using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>Scenarios for the rest of the 3D mesh pipeline: authored off-mesh links, agent classes, tile streaming and glTF input.</summary>
internal static class Build3DWorldScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("build3d/links", Links);
        Scenarios.Register("build3d/world", World);
        Scenarios.Register("build3d/streaming", Streaming);
        Scenarios.Register("build3d/gltf", Gltf);
    }

    private static void Floor(List<double> v, List<int> ix, double x0, double z0, double x1, double z1, double y)
    {
        int a = v.Count / 3;
        v.AddRange([x0, y, z0, x1, y, z0, x1, y, z1, x0, y, z1]);
        ix.AddRange([a, a + 3, a + 2, a, a + 2, a + 1]);
    }

    private static void Box(List<double> v, List<int> ix, double x0, double y0, double z0, double x1, double y1, double z1)
    {
        int a = v.Count / 3;
        for (int i = 0; i < 8; i++)
        {
            v.AddRange([(i & 1) == 0 ? x0 : x1, (i & 4) == 0 ? y0 : y1, (i & 2) == 0 ? z0 : z1]);
        }
        int[][] quads = [[4, 6, 7, 5], [0, 1, 3, 2], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]];
        foreach (int[] q in quads)
        {
            ix.AddRange([a + q[0], a + q[1], a + q[2], a + q[0], a + q[2], a + q[3]]);
        }
    }

    private static NavSettings Tiled => new() { CellSize = 0.25, CellHeight = 0.1, TileSize = 16, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, DetailSampleDistance = 1, DetailMaxError = 0.1 };

    private static void Mesh(ScenarioContext ctx, TiledNavMesh m)
    {
        byte[] bytes = NavMeshSerializer.WriteMesh(m);
        ctx.Write(bytes.Length);
        ctx.Write(bytes);
    }

    private static void Path(ScenarioContext ctx, TiledNavMesh m, Vector3D a, Vector3D b)
    {
        PathResult r = m.FindPath(a, b);
        ctx.Write((int)r.Status);
        ctx.Write(r.Points.Count);
        foreach (StraightPathPoint p in r.Points)
        {
            ctx.Write(p.Position.X);
            ctx.Write(p.Position.Y);
            ctx.Write(p.Position.Z);
            ctx.Write((int)p.Flags);
        }
        ctx.Write(r.Cost);
    }

    private static void Links(ScenarioContext ctx)
    {
        var v = new List<double>();
        var ix = new List<int>();
        Floor(v, ix, 0, 0, 10, 10, 0);
        Floor(v, ix, 20, 0, 30, 10, 0.3);
        OffMeshLinkDefinition jump = OffMeshLinkDefinition.Create(new Vector3D(9, 0, 5), new Vector3D(21, 0.3, 5), 42) with { Radius = 0.6, Area = 4, Flags = 3 };
        OffMeshLinkDefinition ladder = OffMeshLinkDefinition.Create(new Vector3D(1, 0, 1), new Vector3D(29, 0.3, 9), 43).OneWay();
        NavSettings s = Tiled with { OffMeshLinks = [jump, ladder] };
        var pipeline = new Mesh3DPipeline(s);
        TileCache c = pipeline.BuildSoupCached(v.ToArray(), ix.ToArray());
        NavMesh m = NavMesh.Build(v.ToArray(), ix.ToArray(), s);
        Mesh(ctx, m.Tiles);
        Mesh(ctx, c.Mesh);
        Path(ctx, m.Tiles, new Vector3D(2, 0, 5), new Vector3D(28, 0.3, 5));
        Path(ctx, m.Tiles, new Vector3D(28, 0.3, 5), new Vector3D(2, 0, 5));
        int o = c.AddObstacle(TileObstacle.Cylinder(5, 0, 8, 1.2, 2));
        c.UpdateAll();
        Mesh(ctx, c.Mesh);
        c.RemoveObstacle(o);
        c.UpdateAll();
        Mesh(ctx, c.Mesh);
        Path(ctx, c.Mesh, new Vector3D(2, 0, 5), new Vector3D(28, 0.3, 5));
    }

    private static void World(ScenarioContext ctx)
    {
        var v = new List<double>();
        var ix = new List<int>();
        Floor(v, ix, 0, 0, 20, 20, 0);
        Box(v, ix, 9, 0, 0, 11, 3, 9.5);
        Box(v, ix, 9, 0, 10.5, 11, 3, 20);
        NavWorld w = NavWorld.Build(v.ToArray(), ix.ToArray(), NavSettings.Small, NavSettings.Humanoid);
        for (int i = 0; i < w.ClassCount; i++)
        {
            Mesh(ctx, w[i].Tiles);
            Path(ctx, w[i].Tiles, new Vector3D(2, 0, 10), new Vector3D(18, 0, 10));
        }
        WorldCrowd crowd = w.CreateCrowd();
        Agent a = crowd.Add(new Vector3D(2, 0, 10), Agent.Small);
        Agent b = crowd.Add(new Vector3D(2, 0, 13), Agent.Humanoid);
        a.SetDestination(new Vector3D(18, 0, 10));
        b.SetDestination(new Vector3D(18, 0, 10));
        for (int i = 0; i < 200; i++)
        {
            crowd.Step(1.0 / 30);
        }
        foreach (Agent g in new[] { a, b })
        {
            ctx.Write(g.Position.X);
            ctx.Write(g.Position.Y);
            ctx.Write(g.Position.Z);
            ctx.Write((int)g.State);
        }
    }

    private static void Streaming(ScenarioContext ctx)
    {
        var v = new List<double>();
        var ix = new List<int>();
        Floor(v, ix, 0, 0, 48, 16, 0);
        NavMesh m = NavMesh.Build(v.ToArray(), ix.ToArray(), Tiled);
        Crowd crowd = m.CreateCrowd();
        Agent a = crowd.Add(new Vector3D(2, 0, 8));
        a.SetDestination(new Vector3D(46, 0, 8));
        TileStreamer st = m.CreateStreamer();
        var held = new List<byte[]>();
        for (int i = 0; i < 400; i++)
        {
            if (i == 5)
            {
                held.AddRange(st.StreamOut(2, 0));
                ctx.Write(held[0].Length);
                ctx.Write(held[0]);
            }
            if (i == 50)
            {
                st.QueueAdd(held[0]);
                st.QueueRemove(0, 0);
                st.QueueAdd(held[0]);
            }
            ctx.Write(st.Update(1));
            crowd.Step(1.0 / 30);
        }
        ctx.Write(a.Position.X);
        ctx.Write(a.Position.Z);
        ctx.Write((int)a.State);
        Mesh(ctx, m.Tiles);
        byte[] snap = m.Tiles.Snapshot();
        ctx.Write(snap.Length);
        ctx.Write(snap);
    }

    private static byte[] Glb(double[] pos, int[] idx, string node)
    {
        var bin = new List<byte>();
        foreach (double d in pos)
        {
            bin.AddRange(BitConverter.GetBytes((float)d));
        }
        foreach (int i in idx)
        {
            bin.AddRange(BitConverter.GetBytes((ushort)i));
        }
        while (bin.Count % 4 != 0)
        {
            bin.Add(0);
        }
        string json = "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"nodes\":[" + node + "],\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0},\"indices\":1}]}],"
            + "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":" + (pos.Length / 3) + ",\"type\":\"VEC3\"},{\"bufferView\":1,\"componentType\":5123,\"count\":" + idx.Length + ",\"type\":\"SCALAR\"}],"
            + "\"bufferViews\":[{\"buffer\":0,\"byteLength\":" + (pos.Length * 4) + "},{\"buffer\":0,\"byteOffset\":" + (pos.Length * 4) + ",\"byteLength\":" + (idx.Length * 2) + "}],\"buffers\":[{\"byteLength\":" + bin.Count + "}]}";
        byte[] j = Encoding.UTF8.GetBytes(json);
        int jp = (j.Length + 3) & ~3;
        var o = new List<byte>();
        o.AddRange(BitConverter.GetBytes(0x46546C67u));
        o.AddRange(BitConverter.GetBytes(2u));
        o.AddRange(BitConverter.GetBytes((uint)(12 + 8 + jp + 8 + bin.Count)));
        o.AddRange(BitConverter.GetBytes((uint)jp));
        o.AddRange(BitConverter.GetBytes(0x4E4F534Au));
        o.AddRange(j);
        for (int i = j.Length; i < jp; i++)
        {
            o.Add(0x20);
        }
        o.AddRange(BitConverter.GetBytes((uint)bin.Count));
        o.AddRange(BitConverter.GetBytes(0x004E4942u));
        o.AddRange(bin);
        return o.ToArray();
    }

    private static void Gltf(ScenarioContext ctx)
    {
        var v = new List<double>();
        var ix = new List<int>();
        Floor(v, ix, 0, 0, 30, 20, 0);
        Box(v, ix, 10, 0, 5, 14, 2, 12);
        // Transform with a non-trivial rotation (a unit quaternion with exact components), a scale and a translation.
        foreach (string node in new[] { "{\"mesh\":0}", "{\"mesh\":0,\"rotation\":[0,1,0,0],\"translation\":[31.5,0,20.25]}", "{\"mesh\":0,\"scale\":[-1,1,1.5],\"translation\":[40,0.5,0]}" })
        {
            GltfScene sc = GltfReader.Read(Glb(v.ToArray(), ix.ToArray(), node));
            ctx.Write(sc.TriangleCount);
            foreach (double d in sc.Vertices)
            {
                ctx.Write(d);
            }
            foreach (int i in sc.Indices)
            {
                ctx.Write(i);
            }
            Mesh(ctx, NavMesh.Build(sc, Tiled).Tiles);
        }
    }
}
