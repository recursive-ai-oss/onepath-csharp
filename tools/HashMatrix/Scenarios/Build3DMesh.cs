using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>Scenarios for the polygon, height detail and tile stages of the 3D builder and the tier-1 build: serialized meshes of generated levels.</summary>
internal static class Build3DMeshScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("build3d/polys", Polys);
        Scenarios.Register("build3d/mesh-levels", Levels);
        Scenarios.Register("build3d/mesh-layers-chunk", LayersAndChunk);
    }

    private static ulong s_seed;

    private static int Next(int n)
    {
        s_seed = (s_seed * 6364136223846793005UL) + 1442695040888963407UL;
        return (int)((uint)(s_seed >> 33) % (uint)n);
    }

    private static void Quad(List<double> v, List<int> ix, double x0, double z0, double x1, double z1, double y)
    {
        int a = v.Count / 3;
        v.AddRange(new[] { x0, y, z0, x1, y, z0, x1, y, z1, x0, y, z1 });
        ix.AddRange(new[] { a, a + 3, a + 2, a, a + 2, a + 1 });
    }

    private static void Box(List<double> v, List<int> ix, double x0, double y0, double z0, double x1, double y1, double z1)
    {
        int b = v.Count / 3;
        for (int i = 0; i < 8; i++)
        {
            v.Add((i & 1) == 0 ? x0 : x1);
            v.Add((i & 4) == 0 ? y0 : y1);
            v.Add((i & 2) == 0 ? z0 : z1);
        }
        void Q(int a, int c, int d, int e)
        {
            ix.AddRange(new[] { b + a, b + c, b + d, b + a, b + d, b + e });
        }
        Q(4, 6, 7, 5);
        Q(0, 1, 3, 2);
        Q(0, 4, 5, 1);
        Q(2, 3, 7, 6);
        Q(0, 2, 6, 4);
        Q(1, 5, 7, 3);
    }

    private static void Terrain(List<double> v, List<int> ix, int size)
    {
        for (int z = 0; z < size; z++)
        {
            for (int x = 0; x < size; x++)
            {
                int a = v.Count / 3;
                foreach ((double px, double pz) in new[] { (x, z), (x + 1.0, z), (x + 1.0, z + 1.0), (x, z + 1.0) })
                {
                    v.AddRange(new[] { px, (0.02 * px) + (0.002 * (px - 20) * (px - 20)) + (0.01 * pz) + (((int)px * 7 + (int)pz * 13) % 5 * 0.01), pz });
                }
                ix.AddRange(new[] { a, a + 3, a + 2, a, a + 2, a + 1 });
            }
        }
    }

    private static void WriteMesh(ScenarioContext ctx, NavMesh m)
    {
        byte[] bytes = NavMeshSerializer.WriteMesh(m.Tiles);
        ctx.Write(bytes.Length);
        ctx.Write(bytes);
    }

    private static NavSettings Settings => new() { CellSize = 0.25, CellHeight = 0.1, TileSize = 16, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, DetailSampleDistance = 1, DetailMaxError = 0.1 };

    private static void Polys(ScenarioContext ctx)
    {
        s_seed = 5;
        for (int n = 0; n < 6; n++)
        {
            var hf = new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 1f, 1f, 40, 40, 24));
            for (int c = 0; c < 40 * 40; c++)
            {
                if (Next(100) < 10)
                {
                    continue;
                }
                hf.AddSpan(c, 0, 1 + (Next(10) < 8 ? 0 : 1), 63, 0);
            }
            var pipe = new RegionPipeline(new SpanSettings { WalkableHeight = 3, WalkableClimb = 1, WalkableRadius = 0 }, new RegionSettings { MinRegionArea = 4, MergeRegionArea = 16 }, new ContourSettings { MaxError = 1.1f, MaxEdgeLength = 6 });
            ContourSet cs = pipe.Run(hf, out CompactHeightfield chf);
            var map = new TileCoordinates(0, 40, 0.3, 12, 0.2, 0);
            PolyMesh mesh = Polygonizer.Build(cs, 6, -1, map);
            ctx.Write(mesh.PolygonCount);
            ctx.Write(mesh.VertexCount);
            ctx.Write(mesh.DegenerateEars);
            foreach (int i in mesh.Vertices)
            {
                ctx.Write(i);
            }
            foreach (int i in mesh.PolygonVertexCounts)
            {
                ctx.Write(i);
            }
            foreach (int i in mesh.PolygonIndices)
            {
                ctx.Write(i);
            }
            DetailMeshSet d = DetailMeshBuilder.Build(mesh, chf, map, new DetailSettings { SampleDistance = 2, MaxError = 0.1 });
            ctx.Write(d.Vertices.Length);
            foreach (float f in d.Vertices)
            {
                ctx.Write(f);
            }
            ctx.Write(d.Triangles);
        }
    }

    private static void Levels(ScenarioContext ctx)
    {
        {
            var v = new List<double>();
            var ix = new List<int>();
            Quad(v, ix, -10, -10, 50, 40, 0);
            Box(v, ix, 18, 0, 5, 22, 3, 20);
            Box(v, ix, 30, 0, 25, 34, 2, 28);
            WriteMesh(ctx, NavMesh.Build(v.ToArray(), ix.ToArray(), Settings));
        }
        {
            var v = new List<double>();
            var ix = new List<int>();
            Terrain(v, ix, 40);
            WriteMesh(ctx, NavMesh.Build(v.ToArray(), ix.ToArray(), Settings));
        }
        {
            var v = new List<double>();
            var ix = new List<int>();
            Terrain(v, ix, 30);
            WriteMesh(ctx, NavMesh.Build(v.ToArray(), ix.ToArray(), NavSettings.Small with { OriginX = -3.5, OriginZ = 2.25 }));
        }
    }

    private static void LayersAndChunk(ScenarioContext ctx)
    {
        var v = new List<double>();
        var ix = new List<int>();
        Quad(v, ix, 0, 0, 30, 30, 0);
        Quad(v, ix, 8, 12, 22, 18, 4);
        WriteMesh(ctx, NavMesh.Build(v.ToArray(), ix.ToArray(), Settings with { Partition = PartitionType.Layers }));

        var voxels = new byte[48 * 48 * 4];
        s_seed = 9;
        for (int z = 0; z < 48; z++)
        {
            for (int x = 0; x < 48; x++)
            {
                int h = 2 + (Next(20) == 0 ? 1 : 0);
                for (int y = 0; y < h; y++)
                {
                    voxels[(((z * 48) + x) * 4) + y] = 1;
                }
            }
        }
        WriteMesh(ctx, NavMesh.Build(VoxelChunk.FromDense(48, 48, 4, voxels), Settings));
    }
}
