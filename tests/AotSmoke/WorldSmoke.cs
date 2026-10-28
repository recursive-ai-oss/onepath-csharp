using System;
using System.Collections.Generic;
using System.Text;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;
using RecursiveAI.OnePath.Mesh.Queries;

/// <summary>Authored off-mesh links, agent classes, tile streaming and glTF input, for the NativeAOT and WebAssembly smoke apps.</summary>
internal static class WorldSmoke
{
    public static bool Run()
    {
        // Two islands, a link between them, two agent classes.
        double[] v = { 0, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0, 20, 0, 0, 20, 0, 10, 30, 0, 10, 30, 0, 0 };
        int[] ix = { 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 };
        var link = OffMeshLinkDefinition.Create(new Vector3D(9, 0, 5), new Vector3D(21, 0, 5), 7);
        NavSettings a = new() { CellSize = 0.5, CellHeight = 0.25, TileSize = 16, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, OffMeshLinks = new[] { link } };
        NavWorld world = NavWorld.Build(v, ix, a, a with { AgentRadius = 0.3f });
        bool ok = world.ClassCount == 2 && world.ClassFor(0.4) == 0;
        PathResult r = world[0].FindPath(new Vector3D(2, 0, 5), new Vector3D(28, 0, 5));
        ok &= r.Status == PathStatus.Found;
        WorldCrowd crowd = world.CreateCrowd();
        Agent agent = crowd.Add(new Vector3D(2, 0, 5), Agent.Humanoid);
        agent.SetDestination(new Vector3D(28, 0, 5));
        for (int i = 0; i < 3000 && !agent.ReachedTarget; i++)
        {
            crowd.Step(1.0 / 30);
        }
        ok &= agent.State == FollowerState.Arrived && agent.Position.X > 26;

        // Streaming a tile column out and back in.
        TileStreamer st = world[0].CreateStreamer();
        string before = Hash(world[0].Tiles);
        List<byte[]> held = st.StreamOut(1, 0);
        ok &= held.Count == 1 && world[0].FindPath(new Vector3D(2, 0, 5), new Vector3D(28, 0, 5)).Status != PathStatus.Found;
        st.QueueAdd(held[0]);
        ok &= st.Update(4) == 1 && Hash(world[0].Tiles) == before;

        // glTF: a .gltf with a data URI builds the same mesh as the soup.
        var bin = new List<byte>();
        foreach (double d in v)
        {
            bin.AddRange(BitConverter.GetBytes((float)d));
        }
        foreach (int i in ix)
        {
            bin.AddRange(BitConverter.GetBytes((ushort)i));
        }
        string json = "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"nodes\":[{\"mesh\":0}],\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0},\"indices\":1}]}],\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":8,\"type\":\"VEC3\"},{\"bufferView\":1,\"componentType\":5123,\"count\":12,\"type\":\"SCALAR\"}],\"bufferViews\":[{\"buffer\":0,\"byteLength\":96},{\"buffer\":0,\"byteOffset\":96,\"byteLength\":24}],\"buffers\":[{\"byteLength\":120,\"uri\":\"data:application/octet-stream;base64," + Convert.ToBase64String(bin.ToArray()) + "\"}]}";
        NavMesh fromGltf = NavMesh.BuildGltf(Encoding.UTF8.GetBytes(json), a);
        ok &= Hash(fromGltf.Tiles) == Hash(NavMesh.Build(v, ix, a).Tiles);
        try
        {
            GltfReader.Read(Encoding.UTF8.GetBytes("{\"asset\":{\"version\":\"1.0\"}}"));
            ok = false;
        }
        catch (GltfException)
        {
        }
        return ok;
    }

    private static string Hash(TiledNavMesh m)
    {
        var all = new List<byte>();
        var refs = new TileRef[4];
        for (int z = 0; z < 2; z++)
        {
            for (int x = 0; x < 3; x++)
            {
                int n = m.GetTilesAt(x, z, refs);
                for (int i = 0; i < n; i++)
                {
                    all.AddRange(NavMeshSerializer.WriteTile(m.GetTile(refs[i])!));
                }
            }
        }
        return Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(all.ToArray()));
    }
}
