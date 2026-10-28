using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

/// <summary>Writes small glTF files for the tests.</summary>
internal static class GltfWriter
{
    public static byte[] Buffer(Soup s, bool shortIndices)
    {
        var b = new List<byte>();
        for (int i = 0; i < s.V.Count; i++)
        {
            b.AddRange(BitConverter.GetBytes((float)s.V[i]));
        }
        foreach (int i in s.I)
        {
            b.AddRange(shortIndices ? BitConverter.GetBytes((ushort)i) : BitConverter.GetBytes(i));
        }
        while (b.Count % 4 != 0)
        {
            b.Add(0);
        }
        return b.ToArray();
    }

    // nodesJson is the JSON array of nodes; node 0 must be the one with "mesh": 0 unless the test builds its own.
    public static string Json(Soup s, bool shortIndices, string nodesJson, string? bufferUri, int? bufferLength = null, string sceneNodes = "[0]")
    {
        int vbytes = s.V.Count * 4, ibytes = s.I.Count * (shortIndices ? 2 : 4);
        int total = bufferLength ?? ((vbytes + ibytes + 3) & ~3);
        string uri = bufferUri is null ? "" : $",\"uri\":\"{bufferUri}\"";
        return string.Create(CultureInfo.InvariantCulture, $@"{{""asset"":{{""version"":""2.0""}},""scene"":0,""scenes"":[{{""nodes"":{sceneNodes}}}],""nodes"":{nodesJson},
""meshes"":[{{""primitives"":[{{""attributes"":{{""POSITION"":0}},""indices"":1}}]}}],
""accessors"":[{{""bufferView"":0,""componentType"":5126,""count"":{s.V.Count / 3},""type"":""VEC3""}},{{""bufferView"":1,""componentType"":{(shortIndices ? 5123 : 5125)},""count"":{s.I.Count},""type"":""SCALAR""}}],
""bufferViews"":[{{""buffer"":0,""byteOffset"":0,""byteLength"":{vbytes}}},{{""buffer"":0,""byteOffset"":{vbytes},""byteLength"":{ibytes}}}],
""buffers"":[{{""byteLength"":{total}{uri}}}]}}");
    }

    public static byte[] Glb(string json, byte[] bin)
    {
        byte[] j = Encoding.UTF8.GetBytes(json);
        int jp = (j.Length + 3) & ~3;
        int total = 12 + 8 + jp + 8 + bin.Length;
        var o = new List<byte>();
        o.AddRange(BitConverter.GetBytes(0x46546C67u));
        o.AddRange(BitConverter.GetBytes(2u));
        o.AddRange(BitConverter.GetBytes((uint)total));
        o.AddRange(BitConverter.GetBytes((uint)jp));
        o.AddRange(BitConverter.GetBytes(0x4E4F534Au));
        o.AddRange(j);
        for (int i = j.Length; i < jp; i++)
        {
            o.Add(0x20);
        }
        o.AddRange(BitConverter.GetBytes((uint)bin.Length));
        o.AddRange(BitConverter.GetBytes(0x004E4942u));
        o.AddRange(bin);
        return o.ToArray();
    }

    public const string Plain = "[{\"mesh\":0}]";

    public static byte[] SimpleGlb(Soup s, string nodes = Plain) => Glb(Json(s, true, nodes, null), Buffer(s, true));

    public static byte[] SimpleGltf(Soup s, string nodes = Plain) =>
        Encoding.UTF8.GetBytes(Json(s, false, nodes, "data:application/octet-stream;base64," + Convert.ToBase64String(Buffer(s, false))));
}

public class GltfTests
{
    private static NavSettings Settings => new() { CellSize = 0.25, CellHeight = 0.1, TileSize = 16, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, DetailSampleDistance = 1, DetailMaxError = 0.1 };

    private static Soup Level() => new Soup().Floor(0, 0, 30, 20, 0).Box(10, 0, 5, 14, 2, 12);

    private static string Tiles(NavMesh m) => AuthoredLinkTests.TileHashes(m.Tiles);

    [Fact]
    public void Glb_and_gltf_build_the_same_mesh_as_the_soup()
    {
        Soup s = Level();
        string expected = Tiles(s.Build(Settings));
        Assert.Equal(expected, Tiles(NavMesh.BuildGltf(GltfWriter.SimpleGlb(s), Settings)));
        Assert.Equal(expected, Tiles(NavMesh.BuildGltf(GltfWriter.SimpleGltf(s), Settings)));
    }

    [Fact]
    public void Node_transforms_are_applied()
    {
        Soup s = Level();
        // Translation.
        string moved = Tiles(NavMesh.BuildGltf(GltfWriter.SimpleGlb(s, "[{\"mesh\":0,\"translation\":[16,0,8]}]"), Settings));
        var s2 = new Soup().Floor(16, 8, 46, 28, 0).Box(26, 0, 13, 30, 2, 20);
        // The translation shifts every vertex by whole numbers (exact in float32), so the soup shifted by hand is the same geometry.
        Assert.Equal(Tiles(s2.Build(Settings)), moved);
        // A matrix (column-major) does the same.
        string asMatrix = Tiles(NavMesh.BuildGltf(GltfWriter.SimpleGlb(s, "[{\"mesh\":0,\"matrix\":[1,0,0,0, 0,1,0,0, 0,0,1,0, 16,0,8,1]}]"), Settings));
        Assert.Equal(moved, asMatrix);
        // A parent's transform composes with the child's.
        string nested = Tiles(NavMesh.BuildGltf(GltfWriter.SimpleGlb(s, "[{\"children\":[1],\"translation\":[10,0,0]},{\"mesh\":0,\"translation\":[6,0,8]}]").AsSpan().ToArray(), Settings));
        Assert.Equal(moved, nested);
        // Half turn about Y (rotation (0,1,0,0)) maps (x, z) to (-x, -z) exactly and keeps the floor facing up.
        string turned = Tiles(NavMesh.BuildGltf(GltfWriter.SimpleGlb(s, "[{\"mesh\":0,\"rotation\":[0,1,0,0]}]"), Settings));
        var s3 = new Soup().Floor(-30, -20, 0, 0, 0).Box(-14, 0, -12, -10, 2, -5);
        Assert.Equal(Tiles(s3.Build(Settings)), turned);
        // A mirror flips the winding back: the floor still faces up.
        string mirrored = Tiles(NavMesh.BuildGltf(GltfWriter.SimpleGlb(s, "[{\"mesh\":0,\"scale\":[-1,1,1]}]"), Settings));
        var s4 = new Soup().Floor(-30, 0, 0, 20, 0).Box(-14, 0, 5, -10, 2, 12);
        Assert.Equal(Tiles(s4.Build(Settings)), mirrored);
    }

    [Fact]
    public void Strips_and_fans_become_triangles_and_other_modes_are_skipped()
    {
        // One quad as a strip (a, d, b, c order for an up-facing floor) and as a fan.
        string Doc(int mode, string idx) => "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"nodes\":[{\"mesh\":0}],\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0},\"mode\":" + mode + ",\"indices\":1}]}],"
            + "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":4,\"type\":\"VEC3\"},{\"bufferView\":1,\"componentType\":5121,\"count\":4,\"type\":\"SCALAR\"}],"
            + "\"bufferViews\":[{\"buffer\":0,\"byteLength\":48},{\"buffer\":0,\"byteOffset\":48,\"byteLength\":4}],\"buffers\":[{\"byteLength\":52,\"uri\":\"data:;base64,\"}]}";
        float[] pos = [0, 0, 0, 0, 0, 10, 10, 0, 0, 10, 0, 10]; // a(0,0) d(0,10) b(10,0) c(10,10)
        var bin = new List<byte>();
        foreach (float f in pos)
        {
            bin.AddRange(BitConverter.GetBytes(f));
        }
        byte[] strip = [0, 1, 2, 3], fan = [0, 1, 3, 2];
        GltfScene fromStrip = GltfReader.Read(Encoding.UTF8.GetBytes(Doc(5, "").Replace("data:;base64,", "data:;base64," + Convert.ToBase64String([.. bin, .. strip]), StringComparison.Ordinal).Replace("\"byteLength\":52", "\"byteLength\":52", StringComparison.Ordinal)));
        GltfScene fromFan = GltfReader.Read(Encoding.UTF8.GetBytes(Doc(6, "").Replace("data:;base64,", "data:;base64," + Convert.ToBase64String([.. bin, .. fan]), StringComparison.Ordinal)));
        Assert.Equal(2, fromStrip.TriangleCount);
        Assert.Equal(2, fromFan.TriangleCount);
        // Every triangle faces up (+Y normal) in both.
        foreach (GltfScene sc in new[] { fromStrip, fromFan })
        {
            for (int t = 0; t < 2; t++)
            {
                Vector3D P(int k) => new(sc.Vertices[3 * sc.Indices[(3 * t) + k]], sc.Vertices[(3 * sc.Indices[(3 * t) + k]) + 1], sc.Vertices[(3 * sc.Indices[(3 * t) + k]) + 2]);
                Vector3D n = Vector3D.Cross(P(1) - P(0), P(2) - P(0));
                Assert.True(n.Y > 0, $"triangle {t} faces down");
            }
        }
        // Points (mode 0) read as an empty scene.
        Assert.Equal(0, GltfReader.Read(Encoding.UTF8.GetBytes(Doc(0, "").Replace("data:;base64,", "data:;base64," + Convert.ToBase64String([.. bin, .. strip]), StringComparison.Ordinal))).TriangleCount);
    }

    [Fact]
    public void External_buffers_come_through_the_resolver_only()
    {
        Soup s = Level();
        string json = GltfWriter.Json(s, false, GltfWriter.Plain, "level.bin");
        byte[] file = Encoding.UTF8.GetBytes(json);
        Assert.Contains("level.bin", Assert.Throws<GltfException>(() => GltfReader.Read(file)).Message, StringComparison.Ordinal);
        string? asked = null;
        GltfScene sc = GltfReader.Read(file, new GltfOptions { ResolveBuffer = u => { asked = u; return GltfWriter.Buffer(s, false); } });
        Assert.Equal("level.bin", asked);
        Assert.Equal(s.I.Count / 3, sc.TriangleCount);
    }

    // A .glb whose header length matches the cut file, so only the chunk lengths are wrong.
    private static byte[] FixLength(byte[] glb, int length)
    {
        byte[] cut = glb[..length];
        BitConverter.GetBytes((uint)length).CopyTo(cut, 8);
        return cut;
    }

    private static void Rejects(byte[] file, string part, GltfOptions? o = null)
    {
        var e = Assert.Throws<GltfException>(() => GltfReader.Read(file, o));
        Assert.Contains(part, e.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void Hostile_files_are_typed_errors()
    {
        Soup s = Level();
        byte[] glb = GltfWriter.SimpleGlb(s);
        Rejects([], "JSON");
        Rejects(Encoding.UTF8.GetBytes("[]"), "object");
        Rejects(Encoding.UTF8.GetBytes("{\"asset\":{\"version\":\"1.0\"}}"), "version");
        Rejects(Encoding.UTF8.GetBytes("{\"asset\":{\"version\":\"2.0\"},\"extensionsRequired\":[\"KHR_draco_mesh_compression\"]}"), "KHR_draco");
        Rejects(Encoding.UTF8.GetBytes("{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[3]}],\"nodes\":[{}]}"), "scenes[0].nodes");
        // Truncated bin chunk, bad index, bad base64, sparse, wrong accessor types.
        Rejects(glb[..(glb.Length - 8)], "header");
        Rejects(FixLength(glb, glb.Length - 8), "claims");
        Rejects(GltfWriter.Glb(GltfWriter.Json(s, true, GltfWriter.Plain, null, 100000), GltfWriter.Buffer(s, true)), "declares");
        Rejects(Encoding.UTF8.GetBytes(GltfWriter.Json(s, false, GltfWriter.Plain, "data:application/octet-stream;base64,@@@")), "base64");
        Rejects(GltfWriter.Glb(GltfWriter.Json(s, true, GltfWriter.Plain, null).Replace("\"count\":" + (s.I.Count), "\"count\":" + (s.I.Count + 9999), StringComparison.Ordinal), GltfWriter.Buffer(s, true)), "truncated");
        Rejects(GltfWriter.Glb(GltfWriter.Json(s, true, GltfWriter.Plain, null).Replace("\"type\":\"VEC3\"", "\"type\":\"VEC3\",\"sparse\":{}", StringComparison.Ordinal), GltfWriter.Buffer(s, true)), "sparse");
        Rejects(GltfWriter.Glb(GltfWriter.Json(s, true, GltfWriter.Plain, null).Replace("5126", "5123", StringComparison.Ordinal), GltfWriter.Buffer(s, true)), "POSITION");
        // An index beyond the vertex count.
        byte[] bin = GltfWriter.Buffer(s, true);
        BitConverter.GetBytes((ushort)60000).CopyTo(bin, s.V.Count * 4);
        Rejects(GltfWriter.Glb(GltfWriter.Json(s, true, GltfWriter.Plain, null), bin), "60000");
        // A NaN position.
        byte[] nan = GltfWriter.Buffer(s, true);
        BitConverter.GetBytes(float.NaN).CopyTo(nan, 8);
        Rejects(GltfWriter.Glb(GltfWriter.Json(s, true, GltfWriter.Plain, null), nan), "not finite");
        // A cycle of nodes, a node tree too deep, a bad quaternion, matrix and TRS together.
        Rejects(GltfWriter.SimpleGlb(s, "[{\"children\":[0],\"mesh\":0}]"), "deeper");
        Rejects(GltfWriter.SimpleGlb(s, "[{\"mesh\":0,\"rotation\":[0,0,0,3]}]"), "unit quaternion");
        Rejects(GltfWriter.SimpleGlb(s, "[{\"mesh\":0,\"scale\":[1,1,1],\"matrix\":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]}]"), "both");
        Rejects(GltfWriter.SimpleGlb(s, "[{\"mesh\":7}]"), "nodes[0].mesh");
        // Limits: the instance count multiplies the triangles.
        Rejects(GltfWriter.SimpleGlb(s, "[{\"children\":[1,2,3]},{\"mesh\":0},{\"mesh\":0},{\"mesh\":0}]"), "triangles", new GltfOptions { MaxTriangles = 2 * (s.I.Count / 3) });
        Rejects(GltfWriter.SimpleGlb(s, "[{\"children\":[1,2,3]},{\"mesh\":0},{\"mesh\":0},{\"mesh\":0}]"), "nodes", new GltfOptions { MaxNodes = 2 });
        // Bad glb headers.
        byte[] badVersion = (byte[])glb.Clone();
        badVersion[4] = 1;
        Rejects(badVersion, "version");
        byte[] badLength = (byte[])glb.Clone();
        BitConverter.GetBytes((uint)glb.Length + 100).CopyTo(badLength, 8);
        Rejects(badLength, "header");
    }

    [Fact]
    public void Deep_json_and_a_deep_node_chain_do_not_overflow()
    {
        var sb = new StringBuilder("{\"asset\":{\"version\":\"2.0\"},\"x\":");
        sb.Append('[', 5000).Append(']', 5000).Append('}');
        Rejects(Encoding.UTF8.GetBytes(sb.ToString()), "JSON");
        var nodes = new StringBuilder("[");
        for (int i = 0; i < 100000; i++)
        {
            nodes.Append(i == 0 ? "" : ",").Append("{\"children\":[").Append(i + 1).Append("]}");
        }
        nodes.Append(",{}]");
        Rejects(Encoding.UTF8.GetBytes("{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"nodes\":" + nodes + "}"), "deeper");
    }

    [Fact]
    public void Mutated_and_truncated_files_never_throw_anything_else()
    {
        Soup s = new Soup().Floor(0, 0, 4, 4, 0);
        byte[][] files = [GltfWriter.SimpleGlb(s, "[{\"mesh\":0,\"translation\":[1,0,0],\"scale\":[2,1,2]}]"), GltfWriter.SimpleGltf(s)];
        var rng = new TestRng(11);
        foreach (byte[] f in files)
        {
            for (int len = 0; len < f.Length; len += 1 + (f.Length / 300))
            {
                Probe(f[..len]);
            }
            for (int round = 0; round < 600; round++)
            {
                byte[] m = (byte[])f.Clone();
                for (int k = 0; k < 1 + rng.Int(3); k++)
                {
                    m[rng.Int(m.Length)] = (byte)rng.Int(256);
                }
                Probe(m);
            }
        }
    }

    private static void Probe(byte[] file)
    {
        try
        {
            GltfScene sc = GltfReader.Read(file, new GltfOptions { MaxTriangles = 1000 });
            Assert.Equal(0, sc.Indices.Length % 3);
            foreach (int i in sc.Indices)
            {
                Assert.InRange(i, 0, (sc.Vertices.Length / 3) - 1);
            }
        }
        catch (GltfException)
        {
        }
    }
}
