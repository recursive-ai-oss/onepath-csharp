using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// Fuzz targets for glTF input and for link authoring with tile streaming.
/// <c>gltf</c>: the bytes are a file; only <see cref="GltfException"/> may come out, and what comes out is well formed (indices in range); a small result
/// with ordinary coordinates is also built into a mesh, which must validate.
/// <c>links-streaming</c>: the bytes are a script over a two-island mesh with authored links: build with hostile link numbers (documented argument errors only),
/// stream tiles out and in (also with damaged bytes, which must be typed <see cref="NavMeshFormatException"/>), switch links, query; every mesh validates and
/// streaming everything back gives the original tiles.
/// </summary>
internal static class GltfFuzz
{
    private static readonly double[] Special = { double.NaN, double.PositiveInfinity, double.NegativeInfinity, 3e6, -3e6, 1e300, -0.0, 4.9e-324, 2097152.0, 0.0 };

    public static void Gltf(byte[] data)
    {
        GltfScene sc;
        try
        {
            sc = GltfReader.Read(data, new GltfOptions { MaxTriangles = 2000, MaxNodes = 2000 });
        }
        catch (GltfException)
        {
            return;
        }
        if (sc.Indices.Length % 3 != 0 || sc.Vertices.Length % 3 != 0)
        {
            throw new InvalidOperationException("glTF scene arrays are not whole triangles/vertices");
        }
        foreach (int i in sc.Indices)
        {
            if ((uint)i >= (uint)(sc.Vertices.Length / 3))
            {
                throw new InvalidOperationException("glTF scene index out of range");
            }
        }
        if (sc.TriangleCount == 0 || sc.TriangleCount > 64)
        {
            return;
        }
        foreach (double d in sc.Vertices)
        {
            if (!(Math.Abs(d) < 64))
            {
                return;
            }
        }
        NavMesh m = NavMesh.Build(sc, new NavSettings { CellSize = 0.5, CellHeight = 0.25, TileSize = 16, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, OriginX = -64, OriginZ = -64 });
        Check(m.Tiles);
    }

    private static void Check(TiledNavMesh m)
    {
        IReadOnlyList<string> problems = NavMeshValidator.Validate(m);
        if (problems.Count > 0)
        {
            throw new InvalidOperationException("mesh does not validate: " + problems[0]);
        }
    }

    private static readonly double[] s_floors = { 0, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0, 20, 0, 0, 20, 0, 10, 30, 0, 10, 30, 0, 0, 40, 0, 0, 40, 0, 10, 50, 0, 10, 50, 0, 0 };
    private static readonly int[] s_index = { 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10, 8, 10, 11 };

    private static double Num(byte[] d, ref int p)
    {
        int v = (p < d.Length ? d[p++] : 0) | ((p < d.Length ? d[p++] : 0) << 8);
        if ((v & 0xF000) == 0xF000)
        {
            return Special[((v >> 4) & 0xFF) % Special.Length];
        }
        return ((v & 0x3FF) / 8.0) - 8.0;
    }

    public static void LinksStreaming(byte[] data)
    {
        if (data.Length < 2)
        {
            return;
        }
        int p = 1;
        var links = new List<OffMeshLinkDefinition>();
        int n = data[0] % 4;
        for (int i = 0; i < n; i++)
        {
            links.Add(new OffMeshLinkDefinition(
                new Vector3D(Num(data, ref p) + 8, Num(data, ref p) / 8, Num(data, ref p) + 4),
                new Vector3D(Num(data, ref p) + 28, Num(data, ref p) / 8, Num(data, ref p) + 4),
                Math.Abs(Num(data, ref p)) / 2,
                (data.Length > p ? data[p++] : 0) % 2 == 0,
                (byte)((data.Length > p ? data[p++] : 0) % 70),
                (ushort)(data.Length > p ? data[p++] : 0),
                (uint)i));
        }
        NavMesh m;
        try
        {
            m = NavMesh.Build(s_floors, s_index, new NavSettings { CellSize = 0.5, CellHeight = 0.25, TileSize = 16, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, OffMeshLinks = links });
        }
        catch (ArgumentException)
        {
            // Documented: a link with a non-finite or out-of-range number, a radius not above 0, or an area above 63.
            return;
        }
        Check(m.Tiles);
        TileStreamer st = m.CreateStreamer();
        var held = new Dictionary<int, byte[]>();
        var originals = new Dictionary<int, byte[]>();
        for (int c = 0; c < 4; c++)
        {
            TileRef r = m.Tiles.GetTileRef(c, 0, 0);
            if (r.IsSet)
            {
                originals[c] = NavMeshSerializer.WriteTile(m.Tiles.GetTile(r)!);
            }
        }
        int ops = 0;
        while (p < data.Length && ops++ < 40)
        {
            int col = data[p++] % 4;
            int op = p < data.Length ? data[p++] % 7 : 0;
            switch (op)
            {
                case 0:
                    if (!held.ContainsKey(col))
                    {
                        List<byte[]> o = st.StreamOut(col, 0);
                        if (o.Count > 0)
                        {
                            held[col] = o[0];
                        }
                    }
                    break;
                case 1:
                    if (held.Remove(col, out byte[]? b))
                    {
                        st.QueueAdd(b);
                    }
                    break;
                case 2:
                    st.QueueRemove(col, 0);
                    break;
                case 3:
                    if (originals.TryGetValue(col, out byte[]? good))
                    {
                        byte[] bad = (byte[])good.Clone();
                        int at = p < data.Length ? data[p++] * 7 % bad.Length : 0;
                        bad[at] ^= (byte)(p < data.Length ? data[p++] | 1 : 1);
                        st.QueueAdd(p < data.Length && (data[p] & 1) == 0 ? bad[..(at % bad.Length)] : bad);
                    }
                    break;
                case 4:
                    try
                    {
                        st.Update(1 + (p < data.Length ? data[p++] % 3 : 0));
                    }
                    catch (NavMeshFormatException)
                    {
                    }
                    break;
                case 5:
                    for (int s = 0; s < m.Tiles.SlotCount; s++)
                    {
                        NavTile? t = m.Tiles.GetTileBySlot(s);
                        if (t is not null && t.OffMeshLinks.Length > 0)
                        {
                            m.Tiles.SetOffMeshLinkEnabled(m.Tiles.GetSlotRef(s), 0, (p < data.Length ? data[p++] : 0) % 2 == 0);
                        }
                    }
                    break;
                default:
                    PathResult pr = m.FindPath(new Vector3D(2, 0, 5), new Vector3D(48, 0, 5));
                    if (pr.Points.Count > 0 && !double.IsFinite(pr.Length))
                    {
                        throw new InvalidOperationException("non-finite path length");
                    }
                    break;
            }
            Check(m.Tiles);
        }
        // Whatever is still pending runs (damaged bytes stop at the typed error and the rest stays queued), then every held tile returns.
        while (st.PendingCount > 0)
        {
            try
            {
                st.UpdateAll();
            }
            catch (NavMeshFormatException)
            {
            }
        }
        foreach (KeyValuePair<int, byte[]> h in held)
        {
            st.QueueAdd(h.Value);
        }
        st.UpdateAll();
        // Each held column came back exactly as it left (a held tile may itself be a damaged-but-valid variant the script streamed in earlier).
        foreach (KeyValuePair<int, byte[]> h in held)
        {
            TileRef r = m.Tiles.GetTileRef(h.Key, 0, 0);
            if (!r.IsSet || !NavMeshSerializer.WriteTile(m.Tiles.GetTile(r)!).AsSpan().SequenceEqual(h.Value))
            {
                throw new InvalidOperationException($"streamed tile {h.Key} came back different");
            }
        }
        Check(m.Tiles);
    }

    public static void WriteSeeds(string dir)
    {
        Directory.CreateDirectory(dir);
        double[] pos = { 0, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0 };
        int[] idx = { 0, 1, 2, 0, 2, 3 };
        var bin = new List<byte>();
        foreach (double d in pos)
        {
            bin.AddRange(BitConverter.GetBytes((float)d));
        }
        foreach (int i in idx)
        {
            bin.AddRange(BitConverter.GetBytes((ushort)i));
        }
        string json = "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"nodes\":[{\"children\":[1],\"translation\":[1,0,1]},{\"mesh\":0,\"rotation\":[0,1,0,0],\"scale\":[1,1,1]}],\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0},\"indices\":1}]}],"
            + "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":4,\"type\":\"VEC3\"},{\"bufferView\":1,\"componentType\":5123,\"count\":6,\"type\":\"SCALAR\"}],"
            + "\"bufferViews\":[{\"buffer\":0,\"byteLength\":48},{\"buffer\":0,\"byteOffset\":48,\"byteLength\":12}],\"buffers\":[{\"byteLength\":60,\"uri\":\"data:application/octet-stream;base64," + Convert.ToBase64String(bin.ToArray()) + "\"}]}";
        File.WriteAllBytes(Path.Combine(dir, "quad.gltf"), Encoding.UTF8.GetBytes(json));
        byte[] j = Encoding.UTF8.GetBytes(json.Replace(",\"uri\":\"data:application/octet-stream;base64," + Convert.ToBase64String(bin.ToArray()) + "\"", "", StringComparison.Ordinal));
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
        File.WriteAllBytes(Path.Combine(dir, "quad.glb"), o.ToArray());
    }
}
