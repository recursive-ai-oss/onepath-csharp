using System;
using System.Collections.Generic;
using System.Text.Json;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>Thrown when a glTF file is malformed, uses something the reader does not support, or exceeds a limit; the message names where.</summary>
public sealed class GltfException : Exception
{
    /// <summary>Creates the exception with a message.</summary>
    public GltfException(string message)
        : base(message)
    {
    }

    /// <summary>Creates the exception with a message and a cause.</summary>
    public GltfException(string message, Exception inner)
        : base(message, inner)
    {
    }
}

/// <summary>Options of <see cref="GltfReader"/>.</summary>
public sealed class GltfOptions
{
    /// <summary>
    /// Supplies the bytes of a buffer whose <c>uri</c> is not a base64 data URI (typically a <c>.bin</c> file next to the <c>.gltf</c>), or null if it
    /// cannot. The reader never touches the file system itself. Default none: such a buffer is an error.
    /// </summary>
    public Func<string, byte[]?>? ResolveBuffer { get; set; }

    /// <summary>Most triangles the scene may expand to (a mesh instanced by many nodes counts every instance). Default 4 000 000.</summary>
    public int MaxTriangles { get; set; } = 4_000_000;

    /// <summary>Most nodes visited (instances included). Default 1 000 000.</summary>
    public int MaxNodes { get; set; } = 1_000_000;
}

/// <summary>Level geometry read from a glTF file: world-space triangles, Y up, ready for <see cref="NavMesh.Build(ReadOnlySpan{double}, ReadOnlySpan{int}, NavSettings?)"/>.</summary>
public sealed class GltfScene
{
    internal GltfScene(double[] vertices, int[] indices)
    {
        Vertices = vertices;
        Indices = indices;
    }

    /// <summary>x, y, z world coordinates, three doubles per vertex (node transforms applied).</summary>
    public double[] Vertices { get; }

    /// <summary>Three vertex indices per triangle, counter-clockwise seen from the front face (as glTF defines it; a floor seen from above).</summary>
    public int[] Indices { get; }

    /// <summary>The number of triangles.</summary>
    public int TriangleCount => Indices.Length / 3;
}

/// <summary>
/// A minimal glTF 2.0 reader for level geometry: <c>.gltf</c> (JSON, with base64 data URIs or buffers the caller supplies) and <c>.glb</c>.
/// It reads the default scene (or scene 0, or every root node when there are no scenes), applies node transforms (matrix or
/// translation/rotation/scale, composed in double with exact operations only), and collects the POSITION and indices of triangle, strip and fan
/// primitives; points and lines are skipped. A mirroring transform flips the winding, as the glTF specification says. No extensions, no
/// sparse accessors, no skins or animation. See <c>docs/build3d-gltf.md</c>.
/// </summary>
public static class GltfReader
{
    private const int MaxDepth = 256;
    private const uint GlbMagic = 0x46546C67;
    private const uint ChunkJson = 0x4E4F534A;
    private const uint ChunkBin = 0x004E4942;

    /// <summary>Reads a <c>.glb</c> (recognised by its magic) or a <c>.gltf</c> file.</summary>
    /// <param name="data">The file's bytes.</param>
    /// <param name="options">Options; null means the defaults.</param>
    /// <exception cref="GltfException">The file is malformed, unsupported or too large; the message says where.</exception>
    public static GltfScene Read(ReadOnlySpan<byte> data, GltfOptions? options = null)
    {
        options ??= new GltfOptions();
        if (options.MaxTriangles < 0 || options.MaxNodes < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(options), "MaxTriangles and MaxNodes must not be negative.");
        }
        ReadOnlySpan<byte> json = data;
        byte[]? glbBin = null;
        if (data.Length >= 4 && System.Buffers.Binary.BinaryPrimitives.ReadUInt32LittleEndian(data) == GlbMagic)
        {
            json = ParseGlb(data, out glbBin);
        }
        JsonDocument doc;
        try
        {
            doc = JsonDocument.Parse(json.ToArray(), new JsonDocumentOptions { MaxDepth = 64, CommentHandling = JsonCommentHandling.Disallow, AllowTrailingCommas = false });
        }
        catch (JsonException e)
        {
            throw new GltfException("The JSON part is not valid: " + e.Message, e);
        }
        using (doc)
        {
            try
            {
                return new Reader(doc.RootElement, glbBin, options).Run();
            }
            catch (Exception e) when (e is InvalidOperationException or FormatException or OverflowException)
            {
                // A value of the wrong JSON kind (or text that is not valid UTF-8) reached a typed getter: the file is malformed, whichever field it was.
                throw new GltfException("The file has a value of the wrong kind or invalid text: " + e.Message, e);
            }
        }
    }

    private static ReadOnlySpan<byte> ParseGlb(ReadOnlySpan<byte> d, out byte[]? bin)
    {
        bin = null;
        if (d.Length < 12)
        {
            throw new GltfException("The .glb header is truncated.");
        }
        uint version = System.Buffers.Binary.BinaryPrimitives.ReadUInt32LittleEndian(d[4..]);
        uint length = System.Buffers.Binary.BinaryPrimitives.ReadUInt32LittleEndian(d[8..]);
        if (version != 2)
        {
            throw new GltfException($"The .glb version is {version}; only 2 is supported.");
        }
        if (length > (uint)d.Length || length < 12)
        {
            throw new GltfException($"The .glb header says {length} bytes but the file has {d.Length}.");
        }
        d = d[..(int)length];
        int pos = 12;
        ReadOnlySpan<byte> json = default;
        bool haveJson = false;
        while (pos < d.Length)
        {
            if (d.Length - pos < 8)
            {
                throw new GltfException($"The .glb chunk header at byte {pos} is truncated.");
            }
            uint len = System.Buffers.Binary.BinaryPrimitives.ReadUInt32LittleEndian(d[pos..]);
            uint type = System.Buffers.Binary.BinaryPrimitives.ReadUInt32LittleEndian(d[(pos + 4)..]);
            pos += 8;
            if (len > (uint)(d.Length - pos))
            {
                throw new GltfException($"The .glb chunk at byte {pos - 8} claims {len} bytes but only {d.Length - pos} remain.");
            }
            ReadOnlySpan<byte> body = d.Slice(pos, (int)len);
            if (type == ChunkJson && !haveJson)
            {
                json = body;
                haveJson = true;
            }
            else if (type == ChunkBin && bin is null)
            {
                bin = body.ToArray();
            }
            // Other chunk types are ignored, as the specification asks.
            pos += (int)len;
            pos = (pos + 3) & ~3;
        }
        if (!haveJson)
        {
            throw new GltfException("The .glb file has no JSON chunk.");
        }
        return json;
    }

    private sealed class Reader
    {
        private readonly JsonElement _root;
        private readonly byte[]? _glbBin;
        private readonly GltfOptions _opt;
        private readonly Dictionary<int, byte[]> _buffers = new();
        private readonly List<double> _verts = new();
        private readonly List<int> _idx = new();
        private int _nodesVisited;

        public Reader(JsonElement root, byte[]? glbBin, GltfOptions opt)
        {
            _root = root;
            _glbBin = glbBin;
            _opt = opt;
        }

        public GltfScene Run()
        {
            if (_root.ValueKind != JsonValueKind.Object)
            {
                throw new GltfException("The top level of the file must be an object.");
            }
            if (!_root.TryGetProperty("asset", out JsonElement asset) || asset.ValueKind != JsonValueKind.Object
                || !asset.TryGetProperty("version", out JsonElement ver) || ver.ValueKind != JsonValueKind.String || !(ver.GetString() ?? "").StartsWith("2.", StringComparison.Ordinal))
            {
                throw new GltfException("asset.version must be a string starting with \"2.\".");
            }
            if (_root.TryGetProperty("extensionsRequired", out JsonElement req) && req.ValueKind == JsonValueKind.Array && req.GetArrayLength() > 0)
            {
                throw new GltfException($"The file requires the extension {Describe(req[0])}; no extensions are supported.");
            }
            JsonElement nodes = Arr("nodes");
            var roots = new List<int>();
            if (_root.TryGetProperty("scenes", out JsonElement scenes) && scenes.ValueKind == JsonValueKind.Array && scenes.GetArrayLength() > 0)
            {
                int si = _root.TryGetProperty("scene", out JsonElement sc) ? Index(sc, "scene", scenes.GetArrayLength()) : 0;
                JsonElement scene = scenes[si];
                if (scene.ValueKind == JsonValueKind.Object && scene.TryGetProperty("nodes", out JsonElement sn))
                {
                    RequireArray(sn, $"scenes[{si}].nodes");
                    foreach (JsonElement n in sn.EnumerateArray())
                    {
                        roots.Add(Index(n, $"scenes[{si}].nodes", nodes.GetArrayLength()));
                    }
                }
            }
            else
            {
                // No scenes: every node nothing else lists as a child is a root.
                int count = nodes.GetArrayLength();
                var isChild = new bool[count];
                for (int i = 0; i < count; i++)
                {
                    if (nodes[i].ValueKind == JsonValueKind.Object && nodes[i].TryGetProperty("children", out JsonElement ch) && ch.ValueKind == JsonValueKind.Array)
                    {
                        foreach (JsonElement c in ch.EnumerateArray())
                        {
                            if (c.ValueKind == JsonValueKind.Number && c.TryGetInt32(out int ci) && (uint)ci < (uint)count)
                            {
                                isChild[ci] = true;
                            }
                        }
                    }
                }
                for (int i = 0; i < count; i++)
                {
                    if (!isChild[i])
                    {
                        roots.Add(i);
                    }
                }
            }
            foreach (int r in roots)
            {
                Walk(r, Identity());
            }
            return new GltfScene(_verts.ToArray(), _idx.ToArray());
        }

        // Depth-first, children in listed order; the stack is explicit so a deep or cyclic node graph cannot overflow the call stack.
        private void Walk(int root, double[] parentMatrix)
        {
            JsonElement nodes = Arr("nodes");
            var stack = new Stack<(int Node, double[] Parent, int Depth)>();
            stack.Push((root, parentMatrix, 0));
            while (stack.Count > 0)
            {
                (int ni, double[] parent, int depth) = stack.Pop();
                if (depth > MaxDepth)
                {
                    throw new GltfException($"nodes[{ni}]: the node tree is deeper than {MaxDepth} levels (or contains a cycle).");
                }
                if (++_nodesVisited > _opt.MaxNodes)
                {
                    throw new GltfException($"The scene visits more than {_opt.MaxNodes} nodes.");
                }
                JsonElement node = nodes[ni];
                if (node.ValueKind != JsonValueKind.Object)
                {
                    throw new GltfException($"nodes[{ni}] must be an object.");
                }
                double[] world = Multiply(parent, LocalMatrix(node, ni));
                if (node.TryGetProperty("mesh", out JsonElement mesh))
                {
                    AddMesh(Index(mesh, $"nodes[{ni}].mesh", Arr("meshes").GetArrayLength()), world);
                }
                if (node.TryGetProperty("children", out JsonElement ch))
                {
                    RequireArray(ch, $"nodes[{ni}].children");
                    int n = ch.GetArrayLength();
                    // Pushed in reverse so the first child is walked first.
                    for (int i = n - 1; i >= 0; i--)
                    {
                        stack.Push((Index(ch[i], $"nodes[{ni}].children", nodes.GetArrayLength()), world, depth + 1));
                    }
                }
            }
        }

        private static double[] Identity() => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

        private static double[] Multiply(double[] a, double[] b)
        {
            // Column-major: element (row r, column c) is m[c * 4 + r].
            var m = new double[16];
            for (int c = 0; c < 4; c++)
            {
                for (int r = 0; r < 4; r++)
                {
                    double s = 0;
                    for (int k = 0; k < 4; k++)
                    {
                        s += a[(k * 4) + r] * b[(c * 4) + k];
                    }
                    m[(c * 4) + r] = s;
                }
            }
            return m;
        }

        private static double[] LocalMatrix(JsonElement node, int ni)
        {
            bool hasMatrix = node.TryGetProperty("matrix", out JsonElement mat);
            bool hasTrs = node.TryGetProperty("translation", out JsonElement t) | node.TryGetProperty("rotation", out JsonElement r) | node.TryGetProperty("scale", out JsonElement s);
            if (hasMatrix && hasTrs)
            {
                throw new GltfException($"nodes[{ni}] has both a matrix and translation/rotation/scale.");
            }
            if (hasMatrix)
            {
                return Numbers(mat, 16, $"nodes[{ni}].matrix");
            }
            if (!hasTrs)
            {
                return Identity();
            }
            double[] tv = node.TryGetProperty("translation", out t) ? Numbers(t, 3, $"nodes[{ni}].translation") : [0, 0, 0];
            double[] q = node.TryGetProperty("rotation", out r) ? Numbers(r, 4, $"nodes[{ni}].rotation") : [0, 0, 0, 1];
            double[] sv = node.TryGetProperty("scale", out s) ? Numbers(s, 3, $"nodes[{ni}].scale") : [1, 1, 1];
            double x = q[0], y = q[1], z = q[2], w = q[3];
            double n2 = (x * x) + (y * y) + (z * z) + (w * w);
            if (!(Math.Abs(n2 - 1) < 1e-3))
            {
                throw new GltfException($"nodes[{ni}].rotation is not a unit quaternion (squared length {n2:R}).");
            }
            // Rotation matrix of a unit quaternion (columns), then scaled by column, then translated.
            double[] m =
            [
                (1 - (2 * ((y * y) + (z * z)))) * sv[0], 2 * ((x * y) + (z * w)) * sv[0], 2 * ((x * z) - (y * w)) * sv[0], 0,
                2 * ((x * y) - (z * w)) * sv[1], (1 - (2 * ((x * x) + (z * z)))) * sv[1], 2 * ((y * z) + (x * w)) * sv[1], 0,
                2 * ((x * z) + (y * w)) * sv[2], 2 * ((y * z) - (x * w)) * sv[2], (1 - (2 * ((x * x) + (y * y)))) * sv[2], 0,
                tv[0], tv[1], tv[2], 1,
            ];
            return m;
        }

        private static double[] Numbers(JsonElement e, int count, string path)
        {
            if (e.ValueKind != JsonValueKind.Array || e.GetArrayLength() != count)
            {
                throw new GltfException($"{path} must be an array of {count} numbers.");
            }
            var v = new double[count];
            int i = 0;
            foreach (JsonElement n in e.EnumerateArray())
            {
                if (n.ValueKind != JsonValueKind.Number || !n.TryGetDouble(out double d) || !double.IsFinite(d))
                {
                    throw new GltfException($"{path}[{i}] must be a finite number.");
                }
                v[i++] = d;
            }
            return v;
        }

        private void AddMesh(int mi, double[] world)
        {
            JsonElement mesh = Arr("meshes")[mi];
            if (mesh.ValueKind != JsonValueKind.Object || !mesh.TryGetProperty("primitives", out JsonElement prims) || prims.ValueKind != JsonValueKind.Array)
            {
                throw new GltfException($"meshes[{mi}] must have a primitives array.");
            }
            double det = (world[0] * ((world[5] * world[10]) - (world[6] * world[9])))
                - (world[4] * ((world[1] * world[10]) - (world[2] * world[9])))
                + (world[8] * ((world[1] * world[6]) - (world[2] * world[5])));
            bool flip = det < 0;
            int pi = 0;
            foreach (JsonElement prim in prims.EnumerateArray())
            {
                string path = $"meshes[{mi}].primitives[{pi++}]";
                if (prim.ValueKind != JsonValueKind.Object)
                {
                    throw new GltfException($"{path} must be an object.");
                }
                int mode = 4;
                if (prim.TryGetProperty("mode", out JsonElement me))
                {
                    if (me.ValueKind != JsonValueKind.Number || !me.TryGetInt32(out mode) || mode < 0 || mode > 6)
                    {
                        throw new GltfException($"{path}.mode must be an integer 0 to 6.");
                    }
                }
                if (mode < 4)
                {
                    continue;
                }
                if (!prim.TryGetProperty("attributes", out JsonElement attrs) || attrs.ValueKind != JsonValueKind.Object || !attrs.TryGetProperty("POSITION", out JsonElement pos))
                {
                    throw new GltfException($"{path} has no POSITION attribute.");
                }
                int pa = Index(pos, path + ".attributes.POSITION", Arr("accessors").GetArrayLength());
                int vcount = ReadPositions(pa, path, out double[] local);
                int[]? indices = null;
                if (prim.TryGetProperty("indices", out JsonElement ie))
                {
                    indices = ReadIndices(Index(ie, path + ".indices", Arr("accessors").GetArrayLength()), vcount, path);
                }
                int n = indices?.Length ?? vcount;
                long tris = mode == 4 ? n / 3 : Math.Max(0, n - 2);
                if ((_idx.Count / 3) + tris > _opt.MaxTriangles)
                {
                    throw new GltfException($"{path}: the scene expands to more than {_opt.MaxTriangles} triangles.");
                }
                int baseV = _verts.Count / 3;
                for (int v = 0; v < vcount; v++)
                {
                    double x = local[3 * v], y = local[(3 * v) + 1], z = local[(3 * v) + 2];
                    _verts.Add((world[0] * x) + (world[4] * y) + (world[8] * z) + world[12]);
                    _verts.Add((world[1] * x) + (world[5] * y) + (world[9] * z) + world[13]);
                    _verts.Add((world[2] * x) + (world[6] * y) + (world[10] * z) + world[14]);
                }
                int At(int i) => baseV + (indices is null ? i : indices[i]);
                for (int t = 0; t < tris; t++)
                {
                    // Strips alternate winding; a fan turns about its first vertex.
                    int a, b, c;
                    switch (mode)
                    {
                        case 4:
                            (a, b, c) = (At(3 * t), At((3 * t) + 1), At((3 * t) + 2));
                            break;
                        case 5:
                            (a, b, c) = (t & 1) == 0 ? (At(t), At(t + 1), At(t + 2)) : (At(t + 1), At(t), At(t + 2));
                            break;
                        default:
                            (a, b, c) = (At(0), At(t + 1), At(t + 2));
                            break;
                    }
                    if (flip)
                    {
                        (b, c) = (c, b);
                    }
                    _idx.Add(a);
                    _idx.Add(b);
                    _idx.Add(c);
                }
            }
        }

        // ---------------------------------------------------------------- accessors

        private int ReadPositions(int ai, string path, out double[] values)
        {
            Accessor a = Open(ai, path);
            if (a.ComponentType != 5126 || a.Components != 3)
            {
                throw new GltfException($"{path}: POSITION accessor {ai} must be float VEC3.");
            }
            values = new double[checked(a.Count * 3)];
            for (int i = 0; i < a.Count; i++)
            {
                int at = a.Offset + (i * a.Stride);
                for (int k = 0; k < 3; k++)
                {
                    float f = BitConverter.Int32BitsToSingle(System.Buffers.Binary.BinaryPrimitives.ReadInt32LittleEndian(a.Data.AsSpan(at + (4 * k))));
                    if (!float.IsFinite(f))
                    {
                        throw new GltfException($"{path}: POSITION accessor {ai} element {i} is not finite.");
                    }
                    values[(3 * i) + k] = f;
                }
            }
            return a.Count;
        }

        private int[] ReadIndices(int ai, int vertexCount, string path)
        {
            Accessor a = Open(ai, path);
            if (a.Components != 1 || (a.ComponentType != 5121 && a.ComponentType != 5123 && a.ComponentType != 5125))
            {
                throw new GltfException($"{path}: indices accessor {ai} must be a scalar of unsigned byte, short or int.");
            }
            var r = new int[a.Count];
            for (int i = 0; i < a.Count; i++)
            {
                ReadOnlySpan<byte> s = a.Data.AsSpan(a.Offset + (i * a.Stride));
                uint v = a.ComponentType == 5121 ? s[0] : a.ComponentType == 5123 ? System.Buffers.Binary.BinaryPrimitives.ReadUInt16LittleEndian(s) : System.Buffers.Binary.BinaryPrimitives.ReadUInt32LittleEndian(s);
                if (v >= (uint)vertexCount)
                {
                    throw new GltfException($"{path}: indices accessor {ai} element {i} is {v} but the primitive has {vertexCount} vertices.");
                }
                r[i] = (int)v;
            }
            return r;
        }

        private readonly record struct Accessor(byte[] Data, int Offset, int Stride, int Count, int ComponentType, int Components);

        // Resolves an accessor to a byte range of a loaded buffer, with every bound checked in long arithmetic before anything is read.
        private Accessor Open(int ai, string path)
        {
            JsonElement a = Arr("accessors")[ai];
            if (a.ValueKind != JsonValueKind.Object)
            {
                throw new GltfException($"accessors[{ai}] must be an object.");
            }
            if (a.TryGetProperty("sparse", out _))
            {
                throw new GltfException($"{path}: accessors[{ai}] is sparse; sparse accessors are not supported.");
            }
            if (!a.TryGetProperty("bufferView", out JsonElement bvE))
            {
                throw new GltfException($"{path}: accessors[{ai}] has no bufferView.");
            }
            int ct = Int(a, "componentType", $"accessors[{ai}]", 5120, 5126);
            int count = Int(a, "count", $"accessors[{ai}]", 1, int.MaxValue / 4);
            int aoff = a.TryGetProperty("byteOffset", out _) ? Int(a, "byteOffset", $"accessors[{ai}]", 0, int.MaxValue) : 0;
            string type = a.TryGetProperty("type", out JsonElement te) && te.ValueKind == JsonValueKind.String ? te.GetString()! : "";
            int comps = type switch { "SCALAR" => 1, "VEC2" => 2, "VEC3" => 3, "VEC4" => 4, _ => throw new GltfException($"accessors[{ai}].type \"{type}\" is not supported (SCALAR, VEC2, VEC3, VEC4).") };
            int csize = ct switch { 5120 or 5121 => 1, 5122 or 5123 => 2, 5125 or 5126 => 4, _ => throw new GltfException($"accessors[{ai}].componentType {ct} is not valid.") };
            int bvi = Index(bvE, $"accessors[{ai}].bufferView", Arr("bufferViews").GetArrayLength());
            JsonElement bv = Arr("bufferViews")[bvi];
            if (bv.ValueKind != JsonValueKind.Object)
            {
                throw new GltfException($"bufferViews[{bvi}] must be an object.");
            }
            int bufIndex = Int(bv, "buffer", $"bufferViews[{bvi}]", 0, int.MaxValue);
            int bvoff = bv.TryGetProperty("byteOffset", out _) ? Int(bv, "byteOffset", $"bufferViews[{bvi}]", 0, int.MaxValue) : 0;
            int bvlen = Int(bv, "byteLength", $"bufferViews[{bvi}]", 1, int.MaxValue);
            int stride = bv.TryGetProperty("byteStride", out _) ? Int(bv, "byteStride", $"bufferViews[{bvi}]", 4, 252) : 0;
            byte[] data = Buffer(bufIndex);
            int elem = csize * comps;
            if (stride == 0)
            {
                stride = elem;
            }
            else if (stride < elem)
            {
                throw new GltfException($"bufferViews[{bvi}].byteStride {stride} is smaller than the element size {elem}.");
            }
            long viewEnd = (long)bvoff + bvlen;
            long last = (long)aoff + ((long)(count - 1) * stride) + elem;
            if (viewEnd > data.Length || last > bvlen)
            {
                throw new GltfException($"{path}: accessors[{ai}] reads bytes up to {last} of a view of {bvlen} bytes (buffer {bufIndex} holds {data.Length}); the data is truncated.");
            }
            return new Accessor(data, bvoff + aoff, stride, count, ct, comps);
        }

        private byte[] Buffer(int bi)
        {
            if (_buffers.TryGetValue(bi, out byte[]? have))
            {
                return have;
            }
            JsonElement buffers = Arr("buffers");
            if ((uint)bi >= (uint)buffers.GetArrayLength())
            {
                throw new GltfException($"buffers[{bi}] does not exist (there are {buffers.GetArrayLength()}).");
            }
            JsonElement b = buffers[bi];
            if (b.ValueKind != JsonValueKind.Object)
            {
                throw new GltfException($"buffers[{bi}] must be an object.");
            }
            int declared = Int(b, "byteLength", $"buffers[{bi}]", 1, int.MaxValue);
            byte[] data;
            if (!b.TryGetProperty("uri", out JsonElement u))
            {
                if (bi != 0 || _glbBin is null)
                {
                    throw new GltfException($"buffers[{bi}] has no uri and is not the .glb binary chunk.");
                }
                data = _glbBin;
            }
            else
            {
                string uri = u.ValueKind == JsonValueKind.String ? u.GetString()! : throw new GltfException($"buffers[{bi}].uri must be a string.");
                if (uri.StartsWith("data:", StringComparison.OrdinalIgnoreCase))
                {
                    int comma = uri.IndexOf(',', StringComparison.Ordinal);
                    if (comma < 0 || !uri.AsSpan(0, comma).EndsWith(";base64", StringComparison.OrdinalIgnoreCase))
                    {
                        throw new GltfException($"buffers[{bi}].uri is a data URI that is not base64.");
                    }
                    try
                    {
                        data = Convert.FromBase64String(uri[(comma + 1)..]);
                    }
                    catch (FormatException e)
                    {
                        throw new GltfException($"buffers[{bi}].uri holds invalid base64.", e);
                    }
                }
                else
                {
                    data = _opt.ResolveBuffer?.Invoke(uri) ?? throw new GltfException($"buffers[{bi}] refers to \"{uri}\" and no buffer was supplied for it (GltfOptions.ResolveBuffer).");
                }
            }
            if (data.Length < declared)
            {
                throw new GltfException($"buffers[{bi}] declares {declared} bytes but holds {data.Length}.");
            }
            _buffers[bi] = data;
            return data;
        }

        // ---------------------------------------------------------------- json helpers

        private JsonElement Arr(string name)
        {
            if (!_root.TryGetProperty(name, out JsonElement e))
            {
                return EmptyArray();
            }
            RequireArray(e, name);
            return e;
        }

        private static JsonElement s_empty;

        private static JsonElement EmptyArray()
        {
            if (s_empty.ValueKind == JsonValueKind.Undefined)
            {
                using JsonDocument d = JsonDocument.Parse("[]");
                s_empty = d.RootElement.Clone();
            }
            return s_empty;
        }

        private static void RequireArray(JsonElement e, string path)
        {
            if (e.ValueKind != JsonValueKind.Array)
            {
                throw new GltfException($"{path} must be an array.");
            }
        }

        private static int Index(JsonElement e, string path, int count)
        {
            if (e.ValueKind != JsonValueKind.Number || !e.TryGetInt32(out int v) || v < 0 || v >= count)
            {
                throw new GltfException($"{path} must be an integer index below {count}.");
            }
            return v;
        }

        private static int Int(JsonElement o, string name, string path, int min, int max)
        {
            if (!o.TryGetProperty(name, out JsonElement e) || e.ValueKind != JsonValueKind.Number || !e.TryGetInt32(out int v) || v < min || v > max)
            {
                throw new GltfException($"{path}.{name} must be an integer from {min} to {max}.");
            }
            return v;
        }

        private static string Describe(JsonElement e) => e.ValueKind == JsonValueKind.String ? "\"" + e.GetString() + "\"" : "(unnamed)";
    }
}
