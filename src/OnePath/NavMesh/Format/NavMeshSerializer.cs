using System;
using System.Buffers.Binary;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.NavMesh;

/// <summary>
/// Reads and writes tiles and whole meshes in the little-endian binary format described in
/// <c>docs/navmesh-format.md</c>. The same mesh always produces the same bytes; every read validates the bytes
/// completely and reports problems as <see cref="NavMeshFormatException"/>, never as an out-of-range access.
/// Stitched links are not stored: they are rebuilt from the tiles.
/// </summary>
public static class NavMeshSerializer
{
    private const uint TileMagic = 0x5450_4E4F; // "ONPT" little-endian
    private const uint MeshMagic = 0x4D50_4E4F; // "ONPM"
    private const int ParamsSize = 60;
    private const int TileFixedHeader = 4 + 2 + 2 + 12 + 24 + (6 * 4);
    private const int PolyRecordSize = 16;
    private const int OffMeshRecordSize = 36;

    private static ushort HeaderFlags => NavMeshFormat.IsUnstable ? NavMeshFormat.FlagUnstable : (ushort)0;

    /// <summary>Serializes one tile.</summary>
    public static byte[] WriteTile(NavTile tile)
    {
        ArgumentNullException.ThrowIfNull(tile);
        var w = new Writer(TileFixedHeader + (tile.Vertices.Length * 4) + (tile.PolyCount * (PolyRecordSize + 4)));
        WriteTileBody(w, tile);
        return w.ToArray();
    }

    /// <summary>Reads and validates one tile built against <paramref name="p"/>.</summary>
    /// <exception cref="NavMeshFormatException">The bytes are not a valid tile for these parameters.</exception>
    public static NavTile ReadTile(ReadOnlySpan<byte> bytes, NavMeshParams p)
    {
        ArgumentNullException.ThrowIfNull(p);
        p.Validate();
        var r = new Reader(bytes);
        NavTile t = ReadTileBody(ref r, p);
        r.ExpectEnd();
        return t;
    }

    /// <summary>Serializes a whole mesh: parameters, then every slot in slot order with its generation.</summary>
    public static byte[] WriteMesh(TiledNavMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        var w = new Writer(256);
        w.U32(MeshMagic);
        w.U16(NavMeshFormat.Version);
        w.U16(HeaderFlags);
        NavMeshParams p = mesh.Params;
        w.F64(p.OriginX);
        w.F64(p.OriginY);
        w.F64(p.OriginZ);
        w.F64(p.TileSize);
        w.F64(p.CellSize);
        w.U16((ushort)p.MaxVertsPerPoly);
        w.U16(0);
        w.F32(p.WalkableClimb);
        w.F32(p.AgentRadius);
        w.F32(p.AgentHeight);
        w.F32(p.AgentMaxSlopeDegrees);
        w.U32((uint)mesh.SlotCount);
        for (int s = 0; s < mesh.SlotCount; s++)
        {
            w.U32(mesh.GetSlotSalt(s));
            NavTile? t = mesh.GetTileBySlot(s);
            if (t is null)
            {
                w.U8(0);
                continue;
            }
            w.U8(1);
            byte[] body = WriteTile(t);
            w.U32((uint)body.Length);
            w.Bytes(body);
        }
        return w.ToArray();
    }

    /// <summary>Reads and validates a whole mesh, including its slot generations, and stitches it.</summary>
    /// <exception cref="NavMeshFormatException">The bytes are malformed.</exception>
    public static TiledNavMesh ReadMesh(ReadOnlySpan<byte> bytes)
    {
        var r = new Reader(bytes);
        if (r.U32("magic") != MeshMagic)
        {
            throw new NavMeshFormatException("Not a navigation mesh file (bad magic).");
        }
        CheckVersion(ref r);
        NavMeshParams p;
        try
        {
            p = new NavMeshParams
            {
                OriginX = r.F64("origin X"),
                OriginY = r.F64("origin Y"),
                OriginZ = r.F64("origin Z"),
                TileSize = r.F64("tile size"),
                CellSize = r.F64("cell size"),
                MaxVertsPerPoly = r.U16("max vertices per polygon"),
                WalkableClimb = SkipReserved(ref r),
                AgentRadius = r.F32("agent radius"),
                AgentHeight = r.F32("agent height"),
                AgentMaxSlopeDegrees = r.F32("agent slope"),
            };
            p.Validate();
        }
        catch (ArgumentOutOfRangeException e)
        {
            throw new NavMeshFormatException("Invalid mesh parameters: " + e.Message, e);
        }
        uint slotCount = r.U32("slot count");
        if (slotCount > NavMeshFormat.MaxTileSlots || slotCount > r.Remaining / 5)
        {
            throw new NavMeshFormatException($"Slot count {slotCount} is not possible for {r.Remaining} remaining bytes.");
        }
        var slots = new List<(uint, NavTile?)>((int)slotCount);
        for (uint s = 0; s < slotCount; s++)
        {
            uint salt = r.U32("slot generation");
            if (salt == 0 || salt > 0xFFFFFF)
            {
                throw new NavMeshFormatException($"Slot {s} has generation {salt}; it must be 1 to 16777215.");
            }
            byte present = r.U8("slot present flag");
            if (present > 1)
            {
                throw new NavMeshFormatException($"Slot {s} has present flag {present}.");
            }
            if (present == 0)
            {
                slots.Add((salt, null));
                continue;
            }
            uint len = r.U32("tile length");
            ReadOnlySpan<byte> body = r.Take((int)Math.Min(len, int.MaxValue), "tile body");
            NavTile tile;
            try
            {
                tile = ReadTile(body, p);
            }
            catch (NavMeshFormatException e)
            {
                throw new NavMeshFormatException($"Slot {s}: {e.Message}", e);
            }
            slots.Add((salt, tile));
        }
        r.ExpectEnd();
        var mesh = new TiledNavMesh(p);
        mesh.RestoreSlots(slots);
        return mesh;
    }

    // The 16-bit pad after MaxVertsPerPoly must be zero; the climb follows it.
    private static float SkipReserved(ref Reader r)
    {
        if (r.U16("reserved") != 0)
        {
            throw new NavMeshFormatException("Reserved field is not zero.");
        }
        return r.F32("walkable climb");
    }

    private static void CheckVersion(ref Reader r)
    {
        ushort v = r.U16("version");
        ushort f = r.U16("flags");
        if (v != NavMeshFormat.Version)
        {
            throw new NavMeshFormatException($"Unsupported format version {v}; this build reads version {NavMeshFormat.Version}.");
        }
        if (f != HeaderFlags)
        {
            throw new NavMeshFormatException($"Unexpected header flags 0x{f:X4}; this build reads 0x{HeaderFlags:X4} (bit 0 marks the draft format).");
        }
    }

    private static void WriteTileBody(Writer w, NavTile t)
    {
        w.U32(TileMagic);
        w.U16(NavMeshFormat.Version);
        w.U16(HeaderFlags);
        w.I32(t.TileX);
        w.I32(t.TileZ);
        w.I32(t.Layer);
        w.F64(t.OriginX);
        w.F64(t.OriginY);
        w.F64(t.OriginZ);
        w.U32((uint)t.VertexCount);
        w.U32((uint)t.PolyCount);
        w.U32((uint)t.PolyIndices.Length);
        w.U32((uint)(t.DetailVertices.Length / 3));
        w.U32((uint)(t.DetailTriangles.Length / 4));
        w.U32((uint)t.OffMeshLinks.Length);
        foreach (float f in t.Vertices)
        {
            w.F32(f);
        }
        for (int i = 0; i < t.PolyCount; i++)
        {
            DetailMesh d = t.DetailMeshes[i];
            w.U8(t.PolyVertexCounts[i]);
            w.U8(t.PolyAreas[i]);
            w.U16(t.PolyFlags[i]);
            w.U32(d.VertexBase);
            w.U32(d.TriangleBase);
            w.U16(d.VertexCount);
            w.U16(d.TriangleCount);
        }
        foreach (ushort i in t.PolyIndices)
        {
            w.U16(i);
        }
        foreach (int n in t.EdgeNeighbors)
        {
            w.I32(n);
        }
        w.Bytes(t.EdgePortalSides);
        foreach (float f in t.DetailVertices)
        {
            w.F32(f);
        }
        w.Bytes(t.DetailTriangles);
        foreach (OffMeshLink l in t.OffMeshLinks)
        {
            w.F32(l.StartX);
            w.F32(l.StartY);
            w.F32(l.StartZ);
            w.F32(l.EndX);
            w.F32(l.EndY);
            w.F32(l.EndZ);
            w.F32(l.Radius);
            w.U8(l.Area);
            w.U8(l.Bidirectional ? (byte)1 : (byte)0);
            w.U16(l.PolyFlags);
            w.U32(l.UserId);
        }
    }

    private static NavTile ReadTileBody(ref Reader r, NavMeshParams p)
    {
        if (r.U32("tile magic") != TileMagic)
        {
            throw new NavMeshFormatException("Not a tile (bad magic).");
        }
        CheckVersion(ref r);
        var d = new NavTileData
        {
            TileX = r.I32("tile X"),
            TileZ = r.I32("tile Z"),
            Layer = r.I32("layer"),
        };
        double ox = r.F64("origin X");
        double oy = r.F64("origin Y");
        double oz = r.F64("origin Z");
        d.OriginY = oy;
        if (BitConverter.DoubleToInt64Bits(ox) != BitConverter.DoubleToInt64Bits(p.TileOriginX(d.TileX))
            || BitConverter.DoubleToInt64Bits(oz) != BitConverter.DoubleToInt64Bits(p.TileOriginZ(d.TileZ)))
        {
            throw new NavMeshFormatException($"Tile ({d.TileX}, {d.TileZ}) stores origin ({ox:R}, {oz:R}), which is not the grid position given by the mesh parameters.");
        }
        uint nv = r.U32("vertex count");
        uint np = r.U32("polygon count");
        uint ni = r.U32("index count");
        uint ndv = r.U32("detail vertex count");
        uint ndt = r.U32("detail triangle count");
        uint nom = r.U32("off-mesh count");

        // Every count is checked against the bytes that remain before anything is allocated.
        ulong need = ((ulong)nv * 12) + ((ulong)np * PolyRecordSize) + ((ulong)ni * 7) + ((ulong)ndv * 12) + ((ulong)ndt * 4) + ((ulong)nom * OffMeshRecordSize);
        if (need != (ulong)r.Remaining)
        {
            throw new NavMeshFormatException($"Tile body is {r.Remaining} bytes but its counts (vertices {nv}, polygons {np}, indices {ni}, detail vertices {ndv}, detail triangles {ndt}, off-mesh links {nom}) need {need}.");
        }

        var verts = new float[nv * 3];
        for (int i = 0; i < verts.Length; i++)
        {
            verts[i] = r.F32("vertex");
        }
        var counts = new byte[np];
        var areas = new byte[np];
        var flags = new ushort[np];
        var dm = new DetailMesh[np];
        for (int i = 0; i < np; i++)
        {
            counts[i] = r.U8("polygon vertex count");
            areas[i] = r.U8("polygon area");
            flags[i] = r.U16("polygon flags");
            dm[i] = new DetailMesh { VertexBase = r.U32("detail base"), TriangleBase = r.U32("detail triangle base"), VertexCount = r.U16("detail vertex count"), TriangleCount = r.U16("detail triangle count") };
        }
        var idx = new ushort[ni];
        for (int i = 0; i < idx.Length; i++)
        {
            idx[i] = r.U16("polygon index");
        }
        var nbr = new int[ni];
        for (int i = 0; i < nbr.Length; i++)
        {
            nbr[i] = r.I32("edge neighbour");
        }
        byte[] sides = r.Take((int)ni, "edge sides").ToArray();
        var dv = new float[ndv * 3];
        for (int i = 0; i < dv.Length; i++)
        {
            dv[i] = r.F32("detail vertex");
        }
        byte[] dt = r.Take((int)(ndt * 4), "detail triangles").ToArray();
        var om = new OffMeshLink[nom];
        for (int i = 0; i < om.Length; i++)
        {
            var l = new OffMeshLink
            {
                StartX = r.F32("off-mesh"),
                StartY = r.F32("off-mesh"),
                StartZ = r.F32("off-mesh"),
                EndX = r.F32("off-mesh"),
                EndY = r.F32("off-mesh"),
                EndZ = r.F32("off-mesh"),
                Radius = r.F32("off-mesh"),
                Area = r.U8("off-mesh area"),
            };
            byte bits = r.U8("off-mesh flags");
            if (bits > 1)
            {
                throw new NavMeshFormatException($"Off-mesh link {i} has unknown flag bits 0x{bits:X2}.");
            }
            l.Bidirectional = bits == 1;
            l.PolyFlags = r.U16("off-mesh poly flags");
            l.UserId = r.U32("off-mesh user id");
            om[i] = l;
        }
        d.Vertices = verts;
        d.PolyVertexCounts = counts;
        d.PolyIndices = idx;
        d.EdgeNeighbors = nbr;
        d.EdgePortalSides = sides;
        d.PolyAreas = areas;
        d.PolyFlags = flags;
        d.DetailMeshes = dm;
        d.DetailVertices = dv;
        d.DetailTriangles = dt;
        d.OffMeshLinks = om;
        try
        {
            return NavTile.Create(d, p);
        }
        catch (ArgumentOutOfRangeException e)
        {
            throw new NavMeshFormatException(e.Message, e);
        }
    }

    private sealed class Writer
    {
        private byte[] _buf;
        private int _len;

        public Writer(int capacity) => _buf = new byte[Math.Max(16, capacity)];

        public byte[] ToArray() => _buf.AsSpan(0, _len).ToArray();

        private Span<byte> Next(int n)
        {
            if (_len + n > _buf.Length)
            {
                Array.Resize(ref _buf, Math.Max(_buf.Length * 2, _len + n));
            }
            Span<byte> s = _buf.AsSpan(_len, n);
            _len += n;
            return s;
        }

        public void U8(byte v) => Next(1)[0] = v;

        public void U16(ushort v) => BinaryPrimitives.WriteUInt16LittleEndian(Next(2), v);

        public void U32(uint v) => BinaryPrimitives.WriteUInt32LittleEndian(Next(4), v);

        public void I32(int v) => BinaryPrimitives.WriteInt32LittleEndian(Next(4), v);

        public void F32(float v) => BinaryPrimitives.WriteInt32LittleEndian(Next(4), BitConverter.SingleToInt32Bits(v));

        public void F64(double v) => BinaryPrimitives.WriteInt64LittleEndian(Next(8), BitConverter.DoubleToInt64Bits(v));

        public void Bytes(ReadOnlySpan<byte> b) => b.CopyTo(Next(b.Length));
    }

    private ref struct Reader
    {
        private readonly ReadOnlySpan<byte> _b;
        private int _pos;

        public Reader(ReadOnlySpan<byte> b)
        {
            _b = b;
            _pos = 0;
        }

        public readonly int Remaining => _b.Length - _pos;

        private ReadOnlySpan<byte> Next(int n, string what)
        {
            if (n < 0 || n > _b.Length - _pos)
            {
                throw new NavMeshFormatException($"Truncated data at byte {_pos}: {what} needs {n} bytes but {_b.Length - _pos} remain.");
            }
            ReadOnlySpan<byte> s = _b.Slice(_pos, n);
            _pos += n;
            return s;
        }

        public ReadOnlySpan<byte> Take(int n, string what) => Next(n, what);

        public byte U8(string what) => Next(1, what)[0];

        public ushort U16(string what) => BinaryPrimitives.ReadUInt16LittleEndian(Next(2, what));

        public uint U32(string what) => BinaryPrimitives.ReadUInt32LittleEndian(Next(4, what));

        public int I32(string what) => BinaryPrimitives.ReadInt32LittleEndian(Next(4, what));

        public float F32(string what) => BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(Next(4, what)));

        public double F64(string what) => BitConverter.Int64BitsToDouble(BinaryPrimitives.ReadInt64LittleEndian(Next(8, what)));

        public readonly void ExpectEnd()
        {
            if (_pos != _b.Length)
            {
                throw new NavMeshFormatException($"{_b.Length - _pos} unexpected trailing bytes after byte {_pos}.");
            }
        }
    }
}
