using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath;

public sealed partial class NavMesh
{
    /// <summary>
    /// Saves the mesh and the <see cref="Settings"/> it was built with as one byte array that <see cref="Load"/> turns back into an equal
    /// <see cref="NavMesh"/>: the same tiles, polygon flags and areas, off-mesh links and settings, so paths, queries and crowds
    /// give the same bits before and after. The array is a small envelope (magic <c>ONNM</c>, version 1, the settings) around the
    /// navigation mesh bytes of <see cref="Mesh.NavMeshSerializer.WriteMesh"/>; the layout is in <c>docs/navmesh-format.md</c>.
    /// The same mesh state always writes the same bytes. Run-time state a host keeps elsewhere (a tile cache, crowds) is not included.
    /// </summary>
    /// <returns>A new array.</returns>
    public byte[] Save() => NavMeshEnvelope.Write(this);

    /// <summary>The number of bytes <see cref="Save"/> writes now. It builds the bytes to count them, so it allocates like <see cref="Save"/>.</summary>
    public int GetSaveSize() => Save().Length;

    /// <summary>Writes the bytes of <see cref="Save"/> into a buffer the caller owns.</summary>
    /// <param name="destination">The buffer; <see cref="GetSaveSize"/> bytes are needed.</param>
    /// <param name="written">The number of bytes written, or 0 when <paramref name="destination"/> is too small (nothing is written then).</param>
    /// <returns>True when the mesh fitted.</returns>
    public bool TrySave(Span<byte> destination, out int written)
    {
        byte[] bytes = Save();
        if (bytes.Length > destination.Length)
        {
            written = 0;
            return false;
        }
        bytes.CopyTo(destination);
        written = bytes.Length;
        return true;
    }

    /// <summary>
    /// Loads a mesh written by <see cref="Save"/>, with the settings it carries. It also accepts the bare navigation mesh bytes of
    /// <see cref="Mesh.NavMeshSerializer.WriteMesh"/>, which carry no settings: then <paramref name="settings"/> is required.
    /// </summary>
    /// <param name="bytes">The saved mesh.</param>
    /// <param name="settings">
    /// The settings the mesh was built with. Null means the ones stored in the bytes (required for bare mesh bytes). When given with a
    /// saved mesh they must describe the same build: every setting that changes tile content or the mesh parameters must match, which
    /// is checked through the build fingerprint each tile carries.
    /// </param>
    /// <returns>The mesh; <see cref="Settings"/> is the stored settings, or <paramref name="settings"/> when it was given.</returns>
    /// <exception cref="Mesh.NavMeshFormatException">The bytes are truncated, not a mesh, inconsistent or hostile (the message says what and where), or are bare mesh bytes and no settings were given.</exception>
    /// <exception cref="Mesh.NavMeshVersionException">The envelope or mesh version is not one this release reads.</exception>
    /// <exception cref="Mesh.NavMeshFingerprintException">The given settings differ from the build the mesh came from.</exception>
    public static NavMesh Load(ReadOnlySpan<byte> bytes, NavSettings? settings = null) => NavMeshEnvelope.Read(bytes, settings);
}

public sealed partial class NavWorld
{
    /// <summary>
    /// Saves every agent class of the world (see <see cref="NavMesh.Save"/> for each mesh) as one byte array for <see cref="Load"/>:
    /// envelope magic <c>ONNW</c>, version 1, the meshes in class order. The same state always writes the same bytes.
    /// </summary>
    /// <returns>A new array.</returns>
    public byte[] Save() => NavMeshEnvelope.WriteWorld(_meshes);

    /// <summary>The number of bytes <see cref="Save"/> writes now. It builds the bytes to count them, so it allocates like <see cref="Save"/>.</summary>
    public int GetSaveSize() => Save().Length;

    /// <summary>Writes the bytes of <see cref="Save"/> into a buffer the caller owns.</summary>
    /// <param name="destination">The buffer; <see cref="GetSaveSize"/> bytes are needed.</param>
    /// <param name="written">The number of bytes written, or 0 when <paramref name="destination"/> is too small (nothing is written then).</param>
    /// <returns>True when the world fitted.</returns>
    public bool TrySave(Span<byte> destination, out int written)
    {
        byte[] bytes = Save();
        if (bytes.Length > destination.Length)
        {
            written = 0;
            return false;
        }
        bytes.CopyTo(destination);
        written = bytes.Length;
        return true;
    }

    /// <summary>Loads a world written by <see cref="Save"/>: the classes in their saved order, each with its stored settings.</summary>
    /// <param name="bytes">The saved world.</param>
    /// <returns>The world.</returns>
    /// <exception cref="Mesh.NavMeshFormatException">The bytes are truncated, not a world, inconsistent or hostile.</exception>
    /// <exception cref="Mesh.NavMeshVersionException">The envelope or a mesh version is not one this release reads.</exception>
    public static NavWorld Load(ReadOnlySpan<byte> bytes) => new(NavMeshEnvelope.ReadWorld(bytes));
}

/// <summary>The tier-1 save envelopes around the frozen navigation mesh bytes (layout in docs/navmesh-format.md).</summary>
internal static class NavMeshEnvelope
{
    private const uint MeshEnvelopeMagic = 0x4D4E_4E4F; // "ONNM"
    private const uint WorldEnvelopeMagic = 0x574E_4E4F; // "ONNW"
    private const ushort Version = 1;
    private const int LinkBytes = (7 * 8) + 1 + 1 + 2 + 4;

    internal static byte[] Write(NavMesh mesh)
    {
        var w = new NavMeshSerializer.Writer(512);
        w.U32(MeshEnvelopeMagic);
        w.U16(Version);
        w.U16(0);
        WriteSettings(w, mesh.Settings);
        byte[] body = NavMeshSerializer.WriteMesh(mesh.Tiles);
        w.U32((uint)body.Length);
        w.Bytes(body);
        return w.ToArray();
    }

    internal static byte[] WriteWorld(NavMesh[] meshes)
    {
        var w = new NavMeshSerializer.Writer(1024);
        w.U32(WorldEnvelopeMagic);
        w.U16(Version);
        w.U16(0);
        w.U32((uint)meshes.Length);
        foreach (NavMesh m in meshes)
        {
            byte[] one = Write(m);
            w.U32((uint)one.Length);
            w.Bytes(one);
        }
        return w.ToArray();
    }

    internal static NavMesh[] ReadWorld(ReadOnlySpan<byte> bytes)
    {
        var r = new NavMeshSerializer.Reader(bytes);
        if (r.U32("magic") != WorldEnvelopeMagic)
        {
            throw new NavMeshFormatException("Not a saved navigation world (bad magic).");
        }
        CheckVersion(ref r);
        uint count = r.U32("class count");
        if (count == 0 || count > r.Remaining / 8)
        {
            throw new NavMeshFormatException($"Class count {count} is not possible for {r.Remaining} remaining bytes.");
        }
        var meshes = new NavMesh[count];
        for (int i = 0; i < meshes.Length; i++)
        {
            uint len = r.U32("class length");
            ReadOnlySpan<byte> body = r.Take((int)Math.Min(len, int.MaxValue), "class body");
            try
            {
                meshes[i] = Read(body, null);
            }
            catch (NavMeshFormatException e)
            {
                throw new NavMeshFormatException($"Agent class {i}: {e.Message}", e);
            }
        }
        r.ExpectEnd();
        return meshes;
    }

    internal static NavMesh Read(ReadOnlySpan<byte> bytes, NavSettings? given)
    {
        var r = new NavMeshSerializer.Reader(bytes);
        uint magic = r.U32("magic");
        if (magic == NavMeshSerializer.MeshMagic)
        {
            // Bare mesh bytes: the caller's settings are the only source of them.
            if (given is null)
            {
                throw new NavMeshFormatException("These are bare navigation mesh bytes, which carry no settings; pass the NavSettings the mesh was built with.");
            }
            TiledNavMesh bare = NavMeshSerializer.ReadMesh(bytes);
            CheckSettings(given, bare, "the supplied settings");
            return new NavMesh(bare, given);
        }
        if (magic != MeshEnvelopeMagic)
        {
            throw new NavMeshFormatException("Not a saved navigation mesh (bad magic).");
        }
        CheckVersion(ref r);
        NavSettings stored = ReadSettings(ref r);
        uint len = r.U32("mesh length");
        ReadOnlySpan<byte> body = r.Take((int)Math.Min(len, int.MaxValue), "mesh bytes");
        r.ExpectEnd();
        TiledNavMesh tiles = NavMeshSerializer.ReadMesh(body);
        CheckSettings(stored, tiles, "the stored settings");
        if (given is not null)
        {
            CheckSettings(given, tiles, "the supplied settings");
        }
        return new NavMesh(tiles, given ?? stored);
    }

    private static void CheckVersion(ref NavMeshSerializer.Reader r)
    {
        ushort v = r.U16("envelope version");
        ushort flags = r.U16("envelope flags");
        if (v != Version)
        {
            throw new NavMeshVersionException(v, Version);
        }
        if (flags != 0)
        {
            throw new NavMeshFormatException($"Unexpected envelope flags 0x{flags:X4}; version {v} requires 0.");
        }
    }

    // The settings and the mesh must describe one build: the parameters the settings produce (grid, agent class and the digest of
    // every content-changing setting) equal the mesh's, which every tile also carries as its fingerprint.
    private static void CheckSettings(NavSettings s, TiledNavMesh mesh, string what)
    {
        NavMeshParams expected;
        try
        {
            expected = s.ToParams();
        }
        catch (ArgumentException e)
        {
            throw new NavMeshFormatException($"{what} cannot describe a mesh: {e.Message}", e);
        }
        var a = new NavMeshSerializer.Writer(160);
        var b = new NavMeshSerializer.Writer(160);
        NavMeshSerializer.WriteParams(a, expected);
        NavMeshSerializer.WriteParams(b, mesh.Params);
        if (!a.ToArray().AsSpan().SequenceEqual(b.ToArray()))
        {
            throw new NavMeshFingerprintException($"The mesh was built with other settings than {what} (grid, agent class, partition, detail or another content-changing setting differ).");
        }
    }

    private static void WriteSettings(NavMeshSerializer.Writer w, NavSettings s)
    {
        w.F64(s.AgentRadius);
        w.F64(s.AgentHeight);
        w.F64(s.WalkableClimb);
        w.F64(s.AgentMaxSlopeDegrees);
        w.F64(s.CellSize);
        w.F64(s.TileSize);
        w.F64(s.CellHeight);
        w.F64(s.OriginX);
        w.F64(s.OriginZ);
        w.I32(s.MaxVertsPerPoly);
        w.F64(s.FloorY);
        w.U8(s.Area);
        w.U16(s.Flags);
        w.U8(s.SnapBits.HasValue ? (byte)1 : (byte)0);
        w.I32(s.SnapBits ?? 0);
        w.I32((int)s.Partition);
        w.I32(s.MinRegionArea);
        w.I32(s.MergeRegionArea);
        w.F64(s.ContourMaxError);
        w.F64(s.MaxEdgeLength);
        w.F64(s.DetailSampleDistance);
        w.F64(s.DetailMaxError);
        w.U32((uint)s.OffMeshLinks.Count);
        foreach (OffMeshLinkDefinition l in s.OffMeshLinks)
        {
            w.F64(l.Start.X);
            w.F64(l.Start.Y);
            w.F64(l.Start.Z);
            w.F64(l.End.X);
            w.F64(l.End.Y);
            w.F64(l.End.Z);
            w.F64(l.Radius);
            w.U8(l.Bidirectional ? (byte)1 : (byte)0);
            w.U8(l.Area);
            w.U16(l.Flags);
            w.U32(l.UserId);
        }
    }

    private static NavSettings ReadSettings(ref NavMeshSerializer.Reader r)
    {
        try
        {
            double agentRadius = r.F64("agent radius"), agentHeight = r.F64("agent height"), climb = r.F64("walkable climb");
            double slope = r.F64("max slope"), cell = r.F64("cell size"), tile = r.F64("tile size"), cellHeight = r.F64("cell height");
            double ox = r.F64("origin x"), oz = r.F64("origin z");
            int verts = r.I32("max verts per polygon");
            double floorY = r.F64("floor y");
            byte area = r.U8("area");
            ushort flags = r.U16("flags");
            byte hasSnap = r.U8("snap bits flag");
            int snap = r.I32("snap bits");
            if (hasSnap > 1)
            {
                throw new NavMeshFormatException($"Snap bits flag is {hasSnap}; it must be 0 or 1.");
            }
            if (hasSnap == 0 && snap != 0)
            {
                throw new NavMeshFormatException($"Snap bits is {snap} with its present flag clear; it must be 0.");
            }
            int partition = r.I32("partition");
            if (!Enum.IsDefined((PartitionType)partition))
            {
                throw new NavMeshFormatException($"Partition {partition} is not a known partition type.");
            }
            int minRegion = r.I32("min region area"), mergeRegion = r.I32("merge region area");
            double contour = r.F64("contour max error"), edge = r.F64("max edge length");
            double detailDist = r.F64("detail sample distance"), detailErr = r.F64("detail max error");
            uint linkCount = r.U32("link count");
            if (linkCount > r.Remaining / LinkBytes)
            {
                throw new NavMeshFormatException($"Link count {linkCount} is not possible for {r.Remaining} remaining bytes.");
            }
            var links = new OffMeshLinkDefinition[linkCount];
            for (int i = 0; i < links.Length; i++)
            {
                var start = new Vector3D(r.F64("link start"), r.F64("link start"), r.F64("link start"));
                var end = new Vector3D(r.F64("link end"), r.F64("link end"), r.F64("link end"));
                double radius = r.F64("link radius");
                byte bidi = r.U8("link bidirectional flag");
                if (bidi > 1)
                {
                    throw new NavMeshFormatException($"Link {i} has bidirectional flag {bidi}.");
                }
                byte lArea = r.U8("link area");
                ushort lFlags = r.U16("link flags");
                uint user = r.U32("link user id");
                links[i] = new OffMeshLinkDefinition(start, end, radius, bidi == 1, lArea, lFlags, user);
            }
            return new NavSettings
            {
                AgentRadius = agentRadius,
                AgentHeight = agentHeight,
                WalkableClimb = climb,
                AgentMaxSlopeDegrees = slope,
                CellSize = cell,
                TileSize = tile,
                CellHeight = cellHeight,
                OriginX = ox,
                OriginZ = oz,
                MaxVertsPerPoly = verts,
                FloorY = floorY,
                Area = area,
                Flags = flags,
                SnapBits = hasSnap == 1 ? snap : null,
                Partition = (PartitionType)partition,
                MinRegionArea = minRegion,
                MergeRegionArea = mergeRegion,
                ContourMaxError = contour,
                MaxEdgeLength = edge,
                DetailSampleDistance = detailDist,
                DetailMaxError = detailErr,
                OffMeshLinks = links,
            };
        }
        catch (ArgumentOutOfRangeException e)
        {
            throw new NavMeshFormatException($"Stored settings are out of range: {e.Message}", e);
        }
    }
}
