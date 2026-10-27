using System;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// Fuzz target: bytes become build settings and a triangle soup (with hostile coordinates and indices mixed in) or a small voxel chunk; the whole 3D
/// pipeline runs to a tiled mesh. Documented input errors (<see cref="ArgumentException"/> including out-of-range, and the documented
/// <see cref="InvalidOperationException"/> limits) are expected; any other exception, a mesh the validator rejects, or a serialization round trip that
/// changes the bytes is a finding.
/// </summary>
internal static class MeshBuildFuzz
{
    private static readonly double[] Special =
    {
        double.NaN, double.PositiveInfinity, double.NegativeInfinity, 3e6, -3e6, 1e300, -0.0, 4.9e-324, 2097152.0, -2097152.0, 2097151.75,
    };

    /// <summary>A built-in random driver: treats the bytes as a seed and runs many generated inputs, printing a failing one.</summary>
    public static void Smoke(byte[] seedBytes)
    {
        ulong s = 1469598103934665603UL;
        foreach (byte b in seedBytes)
        {
            s = (s ^ b) * 1099511628211UL;
        }
        for (int n = 0; n < 300; n++)
        {
            s = (s * 6364136223846793005UL) + 1442695040888963407UL;
            var data = new byte[40 + (int)((s >> 33) % 400)];
            for (int i = 0; i < data.Length; i++)
            {
                s = (s * 6364136223846793005UL) + 1442695040888963407UL;
                data[i] = (byte)(s >> 40);
            }
            try
            {
                Run(data);
            }
            catch (Exception)
            {
                Console.Error.WriteLine("FAILING INPUT: " + Convert.ToHexString(data));
                throw;
            }
        }
    }

    public static void Run(byte[] data)
    {
        int p = 0;
        int B() => p < data.Length ? data[p++] : 0;
        double Coord(bool special)
        {
            int v = (short)(B() | (B() << 8));
            if (special && (v & 0xFF80) == 0xFF80)
            {
                return Special[(v & 0x7F) % Special.Length];
            }
            return ((v & 0x0FFF) / 32.0) - ((v & 0x0800) != 0 ? 128.0 : 0.0);
        }

        double[] cells = { 0.25, 0.5, 0.3, 1.0 };
        double[] heights = { 0.1, 0.2, 0.25 };
        double[] tiles = { 4, 8, 12, 16 };
        float[] radii = { 0f, 0.25f, 0.5f, 1f };
        float[] agentHeights = { 1f, 1.8f, 2f };
        float[] climbs = { 0f, 0.3f, 0.5f, 1f };
        float[] slopes = { 0f, 30f, 45f, 60f, 90f };
        double[] samples = { 0, 0.5, 1, 2 };
        double[] errors = { 0, 0.1, 0.5 };
        var settings = new NavSettings
        {
            CellSize = cells[B() % cells.Length],
            CellHeight = heights[B() % heights.Length],
            TileSize = tiles[B() % tiles.Length],
            AgentRadius = radii[B() % radii.Length],
            AgentHeight = agentHeights[B() % agentHeights.Length],
            WalkableClimb = climbs[B() % climbs.Length],
            AgentMaxSlopeDegrees = slopes[B() % slopes.Length],
            Partition = (PartitionType)(B() % 3),
            MaxVertsPerPoly = 3 + (B() % 6),
            DetailSampleDistance = samples[B() % samples.Length],
            DetailMaxError = errors[B() % errors.Length],
            MinRegionArea = B() % 3 == 0 ? 0 : 4,
            MergeRegionArea = B() % 3 == 0 ? 0 : 16,
            OriginX = (B() % 5) * 0.375 - 0.75,
            OriginZ = (B() % 5) * 0.625 - 1.25,
        };
        try
        {
            NavMesh mesh;
            if ((B() & 7) == 0)
            {
                int w = 1 + (B() % 24), d = 1 + (B() % 24), h = 1 + (B() % 8);
                var voxels = new byte[w * d * h];
                for (int i = 0; i < voxels.Length && p < data.Length; i++)
                {
                    voxels[i] = (byte)(B() % 3);
                }
                mesh = NavMesh.Build(VoxelChunk.FromDense(w, d, h, voxels), settings);
            }
            else
            {
                int nv = 3 + (B() % 40);
                var verts = new double[nv * 3];
                for (int i = 0; i < verts.Length; i++)
                {
                    // Mostly a gently varying floor so that triangles are walkable, with the y coordinate coarse.
                    verts[i] = i % 3 == 1 ? Math.Round(Coord(true) / 8.0, 1) : Coord(true);
                }
                // A triangle spanning thousands of tiles is a legitimate (slow) build, not a bug: keep finite coordinates modest, so runs stay short.
                foreach (double c in verts)
                {
                    if (double.IsFinite(c) && Math.Abs(c) > 4096 && Math.Abs(c) < 2097152)
                    {
                        return;
                    }
                }
                int nt = B() % 40;
                var idx = new int[nt * 3];
                for (int i = 0; i < idx.Length; i++)
                {
                    idx[i] = B() % 17 == 0 ? B() - 64 : B() % nv;
                }
                mesh = NavMesh.Build(verts, idx, settings);
            }
            var issues = NavMeshValidator.Validate(mesh.Tiles);
            if (issues.Count > 0)
            {
                throw new InvalidOperationException("The built mesh failed validation: " + issues[0]);
            }
            byte[] bytes = NavMeshSerializer.WriteMesh(mesh.Tiles);
            byte[] again = NavMeshSerializer.WriteMesh(NavMeshSerializer.ReadMesh(bytes));
            if (!bytes.AsSpan().SequenceEqual(again))
            {
                throw new InvalidOperationException("Serialization round trip changed the bytes.");
            }
        }
        catch (ArgumentException)
        {
        }
        catch (InvalidOperationException e) when (e.Message.StartsWith("Tile (", StringComparison.Ordinal) || e.Message.StartsWith("A polygon's detail", StringComparison.Ordinal))
        {
        }
    }
}
