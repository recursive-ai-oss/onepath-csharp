using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace RecursiveAI.OnePath.Bench;

/// <summary>
/// The noisy tile map of the 2D builder rows: size/2 by size/2 tiles of 2 world units, about 60% floor. The solidity
/// sequence is the one tools/geom/gen_levels.py writes into <c>tilemap.obj</c> (the same 64-bit linear congruential
/// generator and seed rule), so the reference builder and ours see the same footprint.
/// </summary>
internal static class TileMapWorld
{
    /// <summary>World units per tile.</summary>
    public const double TileUnits = 2.0;

    /// <summary>Solidity of the tiles, row-major (row = world Z, column = world X), for a level of the given size and seed.</summary>
    public static bool[] Generate(double size, int seed, out int tiles)
    {
        tiles = (int)(size / TileUnits);
        // The generator seeds its Lcg with seed * 1000003 + 77.
        ulong s = unchecked(((ulong)(((long)seed * 1000003) + 77) * 2862933555777941757UL) + 3037000493UL);
        var cells = new bool[tiles * tiles];
        for (int i = 0; i < cells.Length; i++)
        {
            s = unchecked((s * 6364136223846793005UL) + 1442695040888963407UL);
            cells[i] = (s >> 11) / 9007199254740992.0 < 0.6;
        }
        return cells;
    }

    /// <summary>
    /// Recovers the tiles from a level file written by the generator: every triangle belongs to the tile its centroid
    /// falls in. Returns the tile count per side; the level's bounds start at the origin.
    /// </summary>
    public static bool[] FromObj(string path, out int tiles, out int triangles)
    {
        var verts = new List<(double X, double Z)>();
        var faces = new List<(int A, int B, int C)>();
        double maxX = 0;
        foreach (string line in File.ReadLines(path))
        {
            string[] f = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (f.Length == 4 && f[0] == "v")
            {
                double x = double.Parse(f[1], CultureInfo.InvariantCulture), z = double.Parse(f[3], CultureInfo.InvariantCulture);
                verts.Add((x, z));
                maxX = Math.Max(maxX, x);
            }
            else if (f.Length == 4 && f[0] == "f")
            {
                faces.Add((int.Parse(f[1], CultureInfo.InvariantCulture) - 1, int.Parse(f[2], CultureInfo.InvariantCulture) - 1, int.Parse(f[3], CultureInfo.InvariantCulture) - 1));
            }
        }
        tiles = (int)Math.Round(maxX / TileUnits);
        triangles = faces.Count;
        var cells = new bool[tiles * tiles];
        foreach (var (a, b, c) in faces)
        {
            double cx = (verts[a].X + verts[b].X + verts[c].X) / 3.0, cz = (verts[a].Z + verts[b].Z + verts[c].Z) / 3.0;
            int x = (int)Math.Floor(cx / TileUnits), z = (int)Math.Floor(cz / TileUnits);
            cells[(z * tiles) + x] = true;
        }
        return cells;
    }
}
