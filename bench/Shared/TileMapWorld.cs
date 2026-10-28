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

    /// <summary>
    /// The exact area of the floor points at least <paramref name="radius"/> away from every wall, for a tile map whose walls are the solid
    /// tiles and everything outside the map, with round joins at convex wall corners (the Minkowski erosion of the floor by a disc).
    /// This is the 2D gate's yardstick; it does not use any builder.
    /// <para>
    /// Method: for a radius up to half a tile, wall tiles two or more tiles away are farther than the radius from every point of a
    /// floor tile, so each floor tile only sees its eight neighbours. Inside a tile of side T the lost area is the union of a strip of
    /// width r along each side that borders a wall, plus a quarter disc of radius r at each corner whose two adjacent sides border
    /// floor but whose diagonal neighbour is a wall. For r &lt;= T/2 these pieces overlap only where two strips meet in an r x r corner
    /// square, so with S wall sides, C wall-side pairs meeting at a corner and D quarter discs, lost = 2 r S - r^2 C + (pi r^2 / 4) D.
    /// Everything is a small integer count times a closed form; the only inexact step is the double rounding of pi (relative error
    /// below 2^-52 per tile), so the sum is within 1e-9 relative of the true area for any map this runner uses.
    /// </para>
    /// </summary>
    public static double ExactErodedArea(bool[] floor, int tiles, double radius)
    {
        if (!(radius >= 0 && radius <= TileUnits / 2))
        {
            throw new ArgumentOutOfRangeException(nameof(radius), radius, "the closed form holds for radius in [0, half a tile]");
        }
        bool Floor(int x, int z) => x >= 0 && z >= 0 && x < tiles && z < tiles && floor[(z * tiles) + x];
        double r = radius, t = TileUnits, area = 0;
        for (int z = 0; z < tiles; z++)
        {
            for (int x = 0; x < tiles; x++)
            {
                if (!floor[(z * tiles) + x])
                {
                    continue;
                }
                bool w = !Floor(x - 1, z), e = !Floor(x + 1, z), s = !Floor(x, z - 1), n = !Floor(x, z + 1);
                int sides = (w ? 1 : 0) + (e ? 1 : 0) + (s ? 1 : 0) + (n ? 1 : 0);
                int pairs = ((w && s) ? 1 : 0) + ((s && e) ? 1 : 0) + ((e && n) ? 1 : 0) + ((n && w) ? 1 : 0);
                int discs = ((!w && !s && !Floor(x - 1, z - 1)) ? 1 : 0) + ((!s && !e && !Floor(x + 1, z - 1)) ? 1 : 0)
                          + ((!e && !n && !Floor(x + 1, z + 1)) ? 1 : 0) + ((!n && !w && !Floor(x - 1, z + 1)) ? 1 : 0);
                double lost = (2 * r * sides) - (r * r * pairs) + (Math.PI * r * r / 4 * discs);
                area += (t * t) - lost;
            }
        }
        return area;
    }
}
