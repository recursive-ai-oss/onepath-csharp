using System.Collections.Generic;

namespace RecursiveAI.OnePath.Tests.Grids;

/// <summary>Seeded generators for the map shapes the Moving AI benchmark sets are made of: rooms with doors, mazes, caves, scattered obstacles.</summary>
internal static class HostMaps
{
    internal static Grid Make(string kind, ulong seed, int w, int h)
    {
        var rng = new SeededRandom(seed * 0x9E3779B97F4A7C15UL + 17);
        return kind switch
        {
            "rooms" => Rooms(ref rng, w, h),
            "maze" => Maze(ref rng, w, h),
            "cave" => Cave(ref rng, w, h),
            "areas" => WithAreas(BruteForce.RandomGrid(ref rng, w, true)),
            _ => BruteForce.RandomGrid(ref rng, w, false),
        };
    }

    internal static readonly string[] Kinds = ["random", "rooms", "maze", "cave", "areas"];

    private static Grid WithAreas(Grid g) => g;

    private static Grid Rooms(ref SeededRandom rng, int w, int h)
    {
        var g = new Grid(w, h);
        for (int k = 0; k < 6; k++)
        {
            bool horizontal = rng.NextInt(2) == 0;
            int line = 1 + rng.NextInt(((horizontal ? h : w)) - 2);
            int gap1 = rng.NextInt(horizontal ? w : h);
            int gap2 = rng.NextInt(horizontal ? w : h);
            for (int t = 0; t < (horizontal ? w : h); t++)
            {
                if (t != gap1 && t != gap2)
                {
                    g.SetArea(horizontal ? t : line, horizontal ? line : t, AreaCosts.Wall);
                }
            }
        }
        return g;
    }

    private static Grid Maze(ref SeededRandom rng, int w, int h)
    {
        // Recursive backtracker on odd cells, walls elsewhere; a few extra openings so there are several routes.
        var g = new Grid(w, h);
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                g.SetArea(x, y, AreaCosts.Wall);
            }
        }
        var stack = new Stack<(int X, int Y)>();
        g.SetArea(1, 1, 0);
        stack.Push((1, 1));
        (int, int)[] d = [(2, 0), (-2, 0), (0, 2), (0, -2)];
        while (stack.Count > 0)
        {
            var (cx, cy) = stack.Peek();
            var options = new List<(int, int)>();
            foreach (var (dx, dy) in d)
            {
                int nx = cx + dx;
                int ny = cy + dy;
                if (nx > 0 && ny > 0 && nx < w - 1 && ny < h - 1 && g.GetArea(nx, ny) == AreaCosts.Wall)
                {
                    options.Add((dx, dy));
                }
            }
            if (options.Count == 0)
            {
                stack.Pop();
                continue;
            }
            var (mx, my) = options[rng.NextInt(options.Count)];
            g.SetArea(cx + (mx / 2), cy + (my / 2), 0);
            g.SetArea(cx + mx, cy + my, 0);
            stack.Push((cx + mx, cy + my));
        }
        for (int i = 0; i < (w * h) / 40; i++)
        {
            int x = 1 + rng.NextInt(w - 2);
            int y = 1 + rng.NextInt(h - 2);
            g.SetArea(x, y, 0);
        }
        return g;
    }

    private static Grid Cave(ref SeededRandom rng, int w, int h)
    {
        var cur = new bool[w * h];
        for (int i = 0; i < cur.Length; i++)
        {
            cur[i] = rng.NextDouble() < 0.45;
        }
        for (int round = 0; round < 4; round++)
        {
            var next = new bool[cur.Length];
            for (int y = 0; y < h; y++)
            {
                for (int x = 0; x < w; x++)
                {
                    int walls = 0;
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx < 0 || ny < 0 || nx >= w || ny >= h || cur[(ny * w) + nx])
                            {
                                walls++;
                            }
                        }
                    }
                    next[(y * w) + x] = walls >= 5;
                }
            }
            cur = next;
        }
        var g = new Grid(w, h);
        for (int i = 0; i < cur.Length; i++)
        {
            if (cur[i])
            {
                g.SetArea(i % w, i / w, AreaCosts.Wall);
            }
        }
        return g;
    }
}
