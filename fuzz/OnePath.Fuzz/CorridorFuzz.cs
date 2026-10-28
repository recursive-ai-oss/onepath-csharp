using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// Fuzz target for path corridors and followers. The input is a program: the first byte picks a mesh, the rest is read as
/// operations (place, set destination, step, edit the mesh, corridor moves and shortcuts, replans with small budgets,
/// state round trips and damaged state bytes). Coordinates and time steps are mostly sane but sometimes not: those calls must
/// throw the documented <see cref="ArgumentOutOfRangeException"/> and change nothing. After every operation the follower must
/// hold together: a finite position, a corridor without repeated polygons, and state bytes that read back to the same bytes.
/// Any other exception, or a broken invariant, crashes the target.
/// </summary>
internal static class CorridorFuzz
{
    // Built on first use, not in a type initializer: the coverage instrumentation is not ready inside one.
    private static TiledNavMesh[]? s_meshes;
    private static byte[][]? s_base;

    private ref struct Input
    {
        private readonly ReadOnlySpan<byte> _d;
        private int _i;

        public Input(ReadOnlySpan<byte> d)
        {
            _d = d;
            _i = 0;
        }

        public bool More => _i < _d.Length;

        public int Byte() => _i < _d.Length ? _d[_i++] : 0;

        // Mostly a coordinate inside the world, sometimes something the API must reject.
        public double Coord(double lo, double hi)
        {
            int b = Byte();
            int sel = Byte();
            if (sel == 255)
            {
                return b switch { < 64 => double.NaN, < 128 => double.PositiveInfinity, < 192 => 3e6, _ => -2097152.0 };
            }
            return lo + ((((b << 8) | sel) / 65536.0) * (hi - lo));
        }
    }

    public static void Run(byte[] data)
    {
        if (data.Length < 2)
        {
            return;
        }
        if (s_meshes is null || s_base is null)
        {
            TiledNavMesh[] meshes = [CorridorWorlds.Field(14, 3), CorridorWorlds.TwoFloors(7), CorridorWorlds.Maze(4, 2, out _)];
            s_base = [meshes[0].Snapshot(), meshes[1].Snapshot(), meshes[2].Snapshot()];
            s_meshes = meshes;
        }
        int which = data[0] % s_meshes.Length;
        TiledNavMesh mesh = s_meshes[which];
        mesh.Restore(s_base[which]);
        var q = new NavMeshQuery(mesh, 4096);
        var inp = new Input(data.AsSpan(1));
        double span = which == 0 ? 14 : which == 1 ? 16 : 9;
        double x0 = which == 1 ? -16 : 0;
        var mut = new NavMeshWorlds.Rng(data[1]);
        FollowerOptions options = FollowerOptions.Humanoid with { ReplanBudget = 1 + (data[1] % 32), MaxReplanRounds = 8, StuckWindow = (data[1] & 1) == 0 ? 0.5 : 0 };
        var f = new PathFollower(options);
        var c = new PathCorridor(16);
        var replan = new CorridorReplan();
        var filter = new QueryFilter { ExcludeFlags = (ushort)((data[1] >> 2) & 3) };
        bool placed = false;
        for (int op = 0; op < 200 && inp.More; op++)
        {
            int kind = inp.Byte() % 14;
            try
            {
                switch (kind)
                {
                    case 0:
                        placed = f.Place(q, new Vector3D(x0 + inp.Coord(-1, span), inp.Coord(-1, 5), x0 + inp.Coord(-1, span)), filter) || placed;
                        break;
                    case 1:
                        f.SetDestination(q, new Vector3D(x0 + inp.Coord(-1, span), inp.Coord(-1, 5), x0 + inp.Coord(-1, span)), filter);
                        break;
                    case 2:
                    case 3:
                    case 4:
                        for (int s = 0, n = 1 + (inp.Byte() % 30); s < n; s++)
                        {
                            f.Step(q, (inp.Byte() == 0) ? 0 : 1.0 / (10 + inp.Byte()), filter);
                        }
                        break;
                    case 5:
                        NavMeshMutations.Apply(mesh, mut, 7);
                        break;
                    case 6:
                        c.MovePosition(q, new Vector3D(x0 + inp.Coord(-1, span), inp.Coord(-1, 5), x0 + inp.Coord(-1, span)), filter);
                        break;
                    case 7:
                        c.MoveTarget(q, new Vector3D(x0 + inp.Coord(-1, span), inp.Coord(-1, 5), x0 + inp.Coord(-1, span)), filter);
                        break;
                    case 8:
                        c.OptimizeVisibility(q, new Vector3D(x0 + inp.Coord(-1, span), inp.Coord(-1, 5), x0 + inp.Coord(-1, span)), 1 + inp.Byte(), filter, inp.Byte() % 40);
                        c.OptimizeTopology(q, filter, 1 + inp.Byte(), 3 + (inp.Byte() % 20));
                        break;
                    case 9:
                        if (c.Count == 0 || inp.Byte() % 4 == 0)
                        {
                            NearestPoly np = q.FindNearestPoly(new Vector3D(x0 + inp.Coord(-1, span), 0, x0 + inp.Coord(-1, span)), new Vector3D(3, 6, 3), filter);
                            if (np.Poly.IsSet)
                            {
                                c.Reset(np.Poly, np.Point);
                            }
                        }
                        c.SetTarget(new Vector3D(x0 + inp.Coord(-1, span), inp.Coord(-1, 5), x0 + inp.Coord(-1, span)));
                        break;
                    case 10:
                        if (replan.Begin(q, c, filter) == QueryStatus.InProgress)
                        {
                            for (int s = 0, n = 1 + (inp.Byte() % 5); s < n && replan.Update(1 + inp.Byte()) == QueryStatus.InProgress; s++)
                            {
                            }
                            replan.Apply();
                        }
                        break;
                    case 11:
                        if (c.Validate(q, filter, 1 + inp.Byte()) == CorridorStatus.Invalid)
                        {
                            c.Relocate(q, filter);
                        }
                        c.Replan(q, filter, 1 + (inp.Byte() * 4));
                        break;
                    case 12:
                    {
                        byte[] b = f.ToBytes();
                        var g = new PathFollower(options);
                        g.Read(b);
                        Require(g.ToBytes().AsSpan().SequenceEqual(b), "follower state bytes do not round trip");
                        byte[] cb = c.ToBytes();
                        Require(PathCorridor.Read(cb, out _).ToBytes().AsSpan().SequenceEqual(cb), "corridor state bytes do not round trip");
                        break;
                    }
                    default:
                    {
                        // Damaged state: rejected with the typed error, or accepted and then safe to run.
                        byte[] b = f.ToBytes();
                        for (int k = 0, n = 1 + (inp.Byte() % 3); k < n; k++)
                        {
                            b[(inp.Byte() << 4 | inp.Byte() >> 4) % b.Length] ^= (byte)(1 << (inp.Byte() & 7));
                        }
                        var g = new PathFollower(options);
                        try
                        {
                            g.Read(b);
                            g.Step(q, 1.0 / 30, filter);
                        }
                        catch (OnePathFormatException)
                        {
                        }
                        break;
                    }
                }
            }
            catch (ArgumentOutOfRangeException)
            {
                // Only rejected input may cause this; the invariants below still have to hold.
            }
            Check(f, c, placed);
        }
    }

    private static void Require(bool ok, string what)
    {
        if (!ok)
        {
            throw new InvalidOperationException(what);
        }
    }

    private static void Check(PathFollower f, PathCorridor c, bool placed)
    {
        Vector3D p = f.Position;
        Require(double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z), "follower position not finite");
        Require(double.IsFinite(f.Velocity.X) && double.IsFinite(f.Velocity.Z), "follower velocity not finite");
        Require(!placed || f.Corridor.Count > 0, "placed follower has an empty corridor");
        foreach (PathCorridor pc in new[] { f.Corridor, c })
        {
            ReadOnlySpan<PolyRef> path = pc.Path;
            Require(path.Length <= pc.Capacity, "corridor over capacity");
            for (int i = 0; i < path.Length; i++)
            {
                for (int j = i + 1; j < path.Length; j++)
                {
                    Require(path[i] != path[j], "polygon twice in a corridor");
                }
            }
        }
    }

    /// <summary>Valid seed inputs: a few short programs that place, send and step.</summary>
    public static void WriteSeeds(string dir)
    {
        System.IO.Directory.CreateDirectory(dir);
        byte[][] seeds =
        [
            [0, 5, 0, 40, 0, 40, 0, 1, 200, 0, 200, 0, 2, 30, 10, 12, 2, 30, 20, 12],
            [1, 8, 0, 60, 0, 10, 0, 60, 0, 1, 190, 0, 90, 0, 190, 0, 2, 30, 8, 5, 2, 30, 8, 12],
            [2, 3, 0, 20, 0, 20, 0, 1, 100, 0, 100, 0, 3, 30, 9, 11, 4, 20, 10, 40, 0, 40, 0, 50, 0],
        ];
        for (int i = 0; i < seeds.Length; i++)
        {
            System.IO.File.WriteAllBytes(System.IO.Path.Combine(dir, $"corridor{i}.bin"), seeds[i]);
        }
    }
}
