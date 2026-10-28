using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>A crowd with the agents it was built with, for the crowd tests and hash-matrix scenarios.</summary>
internal sealed class CrowdSim
{
    public CrowdSim(TiledNavMesh mesh, Crowd crowd, List<Agent> agents)
    {
        Mesh = mesh;
        Crowd = crowd;
        Agents = agents;
    }

    public TiledNavMesh Mesh { get; }

    public Crowd Crowd { get; }

    public List<Agent> Agents { get; }
}

/// <summary>Generated worlds for the crowd tests and hash-matrix scenarios: open arenas, rooms joined by a doorway, corridors, scattered fields.</summary>
internal static class CrowdWorlds
{
    public const double Dt = 1.0 / 30.0;

    /// <summary>An open w x h area of unit cells.</summary>
    public static TiledNavMesh Arena(int w, int h)
    {
        var cells = new bool[w * h];
        Array.Fill(cells, true);
        return NavMesh.Build2D(cells, w, h, 1.0, new Vector2D(0, 0), CorridorWorlds.FlatSettings()).Tiles;
    }

    /// <summary>Two rooms (each <paramref name="roomW"/> x <paramref name="h"/>) joined by a one-cell wall with a door of <paramref name="door"/> cells in the middle.</summary>
    public static TiledNavMesh Doorway(int roomW, int h, int door)
    {
        int w = (2 * roomW) + 1;
        var cells = new bool[w * h];
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                bool wall = x == roomW;
                int mid = h / 2;
                bool inDoor = y >= mid - (door / 2) && y < mid - (door / 2) + door;
                cells[(y * w) + x] = !wall || inDoor;
            }
        }
        return NavMesh.Build2D(cells, w, h, 1.0, new Vector2D(0, 0), CorridorWorlds.FlatSettings()).Tiles;
    }

    /// <summary>A corridor <paramref name="length"/> x <paramref name="width"/> cells.</summary>
    public static TiledNavMesh Corridor(int length, int width) => Arena(length, width);

    private static CrowdSim Make(TiledNavMesh mesh, int threads, AgentOptions options, bool constraints = false) =>
        new(mesh, new Crowd(mesh, new CrowdOptions { MaxThreads = threads, RecordConstraints = constraints }), new List<Agent>());

    /// <summary>Places agents on a lattice of the given spacing inside the box, left to right, rows bottom to top, with a seeded jitter of a tenth of the spacing.</summary>
    public static List<Vector3D> Lattice(double x0, double z0, double x1, double z1, double spacing, int count, NavMeshWorlds.Rng rng)
    {
        var l = new List<Vector3D>();
        for (double z = z0; z <= z1 && l.Count < count; z += spacing)
        {
            for (double x = x0; x <= x1 && l.Count < count; x += spacing)
            {
                double jx = ((rng.Int(2001) - 1000) / 1000.0) * spacing * 0.1;
                double jz = ((rng.Int(2001) - 1000) / 1000.0) * spacing * 0.1;
                l.Add(new Vector3D(x + jx, 0, z + jz));
            }
        }
        return l;
    }

    /// <summary>
    /// Two streams cross an open arena: a block of agents on the left walks right, a block at the bottom walks up. Each agent's goal is
    /// its start shifted by a fixed amount, so goals are as far apart as starts (1.4, more than two agent diameters).
    /// </summary>
    public static CrowdSim Crossing(int n, ulong seed, int threads, AgentOptions? options = null, bool constraints = false)
    {
        options ??= Agent.Small;
        const double Sp = 1.4;
        int side = Math.Max(24, (int)Math.Ceiling(Math.Sqrt(n) * 1.5 * Sp * 1.8) + 12);
        TiledNavMesh mesh = Arena(side, side);
        CrowdSim sim = Make(mesh, threads, options, constraints);
        var rng = new NavMeshWorlds.Rng(seed);
        int half = n / 2;
        double span = side - 2.0;
        double cols = Math.Max(1, Math.Ceiling(Math.Sqrt(half))) * Sp;
        List<Vector3D> left = Lattice(1.0, 1.0 + (span * 0.25), 1.0 + cols, span, Sp, half, rng);
        List<Vector3D> bottom = Lattice(1.0 + cols + 3, 1.0, side - 4.0 - cols, 1.0 + span, Sp, n - half, rng);
        double shiftX = side - 3.0 - cols - 1.0;
        for (int i = 0; i < left.Count; i++)
        {
            Agent a = sim.Crowd.Add(left[i], options);
            sim.Agents.Add(a);
            a.SetDestination(new Vector3D(left[i].X + shiftX, 0, left[i].Z));
        }
        double shiftZ = 0;
        foreach (Vector3D b in bottom)
        {
            shiftZ = Math.Max(shiftZ, b.Z);
        }
        shiftZ = side - 3.0 - shiftZ;
        for (int i = 0; i < bottom.Count; i++)
        {
            Agent a = sim.Crowd.Add(bottom[i], options);
            sim.Agents.Add(a);
            a.SetDestination(new Vector3D(bottom[i].X, 0, bottom[i].Z + shiftZ));
        }
        return sim;
    }

    /// <summary>Agents in the left room go to the right room through a door.</summary>
    public static CrowdSim ThroughDoor(int n, ulong seed, int threads, int door = 2, AgentOptions? options = null, bool constraints = false)
    {
        options ??= Agent.Small;
        int h = Math.Max(14, (int)Math.Ceiling(Math.Sqrt(n) * 2.0) + 6);
        int roomW = h;
        TiledNavMesh mesh = Doorway(roomW, h, door);
        CrowdSim sim = Make(mesh, threads, options, constraints);
        var rng = new NavMeshWorlds.Rng(seed);
        List<Vector3D> starts = Lattice(1.0, 1.0, roomW - 1.0, h - 1.0, 1.0, n, rng);
        // Goals on a sparser lattice (1.5 apart) in the other room, in the same order as the starts.
        List<Vector3D> goals = Lattice(roomW + 2.0, 1.0, (2.0 * roomW) - 1.0, h - 1.0, 1.5, starts.Count, new NavMeshWorlds.Rng(seed + 99));
        for (int i = 0; i < starts.Count; i++)
        {
            Agent a = sim.Crowd.Add(starts[i], options);
            sim.Agents.Add(a);
            a.SetDestination(goals[i]);
        }
        return sim;
    }

    /// <summary>A square field of <paramref name="size"/> x <paramref name="size"/> cells of 2 units with scattered blocked cells (about 22%), so every passage is at least 2 units wide: more than three agent diameters.</summary>
    public static TiledNavMesh Field2(int size, ulong seed)
    {
        var cells = new bool[size * size];
        var rng = new NavMeshWorlds.Rng(seed);
        for (int i = 0; i < cells.Length; i++)
        {
            cells[i] = rng.Int(100) >= 22;
        }
        return NavMesh.Build2D(cells, size, size, 2.0, new Vector2D(0, 0), CorridorWorlds.FlatSettings()).Tiles;
    }

    /// <summary>Agents on a scattered-obstacle field of 2-unit cells walk between random places with a route (starts at least 1 apart, goals at least 1.2 apart).</summary>
    public static CrowdSim RandomField(int size, int n, ulong seed, int threads, AgentOptions? options = null, bool constraints = false)
    {
        options ??= Agent.Small;
        TiledNavMesh mesh = Field2(size, seed);
        CrowdSim sim = Make(mesh, threads, options, constraints);
        var rng = new NavMeshWorlds.Rng(seed + 1);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        double side = size * 2.0;
        int tries = 0;
        while (sim.Agents.Count < n && tries++ < n * 40)
        {
            Vector3D a = new(0.1 + (rng.Int((int)(side * 10) - 2) / 10.0), 0, 0.1 + (rng.Int((int)(side * 10) - 2) / 10.0));
            Vector3D b = new(0.1 + (rng.Int((int)(side * 10) - 2) / 10.0), 0, 0.1 + (rng.Int((int)(side * 10) - 2) / 10.0));
            bool apart = true;
            foreach (Agent o in sim.Agents)
            {
                double dx = o.Position.X - a.X, dz = o.Position.Z - a.Z;
                double gx = o.Destination.X - b.X, gz = o.Destination.Z - b.Z;
                if ((dx * dx) + (dz * dz) < 1.0 || (gx * gx) + (gz * gz) < 1.44)
                {
                    apart = false;
                    break;
                }
            }
            if (!apart || q.FindPath(a, b).Status != PathStatus.Found)
            {
                continue;
            }
            // Positions off the mesh snap to its nearest point; keep the snapped places apart too.
            NearestPoly sa = q.FindNearestPoly(a, CorridorWorlds.Extents), sb = q.FindNearestPoly(b, CorridorWorlds.Extents);
            if (!sa.Poly.IsSet || !sb.Poly.IsSet || !ApartFrom(sim.Agents, sa.Point, sb.Point))
            {
                continue;
            }
            a = sa.Point;
            b = sb.Point;
            Agent ag;
            try
            {
                ag = sim.Crowd.Add(a, options);
            }
            catch (ArgumentException)
            {
                continue;
            }
            sim.Agents.Add(ag);
            ag.SetDestination(b);
        }
        return sim;
    }

    /// <summary>Agents on the two-floor mesh with off-mesh links walk between random places.</summary>
    public static CrowdSim TwoFloors(int n, ulong seed, int threads, AgentOptions? options = null)
    {
        options ??= Agent.Small;
        TiledNavMesh mesh = CorridorWorlds.TwoFloors(seed);
        CrowdSim sim = Make(mesh, threads, options);
        var rng = new NavMeshWorlds.Rng(seed + 5);
        List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        int tries = 0;
        while (sim.Agents.Count < n && tries++ < n * 40)
        {
            Vector3D a = CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng);
            Vector3D b = CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng);
            bool apart = true;
            foreach (Agent o in sim.Agents)
            {
                double dx = o.Position.X - a.X, dz = o.Position.Z - a.Z;
                double gx = o.Destination.X - b.X, gz = o.Destination.Z - b.Z;
                if ((dx * dx) + (dz * dz) < 1.0 || ((gx * gx) + (gz * gz) < 1.44 && Math.Abs(o.Destination.Y - b.Y) < 2.0))
                {
                    apart = false;
                    break;
                }
            }
            if (!apart || q.FindPath(a, b).Status != PathStatus.Found)
            {
                continue;
            }
            // Positions off the mesh snap to its nearest point; keep the snapped places apart too.
            NearestPoly sa = q.FindNearestPoly(a, CorridorWorlds.Extents), sb = q.FindNearestPoly(b, CorridorWorlds.Extents);
            if (!sa.Poly.IsSet || !sb.Poly.IsSet || !ApartFrom(sim.Agents, sa.Point, sb.Point))
            {
                continue;
            }
            a = sa.Point;
            b = sb.Point;
            Agent ag;
            try
            {
                ag = sim.Crowd.Add(a, options);
            }
            catch (ArgumentException)
            {
                continue;
            }
            sim.Agents.Add(ag);
            ag.SetDestination(b);
        }
        return sim;
    }

    // True when the start is at least 0.8 from every agent's position and the goal at least 1.2 from every goal (on the same floor).
    private static bool ApartFrom(List<Agent> agents, Vector3D start, Vector3D goal)
    {
        foreach (Agent o in agents)
        {
            double dx = o.Position.X - start.X, dz = o.Position.Z - start.Z;
            double gx = o.Destination.X - goal.X, gz = o.Destination.Z - goal.Z;
            if (((dx * dx) + (dz * dz) < 0.64 && Math.Abs(o.Position.Y - start.Y) < 2.0) || ((gx * gx) + (gz * gz) < 1.44 && Math.Abs(o.Destination.Y - goal.Y) < 2.0))
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>Distance to the nearest point of the mesh from a position, and whether the position is over a polygon (within 1e-6 horizontally).</summary>
    public static bool OnMesh(NavMeshQuery q, Vector3D p)
    {
        NearestPoly np = q.FindNearestPoly(p, new Vector3D(1e-3, 1.0, 1e-3));
        if (!np.Poly.IsSet)
        {
            return false;
        }
        double dx = np.Point.X - p.X, dz = np.Point.Z - p.Z;
        return (dx * dx) + (dz * dz) <= 1e-12;
    }
}
