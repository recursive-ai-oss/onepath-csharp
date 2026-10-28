using System.Collections.Generic;
using BenchmarkDotNet.Attributes;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Micro;

/// <summary>
/// Path corridor and follower throughput on a scattered-obstacle field: one invocation of <c>FollowerStep</c> steps every
/// agent once (agents per millisecond = Agents / the mean time in ms); <c>CorridorMoveAndCorners</c> is the corridor part alone
/// (move the agent along the surface, find the next corners). Agents that arrive are sent somewhere new, from a seeded generator.
/// Run with <c>dotnet run -c Release --project bench/Micro -- --filter '*Corridor*'</c> on a quiet machine.
/// </summary>
[MemoryDiagnoser]
public class CorridorBenchmarks
{
    private const int Size = 96;
    private TiledNavMesh _mesh = null!;
    private NavMeshQuery _query = null!;
    private readonly List<PathFollower> _agents = new();
    private PathCorridor[] _corridors = null!;
    private StraightPathPoint[] _corners = null!;
    private NavMeshWorlds.Rng _rng = null!;

    [Params(100, 1000)]
    public int Agents { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _mesh = CorridorWorlds.Field(Size, 5);
        _query = new NavMeshQuery(_mesh, 65536, PathNodeModel.Crossing);
        _rng = new NavMeshWorlds.Rng(1);
        _agents.Clear();
        while (_agents.Count < Agents)
        {
            var f = new PathFollower();
            if (f.Place(_query, CorridorWorlds.RandomFieldPoint(Size, _rng)))
            {
                f.SetDestination(_query, CorridorWorlds.RandomFieldPoint(Size, _rng));
                _agents.Add(f);
            }
        }
        _corners = new StraightPathPoint[16];
        _corridors = new PathCorridor[Agents];
        for (int i = 0; i < Agents; i++)
        {
            _corridors[i] = new PathCorridor(256);
            NearestPoly np = _query.FindNearestPoly(CorridorWorlds.RandomFieldPoint(Size, _rng), CorridorWorlds.Extents);
            _corridors[i].Reset(np.Poly, np.Point);
            _corridors[i].SetTarget(CorridorWorlds.RandomFieldPoint(Size, _rng));
            _corridors[i].Replan(_query);
        }
        for (int i = 0; i < 200; i++)
        {
            FollowerStep();
        }
    }

    [Benchmark]
    public void FollowerStep()
    {
        foreach (PathFollower f in _agents)
        {
            f.Step(_query, 1.0 / 30.0);
            if (!f.HasPath)
            {
                f.SetDestination(_query, CorridorWorlds.RandomFieldPoint(Size, _rng));
            }
        }
    }

    [Benchmark]
    public void CorridorMoveAndCorners()
    {
        foreach (PathCorridor c in _corridors)
        {
            if (c.FindCorners(_query, _corners, out int n) != PathStatus.Success || n < 2)
            {
                c.SetTarget(CorridorWorlds.RandomFieldPoint(Size, _rng));
                c.Replan(_query);
                continue;
            }
            Vector3D next = _corners[1].Position;
            c.MoveAlong(_query, Vector3D.Lerp(c.Position, next, 0.1), 4);
        }
    }
}
