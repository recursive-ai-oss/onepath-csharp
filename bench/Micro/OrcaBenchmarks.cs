using BenchmarkDotNet.Attributes;
using RecursiveAI.OnePath.Crowds.Orca;
using RecursiveAI.OnePath.HashMatrix;

namespace RecursiveAI.OnePath.Micro;

/// <summary>
/// ORCA step throughput: one benchmark invocation is one simulation step of a settled crowd (agents/ms = Agents / the mean time in ms).
/// Run with <c>dotnet run -c Release --project bench/Micro -- --filter '*Orca*'</c> on a quiet machine.
/// </summary>
[MemoryDiagnoser]
public class OrcaStepBenchmarks
{
    private OrcaWorld _world = null!;

    [Params(100, 1000)]
    public int Agents { get; set; }

    [Params(1)]
    public int Threads { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _world = OrcaWorld.Circle(Agents, Threads);
        // Let the crowd close in so the measured steps are dense ones.
        for (int i = 0; i < 200; i++)
        {
            _world.Step();
        }
    }

    [Benchmark]
    public void Step() => _world.Step();
}

/// <summary>The ORCA linear program on typical neighbour counts.</summary>
[MemoryDiagnoser]
public class OrcaLinearProgramBenchmarks
{
    private HalfPlane[] _lines = null!;
    private HalfPlane[] _scratch = null!;

    [Params(5, 10, 20)]
    public int Constraints { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _lines = new HalfPlane[Constraints];
        _scratch = new HalfPlane[Constraints];
        // Lines whose normals fan out around the circle and which all permit the origin: a feasible, typical case.
        for (int i = 0; i < Constraints; i++)
        {
            double t = (2.0 * i) / Constraints;
            double c = 1 - (t * t / 2);
            var n = new RecursiveAI.OneMath.Vector2D(c, t - (t * t * t / 6));
            n /= n.Length();
            _lines[i] = new HalfPlane(n * -0.2, n);
        }
    }

    [Benchmark]
    public double Solve() => OrcaLinearProgram.Solve(_lines, 0, 1.5, new RecursiveAI.OneMath.Vector2D(1, 0.3), _scratch, out _).X;
}
