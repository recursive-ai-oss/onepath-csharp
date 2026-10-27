using System.Runtime.CompilerServices;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>The cosine stored for every whole and fractional slope angle, so any platform difference in the degree conversion shows.</summary>
internal static class SlopeDegreesScenarios
{
    [ModuleInitializer]
    internal static void Init() => Scenarios.Register("build3d/slope-degrees", Run);

    private static void Run(ScenarioContext ctx)
    {
        var s = new SpanSettings();
        for (int i = 0; i <= 900; i++)
        {
            s.WalkableSlopeDegrees = i * 0.1f;
            ctx.Write(s.WalkableSlopeCosine);
        }
    }
}
