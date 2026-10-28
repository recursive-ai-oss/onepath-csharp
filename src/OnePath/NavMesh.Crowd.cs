
namespace RecursiveAI.OnePath;

public sealed partial class NavMesh
{
    /// <summary>
    /// Creates an empty crowd that walks this mesh: <c>crowd.Add(start, Agent.Humanoid).SetDestination(goal)</c>, then
    /// <see cref="Crowd.Step"/> with a fixed time step. See <see cref="Crowd"/>.
    /// </summary>
    /// <param name="options">Crowd settings; null means <see cref="CrowdOptions.Default"/>.</param>
    /// <exception cref="System.ArgumentOutOfRangeException">A setting is out of range.</exception>
    public Crowd CreateCrowd(CrowdOptions? options = null) => new(Tiles, options);
}
