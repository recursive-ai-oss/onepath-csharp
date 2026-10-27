using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath;

public sealed partial class NavMesh
{
    /// <summary>
    /// Finds the shortest walkable path between two world positions: the nearest polygon to each position is found, the
    /// cheapest corridor searched and string-pulled (see <see cref="NavMeshQuery.FindPath(Vector3D, Vector3D, PathOptions?)"/>).
    /// Each call uses a fresh query object; for many queries create one <see cref="NavMeshQuery"/> over <see cref="Tiles"/> and reuse it.
    /// </summary>
    /// <param name="start">Start position (world, double).</param>
    /// <param name="end">End position.</param>
    /// <param name="options">Search options; null means the defaults.</param>
    /// <exception cref="System.ArgumentOutOfRangeException">A position is non-finite or beyond +-2^21.</exception>
    public PathResult FindPath(Vector3D start, Vector3D end, PathOptions? options = null) =>
        new NavMeshQuery(Tiles).FindPath(start, end, options);
}
