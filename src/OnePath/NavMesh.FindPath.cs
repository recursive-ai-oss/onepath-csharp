using System.Threading;
using System.Threading.Tasks;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath;

public sealed partial class NavMesh
{
    /// <summary>
    /// Finds the shortest walkable path between two world positions: the nearest polygon to each position is found, the
    /// cheapest corridor searched and string-pulled (see <see cref="NavMeshQuery.FindPath(Vector3D, Vector3D, PathOptions?, CancellationToken)"/>).
    /// Each call uses a fresh query object; for many queries create one <see cref="NavMeshQuery"/> over <see cref="Tiles"/> and reuse it.
    /// </summary>
    /// <param name="start">Start position (world, double).</param>
    /// <param name="end">End position.</param>
    /// <param name="options">Search options; null means the defaults.</param>
    /// <param name="cancellationToken">Checked every 256 expansions. A token that is never cancelled changes nothing; cancelling throws and leaves the mesh untouched.</param>
    /// <exception cref="System.ArgumentOutOfRangeException">A position is non-finite or beyond +-2^21.</exception>
    /// <exception cref="System.OperationCanceledException">Cancellation was requested.</exception>
    public PathResult FindPath(Vector3D start, Vector3D end, PathOptions? options = null, CancellationToken cancellationToken = default) =>
        new NavMeshQuery(Tiles).FindPath(start, end, options, cancellationToken);

    /// <summary>
    /// Same as <see cref="FindPath(Vector3D, Vector3D, PathOptions?, CancellationToken)"/>, searched in slices with a yield between
    /// slices so a host stays responsive; the result is identical to the synchronous one (slices count expansions, never time).
    /// Do not edit the mesh while the returned task runs.
    /// </summary>
    /// <param name="start">Start position (world, double).</param>
    /// <param name="end">End position.</param>
    /// <param name="options">Search options; null means the defaults.</param>
    /// <param name="iterationsPerSlice">Expansions between yields, at least 1. Default 256.</param>
    /// <param name="cancellationToken">Checked between slices; the task then ends cancelled.</param>
    /// <returns>A task for the result.</returns>
    /// <exception cref="System.ArgumentOutOfRangeException"><paramref name="iterationsPerSlice"/> is below 1, or a position is non-finite or beyond +-2^21 (thrown through the task).</exception>
    /// <exception cref="System.OperationCanceledException">Cancellation was requested (thrown through the task).</exception>
    public Task<PathResult> FindPathAsync(Vector3D start, Vector3D end, PathOptions? options = null, int iterationsPerSlice = 256, CancellationToken cancellationToken = default) =>
        new NavMeshQuery(Tiles).FindPathAsync(start, end, options, iterationsPerSlice, cancellationToken);
}
