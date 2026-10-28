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
    /// Calls borrow a pooled query object from the mesh (at most four are kept, about 10 MB each at the default node limit, built on first use), so steady-state calls allocate only the returned path; concurrent calls from several threads are safe. For a tight loop on one thread a <see cref="NavMeshQuery"/> of your own over <see cref="Tiles"/> is still the fastest form.
    /// </summary>
    /// <param name="start">Start position (world, double).</param>
    /// <param name="end">End position.</param>
    /// <param name="options">Search options; null means the defaults.</param>
    /// <param name="cancellationToken">Checked every 256 expansions. A token that is never cancelled changes nothing; cancelling throws and leaves the mesh untouched.</param>
    /// <exception cref="System.ArgumentOutOfRangeException">A position is non-finite or beyond +-2^21.</exception>
    /// <exception cref="System.OperationCanceledException">Cancellation was requested.</exception>
    public PathResult FindPath(Vector3D start, Vector3D end, PathOptions? options = null, CancellationToken cancellationToken = default) =>
        Tiles.FindPath(start, end, options, cancellationToken);

    /// <summary>
    /// Same as <see cref="FindPath(Vector3D, Vector3D, PathOptions?, CancellationToken)"/>, searched cooperatively: the search runs on the
    /// caller's synchronization context in slices of <see cref="PathOptions.IterationsPerSlice"/> expansions with a yield between slices, so a
    /// frame loop stays responsive and any filter delegate runs on the caller's thread. The result is identical to the synchronous one
    /// (slices count expansions, never time). It does not move work to the thread pool; to search on another thread call <see cref="FindPath(Vector3D, Vector3D, PathOptions?, CancellationToken)"/>
    /// inside <c>Task.Run</c>. Do not edit the mesh while the returned task runs.
    /// </summary>
    /// <param name="start">Start position (world, double).</param>
    /// <param name="end">End position.</param>
    /// <param name="options">Search options; null means the defaults.</param>
    /// <param name="cancellationToken">Checked between slices; the task then ends cancelled.</param>
    /// <returns>A task for the result.</returns>
    /// <exception cref="System.ArgumentOutOfRangeException">A position is non-finite or beyond +-2^21 (thrown through the task).</exception>
    /// <exception cref="System.OperationCanceledException">Cancellation was requested (thrown through the task).</exception>
    public Task<PathResult> FindPathAsync(Vector3D start, Vector3D end, PathOptions? options = null, CancellationToken cancellationToken = default) =>
        FindPathPooledAsync(start, end, options, cancellationToken);

    private async Task<PathResult> FindPathPooledAsync(Vector3D start, Vector3D end, PathOptions? options, CancellationToken cancellationToken)
    {
        NavMeshQuery q = Tiles.RentQuery();
        try
        {
            return await q.FindPathAsync(start, end, options, cancellationToken);
        }
        finally
        {
            Tiles.ReturnQuery(q);
        }
    }
}
