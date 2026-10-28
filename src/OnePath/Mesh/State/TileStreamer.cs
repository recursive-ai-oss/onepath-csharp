using System;
using System.Collections.Generic;
using System.Threading;

namespace RecursiveAI.OnePath.Mesh;

/// <summary>
/// Streams tiles in and out of a mesh at run time under an iteration budget. Tiles come in as the bytes
/// <see cref="NavMeshSerializer.WriteTile(NavTile)"/> wrote; <see cref="Update"/> applies queued operations in the order they were queued,
/// at most a given number per call, so a host spreads the work over frames (or steps) the same way on every machine. One operation
/// is one iteration: add (or replace) one tile, or remove one tile column with all its layers.
/// <para>
/// The mesh's queries, path corridors and crowds treat a streamed tile like any other edit: references into a removed or replaced tile
/// become invalid (the tile's slot generation advances), corridors repair around the change, and links across tile borders
/// and off-mesh links are stitched again. Streaming a tile out and back in gives the same tile bytes and the same query answers;
/// it does not give the same polygon references or slot numbers back.
/// </para>
/// <para>
/// The pending queue is not part of a mesh snapshot: a host that snapshots drains the queue first (<see cref="PendingCount"/> is 0).
/// </para>
/// </summary>
public sealed class TileStreamer
{
    private readonly TiledNavMesh _mesh;
    private readonly Queue<Op> _queue = new();

    private readonly record struct Op(byte[]? Tile, int TileX, int TileZ);

    /// <summary>Creates a streamer for the mesh.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="mesh"/> is null.</exception>
    public TileStreamer(TiledNavMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        _mesh = mesh;
    }

    /// <summary>The mesh being streamed.</summary>
    public TiledNavMesh Mesh => _mesh;

    /// <summary>Operations waiting for <see cref="Update"/>.</summary>
    public int PendingCount => _queue.Count;

    /// <summary>Queues a tile to add, or to replace the tile at its position (same column, row and layer). The bytes are copied and checked when applied.</summary>
    /// <param name="tileBytes">A tile as written by <see cref="NavMeshSerializer.WriteTile(NavTile)"/>, built with this mesh's parameters.</param>
    /// <exception cref="ArgumentNullException"><paramref name="tileBytes"/> is null.</exception>
    public void QueueAdd(ReadOnlySpan<byte> tileBytes) => _queue.Enqueue(new Op(tileBytes.ToArray(), 0, 0));

    /// <summary>Queues the removal of every layer of one tile column. Removing a column that holds no tile does nothing.</summary>
    /// <param name="tileX">Tile column.</param>
    /// <param name="tileZ">Tile row.</param>
    public void QueueRemove(int tileX, int tileZ) => _queue.Enqueue(new Op(null, tileX, tileZ));

    /// <summary>
    /// Applies queued operations, oldest first, at most <paramref name="maxOperations"/> of them. The cancellation token is checked between
    /// operations; one in progress finishes. An operation that fails is dropped from the queue before its exception propagates, and the rest stay queued.
    /// </summary>
    /// <param name="maxOperations">The budget in operations; 0 does nothing.</param>
    /// <param name="cancellationToken">Stops between operations.</param>
    /// <returns>The number of operations applied.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="maxOperations"/> is negative.</exception>
    /// <exception cref="NavMeshFormatException">A queued tile is malformed or was built with other mesh parameters.</exception>
    /// <exception cref="OperationCanceledException">Cancelled; the operations applied so far stay applied.</exception>
    public int Update(int maxOperations, CancellationToken cancellationToken = default)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(maxOperations);
        int done = 0;
        while (done < maxOperations && _queue.Count > 0)
        {
            cancellationToken.ThrowIfCancellationRequested();
            Op op = _queue.Dequeue();
            Apply(op);
            done++;
        }
        return done;
    }

    /// <summary>Applies every queued operation.</summary>
    /// <returns>The number of operations applied.</returns>
    public int UpdateAll(CancellationToken cancellationToken = default) => Update(int.MaxValue, cancellationToken);

    private void Apply(Op op)
    {
        if (op.Tile is null)
        {
            foreach (TileRef r in Column(op.TileX, op.TileZ))
            {
                _mesh.RemoveTile(r);
            }
            return;
        }
        NavTile tile = NavMeshSerializer.ReadTile(op.Tile, _mesh.Params);
        if (_mesh.GetTileRef(tile.TileX, tile.TileZ, tile.Layer).IsSet)
        {
            _mesh.ReplaceTile(tile);
        }
        else
        {
            _mesh.AddTile(tile);
        }
    }

    /// <summary>
    /// Takes a column out of the mesh at once and returns its tiles' bytes, lowest layer first, ready to stream back in with
    /// <see cref="QueueAdd"/>. A column with no tile returns an empty list.
    /// </summary>
    /// <param name="tileX">Tile column.</param>
    /// <param name="tileZ">Tile row.</param>
    public IReadOnlyList<byte[]> StreamOut(int tileX, int tileZ)
    {
        var result = new List<byte[]>();
        TileRef[] refs = Column(tileX, tileZ);
        foreach (TileRef r in refs)
        {
            result.Add(NavMeshSerializer.WriteTile(_mesh.GetTile(r)!));
        }
        foreach (TileRef r in refs)
        {
            _mesh.RemoveTile(r);
        }
        return result;
    }

    private TileRef[] Column(int tileX, int tileZ)
    {
        var refs = new TileRef[_mesh.GetTilesAt(tileX, tileZ, default)];
        _mesh.GetTilesAt(tileX, tileZ, refs);
        return refs;
    }
}
