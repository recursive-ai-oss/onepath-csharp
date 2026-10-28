using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath;

public sealed partial class NavMesh
{
    /// <summary>
    /// Creates a streamer that adds and removes this mesh's tiles at run time under an iteration budget: <c>streamer.QueueAdd(bytes)</c>,
    /// <c>streamer.QueueRemove(x, z)</c>, then <c>streamer.Update(maxOperations)</c> each frame. See <see cref="TileStreamer"/>.
    /// </summary>
    public TileStreamer CreateStreamer() => new(Tiles);
}
