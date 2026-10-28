using System;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// Settings of a <see cref="HpaHierarchy"/> (tier 2). The hierarchy is built for one agent size and one movement rule.
/// </summary>
public sealed class HpaOptions
{
    private int _chunkSize = 16;
    private int _entranceThreshold = 6;
    private int _agentSize = 1;

    /// <summary>Side of a chunk (cluster) in cells, 2..1024. Default 16. The last chunk of a row or column may be smaller.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is outside 2..1024.</exception>
    public int ChunkSize
    {
        get => _chunkSize;
        set
        {
            ArgumentOutOfRangeException.ThrowIfLessThan(value, 2, nameof(ChunkSize));
            ArgumentOutOfRangeException.ThrowIfGreaterThan(value, 1024, nameof(ChunkSize));
            _chunkSize = value;
        }
    }

    /// <summary>
    /// Entrance width from which an entrance gets two transitions (one at each end) instead of one in the middle
    /// (Botea, Müller and Schaeffer 2004 use 6). Default 6; at least 1.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is below 1.</exception>
    public int EntranceThreshold
    {
        get => _entranceThreshold;
        set
        {
            ArgumentOutOfRangeException.ThrowIfLessThan(value, 1, nameof(EntranceThreshold));
            _entranceThreshold = value;
        }
    }

    /// <summary>Agent size in cells, 1..255 (see <see cref="GridOptions.AgentSize"/>). One hierarchy serves one size. Default 1.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is outside 1..255.</exception>
    public int AgentSize
    {
        get => _agentSize;
        set
        {
            ArgumentOutOfRangeException.ThrowIfLessThan(value, 1, nameof(AgentSize));
            ArgumentOutOfRangeException.ThrowIfGreaterThan(value, 255, nameof(AgentSize));
            _agentSize = value;
        }
    }

    /// <summary>Four or eight moves. Default eight.</summary>
    public Connectivity Connectivity { get; set; } = Connectivity.Eight;

    /// <summary>
    /// The diagonal rule. <see cref="DiagonalRule.AllowAll"/> is not supported (a diagonal step across a chunk corner
    /// could then connect cells no straight border move reaches) and makes <see cref="HpaHierarchy"/> throw. Default <see cref="DiagonalRule.NoCornerCutting"/>.
    /// </summary>
    public DiagonalRule Diagonals { get; set; } = DiagonalRule.NoCornerCutting;
}
