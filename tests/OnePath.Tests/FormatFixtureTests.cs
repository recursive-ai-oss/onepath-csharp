using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Security.Cryptography;
using RecursiveAI.OnePath.FormatFixtures;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

/// <summary>
/// Every 1.x release reads every format any earlier 1.x release wrote. The fixtures under Fixtures/&lt;release&gt;/ were written
/// by that release's code (tools/FormatFixtures) and are never regenerated; this test loads each with the current readers and
/// checks what the loaded state does against the expectation recorded beside it. A release that changes a format bumps its
/// version marker, keeps the reader for every earlier version, and adds a fixture directory of its own.
/// </summary>
public class FormatFixtureTests
{
    // The fixtures of release 1.0.0, fixed forever: a name leaves this list only if a format is dropped, which 1.x never does.
    private static readonly string[] Names100 =
    [
        "navmesh-mesh", "navmesh-tile", "navmesh-snapshot-full", "navmesh-snapshot-referenced",
        "tilecache-full", "tilecache-referenced",
        "crowd-state", "crowd-with-mesh-full", "crowd-with-mesh-referenced",
        "path-corridor", "path-follower",
        "grid-square", "grid-hex", "grid-voxel",
        "search-grid-astar", "search-grid-astar-generic", "search-grid-jps", "search-grid-jpsplus",
        "search-astar", "search-jump-block", "search-jump-table", "search-theta", "search-graph",
        "replanner-dstar", "replanner-lpa", "replanner-hex-dstar",
        "flow-square-partial", "flow-square-repair", "flow-hex-partial",
        "graph", "landmarks",
    ];

    // Added by release 1.1.0.
    private static readonly string[] Names110 = ["navmesh-tier1", "navworld-tier1"];

    // Release directories that must exist. A later release appends its own entry (and its own names list when it adds formats).
    private static readonly (string Release, string[] Names)[] Releases = [("1.0.0", Names100), ("1.1.0", Names110)];

    private static string Dir(string release) => Path.Combine(AppContext.BaseDirectory, "Fixtures", release);

    private static Dictionary<string, (long Bytes, string FileHash, string Probe)> Manifest(string release)
    {
        string path = Path.Combine(Dir(release), "fixtures.tsv");
        Assert.True(File.Exists(path), $"fixture manifest for release {release} is missing: {path}");
        var m = new Dictionary<string, (long, string, string)>();
        foreach (string line in File.ReadAllLines(path))
        {
            if (line.Length == 0 || line[0] == '#')
            {
                continue;
            }
            string[] c = line.Split('\t');
            Assert.True(c.Length == 5, "malformed manifest line: " + line);
            m.Add(c[0], (long.Parse(c[1], System.Globalization.CultureInfo.InvariantCulture), c[2], c[3]));
        }
        return m;
    }

    public static TheoryData<string, string> Cases()
    {
        var d = new TheoryData<string, string>();
        foreach ((string release, string[] names) in Releases)
        {
            foreach (string n in names)
            {
                d.Add(release, n);
            }
        }
        return d;
    }

    [Fact]
    public void EveryReleaseDirectoryIsThereAndListsExactlyItsFixtures()
    {
        string root = Path.Combine(AppContext.BaseDirectory, "Fixtures");
        Assert.True(Directory.Exists(root), "the Fixtures directory is missing");
        foreach ((string release, string[] names) in Releases)
        {
            Assert.True(Directory.Exists(Dir(release)), $"fixtures of release {release} are missing");
            Assert.Equal(names.OrderBy(x => x, StringComparer.Ordinal), Manifest(release).Keys.OrderBy(x => x, StringComparer.Ordinal));
            Assert.Equal(names.Length, Directory.GetFiles(Dir(release), "*.bin").Length);
        }
        // A directory nobody listed is a release whose fixtures this test would silently skip.
        foreach (string d in Directory.GetDirectories(root))
        {
            Assert.Contains(Releases, r => r.Release == Path.GetFileName(d));
        }
    }

    [Fact]
    public void EveryFixtureHasAProbe()
    {
        foreach ((_, string[] names) in Releases)
        {
            foreach (string n in names)
            {
                Assert.Contains(n, FixtureProbes.Names.Concat(FixtureProbes.Names11));
            }
        }
    }

    [Theory]
    [MemberData(nameof(Cases))]
    public void CurrentReadersLoadTheFixtureAndItBehavesAsRecorded(string release, string name)
    {
        var (bytes, fileHash, probe) = Manifest(release)[name];
        byte[] read(string n) => File.ReadAllBytes(Path.Combine(Dir(release), n + ".bin"));
        byte[] data = read(name);
        Assert.Equal(bytes, data.Length);
        Assert.Equal(fileHash, Convert.ToHexString(SHA256.HashData(data)).ToLowerInvariant());
        // The fixture is the file as the release wrote it; the probe's expectation is what the loaded state does.
        Assert.Equal(probe, Convert.ToHexString(FixtureProbes.Run(name, read)).ToLowerInvariant());
    }

    [Fact]
    public void FixturesAreSmall()
    {
        long total = 0;
        foreach ((string release, _) in Releases)
        {
            total += Manifest(release).Values.Sum(v => v.Bytes);
        }
        Assert.True(total < 1_000_000, $"the fixtures take {total} bytes");
    }

    // ---------------------------------------------------------------- the list of everything that writes bytes

    // Every public method that returns a byte array or fills a Span<byte> writes something a host could store, unless listed
    // as not persistent. A writer added to the library fails this test until it is given a fixture or listed with the reason.
    private static readonly Dictionary<string, string[]> Writers = new()
    {
        ["NavMeshSerializer.WriteMesh"] = ["navmesh-mesh"],
        ["NavMeshSerializer.WriteTile"] = ["navmesh-tile"],
        ["TiledNavMesh.Snapshot"] = ["navmesh-snapshot-full", "navmesh-snapshot-referenced"],
        ["NavMeshSnapshot.Take"] = ["navmesh-snapshot-full", "navmesh-snapshot-referenced"],
        ["TileCache.Snapshot"] = ["tilecache-full", "tilecache-referenced"],
        ["Crowd.SaveState"] = ["crowd-state"],
        ["Crowd.TrySaveState"] = ["crowd-state"],
        ["Crowd.SaveStateWithMesh"] = ["crowd-with-mesh-full", "crowd-with-mesh-referenced"],
        ["PathCorridor.ToBytes"] = ["path-corridor"],
        ["PathCorridor.Write"] = ["path-corridor"],
        ["PathFollower.ToBytes"] = ["path-follower"],
        ["PathFollower.Write"] = ["path-follower"],
        ["Grid.SaveState"] = ["grid-square"],
        ["HexGrid.SaveState"] = ["grid-hex"],
        ["VoxelGrid.SaveState"] = ["grid-voxel"],
        ["GridSearch.SaveState"] = ["search-grid-astar", "search-grid-astar-generic", "search-grid-jps", "search-grid-jpsplus"],
        ["AStar.SaveState"] = ["search-astar"],
        ["JumpSearch.SaveState"] = ["search-jump-block", "search-jump-table"],
        ["ThetaSearch.SaveState"] = ["search-theta"],
        ["GraphAStar.SaveState"] = ["search-graph"],
        ["GridReplanner.SaveState"] = ["replanner-dstar", "replanner-lpa"],
        ["HexReplanner.SaveState"] = ["replanner-hex-dstar"],
        ["GridFlowField.SaveState"] = ["flow-square-partial", "flow-square-repair", "flow-hex-partial"],
        ["Graph.SaveState"] = ["graph"],
        ["Landmarks.Serialize"] = ["landmarks"],
        ["NavMesh.Save"] = ["navmesh-tier1"],
        ["NavMesh.TrySave"] = ["navmesh-tier1"],
        ["NavWorld.Save"] = ["navworld-tier1"],
        ["NavWorld.TrySave"] = ["navworld-tier1"],
    };

    // Public byte writers that are not saved state: identifiers copied out for a host's own keys, and a scratch classification buffer.
    private static readonly HashSet<string> NotPersistent = new(StringComparer.Ordinal)
    {
        "NavMeshFingerprint.CopyTo",
        "TileContentId.CopyTo",
        "Voxelizer.ClassifyTriangles",
    };

    private static SortedSet<string> FoundWriters()
    {
        var found = new SortedSet<string>(StringComparer.Ordinal);
        foreach (Type t in typeof(Grid).Assembly.GetExportedTypes())
        {
            foreach (MethodInfo m in t.GetMethods(BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly))
            {
                if (m.IsSpecialName)
                {
                    continue;
                }
                bool bytesOut = m.ReturnType == typeof(byte[]) || m.GetParameters().Any(p => p.ParameterType == typeof(Span<byte>));
                if (bytesOut)
                {
                    string type = t.Name.Contains('`', StringComparison.Ordinal) ? t.Name[..t.Name.IndexOf('`', StringComparison.Ordinal)] : t.Name;
                    found.Add(type + "." + m.Name);
                }
            }
        }
        return found;
    }

    [Fact]
    public void EveryPublicWriterHasAFixture()
    {
        var found = FoundWriters().ToList();
        var unlisted = found.Where(w => !Writers.ContainsKey(w) && !NotPersistent.Contains(w)).ToList();
        Assert.True(unlisted.Count == 0, "writers without a fixture (add one in tools/FormatFixtures, or list them as not persistent): " + string.Join(", ", unlisted));
        var gone = Writers.Keys.Concat(NotPersistent).Where(w => !found.Contains(w)).ToList();
        Assert.True(gone.Count == 0, "writers that were removed from the public API (a 1.x release keeps its writers): " + string.Join(", ", gone));
        foreach ((string writer, string[] fixtures) in Writers)
        {
            foreach (string f in fixtures)
            {
                Assert.Contains(f, Names100.Concat(Names110));
            }
        }
        // Every fixture is claimed by a writer.
        foreach (string n in Names100.Concat(Names110))
        {
            Assert.Contains(Writers.Values, v => v.Contains(n));
        }
    }
}
