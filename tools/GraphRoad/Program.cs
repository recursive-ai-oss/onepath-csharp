using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Graphs;

// Road-network sample for the graph searches: expansion counts (and, for information, wall time) of Dijkstra, A*,
// bidirectional Dijkstra, ALT and bidirectional ALT over random queries, every result checked against Dijkstra.
//   GraphRoad --gr <file.gr|file.gr.gz> [--queries N] [--landmarks K] [--seed S]     a 9th DIMACS Challenge graph ("a u v w" arcs)
//   GraphRoad --generate <side> [...]                                                  a generated planar road-like graph of side x side junctions
// Exit code 1 if any variant disagrees with Dijkstra.
string? file = null;
int side = 0;
int queries = 100;
int landmarkCount = 16;
ulong seed = 1;
for (int i = 0; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--gr" when i + 1 < args.Length:
            file = args[++i];
            break;
        case "--generate" when i + 1 < args.Length:
            side = int.Parse(args[++i], CultureInfo.InvariantCulture);
            break;
        case "--queries" when i + 1 < args.Length:
            queries = int.Parse(args[++i], CultureInfo.InvariantCulture);
            break;
        case "--landmarks" when i + 1 < args.Length:
            landmarkCount = int.Parse(args[++i], CultureInfo.InvariantCulture);
            break;
        case "--seed" when i + 1 < args.Length:
            seed = ulong.Parse(args[++i], CultureInfo.InvariantCulture);
            break;
        default:
            Console.Error.WriteLine("usage: GraphRoad (--gr <file> | --generate <side>) [--queries N] [--landmarks K] [--seed S]");
            return 2;
    }
}
if (file is null && side <= 0)
{
    Console.Error.WriteLine("give --gr <file> or --generate <side>");
    return 2;
}

List<GraphEdge> edges = [];
int nodes;
string name;
if (file is not null)
{
    name = Path.GetFileName(file);
    nodes = ReadGr(file, edges);
}
else
{
    name = $"generated {side}x{side}";
    nodes = Generate(side, seed, edges);
}
// The DIMACS files list both directions of every road; the graph is directed as given.
Graph g = Graph.From(edges, nodes, directed: true);
var sw = Stopwatch.StartNew();
Landmarks lm = g.BuildLandmarks(new LandmarkOptions { Count = landmarkCount, Selection = LandmarkSelection.Avoid, Seed = seed });
double landmarkSeconds = sw.Elapsed.TotalSeconds;
Console.WriteLine($"{name}: {nodes} nodes, {edges.Count} arcs, {lm.Count} landmarks built in {landmarkSeconds:F1}s ({lm.Serialize().Length / 1048576.0:F1} MiB)");

var algorithms = new (string Label, GraphOptions Options)[]
{
    ("dijkstra", new GraphOptions { Algorithm = GraphAlgorithm.Dijkstra }),
    ("bidirectional dijkstra", new GraphOptions { Algorithm = GraphAlgorithm.BidirectionalDijkstra }),
    ("ALT (A*)", new GraphOptions { Algorithm = GraphAlgorithm.AStar, Landmarks = lm }),
    ("ALT (bidirectional A*)", new GraphOptions { Algorithm = GraphAlgorithm.BidirectionalAStar, Landmarks = lm }),
};
var expanded = new long[algorithms.Length];
var seconds = new double[algorithms.Length];
var rng = new SeededRandom(seed ^ 0xD1B54A32D192ED03UL);
int mismatches = 0;
int found = 0;
for (int q = 0; q < queries; q++)
{
    int s = rng.NextInt(nodes);
    int t = rng.NextInt(nodes);
    double reference = double.NaN;
    GraphPath? first = null;
    for (int a = 0; a < algorithms.Length; a++)
    {
        sw.Restart();
        GraphPath p = g.FindPath(s, t, algorithms[a].Options);
        seconds[a] += sw.Elapsed.TotalSeconds;
        expanded[a] += p.ExpandedCount;
        if (a == 0)
        {
            first = p;
            reference = p.Cost;
            continue;
        }
        bool same = p.Status == first!.Status && (!p.Found || Math.Abs(p.Cost - reference) <= 1e-9 * Math.Max(1.0, reference));
        if (!same)
        {
            mismatches++;
            Console.WriteLine($"MISMATCH {algorithms[a].Label} {s}->{t}: {p.Status} {p.Cost} vs {first.Status} {reference}");
        }
    }
    if (first!.Found)
    {
        found++;
    }
}
Console.WriteLine($"{queries} random queries ({found} connected), mismatches against Dijkstra: {mismatches}");
Console.WriteLine("| search | nodes expanded (total) | per query | wall seconds (informational) |");
Console.WriteLine("|---|---|---|---|");
for (int a = 0; a < algorithms.Length; a++)
{
    Console.WriteLine($"| {algorithms[a].Label} | {expanded[a]} | {expanded[a] / Math.Max(1, queries)} | {seconds[a]:F3} |");
}
return mismatches == 0 ? 0 : 1;

static int ReadGr(string path, List<GraphEdge> edges)
{
    using Stream raw = File.OpenRead(path);
    using Stream s = path.EndsWith(".gz", StringComparison.OrdinalIgnoreCase) ? new GZipStream(raw, CompressionMode.Decompress) : raw;
    using var reader = new StreamReader(s);
    int n = 0;
    string? line;
    while ((line = reader.ReadLine()) != null)
    {
        if (line.Length == 0 || line[0] == 'c')
        {
            continue;
        }
        string[] f = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (f[0] == "p" && f.Length >= 4)
        {
            n = int.Parse(f[2], CultureInfo.InvariantCulture);
        }
        else if (f[0] == "a" && f.Length >= 4)
        {
            // DIMACS nodes are numbered from 1.
            edges.Add(new GraphEdge(int.Parse(f[1], CultureInfo.InvariantCulture) - 1, int.Parse(f[2], CultureInfo.InvariantCulture) - 1, double.Parse(f[3], CultureInfo.InvariantCulture)));
        }
    }
    return n;
}

// A jittered lattice with random missing streets, slow local roads and a sparse net of fast long edges: planar-ish with
// the hierarchy that makes landmark bounds work on real road maps.
static int Generate(int side, ulong seed, List<GraphEdge> edges)
{
    var rng = new SeededRandom(seed);
    var pos = new (double X, double Y)[side * side];
    for (int y = 0; y < side; y++)
    {
        for (int x = 0; x < side; x++)
        {
            pos[(y * side) + x] = ((x * 100) + (rng.NextDouble() * 60), (y * 100) + (rng.NextDouble() * 60));
        }
    }
    double Dist(int a, int b) => Math.Sqrt(((pos[a].X - pos[b].X) * (pos[a].X - pos[b].X)) + ((pos[a].Y - pos[b].Y) * (pos[a].Y - pos[b].Y)));
    for (int y = 0; y < side; y++)
    {
        for (int x = 0; x < side; x++)
        {
            int a = (y * side) + x;
            bool highwayRow = y % 16 == 0;
            bool highwayCol = x % 16 == 0;
            if (x + 1 < side && (highwayRow || rng.NextDouble() < 0.8))
            {
                double w = Math.Round(Dist(a, a + 1) * (highwayRow ? 0.4 : 1.0 + (rng.NextDouble() * 0.5)));
                edges.Add(new GraphEdge(a, a + 1, w));
                edges.Add(new GraphEdge(a + 1, a, w));
            }
            if (y + 1 < side && (highwayCol || rng.NextDouble() < 0.8))
            {
                double w = Math.Round(Dist(a, a + side) * (highwayCol ? 0.4 : 1.0 + (rng.NextDouble() * 0.5)));
                edges.Add(new GraphEdge(a, a + side, w));
                edges.Add(new GraphEdge(a + side, a, w));
            }
        }
    }
    return side * side;
}
