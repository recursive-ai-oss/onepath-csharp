using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

// The README snippets are what a user writes, so they omit the optional cancellation token that xUnit1051 asks tests to pass.
#pragma warning disable xUnit1051

namespace RecursiveAI.OnePath.Tests;

/// <summary>
/// Runs the README quickstart. Each snippet sits between <c>// quickstart: name</c> and <c>// end quickstart</c> markers;
/// <see cref="Readme_snippets_match_these_tests"/> checks that the README shows exactly the code that runs here.
/// </summary>
public class ReadmeQuickstartTests
{
    private static readonly int[] ThreeNodes = [0, 1, 2];

    [Fact]
    public void Quickstart_grid()
    {
        // quickstart: grid
        var grid = Grid.FromText("""
            ........
            .#####..
            .#...#..
            .#.#.#..
            ...#....
            """);
        GridPath path = grid.FindPath(new Cell(0, 0), new Cell(4, 3));
        if (path.Found)
        {
            // path.Cost is exact for eight-connected octile moves and path.Cells lists the cells; query with
            // WorldPoint arguments (after setting grid.Space) to also get path.Points.
        }
        // end quickstart
        Assert.True(path.Found);
        Assert.Equal(new Cell(0, 0), path.Cells[0]);
        Assert.Equal(new Cell(4, 3), path.Cells[^1]);
    }

    [Fact]
    public void Quickstart_grid_options()
    {
        // quickstart: grid-options
        var wide = new Grid(64, 64);
        GridPath fast = wide.FindPath(new Cell(1, 1), new Cell(60, 50), new GridOptions { Algorithm = GridAlgorithm.JpsPlus });
        GridPath straight = wide.FindPath(new Cell(1, 1), new Cell(60, 50), new GridOptions { Algorithm = GridAlgorithm.ThetaStar });
        GridPath fat = wide.FindPath(new Cell(1, 1), new Cell(60, 50), GridOptions.ForAgentSize(3));
        GridPath avoid = wide.FindPath(new Cell(1, 1), new Cell(60, 50), new GridOptions { Blocked = c => c.X == 30 && c.Y < 60 });
        // end quickstart
        double octile = wide.FindPath(new Cell(1, 1), new Cell(60, 50)).Cost;
        Assert.True(fast.Found);
        Assert.Equal(GridAlgorithm.JpsPlus, fast.Algorithm);
        Assert.Equal(octile, fast.Cost, 9);
        Assert.True(straight.Found && straight.Cost <= octile);
        Assert.True(fat.Found);
        Assert.True(avoid.Found && avoid.Cost > octile);
    }

    [Fact]
    public void Quickstart_hex()
    {
        // quickstart: hex
        var hex = HexGrid.FromText("""
            ......
            ..##..
            ..#...
            ......
            """, HexLayout.PointyOddRow);
        GridPath hexPath = hex.FindPath(new Cell(0, 0), new Cell(5, 3));
        // end quickstart
        Assert.True(hexPath.Found);
    }

    [Fact]
    public void Quickstart_voxel()
    {
        // quickstart: voxel
        var voxels = new VoxelGrid(32, 32, 32);
        for (int y = 0; y < 32; y++)
        {
            for (int z = 0; z < 32; z++)
            {
                voxels.SetWalkable(16, y, z, y == 5 && z == 5);   // a wall with one hole
            }
        }
        VoxelPath flight = voxels.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(31, 31, 31), VoxelOptions.SixConnected);
        // end quickstart
        Assert.True(flight.Found);
        Assert.Contains(new VoxelCell(16, 5, 5), flight.Cells.ToArray());
    }

    [Fact]
    public void Quickstart_graph()
    {
        // quickstart: graph
        var graph = Graph.From(new[] { new GraphEdge(0, 1, 4.0), new GraphEdge(1, 2, 1.5), new GraphEdge(0, 2, 7.0) });
        GraphPath route = graph.FindPath(0, 2);   // route.Nodes is 0, 1, 2 and route.Cost is 5.5
        Landmarks landmarks = graph.BuildLandmarks();
        GraphPath fastRoute = graph.FindPath(0, 2, GraphOptions.WithLandmarks(landmarks));

        var points = PointGraph.From(new[] { new Point3(0, 0), new Point3(1, 0), new Point3(2, 0), new Point3(2, 1) }, new PointGraphOptions { MaxDistance = 1.1 });
        GraphPath walk = points.FindPath(new Point3(0, 0), new Point3(2, 1));
        // end quickstart
        Assert.Equal(ThreeNodes, route.Nodes.ToArray());
        Assert.Equal(5.5, route.Cost);
        Assert.Equal(5.5, fastRoute.Cost);
        Assert.True(walk.Found);
        Assert.Equal(3.0, walk.Cost);
    }

    [Fact]
    public void Quickstart_replan()
    {
        // quickstart: replan
        var yard = new Grid(8, 8);
        using GridReplanner replanner = yard.CreateReplanner(new Cell(0, 0), new Cell(7, 0));
        GridPath straightRoute = replanner.Replan();
        yard.SetRect(3, 0, 1, 7, AreaCosts.Wall);   // a wall appears
        GridPath detour = replanner.Replan();
        replanner.MoveStart(new Cell(1, 1));        // the agent moved
        GridPath onward = replanner.Replan();
        // end quickstart
        Assert.Equal(7.0, straightRoute.Cost);
        Assert.True(detour.Found && detour.Cost > straightRoute.Cost);
        Assert.Equal(yard.FindPath(new Cell(0, 0), new Cell(7, 0)).Cost, detour.Cost, 9);
        Assert.Equal(yard.FindPath(new Cell(1, 1), new Cell(7, 0)).Cost, onward.Cost, 9);
    }

    [Fact]
    public void Quickstart_flow_field()
    {
        // quickstart: flow-field
        var field = new Grid(16, 16);
        using GridFlowField toGoal = field.CreateFlowField(new Cell(15, 15));
        double cost = toGoal.GetCost(new Cell(0, 0));          // least cost from a cell to the goal
        if (toGoal.TryGetNext(new Cell(0, 0), out Cell next))  // the step a unit on that cell takes
        {
            // move from (0, 0) to next
        }
        // end quickstart
        Assert.Equal(field.FindPath(new Cell(0, 0), new Cell(15, 15)).Cost, cost, 9);
        Assert.True(toGoal.TryGetNext(new Cell(0, 0), out Cell step) && step == new Cell(1, 1));
    }

    [Fact]
    public void Quickstart_navmesh_2d()
    {
        // quickstart: navmesh-2d
        var open = new bool[40 * 40];
        Array.Fill(open, true);
        NavMesh mesh = NavMesh.Build2D(open, 40, 40, 1.0, new Vector2D(0, 0));
        PathResult route2D = mesh.FindPath(new Vector3D(3, 0, 3), new Vector3D(36, 0, 36));
        // route2D.HasPath, route2D.Points (the string-pulled path), route2D.Length
        // end quickstart
        Assert.True(route2D.HasPath);
        Assert.True(route2D.Points.Count >= 2);
    }

    [Fact]
    public void Readme_snippets_match_these_tests()
    {
        string readme = Resource("README.md");
        string source = Resource("ReadmeQuickstartTests.cs");
        var blocks = Regex.Matches(readme, "```csharp\\r?\\n// quickstart: (?<name>[\\w-]+)\\r?\\n(?<code>.*?)```", RegexOptions.Singleline);
        Assert.Equal(8, blocks.Count);
        foreach (Match block in blocks)
        {
            string name = block.Groups["name"].Value;
            Match region = Regex.Match(source, "// quickstart: " + Regex.Escape(name) + @"\r?\n(?<code>.*?)// end quickstart", RegexOptions.Singleline);
            Assert.True(region.Success, "no test runs the README snippet '" + name + "'");
            Assert.Equal(Normalize(block.Groups["code"].Value), Normalize(region.Groups["code"].Value));
        }
    }

    private static string Normalize(string code) =>
        string.Join('\n', code.Split('\n').Select(l => l.Trim()).Where(l => l.Length > 0));

    private static string Resource(string name)
    {
        using Stream s = typeof(ReadmeQuickstartTests).Assembly.GetManifestResourceStream(name)!;
        using var r = new StreamReader(s);
        return r.ReadToEnd();
    }
}
