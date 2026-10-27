using System;
using System.Linq;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

/// <summary>The text-map constructors, the single-goal flow field and the option presets of the one-line API.</summary>
public class TierOneApiTests
{
    [Fact]
    public void Text_maps_read_rows_areas_and_walls()
    {
        Grid grid = Grid.FromText("\n..#\r\n.3.\n\n");
        Assert.Equal(3, grid.Width);
        Assert.Equal(2, grid.Height);
        Assert.Equal(0, grid.GetArea(0, 0));
        Assert.Equal(AreaCosts.Wall, grid.GetArea(2, 0));
        Assert.Equal(3, grid.GetArea(1, 1));
        HexGrid hex = HexGrid.FromText("..\n#.", HexLayout.FlatOddColumn);
        Assert.Equal(HexLayout.FlatOddColumn, hex.Layout);
        Assert.Equal(AreaCosts.Wall, hex.GetArea(0, 1));
    }

    [Theory]
    [InlineData("", "no rows")]
    [InlineData("..\n.", "line 2")]
    [InlineData("..\n.x", "line 2")]
    public void Malformed_text_maps_name_the_problem(string text, string expected)
    {
        GridFormatException ex = Assert.Throws<GridFormatException>(() => Grid.FromText(text));
        Assert.Contains(expected, ex.Message);
        Assert.Throws<GridFormatException>(() => HexGrid.FromText(text));
        Assert.Throws<ArgumentNullException>(() => Grid.FromText(null!));
    }

    [Fact]
    public void A_single_goal_flow_field_equals_the_one_goal_span_form()
    {
        Grid grid = Grid.FromText("....#...\n.##.#.#.\n.#..#.#.\n.#.##.#.\n...3....");
        using GridFlowField one = grid.CreateFlowField(new Cell(7, 0));
        using GridFlowField many = grid.CreateFlowField([new Cell(7, 0)]);
        Assert.True(one.Costs.SequenceEqual<double>(many.Costs));
        Assert.True(one.Next.SequenceEqual<int>(many.Next));
        HexGrid hex = HexGrid.FromText("....\n.##.\n....");
        using GridFlowField hexOne = hex.CreateFlowField(new Cell(3, 2));
        Assert.Equal(0.0, hexOne.GetCost(new Cell(3, 2)));
    }

    [Fact]
    public void Landmark_options_default_to_sixteen_landmarks_and_match_the_explicit_build()
    {
        var edges = Enumerable.Range(0, 40).Select(i => new GraphEdge(i, (i + 1) % 40, 1.0 + (i % 3))).ToArray();
        Graph graph = Graph.From(edges, 40, directed: false);
        Landmarks defaults = graph.BuildLandmarks();
        Landmarks explicitOptions = graph.BuildLandmarks(new LandmarkOptions { Count = 16, Selection = LandmarkSelection.Avoid, Seed = 1 });
        Assert.Equal(16, defaults.Count);
        Assert.True(defaults.Serialize().SequenceEqual(explicitOptions.Serialize()));
        Assert.Equal(graph.FindPath(3, 31, null, TestContext.Current.CancellationToken).Cost, graph.FindPath(3, 31, GraphOptions.WithLandmarks(defaults), TestContext.Current.CancellationToken).Cost);
        Assert.Equal(graph.FindPath(3, 31, null, TestContext.Current.CancellationToken).Cost, graph.FindPath(3, 31, GraphOptions.Bidirectional, TestContext.Current.CancellationToken).Cost);
    }
}
