using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

/// <summary>The shared path-result vocabulary: stable numbers, the query-status mapping and the common members.</summary>
public class PathResultVocabularyTests
{
    [Fact]
    public void StatusNumbersAreStable()
    {
        Assert.Equal(0, (int)PathStatus.Found);
        Assert.Equal(1, (int)PathStatus.StartInvalid);
        Assert.Equal(2, (int)PathStatus.GoalInvalid);
        Assert.Equal(3, (int)PathStatus.NoPath);
        Assert.Equal(4, (int)PathStatus.BudgetExhausted);
        Assert.Equal(5, (int)PathStatus.Partial);
        Assert.Equal(6, (int)PathStatus.InProgress);
    }

    [Theory]
    [InlineData(QueryStatus.Success, PathStatus.Found)]
    [InlineData(QueryStatus.Partial, PathStatus.Partial)]
    [InlineData(QueryStatus.InProgress, PathStatus.InProgress)]
    [InlineData(QueryStatus.InvalidStartPoly, PathStatus.StartInvalid)]
    [InlineData(QueryStatus.NoStartNearby, PathStatus.StartInvalid)]
    [InlineData(QueryStatus.InvalidEndPoly, PathStatus.GoalInvalid)]
    [InlineData(QueryStatus.NoEndNearby, PathStatus.GoalInvalid)]
    [InlineData(QueryStatus.InvalidCorridor, PathStatus.NoPath)]
    public void QueryStatusMaps(QueryStatus q, PathStatus p) => Assert.Equal(p, q.ToPathStatus());

    [Fact]
    public void GridPathMembers()
    {
        var grid = Grid.FromText("....\n....");
        GridPath cells = grid.FindPath(new Cell(0, 0), new Cell(3, 0));
        Assert.True(cells.Found && cells.HasPath && !cells.IsPartial);
        Assert.True(double.IsNaN(cells.Length));
        GridPath none = grid.FindPath(new Cell(0, 0), new Cell(9, 9));
        Assert.Equal(PathStatus.GoalInvalid, none.Status);
        Assert.False(none.HasPath);
        Assert.Equal(0, none.Length);
        grid.Space = new GridSpace(0, 0, 2);
        GridPath world = grid.FindPath(new WorldPoint(1, 1), new WorldPoint(7, 1));
        Assert.Equal(6.0, world.Length);
    }
}
