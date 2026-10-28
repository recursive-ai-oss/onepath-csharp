using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

/// <summary>The shared path-result vocabulary: stable numbers, the query-status mapping and the common members.</summary>
public class PathResultVocabularyTests
{
    [Fact]
    public void StatusNumbersAreStable()
    {
        Assert.Equal(0, (int)PathStatus.None);
        Assert.Equal(1, (int)PathStatus.Found);
        Assert.Equal(2, (int)PathStatus.StartInvalid);
        Assert.Equal(3, (int)PathStatus.GoalInvalid);
        Assert.Equal(4, (int)PathStatus.NoPath);
        Assert.Equal(5, (int)PathStatus.BudgetExhausted);
        Assert.Equal(6, (int)PathStatus.Partial);
        Assert.Equal(7, (int)PathStatus.InProgress);
    }

    [Fact]
    public void DefaultNeverReadsAsSuccess()
    {
        // An unassigned result, a default struct field or a zeroed array slot must not claim a path.
        Assert.Equal(PathStatus.None, default);
        Assert.False(default(PathStatus).HasPath());
        Assert.Equal(PathStatus.None, default(QueryStatus).ToPathStatus());
        Assert.Equal(0, (int)QueryStatus.None);
        Assert.Equal(0, (int)AreaQueryStatus.None);
        Assert.Equal(0, (int)RaycastStatus.None);
        Assert.Equal(1, (int)QueryStatus.Success);
        Assert.Equal(1, (int)AreaQueryStatus.Success);
        Assert.Equal(1, (int)RaycastStatus.Ok);
        Assert.Equal(PathStatus.None, new Grid(4, 4).CreateSearch().Status);
        Assert.Equal(RaycastStatus.None, default(RaycastResult).Status);
    }

    [Fact]
    public void GridSearchReportsTheSharedVocabulary()
    {
        var grid = new Grid(8, 8);
        grid.SetWalkable(4, 0, false);
        GridSearch s = grid.CreateSearch();
        Assert.Equal(PathStatus.None, s.Status);
        Assert.Equal(PathStatus.InProgress, s.TryBegin(new Cell(0, 0), new Cell(7, 7)));
        Assert.Equal(PathStatus.InProgress, s.Status);
        Assert.Equal(PathStatus.Found, s.Step(100000));
        Assert.Equal(PathStatus.Found, s.Status);
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
        GridPath cells = grid.FindPath(new Cell(0, 0), new Cell(3, 0), cancellationToken: TestContext.Current.CancellationToken);
        Assert.True(cells.Found && cells.HasPath && !cells.IsPartial);
        Assert.True(double.IsNaN(cells.Length));
        GridPath none = grid.FindPath(new Cell(0, 0), new Cell(9, 9), cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.GoalInvalid, none.Status);
        Assert.False(none.HasPath);
        Assert.Equal(0, none.Length);
        grid.Space = new GridSpace(0, 0, 2);
        GridPath world = grid.FindPath(new WorldPoint(1, 1), new WorldPoint(7, 1), cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(6.0, world.Length);
    }
}
