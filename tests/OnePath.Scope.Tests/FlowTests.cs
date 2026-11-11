using System.IO;
using System.Linq;
using OneScope.Sdk;
using RecursiveAI.OnePath.Grids;
using Xunit;

namespace RecursiveAI.OnePath.Scope.Tests;

public class FlowTests
{
    private static History Open() => new(new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath("grid-flow.opgr")), "f"));

    [Fact]
    public void AfterAGoalEditTheFieldStillDrawsAndEqualsTheLibrarys()
    {
        History h = Open();
        var list = new DrawList();
        int Count(long step, int overlay)
        {
            h.Seek(step);
            list.Clear();
            h.World.Draw(list, new DrawRequest(1UL << overlay));
            return list.Items.Length;
        }

        int before = Count(12, ScopeInfo.GridCosts);
        Assert.True(before > 100);
        foreach (long step in new long[] { 26, 40, 60 })
        {
            Assert.True(Count(step, ScopeInfo.GridCosts) > 100, $"costs at step {step}");
            Assert.True(Count(step, ScopeInfo.GridClosed) > 100, $"settled cells at step {step}");
        }

        // The library, run by hand through the same edits, has the same field.
        var run = GridRun.Read(File.ReadAllBytes(Worlds.DataPath("grid-flow.opgr")), "x");
        Grid grid = Grid.FromState(run.GridState);
        var options = new GridOptions { Connectivity = run.Connectivity, Diagonals = run.Diagonals, AgentSize = run.AgentSize, IterationBudget = run.Slice };
        Cell[] goals = run.Points;
        GridFlowField field = grid.CreateFlowField(goals, options);
        for (int s = 0; s < 40; s++)
        {
            byte[] ops = run.Steps[s];
            for (int i = 0; i < ops.Length; i += GridRun.OpBytes)
            {
                int x = System.BitConverter.ToInt32(ops, i + 1), y = System.BitConverter.ToInt32(ops, i + 5);
                if ((GridRunOpKind)ops[i] == GridRunOpKind.SetArea)
                {
                    grid.SetArea(x, y, ops[i + 9]);
                }
                else
                {
                    goals = [new Cell(x, y)];
                }
            }

            if (ops.Length > 0)
            {
                field.Dispose();
                field = grid.CreateFlowField(goals, options);
            }

            if (field.Status == FlowFieldStatus.Running)
            {
                field.Step(run.Slice);
            }
        }

        h.Seek(40);
        var props = new PropertyList();
        h.World.Inspect(ScopeInfo.GridRoot, props);
        Assert.Equal(field.Settled, props.Rows.Single(r => r.Name == "cells settled").Value.GetInteger());
        int finite = field.Costs.ToArray().Count(c => double.IsFinite(c));
        Assert.Equal(finite, Count(40, ScopeInfo.GridClosed));
    }

    [Fact]
    public void FlowDirectionsDrawAnArrowPerCellWithANextCell()
    {
        History h = Open();
        var list = new DrawList();
        h.Seek(10);
        h.World.Draw(list, new DrawRequest(1UL << ScopeInfo.GridArrows));
        int arrows = list.Items.Length;
        Assert.True(arrows > 100);
        Assert.All(list.Items.ToArray(), i => Assert.Equal(DrawKind.Arrow, i.Kind));
        h.Seek(40);
        list.Clear();
        h.World.Draw(list, new DrawRequest(ulong.MaxValue));
        Assert.Contains(list.Items.ToArray(), i => i.Kind == DrawKind.Arrow);
    }

    [Fact]
    public void AMovedGoalOnAWallIsReportedNotSilentlyEmpty()
    {
        // (44, 3) is a wall in the test maze: the library gives a field with no cells, and the viewer says why.
        var w = new System.Buffers.ArrayBufferWriter<byte>();
        GridRun.WriteFlow(Worlds.MazeGrid(), null, [new Cell(45, 16)], 60, [[GridRunOp.SetGoal(44, 3)], []], w);
        var h = new History(new OnePathModule().Open(w.WrittenMemory, "blocked"));
        h.Scan();
        Assert.Contains(h.Findings, f => f.Code == "flow.goal-blocked");
    }

    [Fact]
    public void DraggingTheGoalPastTheEdgeClampsToTheEdgeCell()
    {
        History h = Open();
        IScopeWorld world = h.World;
        var into = new System.Buffers.ArrayBufferWriter<byte>();
        world.EncodePoke(new PokeCall("move_goal", [ScopeValue.OfEntity(ScopeInfo.GoalMarker), ScopeValue.OfVector(new System.Numerics.Vector3(500.7f, -20f, 0))]), into);
        GridRunOp op = new(GridRunOpKind.SetGoal, System.BitConverter.ToInt32(into.WrittenSpan[1..5]), System.BitConverter.ToInt32(into.WrittenSpan[5..9]), 0);
        Assert.Equal(new GridRunOp(GridRunOpKind.SetGoal, 47, 0, 0), op);
        Assert.Throws<System.ArgumentException>(() => world.EncodePoke(new PokeCall("move_goal", [ScopeValue.OfEntity(ScopeInfo.GoalMarker), ScopeValue.OfVector(new System.Numerics.Vector3(float.NaN, 1, 0))]), into));
        Assert.Throws<System.ArgumentException>(() => world.EncodePoke(new PokeCall("set_wall", [ScopeValue.OfVector(new System.Numerics.Vector3(500, 1, 0)), ScopeValue.OfBool(true)]), into));
    }

    [Fact]
    public void CrowdDiffTextUsesTheInspectorsVectorConvention()
    {
        IScopeSource source = new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath("crowd2d.oprp")), "c");
        IScopeWorld a = source.CreateWorld(), b = source.CreateWorld();
        a.Advance(source.Recording.InputAt(0).Span, TimelineSink.None);
        b.Advance(source.Recording.InputAt(0).Span, TimelineSink.None);
        var entities = new System.Collections.Generic.List<EntityInfo>();
        a.CollectEntities(entities);
        EntityId agent = entities.First(e => e.Kind == "agent").Id;
        var poke = new System.Buffers.ArrayBufferWriter<byte>();
        b.EncodePoke(new PokeCall("teleport_agent", [ScopeValue.OfEntity(agent), ScopeValue.OfVector(new System.Numerics.Vector3(30, 20, 0))]), poke);
        b.Advance(poke.WrittenSpan, TimelineSink.None);
        a.Advance(default, TimelineSink.None);
        Difference d = a.FirstDifference(b)!.Value;
        var props = new PropertyList();
        b.Inspect(agent, props);
        string inspector = props.Rows.Single(r => r.Name == "position").Value.ToString();
        // Both write "(x, z, 0)": the second number is the planar z, and the last is 0.
        Assert.StartsWith("(", d.Right);
        Assert.EndsWith(", 0)", d.Right);
        Assert.EndsWith(", 0)", inspector);
        Assert.Equal("(30, 20, 0)", d.Right);
        Assert.Equal("(30, 20, 0)", inspector);
    }
}
