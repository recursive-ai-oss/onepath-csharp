using System;
using RecursiveAI.OnePath.Grids;

/// <summary>Flow fields for the NativeAOT and WebAssembly smoke apps: tier-1 square and hex fields, repair after an edit, and the generic sliced field.</summary>
internal static class FlowSmoke
{
    public static bool Run()
    {
        var grid = new Grid(6, 4);
        using GridFlowField field = grid.CreateFlowField([new Cell(0, 0)], new GridOptions { StepCost = (_, _, c) => c, Blocked = c => c.X == 3 && c.Y == 3 });
        bool ok = field.Status == FlowFieldStatus.Complete && field.GetCost(new Cell(3, 0)) == 3.0 && field.GetPath(new Cell(5, 3)).Length > 0;
        grid.SetWalkable(1, 0, false);
        grid.SetWalkable(1, 1, false);
        field.Refresh();
        ok &= field.GetCost(new Cell(2, 0)) > 2.0 && field.TryGetNext(new Cell(2, 0), out _);

        var hex = new HexGrid(5, 5, HexLayout.PointyOddRow);
        using GridFlowField hexField = hex.CreateFlowField([new Cell(2, 2), new Cell(0, 0)]);
        ok &= hexField.GetCost(new Cell(2, 2)) == 0.0 && hexField.GetDirection(new Cell(4, 4), out double dx, out double dy) && Math.Abs(((dx * dx) + (dy * dy)) - 1.0) < 1e-12;

        var generic = new FlowField<SquareTopology, DefaultStepCost>(grid.CreateTopology(), default);
        generic.Begin([0]);
        while (generic.Step(2) == FlowFieldStatus.Running)
        {
        }
        ok &= generic.Cost(5) == field.GetCost(new Cell(5, 0));
        return ok;
    }
}
