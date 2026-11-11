using System.IO;
using System.Linq;
using OneScope.Sdk;
using Xunit;

namespace RecursiveAI.OnePath.Scope.Tests;

public class DrawTests
{
    [Theory]
    [InlineData("crowd2d.oprp", 2)]
    [InlineData("crowd3d.oprp", 3)]
    public void CrowdDrawIsTheSameEveryTimeAndEachOverlayDrawsItsOwnItems(string file, int dimensions)
    {
        IScopeSource source = new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath(file)), file);
        var history = new History(source, new ScopeHistoryOptions { KeyframeInterval = 7 });
        var list = new DrawList();
        history.Seek(100);
        history.World.Draw(list, new DrawRequest(ulong.MaxValue));
        ulong first = DrawHash.Of(list);
        int count = list.Items.Length;
        Assert.True(count > 100);
        Assert.Equal(0, list.Dropped);

        // Another path to step 100 (a fresh history, stepping), and drawing twice into a reused list, give the same items.
        var other = new History(source, new ScopeHistoryOptions { KeyframeInterval = 64 });
        for (int i = 0; i < 100; i++)
        {
            other.Step();
        }

        list.Clear();
        other.World.Draw(list, new DrawRequest(ulong.MaxValue));
        Assert.Equal(first, DrawHash.Of(list));
        history.Seek(3);
        history.Seek(100);
        list.Clear();
        history.World.Draw(list, new DrawRequest(ulong.MaxValue));
        Assert.Equal(first, DrawHash.Of(list));

        foreach (OverlayInfo overlay in source.Info.Overlays)
        {
            list.Clear();
            history.World.Draw(list, new DrawRequest(1UL << overlay.Id));
            if (overlay.Name != "Off-mesh links")
            {
                Assert.True(list.Items.Length > 0, overlay.Name);
            }

            Assert.All(list.Items.ToArray(), i => Assert.Equal(overlay.Id, i.Overlay));
        }

        // Agents are pickable: their items carry the agent's entity id.
        list.Clear();
        history.World.Draw(list, new DrawRequest(1UL << ScopeInfo.OvAgents));
        Assert.All(list.Items.ToArray(), i => Assert.True(ScopeInfo.IsAgent(i.Entity, out _)));
        Assert.Equal(dimensions == 2 ? DrawKind.Sphere : DrawKind.Cylinder, list.Items[0].Kind);
        if (dimensions == 2)
        {
            Assert.All(list.Vertices.ToArray(), v => Assert.Equal(0f, v.Z));
        }
    }
}
