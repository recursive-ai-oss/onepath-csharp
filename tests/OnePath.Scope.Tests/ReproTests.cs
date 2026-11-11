using System.IO;
using OneScope.Sdk;
using Xunit;

namespace RecursiveAI.OnePath.Scope.Tests;

/// <summary>Inputs the smoke fuzz found crashing; each must now be played as a viewer would, ending only in a typed error or success.</summary>
public class ReproTests
{
    private static byte[] Repro(string name) => File.ReadAllBytes(Path.Combine(System.AppContext.BaseDirectory, "testdata", "repro", name));

    [Fact]
    public void ASearchWhoseEndsBecomeWallsDoesNotStepAStaleSearch()
    {
        // An edit made the start or goal a wall; the old search used to be stepped and threw "the grid was edited".
        IScopeSource source = new OnePathModule().Open(Repro("repro-unbegun-search-steps.bin"), "repro");
        var history = new History(source, new ScopeHistoryOptions { KeyframeInterval = 5 });
        history.Seek(history.EndStep);
        ulong end = history.StateHash;
        var again = new History(source, new ScopeHistoryOptions { KeyframeInterval = 64 });
        again.Seek(again.EndStep);
        Assert.Equal(end, again.StateHash);
        history.Seek(3);
        history.Seek(history.EndStep);
        Assert.Equal(end, history.StateHash);
    }

    [Theory]
    [InlineData("repro-build-too-tall.bin")]
    [InlineData("repro-build-huge-extent.bin")]
    public void ABuildInputThatAsksForAHugeGridIsRefusedAtOpen(string file)
    {
        // These used to open, then spend half a minute (or throw a library argument error) when the finished mesh was drawn.
        Assert.Throws<ScopeFormatException>(() => new OnePathModule().Open(Repro(file), "repro"));
    }
}
