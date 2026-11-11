using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using OneScope.Attach;
using OneScope.Probe;
using OneScope.Sdk;
using Xunit;

namespace RecursiveAI.OnePath.Scope.Tests;

/// <summary>A host program in miniature: a crowd recorded through a CrowdRecorder, feeding a real ScopeProbe on a Unix-domain socket.</summary>
public class SocketTests
{
    private sealed class Snapshots(Replay.CrowdRecorder rec) : ISnapshotSource
    {
        public void WriteSnapshot(System.Buffers.IBufferWriter<byte> writer) => rec.WriteSnapshot(writer);
    }

    private static string NewDir()
    {
        string dir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "osp-" + Guid.NewGuid().ToString("N")[..8]);
        System.IO.Directory.CreateDirectory(dir);
        return dir;
    }

    private static void WaitFor(Func<bool> condition, string what)
    {
        long start = Stopwatch.GetTimestamp();
        while (!condition())
        {
            if (Stopwatch.GetElapsedTime(start).TotalSeconds > 20)
            {
                throw new TimeoutException("Timed out waiting for " + what);
            }

            Thread.Sleep(1);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(100)]
    public async Task ViewerOnARealSocketMatchesTheHostHashForHash(int attachAfterStep)
    {
        string dir = NewDir();
        Crowd crowd = Worlds.Arena2D().CreateCrowd();
        var rec = new Replay.CrowdRecorder(crowd, Worlds.Dt);
        var probeInfo = new ProbeInfo(ScopeInfo.ModuleId, ScopeInfo.LibraryVersion, ScopeInfo.FormatVersion);
        using ScopeProbe probe = ScopeProbe.Attach(probeInfo, new Snapshots(rec), new ProbeOptions { Directory = dir });
        Assert.True(probe.IsListening);
        rec.Record = r => probe.Record(r);
        var hostHashes = new Dictionary<long, ulong> { [0] = crowd.StateHash() };
        Task<LiveSession>? connect = null;
        var module = new OnePathModule();

        // The host steps its crowd; the viewer connects on another thread when the host reaches attachAfterStep, and the
        // host keeps running (polling) until the probe has handed over its snapshot.
        void Hook(int s)
        {
            hostHashes[crowd.StepCount] = crowd.StateHash();
            probe.Poll();
            if (s + 1 == attachAfterStep)
            {
                connect = Task.Run(() => LiveSession.Connect(probe.SocketPath!, [module]));
            }
        }

        if (attachAfterStep == 0)
        {
            connect = Task.Run(() => LiveSession.Connect(probe.SocketPath!, [module]));
            WaitFor(() =>
            {
                probe.Poll();
                return probe.IsConnected;
            }, "the viewer to attach at step 0");
        }

        Worlds.RunScript(new Worlds.RecordingHost(rec), 11, 0, 24, 18, 22, 39, afterStep: Hook);
        if (attachAfterStep > 0)
        {
            // Not attached yet if the script ended first; keep polling so the snapshot is taken at the final step.
            WaitFor(() =>
            {
                probe.Poll();
                return probe.IsConnected;
            }, "the viewer to attach");
        }

        long hostEnd = crowd.StepCount;
        Assert.Equal(Worlds.Steps, hostEnd);
        probe.Dispose();
        using LiveSession viewer = await connect!;
        WaitFor(() =>
        {
            int n = viewer.Pump();
            return n == 0 && viewer.State != ClientState.Streaming && viewer.Pump() == 0;
        }, "the stream to end");
        Assert.Equal(ClientState.Ended, viewer.State);
        Assert.Null(viewer.StreamError);

        History h = viewer.History;
        long start = h.StartStep;
        Assert.True(start >= attachAfterStep && start <= hostEnd, $"attached at host step {start}");
        if (attachAfterStep == 0)
        {
            Assert.Equal(0, start);
        }

        Assert.Equal(hostEnd, h.EndStep);
        foreach (long step in new[] { hostEnd, start, (start + hostEnd) / 2, start + 1, hostEnd - 1 })
        {
            if (step < start || step > hostEnd)
            {
                continue;
            }

            h.Seek(step);
            Assert.Equal(hostHashes[step], h.StateHash);
        }

        for (long s = start; s <= hostEnd; s += 5)
        {
            h.Seek(s);
            Assert.Equal(hostHashes[s], h.StateHash);
        }

        // Reconnecting mid-run is just another attach: it starts from a fresh snapshot.
        System.IO.Directory.Delete(dir, true);
    }

    [Fact]
    public async Task AViewerThatReconnectsGetsAFreshSnapshotAndTheHostIsUnaffected()
    {
        string dir = NewDir();
        Crowd crowd = Worlds.Arena2D().CreateCrowd();
        var rec = new Replay.CrowdRecorder(crowd, Worlds.Dt);
        using ScopeProbe probe = ScopeProbe.Attach(new ProbeInfo(ScopeInfo.ModuleId, ScopeInfo.LibraryVersion, ScopeInfo.FormatVersion), new Snapshots(rec), new ProbeOptions { Directory = dir });
        rec.Record = r => probe.Record(r);
        var module = new OnePathModule();
        var hashes = new Dictionary<long, ulong>();
        var agents = new List<Agent>();
        for (int i = 0; i < 10; i++)
        {
            agents.Add(rec.Add(new RecursiveAI.OneMath.Vector3D(2 + i, 0, 5), Agent.Small));
            rec.SetDestination(agents[i], new RecursiveAI.OneMath.Vector3D(35, 0, 5 + i));
        }

        var sessions = new List<LiveSession>();
        for (int round = 0; round < 2; round++)
        {
            Task<LiveSession> connect = Task.Run(() => LiveSession.Connect(probe.SocketPath!, [module]));
            for (int i = 0; i < 15; i++)
            {
                probe.Poll();
                rec.Step();
                hashes[crowd.StepCount] = crowd.StateHash();
            }

            WaitFor(() =>
            {
                probe.Poll();
                return probe.IsConnected;
            }, "attach");
            for (int i = 0; i < 20; i++)
            {
                rec.Step();
                hashes[crowd.StepCount] = crowd.StateHash();
            }

            LiveSession s = await connect;
            sessions.Add(s);
            WaitFor(() =>
            {
                s.Pump();
                return s.History.EndStep >= crowd.StepCount;
            }, "records");
            s.History.Seek(s.History.EndStep);
            Assert.Equal(hashes[s.History.EndStep], s.History.StateHash);
            s.Dispose();
            // The host learns the viewer left when its next send fails, so it keeps stepping.
            WaitFor(() =>
            {
                rec.Step();
                hashes[crowd.StepCount] = crowd.StateHash();
                return !probe.IsConnected;
            }, "the host to notice the viewer left");
        }

        Assert.True(sessions[1].History.StartStep > sessions[0].History.StartStep);
        System.IO.Directory.Delete(dir, true);
    }
}
