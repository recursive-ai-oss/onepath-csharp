using System;
using System.Buffers;
using System.Collections.Generic;
using OneScope.Sdk;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Replay;
using RecursiveAI.OnePath.Scope;

// A crowd replay and a build input go through the module: open, seek, draw, fork-and-poke, step along the build stage axis.
const int W = 20, H = 12;
var cells = new bool[W * H];
Array.Fill(cells, true);
NavMesh nm = NavMesh.Build2D(cells, W, H, 1.0, new Vector2D(0, 0), NavSettings.Humanoid with { AgentRadius = 0, CellSize = 0.5, TileSize = 16 });
var rec = new CrowdRecorder(nm.CreateCrowd(), 1.0 / 30.0, keepSteps: true);
var agents = new List<Agent>();
for (int i = 0; i < 5; i++)
{
    agents.Add(rec.Add(new Vector3D(2, 0, 2 + i), Agent.Small));
    rec.SetDestination(agents[i], new Vector3D(17, 0, 2 + i));
}

var hashes = new List<ulong>();
for (int s = 0; s < 40; s++)
{
    rec.Step();
    hashes.Add(rec.Crowd.StateHash());
}

var module = new OnePathModule();
IScopeSource source = module.Open(rec.ToReplay().ToBytes(), "smoke");
var history = new History(source, new ScopeHistoryOptions { KeyframeInterval = 7 });
history.Seek(33);
bool seek = history.StateHash == hashes[32];
history.Seek(5);
seek &= history.StateHash == hashes[4];
var list = new DrawList();
history.World.Draw(list, new DrawRequest(ulong.MaxValue));
bool draw = list.Items.Length > 10;
Branch branch = history.Fork(10);
var ids = new List<EntityInfo>();
branch.History.World.CollectEntities(ids);
EntityInfo agent = ids.Find(e => e.Kind == "agent");
branch.Poke(new PokeCall("stop_agent", [ScopeValue.OfEntity(agent.Id)]));
branch.Step();
bool fork = branch.DivergenceStep == 10;

var bytes = new ArrayBufferWriter<byte>();
double[] v = [0, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0];
int[] idx = [0, 1, 2, 0, 2, 3];
BuildInput.Write(v, idx, NavSettings.Humanoid with { AgentRadius = 0.2, CellSize = 0.5, CellHeight = 0.25, MinRegionArea = 1, MergeRegionArea = 1 }, bytes);
var build = new History(module.Open(bytes.WrittenMemory, "floor"));
bool stages = true;
for (int k = 1; k <= 7; k++)
{
    build.Seek("stage", k);
    list.Clear();
    build.View.Draw(list, new DrawRequest(0b11));
    stages &= list.Items.Length > 2;
}

Console.WriteLine($"seek {seek}, draw {draw}, fork {fork}, stages {stages}");
return seek && draw && fork && stages ? 0 : 1;
