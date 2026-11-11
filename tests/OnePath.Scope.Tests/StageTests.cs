using System;
using System.Collections.Generic;
using System.IO;
using OneScope.Sdk;
using RecursiveAI.OnePath.Mesh.Build3D;
using Xunit;

namespace RecursiveAI.OnePath.Scope.Tests;

public class StageTests
{
    // The library's stages called one after another by this test, with the pipeline's own settings; the module must give the same hashes.
    private static ulong[] DirectHashes(byte[] file)
    {
        BuildInput.Parsed input = BuildInput.Read(file, "x");
        var pipe = new Mesh3DPipeline(input.Settings);
        double[] v = input.Vertices;
        double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;
        foreach (int i in input.Indices)
        {
            minX = Math.Min(minX, v[i * 3]);
            maxX = Math.Max(maxX, v[i * 3]);
            minY = Math.Min(minY, v[(i * 3) + 1]);
            maxY = Math.Max(maxY, v[(i * 3) + 1]);
            minZ = Math.Min(minZ, v[(i * 3) + 2]);
            maxZ = Math.Max(maxZ, v[(i * 3) + 2]);
        }

        float cs = (float)input.Settings.CellSize, ch = (float)input.Settings.CellHeight;
        var frame = HeightfieldFrame.FromBounds(minX, minY - ch, minZ, maxX, maxY + ((pipe.SpanOptions.WalkableHeight + 1) * (double)ch), maxZ, cs, ch);
        var solid = new SolidHeightfield(frame);
        var h = new ulong[7];
        Voxelizer.Rasterize(solid, v, input.Indices, pipe.SpanOptions);
        h[0] = BuildStages.HashSolid(solid);
        SpanFilters.FilterLowHangingWalkableObstacles(solid, pipe.SpanOptions.WalkableClimb);
        SpanFilters.FilterLedgeSpans(solid, pipe.SpanOptions.WalkableHeight, pipe.SpanOptions.WalkableClimb);
        SpanFilters.FilterWalkableLowHeightSpans(solid, pipe.SpanOptions.WalkableHeight);
        h[1] = BuildStages.HashSolid(solid);
        CompactHeightfield chf = CompactHeightfield.Build(solid, pipe.SpanOptions.WalkableHeight, pipe.SpanOptions.WalkableClimb);
        Erosion.ErodeWalkableArea(chf, pipe.SpanOptions.WalkableRadius);
        h[2] = BuildStages.HashCompact(chf);
        Regions.Build(chf, pipe.Regions.RegionOptions);
        h[3] = BuildStages.HashCompact(chf);
        ContourSet contours = Contours.Build(chf, pipe.Regions.ContourOptions);
        h[4] = BuildStages.HashContours(contours);
        int tileCells = Math.Max(frame.Width, frame.Depth);
        var map = new TileCoordinates(0, tileCells, cs, (float)(tileCells * (double)cs), ch, 0);
        PolyMesh poly = Polygonizer.Build(contours, input.Settings.MaxVertsPerPoly, 0, map);
        h[5] = BuildStages.HashPoly(poly);
        h[6] = BuildStages.HashDetail(DetailMeshBuilder.Build(poly, chf, map, pipe.Detail));
        return h;
    }

    [Fact]
    public void EveryStageOnTheAxisEqualsCallingTheStageDirectly()
    {
        byte[] file = File.ReadAllBytes(Worlds.DataPath("terrain.opbi"));
        ulong[] direct = DirectHashes(file);
        Assert.All(direct, x => Assert.NotEqual(0UL, x));
        Assert.Equal(7, new HashSet<ulong>(direct).Count);

        var history = new History(new OnePathModule().Open(file, "terrain"));
        Assert.Contains(history.Source.Info.Axes, a => a.Id == "stage" && a.PositionCount == 7);

        void Check(int k)
        {
            history.Seek("stage", k);
            var view = (BuildWorld)history.View;
            Assert.Equal("stage", history.ViewAxis);
            Assert.Equal(k, history.ViewPosition);
            Assert.Equal(direct[k - 1], view.StageHash(k));
        }

        for (int k = 1; k <= 7; k++)
        {
            Check(k);
        }

        for (int k = 7; k >= 1; k--)
        {
            Check(k);
        }

        foreach (int k in new[] { 4, 2, 6, 1, 7, 3, 5 })
        {
            Check(k);
        }

        // After a time seek (to the end and back) the axis gives the same hashes, and the world's own hash never moved.
        ulong world = history.StateHash;
        history.Seek(1);
        history.Seek(0);
        Check(5);
        Check(1);
        Assert.Equal(world, history.StateHash);
        Assert.True(history.Step("stage", 2));
        Assert.Equal(3, history.ViewPosition);
        history.Seek("stage", 0);
        Assert.Equal(0, ((BuildWorld)history.View).ViewStage);
    }

    [Fact]
    public void StagesDrawSomethingAndDrawStably()
    {
        byte[] file = File.ReadAllBytes(Worlds.DataPath("terrain.opbi"));
        var history = new History(new OnePathModule().Open(file, "terrain"));
        var list = new DrawList();
        for (int k = 1; k <= 7; k++)
        {
            history.Seek("stage", k);
            list.Clear();
            history.View.Draw(list, new DrawRequest(0b11));
            int items = list.Items.Length;
            Assert.True(items > 12, $"stage {k} drew {items} items");
            ulong first = DrawHash.Of(list);
            history.Seek("stage", k);
            list.Clear();
            history.View.Draw(list, new DrawRequest(0b11));
            Assert.Equal(first, DrawHash.Of(list));
        }
    }

    [Fact]
    public void TheFinishedMeshOverlayDrawsPolygons()
    {
        var history = new History(new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath("terrain.opbi")), "terrain"));
        var list = new DrawList();
        history.World.Draw(list, new DrawRequest(1UL << 2));
        Assert.Contains(list.Items.ToArray(), i => i.Kind == DrawKind.Mesh);
        Assert.Contains(list.Items.ToArray(), i => i.Kind == DrawKind.Line);
    }
}

internal static class DrawHash
{
    public static ulong Of(DrawList l)
    {
        ulong h = 0xCBF29CE484222325UL;
        void Add(long v)
        {
            for (int k = 0; k < 8; k++)
            {
                h = (h ^ (byte)(v >> (8 * k))) * 0x100000001B3UL;
            }
        }

        void Vec(System.Numerics.Vector3 v)
        {
            Add(BitConverter.SingleToInt32Bits(v.X));
            Add(BitConverter.SingleToInt32Bits(v.Y));
            Add(BitConverter.SingleToInt32Bits(v.Z));
        }

        foreach (DrawItem i in l.Items)
        {
            Add((int)i.Kind);
            Add(i.Overlay);
            Add(i.Layer);
            Add((long)i.Entity.Value);
            Add(i.Color.R | (i.Color.G << 8) | (i.Color.B << 16) | (i.Color.A << 24));
            Vec(i.A);
            Vec(i.B);
            Vec(i.C);
            Vec(i.Size);
            Add(i.First);
            Add(i.Count);
        }

        foreach (System.Numerics.Vector3 v in l.Vertices)
        {
            Vec(v);
        }

        foreach (int x in l.Indices)
        {
            Add(x);
        }

        Add(l.Dropped);
        return h;
    }
}
