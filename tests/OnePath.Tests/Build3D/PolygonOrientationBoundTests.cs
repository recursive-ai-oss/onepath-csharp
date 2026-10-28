using System;
using RecursiveAI.OneMath.Predicates;
using RecursiveAI.OnePath.Mesh.Build3D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

/// <summary>
/// The polygon stage skips the exact predicate for turns of at least <c>TileCoordinates.SafeCross</c> cell areas, on the claim that rounding the corners to float32
/// cannot change their sign. Here the claim is checked against the exact predicate on random and nearly collinear triples, long and short, for tile sizes and cell sizes
/// well beyond those the builders use.
/// </summary>
public class PolygonOrientationBoundTests
{
    [Theory]
    [InlineData(8, 3, 0.01)]
    [InlineData(64, 8, 0.3)]
    [InlineData(128, 10, 0.05)]
    [InlineData(256, 6, 1.0)]
    [InlineData(512, 20, 0.1)]
    [InlineData(1024, 5, 0.37)]
    [InlineData(4096, 5, 0.013)]
    public void TurnsBeyondTheBoundKeepTheirSignInFloat32(int tileCells, int border, double cellSize)
    {
        var map = new TileCoordinates(border, tileCells, cellSize, tileCells * cellSize, 0.2, 0);
        int span = tileCells + (2 * border);
        var rng = new Random((tileCells * 31) + border);
        long checkedTurns = 0;
        for (int i = 0; i < 400_000; i++)
        {
            long ax = rng.Next(span + 1), az = rng.Next(span + 1), bx = rng.Next(span + 1), bz = rng.Next(span + 1);
            long cx, cz;
            if (i % 2 == 0)
            {
                cx = rng.Next(span + 1);
                cz = rng.Next(span + 1);
            }
            else
            {
                // Close to the line through a and b, where the turn is small however far apart the points are.
                double t = (rng.NextDouble() * 3) - 1;
                cx = Math.Clamp((long)Math.Round(ax + (t * (bx - ax))) + rng.Next(-2, 3), 0, span);
                cz = Math.Clamp((long)Math.Round(az + (t * (bz - az))) + rng.Next(-2, 3), 0, span);
            }

            long cross = ((bx - ax) * (cz - az)) - ((bz - az) * (cx - ax));
            if (Math.Abs(cross) < map.SafeCross)
            {
                continue;
            }

            int exact = ExactPredicates.Orient2D(map.Local(ax), map.Local(az), map.Local(bx), map.Local(bz), map.Local(cx), map.Local(cz));
            Assert.Equal(Math.Sign(cross), Math.Sign(exact));
            checkedTurns++;
        }

        Assert.True(checkedTurns > 1000);
    }
}
