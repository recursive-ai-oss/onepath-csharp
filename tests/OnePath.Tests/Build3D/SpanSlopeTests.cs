using System;
using RecursiveAI.OnePath.Mesh.Build3D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

public class SpanSlopeTests
{
    [Fact]
    public void DefaultDegreesMatchDefaultCosine()
    {
        // 45 degrees must classify exactly as the old constant did, so the bits must be equal.
        var a = new SpanSettings();
        var b = new SpanSettings { WalkableSlopeDegrees = 45f };
        Assert.Equal(BitConverter.SingleToInt32Bits(0.70710677f), BitConverter.SingleToInt32Bits(b.WalkableSlopeCosine));
        Assert.Equal(a.WalkableSlopeCosine, b.WalkableSlopeCosine);
        Assert.Equal(45f, a.WalkableSlopeDegrees);
    }

    [Theory]
    [InlineData(0f, 1f)]
    [InlineData(60f, 0.5f)]
    [InlineData(90f, 0f)]
    public void DegreesGiveTheCosine(float degrees, float expected)
    {
        var s = new SpanSettings { WalkableSlopeDegrees = degrees };
        Assert.True(Math.Abs(s.WalkableSlopeCosine - expected) <= MathF.Max(expected * 1e-5f, 1e-9f), $"{degrees}: {s.WalkableSlopeCosine}");
        Assert.Equal(degrees, s.WalkableSlopeDegrees);
    }

    [Fact]
    public void NearVerticalMatchesTheBclWithinOneUlp()
    {
        var s = new SpanSettings { WalkableSlopeDegrees = 89.9f };
        float expected = (float)Math.Cos(89.9f * (Math.PI / 180.0));
        Assert.True(Math.Abs(s.WalkableSlopeCosine - expected) <= expected * 2e-7f);
        Assert.True(s.WalkableSlopeCosine > 0f);
    }

    [Fact]
    public void LastWriteWins()
    {
        var s = new SpanSettings { WalkableSlopeDegrees = 30f };
        s.WalkableSlopeCosine = 0.25f;
        Assert.Equal(0.25f, s.WalkableSlopeCosine);
        Assert.True(float.IsNaN(s.WalkableSlopeDegrees));
        s.WalkableSlopeDegrees = 60f;
        Assert.True(Math.Abs(s.WalkableSlopeCosine - 0.5f) < 1e-6f);
    }

    [Theory]
    [InlineData(-0.1f)]
    [InlineData(90.01f)]
    [InlineData(float.NaN)]
    [InlineData(float.PositiveInfinity)]
    public void BadDegreesThrowNamingTheProperty(float degrees)
    {
        var s = new SpanSettings();
        var e = Assert.Throws<ArgumentOutOfRangeException>(() => s.WalkableSlopeDegrees = degrees);
        Assert.Equal(nameof(SpanSettings.WalkableSlopeDegrees), e.ParamName);
        Assert.Equal(45f, s.WalkableSlopeDegrees);
    }
}
