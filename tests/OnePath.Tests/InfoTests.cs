using RecursiveAI.OneMath;
using RecursiveAI.OnePath;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

public class InfoTests
{
    [Fact]
    public void VersionIsPresent()
    {
        Assert.True(OnePathInfo.Version.Major >= 0);
    }

    [Fact]
    public void OneMathResolvesAndConverts()
    {
        Assert.True(OnePathInfo.OneMathVersion.Major >= 0);
        Assert.Equal(1.5, FixedBridge.ToDouble(3L << 31));
        Assert.Throws<System.ArgumentOutOfRangeException>(() => FixedBridge.ToDouble(long.MaxValue));
    }
}
