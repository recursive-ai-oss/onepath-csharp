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
        Assert.Equal(1.5, OnePathInfo.FixedToDouble(3L << 31));
        Assert.Throws<System.ArgumentOutOfRangeException>(() => OnePathInfo.FixedToDouble(long.MaxValue));
    }
}
