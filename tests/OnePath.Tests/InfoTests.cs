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
}
