using System;
using RecursiveAI.OnePath;

// Exercises the public surface under NativeAOT and full trimming. Exit code 0 means it ran.
Console.WriteLine($"OnePath {OnePathInfo.Version}");
Console.WriteLine($"OneMath {OnePathInfo.OneMathVersion}");
bool ok = OnePathInfo.FixedToDouble(3L << 31) == 1.5;
try
{
    OnePathInfo.FixedToDouble(long.MaxValue);
    ok = false;
}
catch (ArgumentOutOfRangeException)
{
}
return ok && OnePathInfo.Version.Major >= 0 ? 0 : 1;
