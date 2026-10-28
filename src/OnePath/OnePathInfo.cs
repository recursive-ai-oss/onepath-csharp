using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath;

/// <summary>Version information for the OnePath library.</summary>
public static class OnePathInfo
{
    /// <summary>The version of the loaded OnePath assembly. Never <see langword="null"/>.</summary>
    public static System.Version Version => typeof(OnePathInfo).Assembly.GetName().Version ?? new System.Version(0, 0);

    /// <summary>The version of the loaded RecursiveAI.OneMath assembly that OnePath computes with. Never <see langword="null"/>.</summary>
    public static System.Version OneMathVersion => typeof(FixedBridge).Assembly.GetName().Version ?? new System.Version(0, 0);
}
