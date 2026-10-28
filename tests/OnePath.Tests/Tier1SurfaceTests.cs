using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

/// <summary>
/// The happy path never asks a caller to write a struct that implements one of our callback interfaces. Struct-generic
/// hooks (<c>where TCost : struct, ICost</c>) belong to the middle and raw tiers, which live in sub-namespaces. This test
/// walks every public type in the root namespace <c>RecursiveAI.OnePath</c> and fails on a public member that
/// (a) has a type parameter constrained to a struct, or (b) takes or returns an interface declared by this library
/// (other than the change-listener observers).
/// Plain delegates and BCL interfaces (spans, lists) are fine.
/// </summary>
public class Tier1SurfaceTests
{
    private const BindingFlags Members =
        BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;

    // Observer registration, not a search hook: a class (or lambda-backed object) implements it, never a struct,
    // and nothing on the happy path needs one. Listed so a new interface parameter has to be argued for.
    private static readonly string[] AllowedObservers = ["IGridChangeListener", "IVoxelChangeListener"];

    private static readonly Assembly Library = typeof(NavMesh).Assembly;

    private static IEnumerable<Type> Tier1Types() =>
        Library.GetExportedTypes().Where(t => t.Namespace == "RecursiveAI.OnePath");

    [Fact]
    public void Root_namespace_holds_the_entry_types()
    {
        string[] names = Tier1Types().Select(t => t.Name).ToArray();
        foreach (string expected in new[] { "NavMesh", "Grid", "HexGrid", "Graph", "PointGraph", "Crowd", "Agent", "GridOptions", "NavSettings", "CrowdOptions" })
        {
            Assert.Contains(expected, names);
        }
    }

    [Fact]
    public void No_tier1_member_needs_a_struct_callback()
    {
        var violations = new List<string>();
        foreach (Type type in Tier1Types())
        {
            if (HasStructConstraint(type.GetGenericArguments()))
            {
                violations.Add(type.FullName + ": struct-constrained type parameter");
            }
            foreach (MemberInfo member in type.GetMembers(Members))
            {
                if (!IsVisible(member))
                {
                    continue;
                }
                if (member is MethodBase method)
                {
                    if (method.IsGenericMethod && HasStructConstraint(method.GetGenericArguments()))
                    {
                        violations.Add(Describe(method) + ": struct-constrained type parameter");
                    }
                    foreach (ParameterInfo p in method.GetParameters())
                    {
                        if (IsOurInterface(p.ParameterType) && !AllowedObservers.Contains(p.ParameterType.Name))
                        {
                            violations.Add(Describe(method) + ": parameter " + p.Name + " is " + p.ParameterType.Name);
                        }
                    }
                    if (method is MethodInfo mi && IsOurInterface(mi.ReturnType))
                    {
                        violations.Add(Describe(method) + ": returns " + mi.ReturnType.Name);
                    }
                }
                else if (member is PropertyInfo prop && IsOurInterface(prop.PropertyType))
                {
                    violations.Add(type.Name + "." + prop.Name + ": property of " + prop.PropertyType.Name);
                }
            }
        }
        Assert.True(violations.Count == 0, string.Join(Environment.NewLine, violations));
    }

    private static bool IsVisible(MemberInfo member) => member switch
    {
        MethodBase m => m.IsPublic || m.IsFamily || m.IsFamilyOrAssembly,
        PropertyInfo p => (p.GetMethod?.IsPublic ?? false) || (p.SetMethod?.IsPublic ?? false),
        _ => false,
    };

    private static bool HasStructConstraint(Type[] parameters) =>
        parameters.Any(p => (p.GenericParameterAttributes & GenericParameterAttributes.NotNullableValueTypeConstraint) != 0);

    private static bool IsOurInterface(Type type)
    {
        if (type.IsByRef || type.IsArray || type.IsPointer)
        {
            type = type.GetElementType()!;
        }
        if (type.IsGenericType)
        {
            type = type.GetGenericTypeDefinition();
        }
        return type.IsInterface && type.Assembly == Library;
    }

    private static string Describe(MethodBase m) => m.DeclaringType!.Name + "." + m.Name;
}
