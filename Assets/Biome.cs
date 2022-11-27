using System;
using UnityEngine;

public enum Biome
{
    Forest,
    City,
    Field
}

public static class BiomeExtensions
{
    public static Color ToColor(this Biome b)
    {
        return b switch
        {
            Biome.Forest => Color.green,
            Biome.City => Color.white,
            Biome.Field => Color.yellow,
            _ => throw new ArgumentOutOfRangeException(nameof(b), b, null)
        };
    }
}

