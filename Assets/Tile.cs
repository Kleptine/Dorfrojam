using System;
using System.Collections.Generic;
using UnityEngine.Assertions;
using Utilities;
using Random = UnityEngine.Random;

public class Tile
{
    // 0-6 representing the rotation of the tile (rotates the below arrays).
    public int rotation;

    public Biome[] biomes = new Biome[6];

    /// <summary>
    ///  A region id for every biome. Same order as above.
    /// </summary>
    public int[] regions = new int[6];

    public Biome Biome(Direction dir, int extraRotation = 0)
    {
        return biomes[NumUtils.Mod((int)dir - extraRotation, 6)];
    }

    public void Rotate(int rot)
    {
        rotation = NumUtils.Mod(rotation - rot, 6);
        RotateArray(biomes, -rot);
        RotateArray(regions, -rot);
    }

    public static Tile CreateWithBiomes(params Biome[] biomes)
    {
        Tile t = new()
        {
            biomes = biomes,
        };
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (t.biomes[i] == t.biomes[j])
                {
                    t.regions[i] = j;
                    break;
                }
            }

        }

        return t;
    }

    public static Tile CreateRandom()
    {
        Tile t = new Tile();
        for (int i = 0; i < 6; i++)
        {
            t.biomes[i] = (Biome)Random.Range(0, Enum.GetValues(typeof(Biome)).Length);
        }

        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (t.biomes[i] == t.biomes[j])
                {
                    t.regions[i] = j;
                    break;
                }
            }

        }

        return t;
    }

    private static void RotateArray<T>(T[] array, int rotation)
    {
        var temp = new T[array.Length];

        for (int i = 0; i < array.Length; i++)
        {
            temp[i] = array[NumUtils.Mod(i + rotation, array.Length)];
        }

        for (int i = 0; i < array.Length; i++)
        {
            array[i] = temp[i];
        }
    }
}