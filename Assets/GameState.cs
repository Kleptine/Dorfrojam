using System.Collections.Generic;
using UnityEngine;

public class GameState
{
    public TwoWayDictionary<HexVector, Tile> Tiles = new();
    public List<Tile> Bank = new();

    public Tile Hover;
    public HexVector HoverPosition;

    public Biome? BorderingBiome(HexVector position, Direction dir)
    {
        var spot = position.Neighbor(dir);
        if (Tiles.TryGetValue(spot, out Tile tile))
        {
            return tile.Biome(dir.Reverse());
        }

        return null;
    }

    public bool IsValidSpot(Tile t, HexVector v)
    {
        for (int rot = 0; rot < 6; rot++) // For Each Rotation
        {
            if (IsValidSpot(t, v, rot))
            {
                return true;
            }
        }

        return false;
    }

    public bool IsValidSpot(Tile t, HexVector v, int rotation)
    {
        if (Tiles.ContainsKey(v))
        {
            return false;
        }

        bool allEmpty = true;
        for (int dir = 0; dir < 6; dir++) // For Each Direction
        {
            Biome? target = BorderingBiome(v, (Direction)dir);
            if (target == null)
            {
                continue;
            }

            allEmpty = false;
            Biome biome = t.Biome((Direction)dir, rotation);
            
            if (target.Value != biome)
            {
                return false;
            }
        }

        if (allEmpty)
        {
            return false;
        }

        return true;
    }

    public int? FirstValidRotation(Tile t, HexVector vv)
    {
        for (int i = 0; i < 6; i++)
        {
            if (IsValidSpot(t, vv, i))
            {
                return i;
            }
        }

        return null;
    }

}