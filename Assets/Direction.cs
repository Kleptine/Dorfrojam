// ReSharper disable InconsistentNaming

using System;

public enum Direction
{
    E = 0,
    NE = 1,
    NW = 2,
    W = 3,
    SW = 4,
    SE = 5
}

public static class DirectionExt
{
    public static Direction Reverse(this Direction dir)
    {
        return dir switch
        {
            Direction.E => Direction.W,
            Direction.NE => Direction.SW,
            Direction.NW => Direction.SE,
            Direction.W => Direction.E,
            Direction.SW => Direction.NE,
            Direction.SE => Direction.NW,
            _ => throw new ArgumentOutOfRangeException(nameof(dir), dir, null)
        };
    }
}
