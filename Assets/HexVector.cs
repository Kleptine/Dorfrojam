/// <summary>
///     This is a position, or vector, on the hex grid. Just like a normal vector in 3D space, you can add, subtract,
///     scale these and the math all works!
/// </summary>
/// <remarks>
///     This uses a 3-variable coordinate system, taken from Red Blob Games:
///     https://www.redblobgames.com/grids/hexagons/#coordinates-cube Most of the functions on these vectors are taken
///     directly from that reference.
/// </remarks>
public struct HexVector
{
    public static readonly HexVector[] Neighbors =
    {
        // Order of Direction.Values
        new(0, 1), // E
        new(1, 0), // NE
        new(1, -1), // NW
        new(0, -1), // W
        new(-1, 0), // SW
        new(-1, 1), // SE
    };

    // The base coordinates along the three hex axis.
    // The third coordinate value is implicit. 
    public int row, column;

    public HexVector(int row, int column)
    {
        this.row = row;
        this.column = column;
    }

    public static HexVector zero => new HexVector(0, 0);

    public override string ToString()
    {
        return $"HexVector({row}, {column})";
    }

    public HexVector Neighbor(Direction dir)
    {
        return this + NeighborOffset(dir);
    }

    public static HexVector NeighborOffset(Direction dir)
    {
        return Neighbors[(int)dir];
    }

    public static HexVector operator +(HexVector a, HexVector b)
    {
        return new HexVector(
            a.row + b.row,
            a.column + b.column);
    }

    public static HexVector operator -(HexVector a, HexVector b)
    {
        return new HexVector(
            a.row - b.row,
            a.column - b.column);
    }

    public static HexVector operator *(HexVector a, int scale)
    {
        return new HexVector(
            a.row * scale,
            a.column * scale);
    }
}
