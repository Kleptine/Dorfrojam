using UnityEngine;

namespace Utils
{
    public class GameUtils
    {
        private const float tileWidth = 1;
        private static readonly Vector2 columnBasis = new(tileWidth, 0);
        private static readonly Vector2 rowBasis =
            tileWidth * new Vector3(Mathf.Cos(Mathf.PI / 3f), Mathf.Sin(Mathf.PI / 3f));

        private static readonly Matrix2x2 hexToWorld =
            new Matrix2x2(rowBasis.x, columnBasis.x, rowBasis.y, columnBasis.y);
        private static readonly Matrix2x2 worldToHex = hexToWorld.Inverse();

        public static Vector2 HexToPlane(HexVector v)
        {
            return rowBasis * v.row + columnBasis * v.column;
        }

        public static Vector3 HexToWorld(HexVector v)
        {
            var plane = HexToPlane(v);
            return new Vector3(
                plane.x,
                0f,
                plane.y);
        }
        
        public static HexVector WorldToHex(Vector2 v)
        {
            Vector2 hex = worldToHex * v;
            return new HexVector((int)Mathf.Round(hex.x), (int)Mathf.Round(hex.y));
        }

        public static Vector3 TilePointPosition(int pointNumber)
        {
            float size = tileWidth / Mathf.Sqrt(3);
            float angle = (pointNumber - 0.5f) * (Mathf.PI / 3f);
            return new Vector3(
                size * Mathf.Cos(angle),
                0,
                size * Mathf.Sin(angle)
            );
        }
    }
}


