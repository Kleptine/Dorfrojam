using System.Collections.Generic;
using UnityEngine;
using Utils;

public class TileVisuals : MonoBehaviour
{
    public Tile Tile;
    public Material ConnectionMaterial;

    private List<Mesh> freeMeshes = new List<Mesh>();
    private List<Mesh> usedMeshes = new List<Mesh>();

    private void Update()
    {
        DrawDebugForTile();

        freeMeshes.AddRange(usedMeshes);
        usedMeshes.Clear();
    }

    private void DrawDebugForTile()
    {
        var regions = Tile.regions;
        const float height = .1f;
        for (int i = 0; i < 6; i++)
        {
            bool singleRegion = true;
            for (int j = 0; j < 6; j++)
            {
                if (i != j && regions[j] == regions[i])
                {
                    if (j > i)
                    {
                        Mesh mesh = MeshForConnection(Tile, (Direction)i, (Direction)j);
                        Graphics.DrawMesh(mesh, transform.position + transform.up * height,
                            transform.rotation, ConnectionMaterial, 0, null, 0, null, false, false, true);
                    }

                    singleRegion = false;
                }
            }

            if (singleRegion)
            {
                Mesh mesh = MeshForSingle(Tile, (Direction)i);
                Graphics.DrawMesh(mesh, transform.position + Vector3.up * height,
                    Quaternion.AngleAxis(0, Vector3.up),
                    ConnectionMaterial, 0, null, 0, null, false, false, true);
            }
        }
    }

    private Mesh MeshForConnection(Tile t, Direction first, Direction second)
    {
        var (s1, s2) = ShrinkPoints(GameUtils.TilePointPosition((int)first),
            GameUtils.TilePointPosition((int)first + 1));
        var (e1, e2) = ShrinkPoints(
            GameUtils.TilePointPosition((int)second), GameUtils.TilePointPosition((int)second + 1));

        var c1 = t.biomes[(int)first].ToColor();
        var c2 = t.biomes[(int)second].ToColor();

        Mesh m = GetPooledMesh();

        m.vertices = new[]
        {
            s1, s2, e1, e2
        };
        m.normals = new[]
        {
            Vector3.up,
            Vector3.up,
            Vector3.up,
            Vector3.up,
        };
        m.triangles = new[]
        {
            2, 1, 0, 2, 0, 3
        };
        m.colors = new[]
        {
            c1, c1, c2, c2
        };
        return m;
    }

    private Mesh MeshForSingle(Tile t, Direction dir)
    {
        const float size = .2f;
        var (s1, s2) = ShrinkPoints(GameUtils.TilePointPosition((int)dir),
            GameUtils.TilePointPosition((int)dir + 1));
        var midpoint = (s1 + s2) / 2f;

        var shift = (Vector3.zero - midpoint).normalized * size;

        var (e1, e2) = (s1 + shift, s2 + shift);

        var color = t.biomes[(int)dir].ToColor();

        Mesh m = GetPooledMesh();
        m.vertices = new[]
        {
            s1, s2, e1, e2
        };
        m.normals = new[]
        {
            Vector3.up,
            Vector3.up,
            Vector3.up,
            Vector3.up,
        };
        m.triangles = new[]
        {
            2, 1, 0, 2, 1, 3
        };
        m.colors = new[]
        {
            color, color, color, color
        };
        return m;
    }

    private (Vector3 p1, Vector3 p2) ShrinkPoints(Vector3 p1, Vector3 p2)
    {
        const float width = .1f;
        var midpoint = (p1 + p2) / 2f;
        return (
            (p1 - midpoint).normalized * width / 2f + midpoint,
            (p2 - midpoint).normalized * width / 2f + midpoint);
    }

    private Mesh GetPooledMesh()
    {
        if (freeMeshes.Count > 0)
        {
            Mesh m = freeMeshes[^1];
            freeMeshes.RemoveAt(freeMeshes.Count - 1);
            usedMeshes.Add(m);
            return m;
        }
        else
        {
            Mesh m = new();
            usedMeshes.Add(m);
            if (usedMeshes.Count > 10000)
            {
                Debug.LogWarning("Using way too many meshes for debug drawing");
            }

            return m;
        }
    }

    private void OnDestroy()
    {
        foreach (Mesh m in usedMeshes)
        {
            Destroy(m);
        }

        foreach (Mesh m in freeMeshes)
        {
            Destroy(m);
        }
    }
}
