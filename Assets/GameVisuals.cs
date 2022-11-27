using System.Collections.Generic;
using UnityEngine;
using Utilities.Unity;
using Utils;

namespace DefaultNamespace
{
    public class GameVisuals : MonoBehaviour
    {
        public GameObject TilePrefab;
        public Transform TileBank;
        public Mesh AvailableTileMesh;
        public Material AvailableTileMaterial;

        private readonly TwoWayDictionary<Tile, GameObject> tileObjects = new();
        private GameObject hoverVisual;

        private readonly TwoWayDictionary<Tile, GameObject> bankObjects = new();

        // Cache for the available spaces around the tiles.
        private Matrix4x4[] emptySpotTransforms = new Matrix4x4[0]; // automatically expands
        private int emptySpotCount;

        private void Awake()
        {
            foreach (var child in transform.GetChildren())
            {
                Destroy(child.gameObject);
            }

        }

        public void UpdateVisuals(GameState state)
        {
            UpdateGameBoard(state);
            UpdateBank(state);

            if (hoverVisual == null)
            {
                hoverVisual = InstantiateTileVisual(state.Hover);
            }

            var hoverT = hoverVisual.transform;
            hoverT.position = Vector3.Lerp(hoverT.position, GameUtils.HexToWorld(state.HoverPosition), .1f);
            hoverT.rotation = Quaternion.AngleAxis(state.Hover.rotation * 60, Vector3.up);
        }

        private void UpdateGameBoard(GameState state)
        {
            // Delete the tiles that were removed.
            // Update tile positions.
            List<Tile> toRemove = new();
            foreach (KeyValuePair<Tile, GameObject> pair in tileObjects)
            {
                if (!state.Tiles.ContainsValue(pair.Key))
                {
                    DestroyImmediate(pair.Value); // todo: animate away?
                    toRemove.Add(pair.Key);
                }
            }

            foreach (var r in toRemove)
            {
                tileObjects.Remove(r);
            }

            // Spawn new tiles for those that were created.
            foreach (KeyValuePair<HexVector, Tile> pair in state.Tiles)
            {
                var planePos = GameUtils.HexToPlane(pair.Key);
                Vector3 worldPos = new Vector3(planePos.x, 0, planePos.y);
                if (!tileObjects.ContainsKey(pair.Value))
                {
                    GameObject prefab = InstantiateTileVisual(pair.Value, worldPos, Quaternion.identity, transform);
                    tileObjects.Add(pair.Value, prefab);
                }

                var t = tileObjects[pair.Value].transform;
                t.position = Vector3.Lerp(t.position, worldPos, .1f);
                t.rotation = Quaternion.AngleAxis(pair.Value.rotation * 60, Vector3.up);
            }
        }

        private void UpdateBank(GameState state)
        {
            // Delete the tiles that were removed.
            // Update tile positions.
            // List<Tile> toRemove = new();
            // foreach (KeyValuePair<Tile, GameObject> pair in bankObjects)
            // {
            //     if (!state.Tiles.ContainsValue(pair.Key))
            //     {
            //         DestroyImmediate(pair.Value); // todo: animate away?
            //         toRemove.Add(pair.Key);
            //     }
            // }
            //
            // foreach (var r in toRemove)
            // {
            //     bankObjects.Remove(r);
            // }

            // Spawn new tiles for those that were created.
            int i = 0;
            foreach (Tile tile in state.Bank)
            {
                Vector3 localPos = new(0, i * .3f + 1, 0);
                if (!bankObjects.ContainsKey(tile))
                {
                    var prefab = InstantiateTileVisual(tile, parent: TileBank);
                    bankObjects.Add(tile, prefab);
                }

                var t = bankObjects[tile].transform;
                t.localPosition = Vector3.Lerp(t.localPosition, localPos, .1f);
                t.localRotation = Quaternion.AngleAxis(tile.rotation * 60, Vector3.up);
                i++;
            }
        }

        public void UpdateValidSpaces(HashSet<HexVector> validSpots)
        {
            if (emptySpotTransforms.Length < validSpots.Count)
            {
                emptySpotTransforms = new Matrix4x4[validSpots.Count];
            }

            int j = 0;
            foreach (HexVector pos in validSpots)
            {
                emptySpotTransforms[j] = Matrix4x4.TRS(
                    GameUtils.HexToWorld(pos),
                    Quaternion.identity,
                    Vector3.one);
                j++;
            }

            emptySpotCount = validSpots.Count;
        }

        private void Update()
        {
            // Draw available tiles
            Graphics.DrawMeshInstanced(AvailableTileMesh, 0, AvailableTileMaterial, emptySpotTransforms,
                emptySpotCount);
        }

        public GameObject InstantiateTileVisual(Tile t, Vector3? position = null, Quaternion? rotation = null, Transform parent = null)
        {
            GameObject prefab = Instantiate(TilePrefab, position ?? Vector3.zero, rotation ?? Quaternion.identity, parent == null ? transform : parent);
            prefab.GetComponent<TileVisuals>().Tile = t;
            return prefab;
        }
    }
}
