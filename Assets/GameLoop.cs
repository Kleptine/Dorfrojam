using System.Collections.Generic;
using DefaultNamespace;
using UnityEngine;
using Utilities.Unity;
using Utils;
using static Biome;
using static Direction;

public class GameLoop : MonoBehaviour
{
    public float CameraDragSpeed = .1f;
    public float CameraZoomSpeed = .1f;

    private GameVisuals gameVisuals;
    private ControlMap.DefaultActions controls;

    public readonly GameState state = new();

    private Vector3 targetCameraPosition;
    private Camera cam;

    private HashSet<HexVector> validSpots = new();

    // todo lift hover spot up out of rendering

    private void Awake()
    {
        cam = Camera.main;
        gameVisuals = GetComponent<GameVisuals>();
        var map = new ControlMap();
        map.Enable();
        controls = map.Default;

        targetCameraPosition = cam.transform.position;

        state.Tiles.Add(
            new HexVector(0, 0), Tile.CreateWithBiomes(Forest, City, City, City, City, City)
        );
        state.Tiles.Add(
            new HexVector(-1, 1), Tile.CreateWithBiomes(City, Forest, City, City, City, City)
        );

        state.Hover = Tile.CreateWithBiomes(Field, Field, Forest,Field, City, Field);
        state.HoverPosition = new HexVector(0, 0);

        for (int i = 0; i < 16; i++)
        {
            state.Bank.Add(Tile.CreateRandom());
        }

        UpdateValid();
        gameVisuals.UpdateValidSpaces(validSpots);
    }

    private void Update()
    {
        gameVisuals.UpdateVisuals(state);

        // Move the hover tile around based on the mouse position.
        Vector2 pos = controls.MousePosition.ReadValue<Vector2>();
        Ray r = cam.ScreenPointToRay(pos, Camera.MonoOrStereoscopicEye.Mono);
        r.direction *= 100;

        Plane p = new(Vector3.up, Vector3.zero);
        if (p.Raycast(r, out float enter))
        {
            Vector3 hit = r.origin + r.direction * enter;

            // Move tile by removing and inserting at a different hex.
            HexVector hex = GameUtils.WorldToHex(new Vector2(hit.x, hit.z));
            if (state.IsValidSpot(state.Hover, hex))
            {
                // Rotate the piece into the first valid alignment
                int? rotation = state.FirstValidRotation(state.Hover, hex);

                if (rotation.HasValue)
                {
                    state.HoverPosition = hex;

                    if (rotation != 0)
                    {
                        state.Hover.Rotate(rotation.Value);
                    }
                }
            }
        }

        // On click, place tile.
        if (controls.PlaceTile.WasPressedThisFrame())
        {
            if (!state.Tiles.ContainsKey(state.HoverPosition))
            {
                state.Tiles.Add(state.HoverPosition, state.Hover);
                state.Hover = Tile.CreateRandom();

                UpdateValid();
                gameVisuals.UpdateValidSpaces(validSpots);
            }
        }

        // Rotate tiles.
        if (controls.RotateTileCW.WasPressedThisFrame())
        {
            state.Hover.Rotate(-1);
        }

        if (controls.RotateTileCCW.WasPressedThisFrame())
        {
            state.Hover.Rotate(1);
        }

        // Move Camera:
        if (controls.DragCamera.IsPressed())
        {
            var delta = controls.MouseDelta.ReadValue<Vector2>();
            Vector3 forward = cam.transform.forward.Zero(y: true).normalized;
            Vector3 right = cam.transform.right.Zero(y: true).normalized;
            targetCameraPosition -= forward * delta.y * CameraDragSpeed;
            targetCameraPosition -= right * delta.x * CameraDragSpeed;
        }

        // Zoom Camera:
        if (controls.ZoomCamera.IsInProgress())
        {
            float scroll = controls.ZoomCamera.ReadValue<Vector2>().y;
            targetCameraPosition += cam.transform.forward * CameraZoomSpeed * scroll;
        }

        cam.transform.position = Vector3.Lerp(cam.transform.position, targetCameraPosition, .1f);
    }

    public void UpdateValid()
    {
        // Update Tiles for available spaces.
        validSpots.Clear();
        foreach (KeyValuePair<HexVector, Tile> pair in state.Tiles)
        {
            for (int i = 0; i < 6; i++)
            {
                var spot = pair.Key + HexVector.Neighbors[i];
                if (!state.Tiles.ContainsKey(spot) && !validSpots.Contains(spot) && state.IsValidSpot(state.Hover, spot))
                {
                    validSpots.Add(spot);
                }
            }
        }
    }

}

