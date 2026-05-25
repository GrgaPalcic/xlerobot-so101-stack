# XLeRobot Fiducial Targets

These are the physical ChArUco/ArUco targets for the dual-arm calibration run.
The machine-readable presets are in `charuco_board_presets.yaml`, and the same
names are available in `xlerobot-calib board-presets`.

## Intrinsics Board

Use the large caib.io board for camera intrinsics:

- preset: `intrinsics_a3_11x8_34_25_id2`
- file: `calib.io_charuco_420x297_8x11_34_25_DICT_5X5.pdf`
- page: A3 landscape, 420 mm x 297 mm
- dictionary: DICT_5X5_100
- squares: 11 x 8
- square size: 34 mm
- marker size: 25 mm
- marker ids: 2-45
- outer pattern size: 374.0 mm x 272.0 mm

Use this board for intrinsics because it covers more of the camera image and
gives better lens distortion observability than a small target.

## Workspace/World Plates

These are small rigid workspace ChArUco targets for base/world and camera
extrinsic calibration.

- dictionary: DICT_5X5_100
- plates: Plate A, Plate B, Plate C
- marker ids per plate: Plate A=49-65, Plate B=66-82, Plate C=83-99
- squares: 7 x 5
- ChArUco corners: 24
- ArUco markers per plate: 17
- square size: 20 mm
- marker size: 14 mm
- outer pattern size: 140.0 mm x 100.0 mm

Print the PDF at 100% / actual size. Disable fit-to-page and borderless scaling.
Measure the printed ChArUco squares with calipers. Each square should be 20.0 mm.
Mount each selected print to a rigid flat plate before calibration.

PDF order:

```text
Page 1 top: Plate A, ids 49-65, preset world_plate_a_7x5_20_14_id49
Page 1 bottom: Plate B, ids 66-82, preset world_plate_b_7x5_20_14_id66
Page 2 center: Plate C, ids 83-99, preset world_plate_c_7x5_20_14_id83
```

Use Plate A by default. If the print quality, mounting, glare, or detection is poor,
switch to Plate B or Plate C by applying the matching board preset.

Print file:

```text
xlerobot_world_targets_7x5_20mm_aruco5x5_100_ids49-99_a4.pdf
```

This is an A4 PDF containing three smaller plates. The important dimensions are
the printed square and marker sizes, not the paper size. Mount the selected
plate to a rigid flat backing before calibration.

Use CLI presets rather than hand-exporting board constants:

```bash
xlerobot-calib --workspace "$XLEROBOT_WS" --out "$XLEROBOT_RUN" board-presets
xlerobot-calib --workspace "$XLEROBOT_WS" --out "$XLEROBOT_RUN" use-board-preset world_plate_a_7x5_20_14_id49
xlerobot-calib --workspace "$XLEROBOT_WS" --out "$XLEROBOT_RUN" use-board-preset world_plate_b_7x5_20_14_id66
xlerobot-calib --workspace "$XLEROBOT_WS" --out "$XLEROBOT_RUN" use-board-preset world_plate_c_7x5_20_14_id83
```

## CLI Presets

```bash
xlerobot-calib --workspace "$XLEROBOT_WS" --out "$XLEROBOT_RUN" board-presets
xlerobot-calib --workspace "$XLEROBOT_WS" --out "$XLEROBOT_RUN" use-board-preset intrinsics_a3_11x8_34_25_id2
xlerobot-calib --workspace "$XLEROBOT_WS" --out "$XLEROBOT_RUN" use-board-preset world_plate_a_7x5_20_14_id49
```

Switch to Plate B or C only if Plate A has bad print quality, glare, mounting,
or detection:

```bash
xlerobot-calib --workspace "$XLEROBOT_WS" --out "$XLEROBOT_RUN" use-board-preset world_plate_b_7x5_20_14_id66
xlerobot-calib --workspace "$XLEROBOT_WS" --out "$XLEROBOT_RUN" use-board-preset world_plate_c_7x5_20_14_id83
```
