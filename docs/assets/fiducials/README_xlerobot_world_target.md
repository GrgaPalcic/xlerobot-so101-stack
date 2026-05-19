# XLeRobot World Fiducial Targets

These are small rigid workspace ChArUco targets for the dual-arm calibration run.

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

Use Plate A by default. If the print quality, mounting, glare, or detection is poor,
switch to Plate B or Plate C and change only WORLD_START_ID.

Print file:

```text
xlerobot_world_targets_7x5_20mm_aruco5x5_100_ids49-99_a4.pdf
```

Use these runbook constants for Plate A:

```bash
export WORLD_COLS=7
export WORLD_ROWS=5
export WORLD_SQUARE_M=0.020000
export WORLD_MARKER_M=0.014000
export WORLD_START_ID=49
export WORLD_MARKER_COUNT=17
export WORLD_DICT=DICT_5X5_100
```

Alternative start IDs:

```text
Plate A: WORLD_START_ID=49
Plate B: WORLD_START_ID=66
Plate C: WORLD_START_ID=83
```
