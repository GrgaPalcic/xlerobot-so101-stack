# XLeRobot World Fiducial Target

- dictionary: DICT_5X5_100
- marker ids: 50-67
- squares: 7 x 5
- square size: 20 mm
- marker size: 14 mm
- outer pattern size: 140.0 mm x 100.0 mm

Print the PDF at 100% / actual size. Disable fit-to-page and borderless scaling.
After printing, verify the ruler is exactly 100 mm and the outer pattern is exactly 140 mm x 100 mm.
Mount the print to a rigid flat plate before calibration.

Use these runbook constants:

```bash
export WORLD_COLS=7
export WORLD_ROWS=5
export WORLD_SQUARE_M=0.020000
export WORLD_MARKER_M=0.014000
export WORLD_START_ID=50
export WORLD_MARKER_COUNT=18
export WORLD_DICT=DICT_5X5_100
```
