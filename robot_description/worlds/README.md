# weedNix
# تعبتتتتتتتتت
### the repo contains:

# Generate Plants in Straight Lines

## Workspace (Farm Area)
The plants are generated within the farm area defined in `farmWith1CropRow.world` with boundary points:
- (-18, -10)
- (-18, 10)
- (21, 10)
- (21, -10)

## Usage
1. Run the script:
   ```bash
   python generate_plants_straight_lines.py
   ```
2. The output file `plants.sdf` will be created.

## Parameters
- `num_rows`: Number of rows.
- `row_length`: Length of each row.
- `row_spacing`: Distance between rows.
- `original_x`, `original_y`: Starting position.

The script ensures controlled randomness while maintaining row alignment.


