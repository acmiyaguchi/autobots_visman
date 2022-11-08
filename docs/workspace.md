# workspace calibration

## generating the workspace

We need at least six points (x, y, z) in the world frame in order to perform
direct linear transformation (DLT) from the depth camera. We get these points by
creating a small grid to split our surface into quandrants, with the assumption
that the origin of the grid is also the origin of the surface.

```bash
python src/generate_texture.py \
    --output models/textured_box/materials/textures/grid.png
```
