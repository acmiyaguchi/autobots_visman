"""Script that generates a texture with a checkboard pattern."""
from argparse import ArgumentParser
from typing import Tuple

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np


def draw_checkerboard(
    n_grid: int,
    shape: Tuple[int, int, int] = (512, 512, 3),
    color: Tuple[int, int, int] = (255, 0, 0),
) -> np.array:
    """Create an image that displays a texture."""
    texture = np.zeros(shape, dtype=np.uint8)
    # create a texture with a red background
    texture[:, :, :] = color

    # draw a checkboard pattern, filling in every other quadrant with black
    for i in range(n_grid):
        for j in range(n_grid):
            if (i + j) % 2 != 0:
                continue
            i_size = shape[0] // n_grid
            j_size = shape[1] // n_grid
            texture[i * i_size : (i + 1) * i_size, j * j_size : (j + 1) * j_size, :] = 0

    return texture


def main():
    parser = ArgumentParser()
    parser.add_argument("--n_grid", default=4, type=int, help="number of grid squares")
    parser.add_argument("--texture_width", default=400, type=int, help="texture width")
    parser.add_argument("--output", default="texture.png", help="output file")
    parser.add_argument(
        "--show", action="store_true", help="show the texture", default=False
    )
    args = parser.parse_args()

    texture = draw_checkerboard(
        args.n_grid, (args.texture_width, args.texture_width, 3)
    )

    if args.show:
        plt.imshow(texture)
        plt.show()
    else:
        mpimg.imsave(args.output, texture)


if __name__ == "__main__":
    main()
