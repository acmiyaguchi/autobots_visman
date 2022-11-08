"""Script that generates a 512x512 texture with quandrant colors."""
from argparse import ArgumentParser

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = ArgumentParser()
    parser.add_argument("--output", default="texture.png", help="output file")
    parser.add_argument("--show", action="store_true", help="show the texture")
    args = parser.parse_args()

    texture = np.zeros((512, 512, 3), dtype=np.uint8)
    # create a texture with a red background
    texture[:, :, 0] = 255

    # draw a black cross that goes through the origin
    width = 4
    texture[256 - width : 256 + width, :, :] = 0
    texture[:, 256 - width : 256 + width, :] = 0

    plt.imshow(texture)
    if args.show:
        plt.show()
    else:
        mpimg.imsave(args.output, texture)


if __name__ == "__main__":
    main()
