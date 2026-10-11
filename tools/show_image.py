#!/usr/bin/env python3
"""Pop open an image file in a real window, visible over this project's
own VNC viewer (make vnc + make bash's own DISPLAY=:2) -- there's no
image viewer installed in the container (no eog/feh/xdg-open; checked),
but matplotlib + a real GUI backend (TkAgg, confirmed importable in this
image) is, and it respects $DISPLAY like any other X client.

Usage (inside the bridge container, after `make vnc` and `make bash`):
    python3 tools/show_image.py /tmp/planner_plots/plan_00003_....png
"""
import argparse
import sys

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image_path")
    args = parser.parse_args()

    img = plt.imread(args.image_path)
    fig = plt.figure()
    plt.imshow(img)
    plt.axis("off")
    fig.canvas.manager.set_window_title(args.image_path)
    plt.show()


if __name__ == "__main__":
    main()
