"""3D-printable mount: one plate that holds the Feather and the BNO085 rigidly side by side.

  .venv/bin/python cad/mount.py              # writes cad/mount.stl
  .venv/bin/python cad/mount.py --preview    # also saves cad/mount.png to look at

The boards sit on eight posts and are fixed with M2.5 screws that cut their own thread into the posts.
The IMU's long edge lines up with the Feather's, so the sensor's axes line up with the plate's edges.

Every size is a number at the top of this file. MEASURE YOUR BOARDS' HOLE SPACING before printing:
the hole positions come from Adafruit's standard layout (holes 0.1 inch in from each edge), not from
measuring these two boards.
"""
import argparse
import struct
from pathlib import Path

import numpy as np

# --- Sizes, all in mm ----------------------------------------------------------
INCH = 25.4
FEATHER = (2.0 * INCH, 0.9 * INCH)   # circuit board only; the USB-C socket sticks out a little further
IMU = (1.0 * INCH, 0.9 * INCH)       # BNO085 breakout (Adafruit lists 25.6 x 22.7 mm)
HOLE_INSET = 0.1 * INCH              # hole centre to board edge, on both boards

PLATE_THICK = 2.4
MARGIN = 3.0                         # plate beyond the boards on every side
GAP = 6.0                            # between the two boards: room for the STEMMA QT cable to bend
POST_HEIGHT = 5.0                    # lifts the boards clear of the solder joints underneath
POST_DIAMETER = 5.0
# M2.5 screw cutting its own thread. Printers make holes tight: if the screw won't start, use 2.3.
SCREW_HOLE = 2.1
SEGMENTS = 48                        # facets around each post


# --- Shapes as triangles: arrays of shape (N, 3 corners, 3 coordinates) ---------------
def box(x0, y0, z0, x1, y1, z1):
    """A solid box. Corners are listed anticlockwise seen from outside, which is how an STL marks "outside"."""
    v = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])   # index = 4x + 2y + z
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return np.array([[v[a], v[b], v[c]] for q in quads for a, b, c in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))])


def tube(cx, cy, z0, z1, outer_d, inner_d, n=SEGMENTS):
    """A hollow cylinder standing on end: a post with a screw hole down the middle."""
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = lambda d, z: np.column_stack([cx + d / 2 * np.cos(ang), cy + d / 2 * np.sin(ang), np.full(n, z)])  # noqa: E731
    ob, ot, ib, it = ring(outer_d, z0), ring(outer_d, z1), ring(inner_d, z0), ring(inner_d, z1)
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [[ob[i], ob[j], ot[j]], [ob[i], ot[j], ot[i]],     # outside wall
                 [ib[i], it[j], ib[j]], [ib[i], it[i], it[j]],     # hole wall (faces inward)
                 [ot[i], ot[j], it[j]], [ot[i], it[j], it[i]],     # top rim
                 [ob[i], ib[j], ob[j]], [ob[i], ib[i], ib[j]]]     # bottom rim
    return np.array(tris)


def volume(tris):
    """Volume enclosed by a closed surface. Negative if the triangles face inward."""
    return float(np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum() / 6)


def is_closed(tris):
    """True if the surface has no gaps and one consistent outside: every edge is used once in each direction."""
    key = {tuple(p): i for i, p in enumerate(np.unique(np.round(tris.reshape(-1, 3), 6), axis=0).tolist())}
    ids = [[key[tuple(np.round(p, 6).tolist())] for p in tri] for tri in tris]
    edges = [(t[k], t[(k + 1) % 3]) for t in ids for k in range(3)]
    return len(set(edges)) == len(edges) and all((b, a) in set(edges) for a, b in edges)


def hole_centres(x0, y0, size):
    """The four mounting holes of a board whose corner is at (x0, y0)."""
    return [(x0 + dx, y0 + dy) for dx in (HOLE_INSET, size[0] - HOLE_INSET) for dy in (HOLE_INSET, size[1] - HOLE_INSET)]


def build():
    """Returns (list of solid parts, plate length, plate width). The Feather's USB end is at x = 0."""
    width = max(FEATHER[1], IMU[1]) + 2 * MARGIN
    length = MARGIN + FEATHER[0] + GAP + IMU[0] + MARGIN
    parts = [box(0, 0, 0, length, width, PLATE_THICK)]
    feather_corner = (MARGIN, (width - FEATHER[1]) / 2)
    imu_corner = (MARGIN + FEATHER[0] + GAP, (width - IMU[1]) / 2)
    for cx, cy in hole_centres(*feather_corner, FEATHER) + hole_centres(*imu_corner, IMU):
        # Posts start 0.5 mm inside the plate so the slicer fuses them to it; the plate closes the hole's bottom.
        parts.append(tube(cx, cy, PLATE_THICK - 0.5, PLATE_THICK + POST_HEIGHT, POST_DIAMETER, SCREW_HOLE))
    return parts, length, width


def write_stl(path, tris):
    """Binary STL: 80-byte header, triangle count, then normal + 3 corners + 2 spare bytes per triangle."""
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    with open(path, "wb") as f:
        f.write(b"Feather + BNO085 mount, mm".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for n, t in zip(normals, tris):
            f.write(struct.pack("<12fH", *n, *t.ravel(), 0))


# --- Sanity checks -------------------------------------------------------
assert is_closed(box(0, 0, 0, 2, 3, 4)) and np.isclose(volume(box(0, 0, 0, 2, 3, 4)), 24)
_t = tube(0, 0, 0, 10, 5, 2, n=360)
assert is_closed(_t) and np.isclose(volume(_t), np.pi * (2.5 ** 2 - 1 ** 2) * 10, rtol=1e-3)
assert not is_closed(box(0, 0, 0, 1, 1, 1)[:-1])   # a missing triangle is a gap

_parts, _length, _width = build()
assert len(_parts) == 9 and all(is_closed(p) and volume(p) > 0 for p in _parts)
assert POST_DIAMETER / 2 <= HOLE_INSET + MARGIN, "posts would hang over the plate's edge"
assert POST_DIAMETER - SCREW_HOLE >= 2.4, "post wall too thin to print"
# Hole spacing the boards must match: 45.72 x 17.78 mm (Feather) and 20.32 x 17.78 mm (IMU).
assert np.allclose(np.ptp(hole_centres(0, 0, FEATHER), axis=0), [45.72, 17.78])
assert np.allclose(np.ptp(hole_centres(0, 0, IMU), axis=0), [20.32, 17.78])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Write the Feather + BNO085 mount as an STL file.")
    parser.add_argument("--preview", action="store_true", help="also save a picture, cad/mount.png")
    args = parser.parse_args()

    here = Path(__file__).resolve().parent
    parts, length, width = build()
    tris = np.concatenate(parts)
    write_stl(here / "mount.stl", tris)
    print(f"wrote {here / 'mount.stl'}: {len(tris)} triangles")
    print(f"  plate {length:.1f} x {width:.1f} x {PLATE_THICK} mm, {POST_HEIGHT} mm posts, total height {PLATE_THICK + POST_HEIGHT} mm")
    print(f"  hole spacing to check against the boards: Feather 45.72 x 17.78 mm, IMU 20.32 x 17.78 mm")
    print(f"  screws: 8 x M2.5, 6 mm long, into {SCREW_HOLE} mm holes")

    if args.preview:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        fig = plt.figure(figsize=(9, 5))
        ax = fig.add_subplot(projection="3d")
        ax.add_collection3d(Poly3DCollection(tris, facecolors="#9fb4c7", edgecolors="#44566a", linewidths=0.15))
        ax.set(xlim=(0, length), ylim=(0, width), zlim=(0, PLATE_THICK + POST_HEIGHT),
               xlabel="mm (USB end at 0)", ylabel="mm", zlabel="mm")
        ax.set_box_aspect((length, width, PLATE_THICK + POST_HEIGHT))
        ax.view_init(elev=35, azim=-60)
        fig.tight_layout()
        fig.savefig(here / "mount.png", dpi=130)
        print(f"  saved {here / 'mount.png'}")
