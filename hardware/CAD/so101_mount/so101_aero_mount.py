"""SO-101 wrist roll horn to Aero Hand Open palm: mount ("hub + plate").

Coordinates: millimetres, +Z up, toward the hand. z = 0 is the bottom of the
hub (arm side); the palm sits on the top face of the plate.

    palm  ────────────────────────────────────────────
          ┌──────────────────────────────────────────┐  plate (52 mm)
          └──cb──────────┐ ║  ║ ┌────────────────cb──┘
                         │ ║  ║ │  hub (24 mm)  ← horn screws in wells
                         └─┬──┬─┘  ← web, horn pocket below
    bracket  ▔▔▔▔▔▔▔▔▔ [ horn ] ▔▔▔▔▔▔ (bump up to 4.4 mm above horn face)

Hand side hole pattern checked against hardware/CAD/mount_adapters/
G1_Open_Unitree_Converter.stp: 4 holes at (+-13.44, +-13.44), 52.2 mm palm ring.

Arm side checked against the official SO-ARM100 CAD (STEP/SO101/SO101 Assembly.step,
Wrist_Roll_Follower_SO101.step, STS3215_03a.step):
  * horn: 20 mm dia, 4 holes on a 9.9 mm square, centre screw head ~5.6 mm dia
    sticking up ~2.4 mm above the horn face
  * the original gripper uses a 24 mm hub, a 1 mm deep horn pocket and a 3 mm web
  * around the horn the wrist bracket rises to ~1 mm above the horn face, with one
    bump up to 4.4 mm (radius 15.3-23.1 mm from the roll axis). The plate turns with
    the wrist, so it must clear that bump all the way round.
"""

import build123d as bd
from cad import model

# ---- Hub (arm side) ---------------------------------------------------------
HUB_D = 24.0              # same as the original gripper; bracket opening is ~25.2
HORN_POCKET_D = 20.6      # horn is 20.0 mm
HORN_POCKET_DEPTH = 1.0   # same as the original gripper
WEB_T = 3.0               # same as the original gripper (M3x6 horn screws)
HORN_SQUARE = 9.9
HORN_CLEAR_D = 3.4
HORN_CBORE_D = 6.2        # wells for the horn screw heads, open to the top
CENTER_HOLE_D = 6.0       # horn centre screw head is ~5.6 mm

# ---- Plate (hand side) ------------------------------------------------------
PLATE_D = 52.0            # matches Aero's 52.2 mm palm interface ring
PLATE_T = 8.0
PLATE_LIFT = 5.5          # plate underside height above the horn face (bump is 4.4)
PALM_SQUARE = 26.88
PALM_CLEAR_D = 3.4
PALM_CBORE_D = 6.2
PALM_CBORE_DEPTH = 3.3    # M3 socket head (3.0) fully recessed; leaves 4.7 mm for M3x10

BUMP_TOP = 4.4            # from the SO-101 CAD, above the horn face


def cyl(d, h, z0=0.0, x=0.0, y=0.0):
    return bd.Pos(x, y, z0) * bd.Cylinder(
        d / 2, h, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)
    )


def square_points(side):
    h = side / 2
    return [(h, h), (-h, h), (-h, -h), (h, -h)]


def mount_body(plate_t=PLATE_T, plate_lift=PLATE_LIFT):
    horn_face_z = HORN_POCKET_DEPTH
    plate_z0 = horn_face_z + plate_lift
    top_z = plate_z0 + plate_t
    assert plate_lift - BUMP_TOP >= 1.0, "plate too close to the bracket bump"

    body = cyl(HUB_D, plate_z0 + 0.01) + cyl(PLATE_D, plate_t, plate_z0)

    # Arm side: horn pocket, web holes, wells for the screw heads, centre hole.
    body -= cyl(HORN_POCKET_D, HORN_POCKET_DEPTH)
    body -= cyl(CENTER_HOLE_D, top_z)
    for x, y in square_points(HORN_SQUARE):
        body -= cyl(HORN_CLEAR_D, top_z, x=x, y=y)
        body -= cyl(HORN_CBORE_D, top_z, horn_face_z + WEB_T, x=x, y=y)

    # Hand side: palm screws go up from under the plate.
    for x, y in square_points(PALM_SQUARE):
        body -= cyl(PALM_CLEAR_D, plate_t, plate_z0, x=x, y=y)
        body -= cyl(PALM_CBORE_D, PALM_CBORE_DEPTH, plate_z0, x=x, y=y)

    bb = body.bounding_box()
    assert abs(bb.size.X - PLATE_D) < 1e-3, bb.size
    assert abs(bb.size.Z - top_z) < 1e-3, bb.size

    body.label = "mount"
    body.color = "#4a6fa5"
    return body


@model
def so101_aero_mount():
    return mount_body()


@model
def so101_aero_mount_print():
    """Print orientation: plate top (palm side) flat on the bed, hub up. No supports."""
    body = mount_body()
    top_z = body.bounding_box().max.Z
    flipped = bd.Pos(0, 0, top_z) * bd.Rot(180, 0, 0) * body
    assert abs(flipped.bounding_box().min.Z) < 1e-6
    flipped.label = "mount"
    flipped.color = "#4a6fa5"
    return flipped


if __name__ == "__main__":
    so101_aero_mount()
    so101_aero_mount_print()
