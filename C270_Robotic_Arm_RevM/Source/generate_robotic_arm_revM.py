"""Revision M: 27 mm cap opening, accessible horn screws, true camera tilt.
All dimensions are millimetres. Requires CadQuery 2.6.1 and its VTK runtime.
The full build, analysis, reports and verification are run by build_verify_revM.py.
"""

from __future__ import annotations

import math
from pathlib import Path

import cadquery as cq
from cadquery import exporters


OUT = (Path(__file__).resolve().parent.parent if Path(__file__).parent.name == "Source"
       else Path(__file__).resolve().parent.parent / "outputs" / "C270_Robotic_Arm_RevM")
WORK = OUT.parent / "_cad_work_RevM"
OUT.mkdir(parents=True, exist_ok=True)
PRINT_DIR = OUT / "Printable_Parts"
REF_DIR = OUT / "Reference_Models"
ASM_DIR = OUT / "Assemblies"
DOC_DIR = OUT / "Documentation"
RENDER_DIR = OUT / "Renders"
for directory in (PRINT_DIR, REF_DIR, ASM_DIR, DOC_DIR, RENDER_DIR):
    directory.mkdir(parents=True, exist_ok=True)

# Core arm dimensions from the user's sketch.
ARM_CENTER_SPACING = 80.0
ARM_CENTERLINE_RADIUS = 50.0
ARM_WIDTH = 14.0
ARM_THICKNESS = 10.0
BOSS_DIAMETER = 28.0
PAN_PAD_DIAMETER = 40.0
PAN_PAD_THICKNESS = 4.0

# Common SG90 envelope plus print clearance.  The open flange slots tolerate
# small clone-to-clone differences and let the supplied servo model be checked
# after STEP import.
SERVO_BODY_X = 22.8
SERVO_BODY_Y = 12.2
SERVO_CLEARANCE = 0.8
TILT_AXIS_Z = 94.0

# Logitech C270 bounding data supplied by the user.
C270_WIDTH = 72.91
C270_HEIGHT = 31.91
C270_DEPTH_WITH_CLIP = 66.64
C270_MASS_G = 75.0

# Revision J screwless webcam cradle.  The basket holds the webcam body at its
# lower corners, sides, and top/front corners while leaving the lens/front, the
# middle underside open. Two low rear corner barriers prevent backward sliding,
# while a 62 mm-wide central gap lets the permanently attached C270 lead leave
# straight backward. No
# screw touches the webcam; the only screw interface is between this printed
# cradle and the SG90 horn.
CRADLE_BODY_DEPTH = 22.0
CRADLE_FIT_CLEARANCE = 0.60
CRADLE_WALL = 2.80
CRADLE_FRONT_Y = 9.0 - CRADLE_FIT_CLEARANCE
CRADLE_BACK_Y = 9.0 + CRADLE_BODY_DEPTH + CRADLE_FIT_CLEARANCE
CRADLE_BOTTOM_Z = 6.0 - CRADLE_FIT_CLEARANCE
CRADLE_TOP_Z = 6.0 + C270_HEIGHT + CRADLE_FIT_CLEARANCE
CRADLE_INNER_HALF_X = C270_WIDTH / 2.0 + CRADLE_FIT_CLEARANCE
CRADLE_OUTER_HALF_X = CRADLE_INNER_HALF_X + CRADLE_WALL
CRADLE_REAR_KEEPER_GAP_WIDTH = 62.0
CRADLE_REAR_KEEPER_INNER_X = CRADLE_REAR_KEEPER_GAP_WIDTH / 2.0
CRADLE_REAR_KEEPER_HEIGHT = 8.0
CRADLE_SIDE_GRIP_PRELOAD = 0.15
CRADLE_SIDE_GRIP_RADIUS = 2.0

# Revision F preserves the Revision E pan-servo stability base.  The 140 mm circular footprint is
# uniform in every pan direction and fits common 220 x 220 mm FDM beds.
BASE_DIAMETER = 140.0
BASE_THICKNESS = 6.0
BASE_BOTTOM_Z = -27.0
BASE_TOP_Z = BASE_BOTTOM_Z + BASE_THICKNESS
BASE_MOUNT_RADIUS = 50.0
BASE_SIDE_PORT_WIDTH = 14.0
BASE_SIDE_PORT_HEIGHT = 9.0
BASE_SIDE_GROOVE_WIDTH = 14.0
BASE_SIDE_GROOVE_DEPTH = 4.0
ARM_SIDE_PORT_LENGTH = 16.0
ARM_SIDE_PORT_HEIGHT = 10.0
ARM_SIDE_PORT_CENTER_Y = 10.5

# Verified printed fastener patterns.  The base holes match the 27.5 mm
# centre-to-centre ear-slot pattern measured from User Library-SG90.sldprt.
SG90_EAR_HOLE_SPACING = 27.5
SG90_EAR_PILOT_DIAMETER = 1.9
CAP_SCREW_CENTER_X = 17.0
CAP_SCREW_CENTER_Z_OFFSET = 8.5
CAP_PILOT_DIAMETER = 1.8
CAP_CLEARANCE_DIAMETER = 2.4
CAP_WIDTH = 42.0
CAP_HEIGHT = 26.0
CAP_OUTPUT_OPENING_LENGTH = 27.0
CAP_OUTPUT_OPENING_HEIGHT = 15.0
CAP_SCREW_HEAD_PROXY_DIAMETER = 4.0
HORN_CENTER_CLEARANCE_DIAMETER = 3.2
HORN_PILOT_DIAMETER = 2.0
HORN_PILOT_RADIUS = 7.0


def box_at(x1: float, x2: float, y1: float, y2: float, z1: float, z2: float) -> cq.Workplane:
    """Axis-aligned box from minima/maxima."""
    return (
        cq.Workplane("XY")
        .box(x2 - x1, y2 - y1, z2 - z1, centered=(True, True, False))
        .translate(((x1 + x2) / 2.0, (y1 + y2) / 2.0, z1))
    )


def cyl_y(radius: float, y1: float, y2: float, x: float, z: float) -> cq.Workplane:
    solid = cq.Solid.makeCylinder(
        radius,
        y2 - y1,
        cq.Vector(x, y1, z),
        cq.Vector(0, 1, 0),
    )
    return cq.Workplane(obj=solid)


def cyl_x(radius: float, x1: float, x2: float, y: float, z: float) -> cq.Workplane:
    solid = cq.Solid.makeCylinder(
        radius,
        x2 - x1,
        cq.Vector(x1, y, z),
        cq.Vector(1, 0, 0),
    )
    return cq.Workplane(obj=solid)


def cyl_z(radius: float, z1: float, z2: float, x: float = 0.0, y: float = 0.0) -> cq.Workplane:
    solid = cq.Solid.makeCylinder(
        radius,
        z2 - z1,
        cq.Vector(x, y, z1),
        cq.Vector(0, 0, 1),
    )
    return cq.Workplane(obj=solid)


def side_capsule_x(
    x1: float,
    x2: float,
    y_center: float,
    length: float,
    height: float,
    z_center: float,
) -> cq.Workplane:
    """Rounded cable window in a YZ side face, extruded along X."""
    radius = height / 2.0
    half_span = (length - height) / 2.0
    opening = box_at(
        x1,
        x2,
        y_center - half_span,
        y_center + half_span,
        z_center - radius,
        z_center + radius,
    )
    opening = opening.union(cyl_x(radius, x1, x2, y_center - half_span, z_center))
    opening = opening.union(cyl_x(radius, x1, x2, y_center + half_span, z_center))
    return opening.clean()


def arm_ribbon() -> cq.Workplane:
    """Curved 14 mm ribbon using exact analytic arcs, not segmented lines."""
    bottom_z = 14.0
    arc_cx = 30.0
    arc_cz = bottom_z + ARM_CENTER_SPACING / 2.0
    outer_r = ARM_CENTERLINE_RADIUS + ARM_WIDTH / 2.0
    inner_r = ARM_CENTERLINE_RADIUS - ARM_WIDTH / 2.0
    start_deg = math.degrees(math.atan2(bottom_z - arc_cz, -arc_cx))
    end_deg = start_deg - math.degrees(
        2.0 * math.asin(ARM_CENTER_SPACING / (2.0 * ARM_CENTERLINE_RADIUS))
    )

    mid_deg = (start_deg + end_deg) / 2.0

    def point(radius: float, angle_deg: float) -> tuple[float, float]:
        angle = math.radians(angle_deg)
        return (arc_cx + radius * math.cos(angle), arc_cz + radius * math.sin(angle))

    outer_start = point(outer_r, start_deg)
    outer_mid = point(outer_r, mid_deg)
    outer_end = point(outer_r, end_deg)
    inner_end = point(inner_r, end_deg)
    inner_mid = point(inner_r, mid_deg)
    inner_start = point(inner_r, start_deg)

    ribbon = (
        cq.Workplane("XZ")
        .moveTo(*outer_start)
        .threePointArc(outer_mid, outer_end)
        .lineTo(*inner_end)
        .threePointArc(inner_mid, inner_start)
        .close()
        .extrude(ARM_THICKNESS / 2.0, both=True)
    )
    bottom_boss = (
        cq.Workplane("XZ")
        .center(0, bottom_z)
        .circle(BOSS_DIAMETER / 2.0)
        .extrude(ARM_THICKNESS / 2.0, both=True)
    )
    top_boss = (
        cq.Workplane("XZ")
        .center(0, bottom_z + ARM_CENTER_SPACING)
        .circle(BOSS_DIAMETER / 2.0)
        .extrude(ARM_THICKNESS / 2.0, both=True)
    )
    smooth_arm = ribbon.union(bottom_boss).union(top_boss).clean()
    # Round the two broad-profile edges to remove the hard ridge and improve grip/appearance.
    try:
        smooth_arm = smooth_arm.edges("|Y").fillet(1.0)
    except Exception:
        # The exact arcs remain smooth even if a kernel build rejects a cosmetic fillet.
        pass
    return smooth_arm


def make_arm_and_holder() -> cq.Workplane:
    # Horizontal pan-horn pad: pan axis is Z; tilt axis is Y, exactly 90 degrees apart.
    # The pad starts at Z=4 mm, leaving room below it for the SG90 horn.
    pad = (
        cq.Workplane("XY")
        .circle(PAN_PAD_DIAMETER / 2.0)
        .extrude(PAN_PAD_THICKNESS)
        .translate((0, 0, 4.0))
    )
    pad = pad.edges("%CIRCLE").fillet(1.0)
    pad = pad.cut(cyl_z(HORN_CENTER_CLEARANCE_DIAMETER / 2.0, 3.5, 8.5))
    for x, y in (
        (HORN_PILOT_RADIUS, 0),
        (-HORN_PILOT_RADIUS, 0),
        (0, HORN_PILOT_RADIUS),
        (0, -HORN_PILOT_RADIUS),
    ):
        pad = pad.cut(cyl_z(HORN_PILOT_DIAMETER / 2.0, 3.5, 8.5, x, y))

    # Trim the vertical profile at the pan-pad interface; it no longer protrudes into the horn.
    upright = arm_ribbon().intersect(box_at(-100, 100, -100, 100, 7.5, 200.0))
    arm = pad.union(upright)

    # Compact sideways SG90 cage.  Servo slides in from +Y; output shaft points +Y.
    holder = box_at(-17.0, 17.0, 2.5, 6.0, 84.0, 104.0)  # back
    holder = holder.union(box_at(-15.5, 15.5, 2.5, 24.0, 84.0, 87.0))  # lower rail
    holder = holder.union(box_at(-15.5, -12.2, 2.5, 24.0, 84.0, 104.0))
    holder = holder.union(box_at(12.2, 15.5, 2.5, 24.0, 84.0, 104.0))
    holder = holder.union(box_at(-15.5, 15.5, 2.5, 24.0, 101.0, 104.0))

    # Openings for the standard SG90 mounting ears near the output end.
    holder = holder.cut(box_at(-17.0, -12.0, 18.0, 24.5, 87.0, 101.0))
    holder = holder.cut(box_at(12.0, 17.0, 18.0, 24.5, 87.0, 101.0))

    # Four cap pilot holes, axis Y.  They accept M2 self-tapping screws.
    for x in (-CAP_SCREW_CENTER_X, CAP_SCREW_CENTER_X):
        for z in (TILT_AXIS_Z - CAP_SCREW_CENTER_Z_OFFSET, TILT_AXIS_Z + CAP_SCREW_CENTER_Z_OFFSET):
            holder = holder.cut(cyl_y(CAP_PILOT_DIAMETER / 2.0, 1.5, 24.5, x, z))

    combined = arm.union(holder).clean()

    # Two rounded side-entry windows replace the incorrect rear-facing loop.
    # Either window passes a typical three-pin SG90 plug or USB-A plug while
    # leaving the central back wall intact.  The cable then runs down the
    # outside face of the curved arm with a loose service loop.
    left_port = side_capsule_x(
        -17.5,
        -11.5,
        ARM_SIDE_PORT_CENTER_Y,
        ARM_SIDE_PORT_LENGTH,
        ARM_SIDE_PORT_HEIGHT,
        TILT_AXIS_Z,
    )
    right_port = side_capsule_x(
        11.5,
        17.5,
        ARM_SIDE_PORT_CENTER_Y,
        ARM_SIDE_PORT_LENGTH,
        ARM_SIDE_PORT_HEIGHT,
        TILT_AXIS_Z,
    )
    combined = combined.cut(left_port).cut(right_port)

    # Re-cut every screw path after all unions.  This prevents the curved-arm
    # bosses from partially filling holes that were drilled in the pad or cage
    # before those solids were joined.
    combined = combined.cut(
        cyl_z(HORN_CENTER_CLEARANCE_DIAMETER / 2.0, 3.5, 8.5)
    )
    for x, y in (
        (HORN_PILOT_RADIUS, 0),
        (-HORN_PILOT_RADIUS, 0),
        (0, HORN_PILOT_RADIUS),
        (0, -HORN_PILOT_RADIUS),
    ):
        combined = combined.cut(
            cyl_z(HORN_PILOT_DIAMETER / 2.0, 3.5, 8.5, x, y)
        )
    for x in (-CAP_SCREW_CENTER_X, CAP_SCREW_CENTER_X):
        for z in (
            TILT_AXIS_Z - CAP_SCREW_CENTER_Z_OFFSET,
            TILT_AXIS_Z + CAP_SCREW_CENTER_Z_OFFSET,
        ):
            combined = combined.cut(
                cyl_y(CAP_PILOT_DIAMETER / 2.0, 1.5, 24.5, x, z)
            )
    # New outboard bosses carry the wider 34 mm cap screw pattern.
    for x in (-CAP_SCREW_CENTER_X, CAP_SCREW_CENTER_X):
        for z in (TILT_AXIS_Z - CAP_SCREW_CENTER_Z_OFFSET, TILT_AXIS_Z + CAP_SCREW_CENTER_Z_OFFSET):
            combined = combined.union(cyl_y(4.0, 14.0, 24.0, x, z))
            combined = combined.cut(cyl_y(0.9, 13.5, 24.5, x, z))
    # Ear slots must also trim the new bosses and make room for cap contact pads.
    for x1,x2 in ((-16.4,-12.0),(12.0,16.4)):
        combined = combined.cut(box_at(x1,x2,18.0,24.5,87.7,100.3))
    # Keep the side windows open through the enlarged holder.
    for x1, x2 in ((-22.0, -11.5), (11.5, 22.0)):
        combined = combined.cut(side_capsule_x(x1, x2, 10.5, 16.0, 10.0, TILT_AXIS_Z))
    # Stock shaft screw passes through the print and seats on the original horn.
    # Continue through the lower arm boss so a screwdriver can reach it.
    combined = combined.cut(cyl_z(3.2, 3.5, 150.0))
    # Only the two Y-offset horn screws are used; their heads are accessible.
    # Recessed seats leave 1.2 mm of printed material above the horn interface.
    for y in (-7.0, 7.0):
        combined = combined.cut(cyl_z(1.2, 3.5, 8.5, 0.0, y))
        combined = combined.cut(cyl_z(2.3, 5.2, 41.0, 0.0, y))
    # Zip-tie tabs carry the exterior USB/servo bundle without threading plugs.
    for z in (40.0, 65.0):
        tab = box_at(-30, -21, -5, 0, z-3, z+3)
        tab = tab.cut(box_at(-28, -24, -5.5, 0.5, z-1, z+1))
        combined = combined.union(tab)
    return combined.clean()


def make_servo_cap() -> cq.Workplane:
    cap = box_at(
        -CAP_WIDTH / 2.0,
        CAP_WIDTH / 2.0,
        24.0,
        27.0,
        TILT_AXIS_Z - CAP_HEIGHT / 2.0,
        TILT_AXIS_Z + CAP_HEIGHT / 2.0,
    )
    # 27 x 15 mm opening. Outboard screw bosses provide 2.3 mm hole-edge web.
    opening_radius = CAP_OUTPUT_OPENING_HEIGHT / 2.0
    opening_half_span = CAP_OUTPUT_OPENING_LENGTH / 2.0
    opening = box_at(
        -opening_half_span,
        opening_half_span,
        23.5,
        27.5,
        TILT_AXIS_Z - opening_radius,
        TILT_AXIS_Z + opening_radius,
    )
    cap = cap.cut(opening.clean())
    for x in (-CAP_SCREW_CENTER_X, CAP_SCREW_CENTER_X):
        for z in (TILT_AXIS_Z - CAP_SCREW_CENTER_Z_OFFSET, TILT_AXIS_Z + CAP_SCREW_CENTER_Z_OFFSET):
            cap = cap.cut(cyl_y(CAP_CLEARANCE_DIAMETER / 2.0, 23.5, 27.5, x, z))
    # Rear contact pads retain the mounting ears with 0.2 mm nominal axial play.
    for x1, x2 in ((-16.2, -13.5), (13.5, 16.2)):
        cap = cap.union(box_at(x1, x2, 21.4, 24.1, TILT_AXIS_Z-6.0, TILT_AXIS_Z+6.0))
    return cap.clean()


def make_camera_basket_original_coordinates() -> cq.Workplane:
    """One-piece basket that captures the C270 without screwing into it.

    The camera lowers into the open-front basket.  Corner shelves carry the
    body but leave the permanent monitor clip free in the large centre opening.
    Thin top and front corner lips prevent the webcam from lifting or moving
    forward.  The side walls flex slightly during insertion, and two shallow
    side-grip ribs provide a light retention preload. Two low barriers catch
    the rear lower corners so the webcam cannot slide backward. The 62 mm-wide
    center remains completely open for the attached lead before it bends toward
    either blue arm side window.
    """
    # Horn boss and stiff mounting web live below the webcam, keeping the lens
    # face completely open.
    mount_web = box_at(
        -CRADLE_OUTER_HALF_X,
        CRADLE_OUTER_HALF_X,
        4.0,
        CRADLE_FRONT_Y,
        0.0,
        CRADLE_BOTTOM_Z,
    )
    try:
        mount_web = mount_web.edges("|Y").fillet(2.0)
    except Exception:
        pass
    cradle = mount_web

    # Two corner shelves support the webcam body.  The 62 mm-wide centre gap
    # remains open for the fixed monitor clip, so no clip geometry is forced.
    shelf_inner_x = 31.0
    shelf_y_back = CRADLE_BACK_Y + CRADLE_WALL
    cradle = cradle.union(
        box_at(
            -CRADLE_OUTER_HALF_X,
            -shelf_inner_x,
            CRADLE_FRONT_Y,
            shelf_y_back,
            CRADLE_BOTTOM_Z - CRADLE_WALL,
            CRADLE_BOTTOM_Z,
        )
    )
    cradle = cradle.union(
        box_at(
            shelf_inner_x,
            CRADLE_OUTER_HALF_X,
            CRADLE_FRONT_Y,
            shelf_y_back,
            CRADLE_BOTTOM_Z - CRADLE_WALL,
            CRADLE_BOTTOM_Z,
        )
    )

    # Full-height side cheeks define the width.  There is deliberately no rear
    # wall, rail, or cross-member between them.
    cradle = cradle.union(
        box_at(
            -CRADLE_OUTER_HALF_X,
            -CRADLE_INNER_HALF_X,
            CRADLE_FRONT_Y,
            shelf_y_back,
            CRADLE_BOTTOM_Z - CRADLE_WALL,
            CRADLE_TOP_Z + CRADLE_WALL,
        )
    )
    cradle = cradle.union(
        box_at(
            CRADLE_INNER_HALF_X,
            CRADLE_OUTER_HALF_X,
            CRADLE_FRONT_Y,
            shelf_y_back,
            CRADLE_BOTTOM_Z - CRADLE_WALL,
            CRADLE_TOP_Z + CRADLE_WALL,
        )
    )
    # Small front and top corner returns turn the structure into a cradle, not
    # a flat blade.  They overlap only the outer body corners so the lens and
    # microphone area remain unobstructed.
    corner_inner_x = C270_WIDTH / 2.0 - 2.0
    for x0, x1 in (
        (-CRADLE_OUTER_HALF_X, -corner_inner_x),
        (corner_inner_x, CRADLE_OUTER_HALF_X),
    ):
        cradle = cradle.union(
            box_at(
                x0,
                x1,
                CRADLE_FRONT_Y - 2.0,
                CRADLE_FRONT_Y,
                CRADLE_BOTTOM_Z,
                CRADLE_TOP_Z + CRADLE_WALL,
            )
        )
        cradle = cradle.union(
            box_at(
                x0,
                x1,
                CRADLE_FRONT_Y,
                shelf_y_back,
                CRADLE_TOP_Z,
                CRADLE_TOP_Z + CRADLE_WALL,
            )
        )

    # Shallow opposed grip ribs sit on the side faces, not behind the webcam.
    # Their 0.15 mm nominal preload is intentionally small so the long cheeks
    # can flex during insertion.  The rear cable aperture stays unobstructed.
    grip_y = CRADLE_FRONT_Y + 0.70 * CRADLE_BODY_DEPTH
    grip_z = CRADLE_BOTTOM_Z + 0.52 * C270_HEIGHT
    cradle = cradle.union(
        cyl_x(
            CRADLE_SIDE_GRIP_RADIUS,
            -CRADLE_OUTER_HALF_X,
            -C270_WIDTH / 2.0 + CRADLE_SIDE_GRIP_PRELOAD,
            grip_y,
            grip_z,
        )
    )
    cradle = cradle.union(
        cyl_x(
            CRADLE_SIDE_GRIP_RADIUS,
            C270_WIDTH / 2.0 - CRADLE_SIDE_GRIP_PRELOAD,
            CRADLE_OUTER_HALF_X,
            grip_y,
            grip_z,
        )
    )

    # Split rear keepers catch only the webcam's lower outer corners. They start
    # behind the nominal body envelope, so they retain without squeezing it.
    # The 62 mm center stays empty for the fixed USB lead and strain relief.
    keeper_top_z = CRADLE_BOTTOM_Z + CRADLE_REAR_KEEPER_HEIGHT
    cradle = cradle.union(
        box_at(
            -CRADLE_OUTER_HALF_X,
            -CRADLE_REAR_KEEPER_INNER_X,
            CRADLE_BACK_Y,
            shelf_y_back,
            CRADLE_BOTTOM_Z,
            keeper_top_z,
        )
    )
    cradle = cradle.union(
        box_at(
            CRADLE_REAR_KEEPER_INNER_X,
            CRADLE_OUTER_HALF_X,
            CRADLE_BACK_Y,
            shelf_y_back,
            CRADLE_BOTTOM_Z,
            keeper_top_z,
        )
    )

    # Final clearance cut guarantees the full 62 mm central cable channel is
    # empty. It trims the shelves and top lips without removing the new corner
    # keepers.
    cradle = cradle.cut(
        box_at(
            -CRADLE_REAR_KEEPER_INNER_X,
            CRADLE_REAR_KEEPER_INNER_X,
            CRADLE_BACK_Y - 0.4,
            shelf_y_back + 0.5,
            CRADLE_BOTTOM_Z - CRADLE_WALL - 0.5,
            CRADLE_TOP_Z + CRADLE_WALL + 0.5,
        )
    )

    # Horn attachment pattern: the screws fasten the printed cradle to the
    # supplied SG90 horn.  They never enter or touch the webcam.
    return cradle.clean()


CAMERA_CENTER_Y = CRADLE_OUTER_HALF_X + 5.0
CAMERA_CENTER_Z_ORIGINAL = 6.0 + C270_HEIGHT / 2.0


def orient_camera(shape):
    """Point the lens along +X, perpendicular to the Y tilt shaft."""
    return shape.rotate((0,0,0), (0,0,1), 90).translate(
        (20.0, CAMERA_CENTER_Y, -CAMERA_CENTER_Z_ORIGINAL))


def make_camera_screwless_cradle():
    basket = orient_camera(make_camera_basket_original_coordinates())
    side_mount = box_at(-15, 15, 0, 3.0, -17, 17)
    side_mount = side_mount.union(box_at(-12, 12, 2.5, 8.0, -16, 16))
    cradle = basket.union(side_mount)
    # Open the stock center-screw path through both sidewalls; insert camera last.
    cradle = cradle.cut(cyl_y(3.2, -0.5, CAMERA_CENTER_Y+CRADLE_OUTER_HALF_X+1, 0, 0))
    # Two horn screws along local Z avoid the rear cable corridor.
    for z in (-7.0, 7.0):
        cradle = cradle.cut(cyl_y(1.2, -0.5, 8.5, 0, z))
        cradle = cradle.cut(cyl_y(2.3, 1.2, CAMERA_CENTER_Y+CRADLE_OUTER_HALF_X+1, 0, z))
    return cradle.clean()


def make_pan_servo_stability_base() -> cq.Workplane:
    """Wide, low base that supports and fastens the upright pan SG90.

    The SG90 reference body rests on the plate at Z=-21 mm.  Two side towers
    sit clear of the body and support its mounting ears.  Pilot holes accept
    common M2 servo screws, while the outer M4 holes can fasten the base to a
    board or accept rubber-foot hardware.  Strap slots provide a clone-tolerant
    fallback when ear-hole spacing differs.
    """
    base = (
        cq.Workplane("XY")
        .circle(BASE_DIAMETER / 2.0)
        .extrude(BASE_THICKNESS)
        .translate((0, 0, BASE_BOTTOM_Z))
    )
    try:
        base = base.edges("%CIRCLE").fillet(1.8)
    except Exception:
        pass

    # Upright supports and ledges for the two horizontal SG90 mounting ears.
    left_tower = box_at(-18.5, -12.2, -8.5, 8.5, BASE_TOP_Z, -8.4)
    right_tower = box_at(12.2, 18.5, -8.5, 8.5, BASE_TOP_Z, -8.4)
    left_ledge = box_at(-19.0, -11.8, -10.5, 10.5, -10.6, -8.4)
    right_ledge = box_at(11.8, 19.0, -10.5, 10.5, -10.6, -8.4)
    base = base.union(left_tower).union(right_tower).union(left_ledge).union(right_ledge)

    # Large side windows align with either possible SG90 cable-exit side.  The
    # connector can pass through before the servo ears are screwed down.
    port_y = BASE_SIDE_PORT_WIDTH / 2.0
    port_z1 = -20.0
    port_z2 = port_z1 + BASE_SIDE_PORT_HEIGHT
    base = base.cut(box_at(-19.5, -11.5, -port_y, port_y, port_z1, port_z2))
    base = base.cut(box_at(11.5, 19.5, -port_y, port_y, port_z1, port_z2))

    # Shallow left/right grooves continue each side window to the plate edge.
    # A 2 mm floor remains under the 4 mm-deep cable recess.
    groove_y = BASE_SIDE_GROOVE_WIDTH / 2.0
    groove_z1 = BASE_TOP_Z - BASE_SIDE_GROOVE_DEPTH
    base = base.cut(box_at(-71.0, -18.0, -groove_y, groove_y, groove_z1, BASE_TOP_Z + 0.5))
    base = base.cut(box_at(18.0, 71.0, -groove_y, groove_y, groove_z1, BASE_TOP_Z + 0.5))

    # M2 self-tapping pilot holes beneath the standard SG90 ear locations.
    for x in (-SG90_EAR_HOLE_SPACING / 2.0, SG90_EAR_HOLE_SPACING / 2.0):
        base = base.cut(cyl_z(SG90_EAR_PILOT_DIAMETER / 2.0, -15.0, -7.8, x, 0.0))

    # Two through-slots let a narrow zip tie retain unusually shaped SG90 clones.
    base = base.cut(box_at(-24.5, -21.5, -8.0, 8.0, BASE_BOTTOM_Z - 0.5, BASE_TOP_Z + 0.5))
    base = base.cut(box_at(21.5, 24.5, -8.0, 8.0, BASE_BOTTOM_Z - 0.5, BASE_TOP_Z + 0.5))

    # Four M4-class table/board mounting holes with shallow top counterbores.
    offset = BASE_MOUNT_RADIUS / math.sqrt(2.0)
    for x in (-offset, offset):
        for y in (-offset, offset):
            base = base.cut(cyl_z(2.2, BASE_BOTTOM_Z - 0.5, BASE_TOP_Z + 0.5, x, y))
            base = base.cut(cyl_z(4.2, -23.2, BASE_TOP_Z + 0.5, x, y))

    return base.clean()


def make_sg90_reference() -> tuple[cq.Workplane, cq.Workplane]:
    """A dimensionally representative SG90 body and horn for assembly visualization."""
    # Upright reference: output axis is +Z at the origin; body extends downward.
    body = box_at(-11.4, 11.4, -6.1, 6.1, -21.0, -4.0)
    body = body.union(box_at(-16.2, 16.2, -6.1, 6.1, -8.0, -5.8))
    body = body.union(cyl_z(6.0, -4.0, -0.7, 0.0, 0.0))
    body = body.union(cyl_z(4.0, -4.0, -0.7, 6.0, 0.0))
    body = body.union(cyl_z(2.35, -0.7, 2.0, 0.0, 0.0))
    for x in (-SG90_EAR_HOLE_SPACING / 2.0, SG90_EAR_HOLE_SPACING / 2.0):
        body = body.cut(cyl_z(1.1, -8.5, -5.3, x, 0.0))
    try:
        body = body.edges("|Z").fillet(0.8)
    except Exception:
        pass

    # Cross horn is a separate reference solid so it can be colored white.
    horn = cyl_z(4.8, 2.0, 3.8)
    horn = horn.union(box_at(-16.0, 16.0, -2.0, 2.0, 2.4, 3.8))
    horn = horn.union(box_at(-2.0, 2.0, -11.0, 11.0, 2.4, 3.8))
    horn = horn.cut(cyl_z(1.0, 1.8, 4.0))
    return body.clean(), horn.clean()


def make_c270_reference_original_coordinates() -> tuple[cq.Workplane, cq.Workplane]:
    """A simplified C270 body and fixed-clip envelope for assembly visualization."""
    # Local tilt axis is Y through (0,0,0). The clip blade top is Z=6 mm.
    webcam = box_at(-C270_WIDTH / 2, C270_WIDTH / 2, 9.0, 31.0, 6.0, 6.0 + C270_HEIGHT)
    try:
        webcam = webcam.edges("|Y").fillet(7.0)
    except Exception:
        pass
    lens = cyl_y(7.0, 5.0, 9.2, -10.0, 6.0 + C270_HEIGHT * 0.58)
    lens = lens.union(cyl_y(3.8, 3.8, 5.2, -10.0, 6.0 + C270_HEIGHT * 0.58))
    webcam = webcam.union(lens).clean()

    # The clip is deliberately simplified but uses the complete supplied depth envelope.
    clip = box_at(-30.0, 30.0, 18.0, 18.0 + C270_DEPTH_WITH_CLIP - 22.0, -4.0, 8.0)
    try:
        clip = clip.edges("|Y").fillet(3.0)
    except Exception:
        pass
    return webcam, clip.clean()


def make_c270_reference():
    body, clip = make_c270_reference_original_coordinates()
    return orient_camera(body), orient_camera(clip)


def positioned_reference_models() -> dict[str, cq.Workplane]:
    servo_body, servo_horn = make_sg90_reference()
    pan_body = servo_body
    pan_horn = servo_horn
    tilt_body = servo_body.rotate((0, 0, 0), (1, 0, 0), -90.0).translate((0, 27.0, TILT_AXIS_Z))
    tilt_horn = servo_horn.rotate((0, 0, 0), (1, 0, 0), -90.0).translate((0, 27.0, TILT_AXIS_Z))
    webcam, clip = make_c270_reference()
    webcam = webcam.translate((0, 31.0, TILT_AXIS_Z))
    clip = clip.translate((0, 31.0, TILT_AXIS_Z))
    return {
        "SG90_Pan_Body": pan_body,
        "SG90_Pan_Horn": pan_horn,
        "SG90_Tilt_Body": tilt_body,
        "SG90_Tilt_Horn": tilt_horn,
        "C270_Webcam_Body": webcam,
        "C270_Fixed_Clip": clip,
    }


def export_part(shape: cq.Workplane, stem: str) -> None:
    exporters.export(shape, str(PRINT_DIR / f"{stem}.step"))
    exporters.export(
        shape,
        str(PRINT_DIR / f"{stem}.stl"),
        tolerance=0.03,
        angularTolerance=0.04,
    )


def export_reference_models() -> None:
    servo_body, servo_horn = make_sg90_reference()
    servo_assembly = cq.Assembly(name="SG90_Visual_Reference")
    servo_assembly.add(servo_body, name="SG90_Body", color=cq.Color(0.12, 0.35, 0.78))
    servo_assembly.add(servo_horn, name="SG90_Cross_Horn", color=cq.Color(0.95, 0.95, 0.92))
    servo_assembly.save(str(REF_DIR / "SG90_Visual_Reference.step"))

    webcam, clip = make_c270_reference()
    webcam_assembly = cq.Assembly(name="Logitech_C270_Visual_Reference")
    webcam_assembly.add(webcam, name="C270_Body", color=cq.Color(0.08, 0.09, 0.11))
    webcam_assembly.add(clip, name="C270_Fixed_Clip", color=cq.Color(0.18, 0.19, 0.22))
    webcam_assembly.save(str(REF_DIR / "Logitech_C270_Visual_Reference.step"))


def export_positioned_assemblies(
    arm: cq.Workplane, cap: cq.Workplane, camera: cq.Workplane, base: cq.Workplane
) -> None:
    printable = cq.Assembly(name="C270_PanTilt_Printable_Assembly")
    printable.add(base, name="PanServo_StabilityBase", color=cq.Color(0.28, 0.31, 0.36))
    printable.add(arm, name="CurvedArm_SG90_TiltHolder", color=cq.Color(0.18, 0.55, 0.92))
    printable.add(cap, name="SG90_RetainingCap", color=cq.Color(0.95, 0.62, 0.18))
    printable.add(
        camera,
        name="Logitech_C270_ScrewlessCradle",
        loc=cq.Location(cq.Vector(0, 31.0, TILT_AXIS_Z)),
        color=cq.Color(0.18, 0.82, 0.58),
    )
    printable.save(str(ASM_DIR / "C270_PanTilt_Printable_Assembly.step"))

    assembly = cq.Assembly(name="C270_PanTilt_Full_Assembly_With_Servos")
    assembly.add(base, name="PanServo_StabilityBase", color=cq.Color(0.28, 0.31, 0.36))
    assembly.add(arm, name="CurvedArm_SG90_TiltHolder", color=cq.Color(0.18, 0.55, 0.92))
    assembly.add(cap, name="SG90_RetainingCap", color=cq.Color(0.95, 0.62, 0.18))
    assembly.add(
        camera,
        name="Logitech_C270_ScrewlessCradle",
        loc=cq.Location(cq.Vector(0, 31.0, TILT_AXIS_Z)),
        color=cq.Color(0.18, 0.82, 0.58),
    )
    refs = positioned_reference_models()
    for name, shape in refs.items():
        if "Horn" in name:
            color = cq.Color(0.95, 0.95, 0.92)
        elif "SG90" in name:
            color = cq.Color(0.12, 0.35, 0.78)
        elif "Webcam" in name:
            color = cq.Color(0.07, 0.08, 0.10)
        else:
            color = cq.Color(0.18, 0.19, 0.22)
        assembly.add(shape, name=name, color=color)
    assembly.save(str(ASM_DIR / "C270_PanTilt_Full_Assembly_With_Servos.step"))




def make_previews(
    arm: cq.Workplane,
    cap: cq.Workplane,
    camera: cq.Workplane,
    base: cq.Workplane,
) -> None:
    # VTK is bundled with the CadQuery kernel.  Use off-screen rendering so the
    # user gets a quick visual without changing their live SolidWorks session.
    from vtkmodules.vtkIOGeometry import vtkSTLReader
    from vtkmodules.vtkRenderingCore import (
        vtkActor,
        vtkPolyDataMapper,
        vtkRenderer,
        vtkRenderWindow,
    )
    from vtkmodules.vtkRenderingOpenGL2 import vtkOpenGLRenderer, vtkOpenGLRenderWindow  # noqa: F401
    from vtkmodules.vtkIOImage import vtkPNGWriter
    from vtkmodules.vtkRenderingCore import vtkWindowToImageFilter

    preview_dir = WORK / "preview_revM"
    preview_dir.mkdir(parents=True, exist_ok=True)
    def render_scene(filename, scene, camera_position, focal_point):
        renderer = vtkRenderer()
        renderer.SetBackground(0.965, 0.975, 0.99)
        for index, (name, shape, color, opacity) in enumerate(scene):
            path = preview_dir / f"{filename}_{index}_{name}.stl"
            exporters.export(shape, str(path), tolerance=0.04, angularTolerance=0.05)
            reader = vtkSTLReader()
            reader.SetFileName(str(path))
            mapper = vtkPolyDataMapper()
            mapper.SetInputConnection(reader.GetOutputPort())
            actor = vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*color)
            actor.GetProperty().SetOpacity(opacity)
            actor.GetProperty().SetSpecular(0.22)
            actor.GetProperty().SetSpecularPower(20)
            renderer.AddActor(actor)

        window = vtkRenderWindow()
        window.SetOffScreenRendering(1)
        window.SetMultiSamples(8)
        window.SetSize(1400, 1000)
        window.AddRenderer(renderer)
        vtk_camera = renderer.GetActiveCamera()
        vtk_camera.SetPosition(*camera_position)
        vtk_camera.SetFocalPoint(*focal_point)
        vtk_camera.SetViewUp(0, 0, 1)
        renderer.ResetCameraClippingRange()
        window.Render()
        capture = vtkWindowToImageFilter()
        capture.SetInput(window)
        capture.ReadFrontBufferOff()
        capture.Update()
        writer = vtkPNGWriter()
        writer.SetFileName(str(RENDER_DIR / filename))
        writer.SetInputConnection(capture.GetOutputPort())
        writer.Write()

    blue = (0.12, 0.45, 0.86)
    orange = (0.96, 0.55, 0.12)
    green = (0.10, 0.72, 0.49)
    servo_blue = (0.08, 0.24, 0.62)
    white = (0.94, 0.94, 0.90)
    black = (0.06, 0.07, 0.09)
    dark_gray = (0.18, 0.19, 0.22)
    base_gray = (0.26, 0.29, 0.34)
    refs = positioned_reference_models()
    neutral_scene = [
        ("base", base, base_gray, 1.0),
        ("arm", arm, blue, 1.0),
        ("cap", cap, orange, 1.0),
        ("cradle", camera.translate((0, 31.0, TILT_AXIS_Z)), green, 1.0),
        ("pan_servo", refs["SG90_Pan_Body"], servo_blue, 1.0),
        ("pan_horn", refs["SG90_Pan_Horn"], white, 1.0),
        ("tilt_servo", refs["SG90_Tilt_Body"], servo_blue, 1.0),
        ("tilt_horn", refs["SG90_Tilt_Horn"], white, 1.0),
        ("webcam", refs["C270_Webcam_Body"], black, 1.0),
        ("webcam_clip", refs["C270_Fixed_Clip"], dark_gray, 1.0),
    ]
    render_scene(
        "01_Full_Assembly_With_Servos.png",
        neutral_scene,
        (230, -330, 190),
        (0, 14, 42),
    )

    exploded_scene = [
        ("base", base.translate((-65, 35, -12)), base_gray, 1.0),
        ("arm", arm.translate((-50, -28, 18)), blue, 1.0),
        ("cap", cap.translate((25, -10, -55)), orange, 1.0),
        ("cradle", camera.translate((88, 0, 66)), green, 1.0),
    ]
    render_scene(
        "02_Exploded_Printable_Parts.png",
        exploded_scene,
        (265, -365, 210),
        (0, 0, 38),
    )

    webcam_local, clip_local = make_c270_reference()
    moving_local = cq.Compound.makeCompound([camera.val(), webcam_local.val(), clip_local.val()])
    tilt_scene = [
        ("base", base, base_gray, 1.0),
        ("arm", arm, blue, 1.0),
        ("cap", cap, orange, 1.0),
        ("pan_servo", refs["SG90_Pan_Body"], servo_blue, 1.0),
        ("tilt_servo", refs["SG90_Tilt_Body"], servo_blue, 1.0),
    ]
    for label, angle, color, opacity in (
        ("minus45", -45.0, (0.22, 0.67, 0.96), 0.35),
        ("neutral", 0.0, green, 0.82),
        ("plus45", 45.0, (0.63, 0.32, 0.91), 0.35),
    ):
        moved = cq.Workplane(obj=moving_local).rotate((0, 0, 0), (0, 1, 0), angle).translate(
            (0, 31.0, TILT_AXIS_Z)
        )
        tilt_scene.append((label, moved, color, opacity))
    render_scene(
        "03_Tilt_Range_Minus45_to_Plus45.png",
        tilt_scene,
        (235, -345, 175),
        (0, 20, 55),
    )

    base_scene = [
        ("base", base, base_gray, 1.0),
        ("pan_servo", refs["SG90_Pan_Body"], servo_blue, 1.0),
        ("pan_horn", refs["SG90_Pan_Horn"], white, 1.0),
    ]
    render_scene(
        "04_Pan_Servo_Stability_Base.png",
        base_scene,
        (145, -205, 92),
        (0, 0, -9),
    )

    holder_scene = [
        ("arm", arm, blue, 1.0),
        ("tilt_servo", refs["SG90_Tilt_Body"], servo_blue, 0.25),
    ]
    render_scene(
        "05_Arm_Side_Cable_Exit.png",
        holder_scene,
        (180, 120, 112),
        (0, 10, 93),
    )

    webcam_local, clip_local = make_c270_reference()
    rear_cable_proxy = orient_camera(cyl_y(
        3.0,
        CRADLE_BACK_Y - 2.0,
        CRADLE_BACK_Y + 42.0,
        0.0,
        CRADLE_BOTTOM_Z + 0.52 * C270_HEIGHT,
    ))
    cradle_scene = [
        ("cradle", camera, green, 1.0),
        ("webcam", webcam_local, black, 0.82),
        ("fixed_clip", clip_local, dark_gray, 0.45),
        ("rear_usb_cable", rear_cable_proxy, orange, 1.0),
    ]
    render_scene(
        "06_Screwless_C270_Cradle.png",
        cradle_scene,
        (160, 220, 95),
        (0, 45, 0),
    )

    cap_scene = [
        ("cap", cap, orange, 1.0),
        ("tilt_servo", refs["SG90_Tilt_Body"], servo_blue, 0.35),
    ]
    render_scene(
        "07_SG90_RetainingCap_Rectangular_Clearance.png",
        cap_scene,
        (85, 145, 105),
        (0, 25, TILT_AXIS_Z),
    )


def main() -> None:
    from build_verify_revM import main as build_and_verify
    build_and_verify()


if __name__ == "__main__":
    main()
