# C270 robotic arm Revision M

Revision M enlarges the cap opening to **27 × 15 mm**, provides direct access to the original servo shaft screws, and turns the webcam mount so the upper servo aims the camera up and down. The arm is now 10 mm thick to reduce sideways flex.

**Print three updated parts: the arm, cap, and webcam cradle. Reuse the Revision L base.** The base STL in this package is unchanged. Revision M cap holes use a new **34 × 17 mm** pattern and require the matching Revision M arm.

| File in Printable_Parts | Quantity | Action |
| --- | --- | --- |
| CurvedArm_SG90_TiltHolder.stl | 1 | Print the new thicker arm with screw access and cable tie tabs |
| SG90_RetainingCap.stl | 1 | Print the 27 × 15 mm opening and matching screw pattern |
| Logitech_C270_ScrewlessCradle.stl | 1 | Print the new side-mounted cradle for actual camera tilt |
| PanServo_StabilityBase.stl | 1 total | Reuse your existing Revision L base |

Print individual STLs. The STEP assemblies show placement and can be imported into SolidWorks; they are not files to print as a single object. The servos and camera in the full assembly are visualization references.

## Stability requirement

**Secure the gray base to a rigid work surface using its four M4 mounting holes.** The analysis does not support claiming that the lightweight base will resist cable pull while free-standing. The revised arm is stiffer, but printed-part flex, servo backlash and shaft-bearing compliance still require a physical check.

## Read before assembly

- `Documentation/Assembly_and_Printing_Guide.md` explains the screws, assembly order and what to print.
- `Documentation/Wiring_and_Motion.md` explains both USB connections, servo power, cable slack and travel limits.
- `Documentation/Engineering_Check.md` gives the assumptions, torque calculation and tipping calculation.
- `Documentation/CAD_and_Engineering_Results.json` contains the computed measurements and individual check results.
- `Software/Camera_RevM.cpp` and `Camera_RevM.exe` provide a separate tracker build with narrower commands and slower outgoing motion. Read the wiring guide before using it; it has not been run against hardware.

The previous Word guides remain with their older revisions. This revision's guides supersede their cap, cradle, screw-access, axis and stability instructions.

## Main dimensions

- Rectangular cap opening: 27 × 15 mm; this adds 5 mm to the former 22 mm side.
- Cap outer face: 42 × 26 mm; face thickness: 3 mm. Ear retaining pads extend 2.6 mm behind that face.
- Cap holes: 2.4 mm clearance; centers: 34 × 17 mm. Holder pilots: 1.8 mm.
- Original shaft-screw access: 6.4 mm diameter in the arm and cradle.
- Printed horn-screw seats: 1.2 mm material thickness after recessing.
- Arm ribbon: 14 mm wide, 10 mm thick; nominal 80 mm end-center spacing.
- Tilt: rotation about the upper servo's Y shaft; modeled movement −45° to +45°.
- Pan: rotation about vertical Z. These are two independent aiming rotations.

The screw-access changes address the blocked shaft-screw path found in the model. Actual screw head size, thread and under-head length have not been measured from your hardware. Compare them with the fastener table before tightening.

## Rebuilding the CAD

The `Source` folder contains the generator and build/verification script. Install the dependencies in an isolated Python environment, then run `python build_verify_revM.py` from that folder. The script regenerates the CAD, previews and numerical results in this revision folder; the written guides are maintained separately. Generated STEP files contain solids rather than a native SolidWorks feature history.
