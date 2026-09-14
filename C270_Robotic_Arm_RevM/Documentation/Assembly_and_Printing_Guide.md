# Revision M assembly and printing guide

Assemble with the camera removed and the servos unpowered except when centering them. The camera inserts last so it cannot obstruct screwdriver access across the cradle.

## Screws and their jobs

| Connection | Fastener and quantity | How it reaches |
| --- | --- | --- |
| Each servo output shaft to its original plastic horn | The original shaft screw supplied with that servo; 1 each | Its head passes through the 6.4 mm access opening and seats directly on the horn. Printed plastic adds no thickness under this screw head. |
| Printed arm to pan horn | Two approximately 2 mm diameter self-tapping horn screws, nominally 4 mm under the head | Recesses leave 1.2 mm of print, followed by the horn. Use the two holes at Y = ±7 mm. |
| Printed cradle to tilt horn | Two approximately 2 mm diameter self-tapping horn screws, nominally 4 mm under the head | Recesses leave 1.2 mm of print. Use the two holes at Z = ±7 mm, with camera absent. |
| Orange cap to blue holder | Four M2-class self-tapping screws, 6 mm under the head | 3 mm of cap plus 3 mm into the printed 1.8 mm pilot. Pattern is 34 × 17 mm. |
| Pan servo ears to base | Two approximately 2 mm diameter self-tapping servo mounting screws | Original ear spacing is 27.5 mm. Select length from the actual ear thickness plus about 3 mm engagement into the printed base pilots. |
| Base to rigid work surface | Four M4 bolts, washers and nuts appropriate to the work surface | Existing 4.4 mm through-holes; choose length from the actual surface thickness, base thickness, washers and nut. |

The small screw into the servo shaft is not interchangeable with the screws that attach the horn or servo ears. Do not use nails, force an M2 machine screw into an unknown shaft thread, or fit an arbitrarily longer shaft screw: it can bottom out inside the servo.

The model provides clearance for a shaft-screw head up to approximately 6 mm and a driver tip smaller than the 6.4 mm opening. Measure the actual head before use. The 4 mm horn screw is a starting selection for the modeled 1.4 mm horn blade, not a claim that every SG90 horn has that thickness or hole pattern. A 4 mm screw through the 1.2 mm print and modeled 0.2 mm gap leaves 2.6 mm reach into and through the horn. Check the back of the actual horn for tip clearance before fitting it to the servo. The two horn hole centers must match the actual horn's holes at ±7 mm; mark or adapt the plastic horn off the servo if they do not match.

## Assembly order

1. Reuse the base and check that its side windows and grooves are clear. Fasten it to a rigid work surface using the four M4 holes. Their centers are at X/Y = ±35.355 mm: a 70.711 mm square. Place washers beneath the mounting screw heads. The modeled counterbore is 8.4 mm diameter and 2.2 mm deep, so select washers that fit it or seat on the surrounding flat surface.
2. Seat the pan servo on the base with its shaft upward. Pass its wire through the matching SIDE window, lay it in that side's groove, and fasten the servo ears. Screw length should give about 3 mm of engagement after passing the actual ear; check that the screw does not bottom in the pilot.
3. With the horn detached from the servo, fasten the printed arm to the pan horn using the two recessed mounting holes. Do this before installing the tilt servo. Use a short driver/bit for the pad recesses. The center access opening remains empty at this stage.
4. Center the pan servo electrically, power it off, then seat the horn/arm assembly on its spline. Insert the servo's original center screw through the arm's enlarged access channel. The screw head must seat on the original horn below the printed arm. Tighten gently.
5. Route the tilt servo's own cable through either enlarged SIDE opening in the upper holder and slide the servo into its cage. The shaft points along Y. Ensure the wire is clear of the ear slots and cap contact pads.
6. Fit the Revision M cap over the output side and install four 6 mm M2-class self-tapping screws into the matching Revision M holder. The 27 × 15 mm opening surrounds the raised servo portion; two rear pads retain its ears. Do not use the old 27.7 × 15 mm cap pattern. Check that the servo seats and is held without crushing its case.
7. With the webcam absent, attach the corrected cradle to the original tilt horn using its two recessed holes. Center the tilt servo electrically, power it off, and put the horn/cradle on the spline with the camera position level.
8. Insert the tilt servo's original shaft screw through the access holes across the empty cradle. It seats on the horn, below the print. Check that both horn screws and the shaft screw are secure and that their tips do not contact the servo case.
9. Gently install the webcam. The shelves carry the body; side cheeks and corner returns retain it. Its fixed clip hangs through the open underside. The split low rear barriers help prevent backward movement while leaving the central rear opening clear.
10. Route the webcam USB lead straight out of the back, then form a loose loop toward the arm. Keep the full USB plug outside the holder; use the new tie tabs for the external cable bundle. Follow the wiring guide before applying power.
11. Support the webcam by hand during the first powered test. Start near neutral, move one axis at a time, and gradually check both directions. Stop for wire tension, sag, case movement, rubbing, buzzing, or unexpected direction. Measure the real tilt angle before allowing the full ±45° CAD travel.

The original shaft screw holds the horn to the spline. The separate horn screws keep the printed part from rotating on the horn. Both connections are necessary.

## Printing

Print one new arm, one new cap and one new cradle. The base is unchanged from Revision L. Use millimetres and 100% model scale.

| Part | Orientation to discuss with the print operator | Suggested starting settings |
| --- | --- | --- |
| Arm and holder | Broad curved side down, with the curved ribbon in the bed plane | 0.20 mm layers, 5–6 walls, about 50% infill; supports only where the holder/pad creates unsupported starts |
| Retaining cap | Flat outer 42 × 26 mm face down; rear ear pads face upward | 0.20 mm layers, 4 walls or solid infill; no supports in the rectangular opening or screw holes |
| Corrected webcam cradle | Broad servo mounting face down; inspect the opposite cheek and corner returns in the slicer | 0.20 mm layers, 4 walls, 35–45% infill; brim as needed and removable supports under unsupported starts/overhangs |
| Existing base, if a replacement is needed | Circular underside down | 0.20 mm layers, 4 walls, 35–45% infill; keep both side cable ports and grooves clear |

The print operator should decide final supports and settings from the slicer preview. The cradle has overhangs and may require supports inside it; this has not been sliced on your printer. Remove supports fully before inserting the camera or screws. PLA/PLA+ favors stiffness for indoor use; PETG is another option for the flexible cradle, but either material needs a real fit and load test. Do not force the camera against a rigid or poorly printed lip.

The calculated mass bounds are deliberately broad. Weigh the finished parts: nominal infill percentage alone does not give an accurate mass or stiffness.

## Before another full print run

Check the cap opening against the actual servo's raised feature, and check the screw head against the access opening. Print the small cap first if the hardware dimensions are uncertain. The cap cannot be fastened to the old arm because its hole pattern has changed. Do not scale the whole model to fix a local fit problem; that changes every screw position.
