# Revision M wiring and motion

Keep the Arduino on the stationary work surface next to the anchored base. No Arduino model or mounting-hole pattern has been confirmed, so this revision does not add a board mount. Both servos move the mechanism; neither USB connection should carry its mechanical load.

## Electrical connections

| Connection | Destination |
| --- | --- |
| Webcam USB cable | Computer USB port |
| Arduino USB cable | Computer USB port for serial commands and Arduino power |
| Both servo red wires | Positive terminal of a regulated external servo supply matched to the actual servos |
| Both servo brown/black wires | External supply ground |
| Arduino GND | The same external supply ground |
| Pan servo signal wire | The Arduino output pin named for PAN in your firmware |
| Tilt servo signal wire | The Arduino output pin named for TILT in your firmware |

Verify wire functions on the actual servo. Brown/black ground, red positive and orange/yellow signal are common SG90 colors. Do not run the servo current through an Arduino I/O pin. With the Arduino powered by USB, keep the external servo positive rail separate from the Arduino 5 V pin unless a specific power-sharing design is provided.

A regulated 5 V supply is a candidate only if the labels/specifications of your actual servos permit it. Size the supply for both motors' simultaneous current, including startup/stall; exact clone current has not been measured. TowerPro's official SG90 page lists 4.8 V operation and its manufacturer response permits 4.8–6 V, but this does not identify your specific servo clone.

Arduino advises using an external servo supply when board power is insufficient and joining the Arduino and supply grounds. See [Arduino servo troubleshooting](https://support.arduino.cc/hc/en-us/articles/360017053760-Troubleshoot-servo-motors) and [TowerPro SG90 specifications](https://towerpro.com.tw/product/sg90-7/).

## Physical cable routing

1. The pan servo lead exits from its SIDE and lies in the matching base groove. Keep it below the rotating horn and arm pad.
2. The tilt servo lead exits a SIDE window in the upper holder. It moves with pan but does not need to rotate with the camera cradle.
3. The webcam cable exits the open BACK of the cradle. Leave the first section free of the rear retention corners and the clip. Its six-millimetre-diameter, 42 mm straight exit envelope was checked in CAD; the actual molded strain relief and the rest of the cable still need a physical check.
4. Make a generous U-shaped service loop between the camera and the arm so tilt can move in both directions without pulling the connector. Then tie the external bundle loosely to the two new arm tabs. Their 4 × 2 mm slots accept narrow zip ties; they are tie slots, not holes for USB plugs.
5. Leave another loop between the rotating arm and the stationary base/Arduino. It must accommodate the full permitted pan movement. Anchor the stationary cable bundle to the work surface close to the base so pulling a computer cable does not pull the camera directly.
6. Move to all four combinations of minimum/maximum pan and tilt. The loops must remain loose, clear of horns and screw heads, and above sharp edges. The flexible loops are not fully represented by the rigid CAD cable proxy.

Do not thread a large molded USB connector through the servo cage, wind wiring around either shaft, or allow unlimited pan. Match the cable's bend radius to its actual construction and leave additional slack rather than forcing a tight bend.

## What the two axes now do

At neutral, the camera looks along CAD +X. Pan rotates around vertical Z, aiming left/right. Tilt rotates around the upper servo's Y shaft, which runs across the camera's width, aiming up/down. This gives two independent aiming angles; it is not a planar XY translation mechanism.

The older cradle pointed the lens parallel to the Y shaft, which would roll the image. The Revision M cradle corrects that orientation.

With pan angle p and tilt angle t measured from neutral, the camera viewing direction is

`(cos(p) cos(t), sin(p) cos(t), -sin(t))`.

The sign of a real servo's movement depends on horn placement and firmware. Check it at low travel before enabling tracking.

## Controller limits and first test

The existing `Camera.cpp` sends `pan,tilt` followed by a newline at 115200 baud. It currently clamps both requested angles to 0–180. That is broader than the travel checked for this mechanism.

The separate `Software/Camera_RevM.cpp` and compiled `Camera_RevM.exe` apply a starting logical range of **45–135**, with logical neutral at 90, and limit outgoing movement to one degree per axis at intervals of at least 33 ms. Apply limits in the Arduino firmware too. These command numbers do not prove the physical angle: calibrate the actual servo endpoints so the measured camera tilt stays within ±45° and neither servo reaches its stop. Begin with a much smaller range around neutral while checking the wiring.

No firmware was found in the supplied project folder, and the Arduino signal pins have not been confirmed. Original `Camera.cpp` and `Camera.exe` remain unchanged; the new executable has been compiled but not run against the hardware. Launch it with `Camera Tracker C++` as the working directory so it can find the existing model files, using the same OpenCV DLL environment as the original tracker. It still uses the existing COM5 port and 115200 baud settings. Confirm these match the connected Arduino. This guide does not assert that the existing firmware already enforces the limits.

Keep the servos powered to hold a position during use. Power loss, gear backlash or a slipping horn can still allow movement; this design is not a brake or a self-leveling mechanism.
