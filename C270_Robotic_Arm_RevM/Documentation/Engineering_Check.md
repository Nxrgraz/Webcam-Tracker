# Revision M engineering check

The geometry supports two independent camera aiming rotations and provides access to the shaft screws. The tipping calculation requires the base to be anchored. These checks establish geometric clearance and stated load estimates; they do not establish the strength of an unknown print or servo clone.

## Geometry and movement

The camera looks along +X at neutral. The pan shaft is vertical Z and the tilt shaft is Y. For pan p and tilt t, the viewing direction is `(cos(p)cos(t), sin(p)cos(t), -sin(t))`. At tilt −45°, 0° and +45°, the vertical component is respectively +0.7071, 0 and −0.7071. The upper joint therefore changes camera elevation rather than rolling the image.

The CAD checks sample tilt every 5° from −45° to +45°, checking the cradle, camera, clip, rotating horn and a straight rear cable proxy against the fixed parts. Those checks pass. The minimum sampled camera/cradle-to-fixed clearance is about 2.17 mm. Rotation about Y preserves every point's Y coordinate, and the moving camera/cradle assembly has at least 2 mm of axial separation from the upper fixed parts. A separate radial bounding check keeps it above the base throughout the motion, providing clearance bounds between the sampled angles as well. The flexible cable loops and actual molded connectors are not represented by those bounds.

The cap opening is 27 × 15 mm. Moving its screws to 34 × 17 mm centers retains at least 2.3 mm of material between the opening and the hole edges using a conservative horizontal measurement. A 6 mm under-head cap screw crosses 3 mm of cap and engages 3 mm of holder pilot. The modeled pilot surroundings contain material over that engagement length.

The arm and cradle have 6.4 mm center access openings. Original shaft screws seat on the original servo horns, so printed material does not consume their reach. Separate recessed horn screw seats leave 1.2 mm of print. Actual screw threads, heads, lengths and horn hole positions must be checked against the hardware.

## Tilt holding torque

For a mass m at perpendicular radius r from the tilt shaft, the maximum gravitational torque is `m g r`. In the customary kg·cm servo unit, use mass in kilograms multiplied by radius in centimetres.

The camera plus fixed clip is assigned the supplied 75 g total mass. Rather than guessing its exact center of mass, the calculation allows that mass center anywhere inside the camera/clip CAD envelope. Its maximum radial distance from Y is 49.92 mm. Printed cradle mass uses its CAD volume and a maximum effective density of 1.27 g/cm³.

| Quantity | Result |
| --- | --- |
| Camera and cradle gravity bound | 0.381 kg·cm |
| Gravity allowance for motion, chosen factor | 1.5 × |
| Assumed residual cable drag | 0.20 N at a 60 mm radius |
| Cable torque | 0.122 kg·cm |
| Estimated combined design demand | 1.5 × 0.381 + 0.122 = **0.694 kg·cm** |
| TowerPro SG90 published stall torque at 4.8 V | 1.8 kg·cm |
| Published stall torque divided by estimated demand | 2.59 |

The 0.20 N cable drag and 1.5 motion multiplier are design assumptions, not measurements or a simulation of the actual acceleration. A snag or fast reversal can exceed them. The published stall rating is not a continuous operating rating or a guarantee of stiffness. Actual servo type, voltage, temperature, wear and gear backlash affect holding performance. Check the powered assembly for sag and heating; do not describe this estimate as a proof that the camera can never droop.

The original [TowerPro SG90 specification](https://towerpro.com.tw/product/sg90-7/) is the source for the 1.8 kg·cm stall figure. It is a reference for a genuine TowerPro unit, not identification of the supplied hardware.

## Base tipping

For component masses m_i and positions r_i, the combined center of mass is `r_COM = sum(m_i r_i) / sum(m_i)`. On a circular base, a gravity-only stability margin is `R - horizontal_distance(r_COM, base_center)`.

The printed-part analysis samples effective density from 0.434 to 1.27 g/cm³ independently for each part. This deliberately broad range spans a light print through a solid plastic part; it is not a conversion from the slicer's infill percentage. The camera center of mass is allowed anywhere in its full envelope. The servos are assigned 9 g each and the horns 2 g each. Component centers come from CAD, with each moving part transformed through the tilt range.

To account for the rounded bottom edge, usable base support radius is taken as 68 mm rather than the nominal 70 mm. The worst sampled combination gives:

| Quantity | Conservative sampled result |
| --- | --- |
| Total moving and fixed assembly mass | 154.20 g |
| Horizontal center-of-mass radius | 63.78 mm |
| Remaining edge margin | **4.22 mm** |
| Restoring moment `M = total_mass × g × margin` | **0.00638 N·m** |
| Horizontal force at 180 mm height that equals that restoring moment | **0.0355 N** |

This is an intentionally conservative corner-of-envelope case, not a prediction of the exact physical center of mass. It nevertheless shows why a blanket promise of free-standing stability would be unjustified. Even when a real sample is better balanced, a cable pull can dominate the small restoring moment.

**Fasten the base to a rigid work surface with all four M4 holes, and anchor the stationary cable bundle beside it.** The holes form a 70.711 mm square. Use suitable washers and nuts and confirm the printed recesses are sound. A 1 N horizontal pull at 180 mm height creates 0.18 N·m; allowing twice that moment corresponds to approximately 5.1 N of additional force across a 70.711 mm bolt-row spacing. This estimates the anchor reaction, not a certified allowable load for the print, washers or work surface. A lightweight loose board can tip with the assembly and is not equivalent to a secured work surface.

## Arm flex

The ribbon thickness increases from 6 to 10 mm while its nominal width remains 14 mm. For the same homogeneous material, weak-direction rectangular-section inertia is `I = b t³ / 12`:

- Previous nominal section: 252 mm⁴.
- Revised nominal section: 1166.7 mm⁴.
- Ratio: 4.63.

This is a section comparison, not a claim that the complete printed arm is exactly 4.63 times stiffer. Curvature, screw access cutouts, layer orientation, infill, local stress concentrations and joints change the real result. The calculation does not include servo shaft/bearing deflection or horn compliance. Anchoring prevents base movement; a load test is still needed to check arm and joint sag.

## Acceptance checks on the real assembly

Measure the relevant servo feature before printing the new cap. Weigh the printed parts and record the material and settings. With the base anchored, support the camera during initial power-up and test one axis at a time near neutral. Increase travel gradually, confirming there is no wire tension, rubbing, case movement or loss of level at neutral. Do not force an unpowered geared servo through its travel. Observe whether the powered camera sags or the servos heat up under sustained load. Confirm physical travel limits before tracking.

The supplied tracker build reduces commands to the logical range 45–135 and at most one degree per 33 ms. Those limits reduce commanded motion; they do not measure cable force, guarantee physical servo angle or prevent droop after power loss.
