"""Build and audit Revision M. Run with the supplied CAD generator beside this file."""
import contextlib
import hashlib
import io
import itertools
import json
import math
from pathlib import Path

import cadquery as cq
import generate_robotic_arm_revM as g


def vol(shape):
    return sum(s.Volume() for s in shape.solids().vals())


def overlap(a, b):
    return vol(a.intersect(b))


def point(p):
    return [p.x, p.y, p.z]


def rotate_point(p, angle):
    x, y, z = p
    a = math.radians(angle)
    return (x*math.cos(a)+z*math.sin(a), y, -x*math.sin(a)+z*math.cos(a))


def main():
    arm = g.make_arm_and_holder()
    cap = g.make_servo_cap()
    cradle = g.make_camera_screwless_cradle()
    base = g.make_pan_servo_stability_base()
    parts = dict(CurvedArm_SG90_TiltHolder=arm, SG90_RetainingCap=cap,
                 Logitech_C270_ScrewlessCradle=cradle, PanServo_StabilityBase=base)
    checks = {}
    def check(name, passed, value=None):
        checks[name] = dict(pass_check=bool(passed), value=value)
        print(('PASS ' if passed else 'FAIL ') + name + (': '+str(value) if value is not None else ''), flush=True)

    for name, shape in parts.items():
        check(name+' one valid solid', len(shape.solids().vals()) == 1 and shape.val().isValid())
        g.export_part(shape, name)
    g.export_reference_models()
    g.export_positioned_assemblies(arm, cap, cradle, base)
    for path in sorted(g.OUT.rglob('*.step')):
        imported = cq.importers.importStep(str(path))
        expected = 10 if 'Full_Assembly' in path.name else 4 if 'Printable_Assembly' in path.name else 2 if 'Reference' in path.name else 1
        check('STEP '+path.name, len(imported.solids().vals()) == expected and all(s.isValid() for s in imported.solids().vals()))

    refs = g.positioned_reference_models()
    opening = g.box_at(-13.49,13.49,21.3,27.1,86.51,101.49)
    check('27 x 15 mm rectangular cap opening', overlap(cap,opening)<1e-5, round(overlap(cap,opening),8))
    web = 17-13.5-1.2
    check('opening to screw hole web >= 2 mm',web>=2,web)
    check('new screw pattern 34 x 17 mm',g.CAP_SCREW_CENTER_X*2==34 and g.CAP_SCREW_CENTER_Z_OFFSET*2==17)
    for x in (-17,17):
        for z in (85.5,102.5):
            check(f'cap hole {x} {z}',overlap(cap,g.cyl_y(1.19,23.9,27.1,x,z))<1e-5)
            check(f'holder pilot {x} {z}',overlap(arm,g.cyl_y(.89,14,24.1,x,z))<1e-5)
            # A 6 mm under-head screw crosses 3 mm cap and engages 3 mm holder.
            annulus = g.cyl_y(1.5,21.0,23.9,x,z).cut(g.cyl_y(.91,20.9,24,x,z))
            check(f'3 mm engagement material {x} {z}',overlap(arm,annulus)>0.95*vol(annulus))
    check('pan stock screw and tool corridor', overlap(arm,g.cyl_z(3.1,3.5,150))<1e-5)
    check('tilt stock screw and tool corridor',overlap(cradle,g.cyl_y(3.1,-.1,86,0,0))<1e-5)
    for y in (-7,7):
        check(f'pan recessed screw seat {y}',overlap(arm,g.cyl_z(2.2,5.21,40,0,y))<1e-5)
    for z in (-7,7):
        check(f'tilt recessed screw seat {z}',overlap(cradle,g.cyl_y(2.2,1.21,86,0,z))<1e-5)
    for x1,x2 in ((-22,-11.6),(11.6,22)):
        probe=g.side_capsule_x(x1,x2,10.5,15.8,9.8,94)
        check(f'upper servo side wire exit {x1}',overlap(arm,probe)<1e-5)
    for x1,x2 in ((-19.4,-11.6),(11.6,19.4)):
        check(f'base side wire exit {x1}',overlap(base,g.box_at(x1,x2,-6.9,6.9,-19.9,-11.1))<1e-5)
    rear_probe = g.orient_camera(g.box_at(-30.9,30.9,31.7,70,5.5,40))
    check('rear USB central opening stays unobstructed',overlap(cradle,rear_probe)<1e-5)

    for name,a,b in [('cap against servo',cap,refs['SG90_Tilt_Body']),
                     ('arm against tilt servo',arm,refs['SG90_Tilt_Body']),
                     ('cap against arm',cap,arm),
                     ('base against pan servo',base,refs['SG90_Pan_Body']),
                     ('arm against pan horn',arm,refs['SG90_Pan_Horn'])]:
        value=overlap(a,b)
        check(name,value<.01,round(value,6))

    body,clip=g.make_c270_reference()
    check('webcam clip against cradle',overlap(clip,cradle)<.01,round(overlap(clip,cradle),6))
    preload=overlap(body,cradle)
    check('webcam only intended flexible grip preload',0.1<preload<4.0,round(preload,6))
    fixed = cq.Workplane(obj=cq.Compound.makeCompound([arm.val(),cap.val(),base.val(),refs['SG90_Tilt_Body'].val()]))
    moving = cq.Workplane(obj=cq.Compound.makeCompound([cradle.val(),body.val(),clip.val()]))
    # Rotation about Y preserves every Y coordinate. This separation bounds
    # clearance continuously between the sampled angles for the upper assembly.
    upper_max_y=max(p.val().BoundingBox().ymax for p in (arm,cap,refs['SG90_Tilt_Body']))
    moving_min_y=moving.val().BoundingBox().ymin+31
    check('continuous upper assembly axial separation >= 2 mm',moving_min_y-upper_max_y>=1.999,
          round(moving_min_y-upper_max_y,3))
    mb=moving.val().BoundingBox()
    radial_bound=max(math.hypot(x,z) for x,z in itertools.product((mb.xmin,mb.xmax),(mb.zmin,mb.zmax)))
    check('continuous moving assembly separation above base',94-radial_bound>base.val().BoundingBox().zmax,
          round(94-radial_bound-base.val().BoundingBox().zmax,3))
    cable = g.orient_camera(g.cyl_y(3,31.6,73.6,0,6+g.C270_HEIGHT/2))
    motion=[]
    for angle in range(-45,46,5):
        moved=moving.rotate((0,0,0),(0,1,0),angle).translate((0,31,94))
        moved_cable=cable.rotate((0,0,0),(0,1,0),angle).translate((0,31,94))
        horn=refs['SG90_Tilt_Horn'].rotate((0,31,94),(0,32,94),angle)
        collision=overlap(moved,fixed)
        cable_collision=overlap(moved_cable,fixed)
        horn_collision=overlap(horn,fixed)
        clearance=moved.val().distance(fixed.val())
        direction=rotate_point((1,0,0),angle)
        motion.append(dict(tilt_deg=angle,collision_mm3=collision,clearance_mm=clearance,
                           cable_collision_mm3=cable_collision,horn_collision_mm3=horn_collision,
                           view_direction=direction))
        check(f'tilt {angle:+d} deg solids cable horn clearance',max(collision,cable_collision,horn_collision)<.01,
              [round(collision,5),round(cable_collision,5),round(horn_collision,5),round(clearance,3)])
    check('tilt changes camera elevation by 90 degrees',
          abs(motion[0]['view_direction'][2]-.70710678)<1e-6 and abs(motion[-1]['view_direction'][2]+.70710678)<1e-6)
    # Z pan rotates both the tilt axis and the viewing vector. Perpendicularity
    # and independent elevation hold at every pan angle; there is no roll input.
    check('pan and tilt shaft directions perpendicular',sum(a*b for a,b in zip((0,0,1),(0,1,0)))==0)

    # Uncertain effective density covers sparse base/arm through fully solid PLA/PETG.
    # It is NOT the slicer's infill fraction converted to an exact printed mass.
    densities=(.35*1.24,1.27) # g/cm^3
    samples=[]
    centers={n:point(p.val().Center()) for n,p in parts.items()}
    camera_bb=cq.Compound.makeCompound([body.val(),clip.val()]).BoundingBox()
    camera_corners=list(itertools.product((camera_bb.xmin,camera_bb.xmax),
                                         (camera_bb.ymin,camera_bb.ymax),
                                         (camera_bb.zmin,camera_bb.zmax)))
    base_radius=68.0 # conservatively account for the 1.8 mm rounded bottom edge
    for db,da,dc,dk in itertools.product(densities,repeat=4):
        mass={n:vol(parts[n])/1000*d for n,d in zip(parts,(da,dk,dc,db))}
        for angle in range(-45,46,5):
            cm=rotate_point(centers['Logitech_C270_ScrewlessCradle'],angle)
            cradle_cm=(cm[0],cm[1]+31,cm[2]+94)
            for corner in camera_corners:
                cp=rotate_point(corner,angle)
                camera_cm=(cp[0],cp[1]+31,cp[2]+94)
                weighted=[(mass['PanServo_StabilityBase'],centers['PanServo_StabilityBase']),
                          (mass['CurvedArm_SG90_TiltHolder'],centers['CurvedArm_SG90_TiltHolder']),
                          (mass['SG90_RetainingCap'],centers['SG90_RetainingCap']),
                          (mass['Logitech_C270_ScrewlessCradle'],cradle_cm),
                          (75,camera_cm),(9,(0,0,-10)),(9,(0,16,94)),(2,(0,0,3)),(2,(0,30,94))]
                total=sum(m for m,p in weighted)
                cg=[sum(m*p[i] for m,p in weighted)/total for i in range(3)]
                margin=base_radius-math.hypot(cg[0],cg[1])
                resisting=total/1000*9.80665*margin/1000
                samples.append(dict(total_g=total,tilt_deg=angle,cg_mm=cg,edge_margin_mm=margin,
                                    restoring_Nm=resisting,cable_force_threshold_N=resisting/.18))
    worst=min(samples,key=lambda s:s['restoring_Nm'])
    # A conservative camera COM anywhere inside the proxy envelope bounds gravity.
    camera_r=max(math.hypot(x,z) for x,y,z in camera_corners)
    cradle_cm=centers['Logitech_C270_ScrewlessCradle']
    cradle_mass_max=vol(cradle)/1000*1.27
    gravity_kgcm=.075*camera_r/10+(cradle_mass_max/1000)*math.hypot(cradle_cm[0],cradle_cm[2])/10
    cable_assumption_N=.2
    cable_radius_mm=60
    cable_kgcm=cable_assumption_N*cable_radius_mm/1000/.0980665
    motion_allowance=1.5
    tilt_design_kgcm=motion_allowance*gravity_kgcm+cable_kgcm
    axis_formula='d(p,t) = (cos(p)*cos(t), sin(p)*cos(t), -sin(t)); angles in radians'
    engineering=dict(assumptions=dict(webcam_g=75,servo_g_each=9,horn_g_each=2,
        effective_density_g_cm3=densities,full_camera_COM_envelope=True,
        support_radius_mm=68,cable_pull_height_mm=180,cable_drag_N=.2,
        cable_torque_radius_mm=60,gravity_motion_multiplier=1.5),
        worst_unanchored_case=worst,camera_radial_COM_bound_mm=camera_r,
        tilt_gravity_bound_kgcm=gravity_kgcm,tilt_with_motion_and_cable_kgcm=tilt_design_kgcm,
        manufacturer_nominal_stall_kgcm=1.8,stall_to_estimated_demand_ratio=1.8/tilt_design_kgcm,
        axis_formula=axis_formula,
        arm_section_I_old_mm4=14*6**3/12,
        arm_section_I_new_mm4=14*10**3/12,
        arm_nominal_section_stiffness_ratio=(10/6)**3,
        arm_note='Section comparison only; curved geometry, access holes, layer adhesion, '
                 'horn compliance and SG90 bearing stiffness require physical load testing.',
        decision='Secure the base to a rigid work surface using its four M4 holes. '
                 'Unanchored stability and servo stiffness are not certified by this model. '
                 'Weigh the prints and measure real cable drag before relying on free-standing use.')
    result=dict(checks=checks,motion_samples=motion,engineering=engineering,
                parts={n:dict(volume_mm3=vol(p),centroid_mm=centers[n],
                    bbox_mm=[p.val().BoundingBox().xlen,p.val().BoundingBox().ylen,p.val().BoundingBox().zlen]) for n,p in parts.items()})
    (g.DOC_DIR/'CAD_and_Engineering_Results.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    failures=[n for n,r in checks.items() if not r['pass_check']]
    print('ENGINEERING',json.dumps(engineering),flush=True)
    if failures:
        raise RuntimeError('Failed checks: '+', '.join(failures))
    g.make_previews(arm,cap,cradle,base)
    print('REVISION M CAD CHECKS PASSED',flush=True)


if __name__=='__main__':
    main()
