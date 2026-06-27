# Claude PCB connecting-beams tool v2 for "Main Robot".
# Phases (MODE='all'):  center turret -> build beams+holes -> verify clearance -> export f3z
# Internal Fusion units = cm. All design data below in mm, Y is UP, assembly/root space.
import adsk.core, adsk.fusion, traceback, json, os

# ----------------------------------------------------------------------------
MODE = 'export'       # 'all'|'center'|'build'|'verify'|'export'|'inspect_ir'|'isolate'|'showall'|'stl'|'saveas'
COMP_NAME = 'Claude Support Beams'
EXPORT_PATH = r'C:\Users\kusha\Downloads\RCJ\RCJ Main 2026\CAD\Fusion 1.0\Main_Robot_after_Claude.f3z'
DO_CENTER = True
# ----------------------------------------------------------------------------

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'out')
CM = 0.1              # mm -> cm

# ===== Step 0 centering: move the whole sensor turret to the footprint center.
# footprint center (top base + Base with Blocks) xz = (128.646, -28.952) mm
# IR_PCB bbox center xz = (129.248, -22.028) -> delta to centre =
DELTA = (-0.602, 0.0, -6.924)          # (dx, dy, dz) mm applied to turret occs
TURRET = ['IR Cover', 'IR_PCB', 'Ultrasonic_PCB', 'Ultrasonic_PCB 2.0']
FOOTPRINT_CENTER_XZ = (128.646, -28.952)

# Keep-out: Teensy micro-USB cable envelope (mm) [xmin,xmax, ymin,ymax, zmin,zmax]
USB_ENVELOPE = [160.8, 188.0, 66.5, 80.0, -31.0, -15.0]

# ===== Stack Y planes (mm, Y up) -- unchanged by the in-plane centering move
Y_MAIN_TOP = 62.54     # Main PCB top face
Y_PWR_TOP  = 86.52     # Power PCB top
Y_PWR_BOT  = 85.00     # Power PCB underside
Y_COVER    = 105.08    # IR Cover underside (seat plane for Group A tops)
Y_IR_TOP   = 117.81    # IR PCB top
Y_US_BOT   = 133.0     # Ultrasonic mounting-PCB underside (real Ø6 holes ~y133.3)
YB = 81.2              # capture-structure bottom (clears motors @80.7)
YTHIN = 83.0           # thin-rail top (clears terminal legs @83)
YT = 85.0              # boss top (reaches Power PCB underside @85)

# ----- verified holes (mm). Main PCB + Power PCB DO NOT move; IR + US move by DELTA.
def shifted(x, z):
    return (x + DELTA[0], z + DELTA[2])

# ===========================================================================
def beamA(side):
    """Group A L-beam. side=-1 left, +1 right. L-shaped to clear motors/USB/terminals.
    Captures the Power PCB at two corners (the two beams together pin all 4 corners),
    terminates under the IR Cover. Holes: Main Ø6, Power Ø6 x2, IR-Cover Ø4.5."""
    if side < 0:        # LEFT
        main_hole = (86.75, -22.97)
        pwr_f = (73.0, -32.0); pwr_r = (73.0, -89.0)
        col = (71.0, -11.5); rail_cx = 63.0
        name = 'Group A Beam L'
    else:               # RIGHT (clear USB cable + CW-019 module)
        main_hole = (171.75, -22.97)
        pwr_f = (184.5, -32.0); pwr_r = (184.5, -88.5)
        col = (183.0, -11.5); rail_cx = 195.5
        name = 'Group A Beam R'
    mhx, mhz = main_hole
    cx, cz = col
    fx, fz = pwr_f
    rx, rz = pwr_r
    boxes = []
    boxes.append((mhx, mhz, Y_MAIN_TOP, 66.0, 12, 10))               # foot pad on Main PCB
    ax0, ax1 = sorted([mhx, cx]); az0, az1 = sorted([mhz, cz])
    boxes.append(((ax0+ax1)/2, (az0+az1)/2, Y_MAIN_TOP, 66.0,
                  (ax1-ax0)+6, (az1-az0)+4))                          # low arm (below USB)
    boxes.append((cx, cz, Y_MAIN_TOP, Y_COVER, 8, 5))                # column up to cover
    lx0, lx1 = sorted([cx, fx]); lz0, lz1 = sorted([cz, fz])
    boxes.append(((lx0+lx1)/2, (lz0+lz1)/2, YB, YT, abs(lx1-lx0)+8, abs(lz1-lz0)+8))  # link
    boxes.append((fx, fz, YB, YT, 8, 10))                            # front boss -> Power
    boxes.append((rail_cx, (fz+rz)/2, YB, YTHIN, 8, abs(fz-rz)+8))   # outboard thin rail
    cb0, cb1 = sorted([rail_cx, fx])
    boxes.append(((cb0+cb1)/2, fz, YB, YTHIN, abs(rail_cx-fx)+8, 10))
    rb0, rb1 = sorted([rail_cx, rx])
    boxes.append(((rb0+rb1)/2, rz, YB, YTHIN, abs(rail_cx-rx)+8, 10))
    boxes.append((rx, rz, YB, YT, 12, 10))                           # rear boss -> Power
    holes = [(mhx, mhz, 60.0, 68.0, 6.0),     # Main PCB Ø6
             (cx, cz, 100.0, 107.0, 4.5),     # IR Cover end Ø4.5 (nominal seat hole)
             (fx, fz, 79.0, 87.0, 6.0),       # Power PCB Ø6 front corner
             (rx, rz, 79.0, 87.0, 6.0)]       # Power PCB Ø6 rear corner
    return boxes, holes, [], name

def beamB(ir_xz, us_xz, name, avoid=None):
    """Group B post: IR PCB (Ø5) -> Ultrasonic case (Ø6). Coords already DELTA-shifted.
    `avoid` = list of (x0,x1,z0,z1,ytop) component AABBs (current world mm) to notch
    out of the lower post so it clears vertical resistors etc. on the IR board."""
    irx, irz = ir_xz; usx, usz = us_xz
    cx = (irx+usx)/2; cz = (irz+usz)/2
    dx = abs(irx-usx)+7; dz = abs(irz-usz)+7
    boxes = [(cx, cz, Y_IR_TOP, Y_US_BOT, dx, dz)]
    holes = [(irx, irz, 115.0, 130.0, 5.0),   # IR end Ø5
             (usx, usz, 123.0, 134.0, 6.0)]   # US end Ø6
    cuts = []
    M = 0.8                                    # clearance margin (mm) around the part
    for (x0, x1, z0, z1, ytop) in (avoid or []):
        ccx = (x0+x1)/2; ccz = (z0+z1)/2
        cuts.append((ccx, ccz, Y_IR_TOP-3.0, ytop+0.7, (x1-x0)+2*M, (z1-z0)+2*M))
    return boxes, holes, cuts, name

def beam_specs():
    rl_ir = shifted(97.272, -49.528); rl_us = shifted(97.5,  -54.294)
    rr_ir = shifted(158.182, -49.528); rr_us = shifted(160.5, -54.294)
    fl_ir = shifted(97.222, 8.472);   fl_us = shifted(97.5,  8.706)
    fr_ir = shifted(158.182, 8.472);  fr_us = shifted(160.5, 8.706)
    # vertical resistors crowding each IR hole (current world AABB: x0,x1,z0,z1,ytop)
    RL_AV = [(89.0, 94.4, -65.0, -59.5, 124.5)]
    RR_AV = [(158.7, 164.2, -68.6, -63.1, 124.5)]
    FL_AV = [(92.6, 98.1, 4.7, 10.2, 124.5)]
    FR_AV = [(162.3, 167.8, 1.1, 6.6, 124.5)]
    return [beamA(-1), beamA(+1),
            beamB(rl_ir, rl_us, 'Group B Post RL', RL_AV),
            beamB(rr_ir, rr_us, 'Group B Post RR', RR_AV),
            beamB(fl_ir, fl_us, 'Group B Post FL', FL_AV),
            beamB(fr_ir, fr_us, 'Group B Post FR', FR_AV)]

# ===========================================================================
def mkbox(tbm, cx, cz, y0, y1, dx, dz):
    c = adsk.core.Point3D.create(cx*CM, (y0+y1)/2*CM, cz*CM)
    obb = adsk.core.OrientedBoundingBox3D.create(
        c, adsk.core.Vector3D.create(1, 0, 0), adsk.core.Vector3D.create(0, 0, 1),
        dx*CM, dz*CM, abs(y1-y0)*CM)
    return tbm.createBox(obb)

def mkcyl(tbm, x, z, y0, y1, dia):
    p1 = adsk.core.Point3D.create(x*CM, y0*CM, z*CM)
    p2 = adsk.core.Point3D.create(x*CM, y1*CM, z*CM)
    return tbm.createCylinderOrCone(p1, dia/2*CM, p2, dia/2*CM)

def _bt(names, intval):
    for n in names:
        if hasattr(adsk.fusion.BooleanTypes, n):
            return getattr(adsk.fusion.BooleanTypes, n)
    return intval
BOOL_UNION = _bt(['UnionBooleanType', 'UnionType'], 0)
BOOL_DIFF = _bt(['DifferenceBooleanType', 'DifferenceType'], 1)

def build_body(tbm, boxes, holes, cuts=None):
    base = None
    for bx in boxes:
        b = mkbox(tbm, *bx)
        if base is None:
            base = b
        else:
            tbm.booleanOperation(base, b, BOOL_UNION)
    for cut in (cuts or []):
        cb = mkbox(tbm, *cut)
        tbm.booleanOperation(base, cb, BOOL_DIFF)
    for h in holes:
        c = mkcyl(tbm, *h)
        tbm.booleanOperation(base, c, BOOL_DIFF)
    return base

def aabb_overlap(a, b):
    ox = min(a[1], b[1]) - max(a[0], b[0])
    oy = min(a[3], b[3]) - max(a[2], b[2])
    oz = min(a[5], b[5]) - max(a[4], b[4])
    if ox > 0 and oy > 0 and oz > 0:
        return round(min(ox, oy, oz), 3), [round(ox, 2), round(oy, 2), round(oz, 2)]
    return None

def box_aabb(bx):
    cx, cz, y0, y1, dx, dz = bx
    return [cx-dx/2, cx+dx/2, min(y0, y1), max(y0, y1), cz-dz/2, cz+dz/2]

# ===========================================================================
def find_root_occ(root, comp_name):
    for occ in root.occurrences:
        if occ.component.name == comp_name:
            return occ
    return None

def occ_world_center_xz(occ):
    bb = occ.boundingBox
    return ((bb.minPoint.x+bb.maxPoint.x)/2*10.0, (bb.minPoint.z+bb.maxPoint.z)/2*10.0)

def center_turret(design, root, out):
    """Shift the four turret occurrences by DELTA (parent/world frame, identity rot)."""
    out['center'] = {'delta_mm': list(DELTA), 'moved': [], 'before': {}, 'after': {}}
    # capture before
    for nm in TURRET:
        occ = find_root_occ(root, nm)
        if occ:
            try:
                out['center']['before'][nm] = [round(v, 3) for v in occ_world_center_xz(occ)]
            except:
                pass
    # idempotency: skip if the turret is already at the footprint centre (e.g. re-run)
    occ_ir0 = find_root_occ(root, 'IR_PCB')
    if occ_ir0:
        try:
            cx0, cz0 = occ_world_center_xz(occ_ir0)
            if abs(cx0-FOOTPRINT_CENTER_XZ[0]) < 1.0 and abs(cz0-FOOTPRINT_CENTER_XZ[1]) < 1.0:
                out['center']['skipped'] = 'already centred (IR_PCB within 1mm of footprint)'
                out['center']['ok'] = True
                out['center']['after'] = dict(out['center']['before'])
                return True
        except:
            pass
    # apply move
    for nm in TURRET:
        occ = find_root_occ(root, nm)
        if not occ:
            out['center'].setdefault('missing', []).append(nm)
            continue
        try:
            if occ.isGrounded:
                occ.isGrounded = False
        except:
            pass
        m = occ.transform2
        t = m.translation
        t.x += DELTA[0]*CM; t.y += DELTA[1]*CM; t.z += DELTA[2]*CM
        m.translation = t
        occ.transform2 = m
        out['center']['moved'].append(nm)
    # persist (parametric needs a snapshot)
    try:
        if design.snapshots.hasPendingSnapshot:
            design.snapshots.add()
            out['center']['snapshot'] = True
    except Exception as e:
        out['center']['snapshot_err'] = str(e)
    # capture after + check
    ok = True
    for nm in TURRET:
        occ = find_root_occ(root, nm)
        if occ:
            try:
                c = [round(v, 3) for v in occ_world_center_xz(occ)]
                out['center']['after'][nm] = c
            except:
                pass
    irc = out['center']['after'].get('IR_PCB')
    if irc:
        out['center']['IR_PCB_to_footprint_mm'] = [round(irc[0]-FOOTPRINT_CENTER_XZ[0], 3),
                                                    round(irc[1]-FOOTPRINT_CENTER_XZ[1], 3)]
        if abs(irc[0]-FOOTPRINT_CENTER_XZ[0]) > 1.0 or abs(irc[1]-FOOTPRINT_CENTER_XZ[1]) > 1.0:
            ok = False
    out['center']['ok'] = ok
    return ok

# ===========================================================================
def do_build(app, design, root, out):
    tbm = adsk.fusion.TemporaryBRepManager.get()
    for occ in list(root.occurrences):
        if occ.component.name == COMP_NAME:
            occ.deleteMe()
    specs = beam_specs()
    newOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = newOcc.component
    comp.name = COMP_NAME
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    made = []
    for boxes, holes, cuts, name in specs:
        body = build_body(tbm, boxes, holes, cuts)
        nb = comp.bRepBodies.add(body, bf)
        nb.name = name
        made.append((name, boxes, holes, cuts))
    bf.finishEdit()
    out['bodies'] = []
    for occ in root.occurrences:
        if occ.component.name == COMP_NAME:
            for b in occ.bRepBodies:
                out['bodies'].append({'name': b.name, 'vol_cm3': round(b.volume, 3)})
    _clearance_check(root, made, out)

def _covered_by_cut(oa, cuts_aabb, eps=0.05):
    """True if component AABB oa is fully inside (xz + top) one notch cut -> removed."""
    for c in cuts_aabb:
        if (oa[0] >= c[0]-eps and oa[1] <= c[1]+eps and
                oa[4] >= c[4]-eps and oa[5] <= c[5]+eps and
                oa[3] <= c[3]+eps):
            return True
    return False

def _clearance_check(root, made, out):
    TOL = 0.4
    beam_boxes = []
    cuts_aabb = []
    for name, boxes, holes, cuts in made:
        for bx in boxes:
            beam_boxes.append((name, box_aabb(bx)))
        for cu in cuts:
            cuts_aabb.append(box_aabb(cu))
    collisions = []
    for occ in root.allOccurrences:
        if occ.component.name == COMP_NAME:
            continue
        if occ.childOccurrences.count > 0:
            continue
        try:
            nm_mesh = occ.component.meshBodies.count
        except:
            nm_mesh = 0
        if occ.bRepBodies.count == 0 and nm_mesh == 0:
            continue
        try:
            bb = occ.boundingBox
            oa = [bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.y*10,
                  bb.maxPoint.y*10, bb.minPoint.z*10, bb.maxPoint.z*10]
        except:
            continue
        for bname, abx in beam_boxes:
            r = aabb_overlap(abx, oa)
            if r and r[0] > TOL:
                if _covered_by_cut(oa, cuts_aabb):
                    continue   # notched away in the actual solid
                collisions.append({'beam': bname, 'hits': occ.fullPathName,
                                   'pen_mm': r[0], 'overlap_xyz': r[1]})
    usb = list(USB_ENVELOPE)
    for bname, abx in beam_boxes:
        r = aabb_overlap(abx, usb)
        if r and r[0] > 0.0:
            collisions.append({'beam': bname, 'hits': 'USB_ENVELOPE',
                               'pen_mm': r[0], 'overlap_xyz': r[1]})
    seen = {}
    for c in collisions:
        k = (c['beam'], c['hits'])
        if k not in seen or c['pen_mm'] > seen[k]['pen_mm']:
            seen[k] = c
    out['collisions'] = sorted(seen.values(), key=lambda c: -c['pen_mm'])
    out['collision_count'] = len(out['collisions'])

def do_stl(design, root, out):
    import time
    em = design.exportManager
    occ = find_root_occ(root, COMP_NAME)
    path = r'C:\Users\kusha\Downloads\RCJ\RCJ Main 2026\CAD\Fusion 1.0\Support_Beams_after_Claude.stl'
    out['stl'] = path
    try:
        opts = em.createSTLExportOptions(occ.component, path)
        try:
            opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
        except Exception:
            pass
        em.execute(opts)
    except Exception:
        out['stl_error'] = traceback.format_exc()
    for _ in range(4):
        time.sleep(1)
        if os.path.exists(path):
            break
    out['stl_exists'] = os.path.exists(path)
    try:
        out['stl_size'] = os.path.getsize(path)
    except Exception:
        out['stl_size'] = 0

def do_inspect_ir(root, out):
    """Dump world AABBs (mm) of leaf parts under IR_PCB:1 near the 4 Group-B holes,
    so the posts can be shaped to clear nearby resistors/components."""
    holes = []
    for (irx, irz) in [shifted(97.272, -49.528), shifted(158.182, -49.528),
                       shifted(97.222, 8.472), shifted(158.182, 8.472)]:
        holes.append((irx, irz))
    parts = []
    for occ in root.allOccurrences:
        if 'IR_PCB:1' not in occ.fullPathName:
            continue
        if occ.childOccurrences.count > 0:
            continue
        if occ.bRepBodies.count == 0:
            continue
        try:
            bb = occ.boundingBox
            a = [bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.y*10,
                 bb.maxPoint.y*10, bb.minPoint.z*10, bb.maxPoint.z*10]
        except:
            continue
        cx = (a[0]+a[1])/2; cz = (a[4]+a[5])/2
        # nearest hole distance (xz)
        nd = min(((cx-hx)**2+(cz-hz)**2)**0.5 for hx, hz in holes)
        if nd < 12.0:   # within 12mm of any post centre
            nm = occ.component.name
            parts.append({'name': nm, 'near_mm': round(nd, 2),
                          'bb': [round(v, 2) for v in a],
                          'cx': round(cx, 2), 'cz': round(cz, 2),
                          'ytop': round(a[3], 2)})
    parts.sort(key=lambda p: p['near_mm'])
    out['ir_parts'] = parts
    out['ir_holes'] = [[round(h[0], 2), round(h[1], 2)] for h in holes]

def do_export(design, out):
    import time
    out['exported'] = EXPORT_PATH
    try:
        opts = design.exportManager.createFusionArchiveExportOptions(EXPORT_PATH)
        res = design.exportManager.execute(opts)
        out['export_result'] = str(res)
    except Exception:
        out['export_error'] = traceback.format_exc()
    for _ in range(6):
        time.sleep(1)
        if os.path.exists(EXPORT_PATH):
            break
    out['exists'] = os.path.exists(EXPORT_PATH)
    try:
        out['size_bytes'] = os.path.getsize(EXPORT_PATH)
    except Exception:
        out['size_bytes'] = 0

# ===========================================================================
def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    out = {'ok': False, 'mode': MODE}
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        out['doc'] = app.activeDocument.name
        if design.designType != adsk.fusion.DesignTypes.ParametricDesignType:
            design.designType = adsk.fusion.DesignTypes.ParametricDesignType

        if MODE in ('all', 'center') and DO_CENTER:
            cok = center_turret(design, root, out)
            if MODE == 'all' and not cok:
                out['aborted'] = 'centering check failed; build skipped'
                raise RuntimeError(out['aborted'])

        if MODE == 'inspect_ir':
            do_inspect_ir(root, out)

        if MODE == 'isolate':       # show only the beams (for a clean screenshot)
            for occ in root.occurrences:
                try:
                    occ.isLightBulbOn = (occ.component.name == COMP_NAME)
                except:
                    pass
            out['isolated'] = True

        if MODE == 'showall':       # restore all top-level components visible
            for occ in root.occurrences:
                try:
                    occ.isLightBulbOn = True
                except:
                    pass
            out['shownall'] = True

        if MODE in ('all', 'build'):
            do_build(app, design, root, out)

        if MODE in ('all', 'export'):
            do_export(design, out)

        if MODE == 'stl':           # export beam component as STL for printing
            do_stl(design, root, out)

        if MODE == 'saveas':        # save a NEW cloud document, original untouched
            doc = app.activeDocument
            try:
                folder = doc.dataFile.parentFolder
            except Exception:
                folder = app.data.activeProject.rootFolder
            try:
                r = doc.saveAs('Main Robot after Claude', folder,
                               'PCB connecting beams added; sensor turret centered (Claude).',
                               'claude-beams')
                out['saveas_result'] = str(r)
                out['saveas_folder'] = folder.name if folder else None
                out['active_after'] = app.activeDocument.name
            except Exception:
                out['saveas_error'] = traceback.format_exc()

        out['ok'] = True
    except Exception:
        out['traceback'] = traceback.format_exc()

    try:
        os.makedirs(OUT_DIR, exist_ok=True)
        with open(os.path.join(OUT_DIR, 'v2_' + MODE + '.json'), 'w') as f:
            json.dump(out, f, indent=1)
    except Exception:
        pass

    try:
        msg = 'Claude beams v2 [%s]: ok=%s' % (MODE, out['ok'])
        if 'center' in out:
            msg += '\nCenter ok=%s  IR_PCB->footprint=%s' % (
                out['center'].get('ok'), out['center'].get('IR_PCB_to_footprint_mm'))
        if 'bodies' in out:
            msg += '\nBodies: %d   Collisions: %s' % (
                len(out['bodies']), out.get('collision_count', '?'))
        if 'exported' in out:
            msg += '\nExported: %s (exists=%s)' % (out['exported'], out.get('exists'))
        if not out['ok']:
            msg += '\n\n' + out.get('traceback', '')[-1400:]
        ui.messageBox(msg, 'Claude Beams v2')
    except:
        pass
