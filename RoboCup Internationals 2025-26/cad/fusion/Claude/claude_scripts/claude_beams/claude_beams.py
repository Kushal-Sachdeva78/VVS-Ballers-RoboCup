# Claude PCB connecting-beams tool for "Main Robot".
# Reads REAL geometry, then (build mode) creates beam bodies + clearance holes,
# then self-checks clearance vs every component and the USB envelope.
# Internal units cm (Fusion native). All design data below in mm, Y is UP.
import adsk.core, adsk.fusion, traceback, json, os

# ----------------------------------------------------------------------------
MODE = 'build'        # 'inspect' | 'build' | 'verify' | 'export'
COMP_NAME = 'Claude Support Beams'
EXPORT_PATH = r'C:\Users\kusha\Downloads\RCJ\RCJ Main 2026\CAD\Fusion 1.0\Main_Robot_after_Claude.f3z'
# ----------------------------------------------------------------------------

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'out')
CM = 0.1              # mm -> cm

# Keep-out: Teensy micro-USB cable envelope (mm) [xmin,xmax, ymin,ymax, zmin,zmax]
USB_ENVELOPE = [160.8, 188.0, 66.5, 80.0, -31.0, -15.0]

# ===== GEOMETRY (all mm, Y up). Boxes: (cx,cz,y0,y1,dx,dz). Holes:(x,z,y0,y1,dia)
Y_MAIN_TOP = 62.54     # Main PCB top face
Y_PWR_TOP  = 86.52     # Power PCB top
Y_PWR_BOT  = 85.00     # Power PCB underside
Y_COVER    = 105.08    # IR Cover underside (seat plane for Group A tops)
Y_IR_TOP   = 117.81    # IR PCB top
Y_US_BOT   = 133.0     # Ultrasonic mounting-PCB underside (real Ø6 holes ~y133.3)
# capture band: above motors (y<=80.7) and terminal legs (y>=83), below board (85)
YB = 81.2              # capture-structure bottom (clears motors @80.7)
YTHIN = 83.0           # thin-rail top (clears terminal legs @83)
YT = 85.0             # boss top (reaches Power PCB underside @85)

def beamA(side):
    """Group A L-beam. side=-1 left, +1 right. L-shaped to clear motors/USB/terminals."""
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
    # foot pad on Main PCB (sits in the motor-free z-gap, below USB y66.5)
    boxes.append((mhx, mhz, Y_MAIN_TOP, 66.0, 12, 10))
    # low arm foot -> column base (below USB, kept clear of front motor @ z-8.6)
    ax0, ax1 = sorted([mhx, cx]); az0, az1 = sorted([mhz, cz])
    boxes.append(((ax0+ax1)/2, (az0+az1)/2, Y_MAIN_TOP, 66.0,
                  (ax1-ax0)+6, (az1-az0)+4))
    # column up to cover (in the clear front slot between USB and front motor)
    boxes.append((cx, cz, Y_MAIN_TOP, Y_COVER, 8, 5))
    # link column -> front boss, in the capture band (above motors)
    lx0, lx1 = sorted([cx, fx]); lz0, lz1 = sorted([cz, fz])
    boxes.append(((lx0+lx1)/2, (lz0+lz1)/2, YB, YT, abs(lx1-lx0)+8, abs(lz1-lz0)+8))
    # front boss (reaches Power underside) at the Ø6 corner
    boxes.append((fx, fz, YB, YT, 8, 10))
    # outboard THIN rail front->rear (x outboard of relay, y above motors/legs)
    boxes.append((rail_cx, (fz+rz)/2, YB, YTHIN, 8, abs(fz-rz)+8))
    # connectors rail<->bosses (thin)
    cb0, cb1 = sorted([rail_cx, fx])
    boxes.append(((cb0+cb1)/2, fz, YB, YTHIN, abs(rail_cx-fx)+8, 10))
    rb0, rb1 = sorted([rail_cx, rx])
    boxes.append(((rb0+rb1)/2, rz, YB, YTHIN, abs(rail_cx-rx)+8, 10))
    # rear boss (reaches Power underside) at the Ø6 corner
    boxes.append((rx, rz, YB, YT, 12, 10))
    holes = [(mhx, mhz, 60.0, 68.0, 6.0),     # Main PCB Ø6
             (cx, cz, 100.0, 107.0, 4.5),     # IR Cover end Ø4.5 (clearance)
             (fx, fz, 79.0, 87.0, 6.0),       # Power PCB Ø6 front corner
             (rx, rz, 79.0, 87.0, 6.0)]       # Power PCB Ø6 rear corner
    return boxes, holes, name

def beamB(ir_xz, us_xz, name):
    """Group B post: IR(Ø5) -> Ultrasonic mounting PCB(Ø6)."""
    irx, irz = ir_xz; usx, usz = us_xz
    cx = (irx+usx)/2; cz = (irz+usz)/2
    dx = abs(irx-usx)+7; dz = abs(irz-usz)+7
    boxes = [(cx, cz, Y_IR_TOP, Y_US_BOT, dx, dz)]
    holes = [(irx, irz, 115.0, 130.0, 5.0),   # IR end Ø5
             (usx, usz, 123.0, 134.0, 6.0)]   # US end Ø6
    return boxes, holes, name

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
BOOL_INTERSECT = _bt(['IntersectionBooleanType', 'IntersectionType'], 2)

def build_body(tbm, boxes, holes):
    base = None
    for bx in boxes:
        b = mkbox(tbm, *bx)
        if base is None:
            base = b
        else:
            tbm.booleanOperation(base, b, BOOL_UNION)
    for h in holes:
        c = mkcyl(tbm, *h)
        tbm.booleanOperation(base, c, BOOL_DIFF)
    return base

def aabb_overlap(a, b):
    # a,b = [xmin,xmax,ymin,ymax,zmin,zmax]; return min penetration over axes (mm) or None
    ox = min(a[1], b[1]) - max(a[0], b[0])
    oy = min(a[3], b[3]) - max(a[2], b[2])
    oz = min(a[5], b[5]) - max(a[4], b[4])
    if ox > 0 and oy > 0 and oz > 0:
        return round(min(ox, oy, oz), 3), [round(ox, 2), round(oy, 2), round(oz, 2)]
    return None

def box_aabb(bx):
    cx, cz, y0, y1, dx, dz = bx
    return [cx-dx/2, cx+dx/2, min(y0, y1), max(y0, y1), cz-dz/2, cz+dz/2]


def do_build(app, design, root, out):
    tbm = adsk.fusion.TemporaryBRepManager.get()
    if design.designType != adsk.fusion.DesignTypes.ParametricDesignType:
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType

    # remove a previous run
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
    for boxes, holes, name in specs:
        body = build_body(tbm, boxes, holes)
        nb = comp.bRepBodies.add(body, bf)
        nb.name = name
        made.append((name, boxes, holes))
    bf.finishEdit()

    out['bodies'] = []
    for occ in root.occurrences:
        if occ.component.name == COMP_NAME:
            for b in occ.bRepBodies:
                bb = b.boundingBox
                out['bodies'].append({'name': b.name, 'vol_cm3': round(b.volume, 3)})

    # ---- world-space AABB clearance check (occ.boundingBox IS world) --------
    TOL = 0.4   # mm; ignore mere face-contact / seating planes
    beam_boxes = []
    for name, boxes, holes in made:
        for bx in boxes:
            beam_boxes.append((name, box_aabb(bx)))

    collisions = []
    for occ in root.allOccurrences:
        if occ.component.name == COMP_NAME:
            continue
        if occ.childOccurrences.count > 0:
            continue   # only true leaf occurrences give a tight world bbox
        nb = occ.bRepBodies.count
        try:
            nm_mesh = occ.component.meshBodies.count
        except:
            nm_mesh = 0
        if nb == 0 and nm_mesh == 0:
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
                collisions.append({'beam': bname, 'hits': occ.fullPathName,
                                   'pen_mm': r[0], 'overlap_xyz': r[1],
                                   'obj_bb': [round(v, 1) for v in oa]})
    # USB envelope (mm world AABB)
    usb = [USB_ENVELOPE[0], USB_ENVELOPE[1], USB_ENVELOPE[2],
           USB_ENVELOPE[3], USB_ENVELOPE[4], USB_ENVELOPE[5]]
    for bname, abx in beam_boxes:
        r = aabb_overlap(abx, usb)
        if r and r[0] > 0.0:
            collisions.append({'beam': bname, 'hits': 'USB_ENVELOPE',
                               'pen_mm': r[0], 'overlap_xyz': r[1], 'obj_bb': usb})

    seen = {}
    for c in collisions:
        k = (c['beam'], c['hits'])
        if k not in seen or c['pen_mm'] > seen[k]['pen_mm']:
            seen[k] = c
    out['collisions'] = sorted(seen.values(), key=lambda c: -c['pen_mm'])
    out['collision_count'] = len(out['collisions'])


def beam_specs():
    return [beamA(-1), beamA(+1),
            beamB((97.272, -49.528), (97.5, -54.294), 'Group B Post RL'),
            beamB((158.182, -49.528), (160.5, -54.294), 'Group B Post RR'),
            beamB((97.222, 8.472), (97.5, 8.706), 'Group B Post FL'),
            beamB((158.182, 8.472), (160.5, 8.706), 'Group B Post FR')]


def do_verify(root, out):
    # Check the PLANNED solid sub-boxes (tight) vs current world geometry.
    beams = []
    for boxes, holes, name in beam_specs():
        for i, bx in enumerate(boxes):
            beams.append(('%s#%d' % (name, i), box_aabb(bx)))
    # reliability references (compare to inspect: IR_PCB board top should be ~117.8)
    refbb = {}
    for occ in root.allOccurrences:
        fp = occ.fullPathName
        for tok in ['IR_PCB_PCB', 'IR Cover:1', 'Power PCB_PCB',
                    'Ultrasonic_PCB 2.0:1', 'Main_PCB 2.0_PCB']:
            if tok in fp and tok not in refbb:
                try:
                    bb = occ.boundingBox
                    refbb[tok] = [round(bb.minPoint.x*10, 1), round(bb.maxPoint.x*10, 1),
                                  round(bb.minPoint.y*10, 1), round(bb.maxPoint.y*10, 1),
                                  round(bb.minPoint.z*10, 1), round(bb.maxPoint.z*10, 1)]
                except:
                    pass
    out['ref'] = refbb
    # keep-outs: leaf occurrences (world bbox)
    keep = []
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
            keep.append((occ.fullPathName,
                         [bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.y*10,
                          bb.maxPoint.y*10, bb.minPoint.z*10, bb.maxPoint.z*10]))
        except:
            pass
    TOL = 0.4
    cols = []
    for bn, ba in beams:
        for kn, ka in keep:
            r = aabb_overlap(ba, ka)
            if r and r[0] > TOL:
                cols.append({'beam': bn, 'hits': kn, 'pen_mm': r[0],
                             'ovl': r[1], 'obj': [round(v, 1) for v in ka]})
    usb = list(USB_ENVELOPE)
    for bn, ba in beams:
        r = aabb_overlap(ba, usb)
        if r and r[0] > 0.0:
            cols.append({'beam': bn, 'hits': 'USB_ENVELOPE', 'pen_mm': r[0],
                         'ovl': r[1], 'obj': usb})
    seen = {}
    for c in cols:
        k = (c['beam'], c['hits'])
        if k not in seen or c['pen_mm'] > seen[k]['pen_mm']:
            seen[k] = c
    out['beams'] = [{'name': n, 'bb': [round(v, 1) for v in a]} for n, a in beams]
    out['collisions'] = sorted(seen.values(), key=lambda c: -c['pen_mm'])
    out['collision_count'] = len(out['collisions'])


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    out = {'ok': False, 'mode': MODE}
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        out['doc'] = app.activeDocument.name
        if MODE == 'build':
            do_build(app, design, root, out)
        elif MODE == 'verify':
            do_verify(root, out)
        elif MODE == 'export':
            opts = design.exportManager.createFusionArchiveExportOptions(EXPORT_PATH)
            design.exportManager.execute(opts)
            out['exported'] = EXPORT_PATH
            out['exists'] = os.path.exists(EXPORT_PATH)
        out['ok'] = True
    except Exception:
        out['traceback'] = traceback.format_exc()

    try:
        os.makedirs(OUT_DIR, exist_ok=True)
        with open(os.path.join(OUT_DIR, MODE + '.json'), 'w') as f:
            json.dump(out, f, indent=1)
    except Exception:
        pass

    try:
        msg = 'Claude beams [%s]: ok=%s' % (MODE, out['ok'])
        if MODE == 'build':
            msg += '\nBodies: %d   Collisions: %s' % (
                len(out.get('bodies', [])), out.get('collision_count', '?'))
        if not out['ok']:
            msg += '\n\n' + out.get('traceback', '')[-1400:]
        ui.messageBox(msg, 'Claude Beams')
    except:
        pass
