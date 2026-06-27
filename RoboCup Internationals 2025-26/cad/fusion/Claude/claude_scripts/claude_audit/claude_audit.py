"""Claude final verify: beam bodies, External1 alignment, document state."""
import adsk.core, adsk.fusion, traceback, json, os

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'out')


def vec(p):
    return [round(p.x, 5), round(p.y, 5), round(p.z, 5)]


def run(context):
    app = adsk.core.Application.get()
    out = {'ok': False}
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        out['docName'] = app.activeDocument.name
        out['docModified'] = app.activeDocument.isModified
        out['beams'] = []
        out['external1Faces'] = []
        for occ in root.occurrences:
            if occ.component.name == 'Claude Support Beams':
                for b in occ.bRepBodies:
                    bb = b.boundingBox
                    out['beams'].append({'name': b.name, 'volume_cm3': round(b.volume, 3),
                                         'bb_min': vec(bb.minPoint), 'bb_max': vec(bb.maxPoint)})
            if occ.component.name == 'External1':
                for b in occ.bRepBodies:
                    for f in b.faces:
                        g = f.geometry
                        if isinstance(g, adsk.core.Plane):
                            out['external1Faces'].append(
                                {'normal': vec(g.normal), 'origin': vec(g.origin)})
                bb = occ.boundingBox
                out['external1BB'] = [vec(bb.minPoint), vec(bb.maxPoint)]
        out['ok'] = True
    except Exception:
        out['traceback'] = traceback.format_exc()
    try:
        os.makedirs(OUT_DIR, exist_ok=True)
        with open(os.path.join(OUT_DIR, 'verify.json'), 'w') as f:
            json.dump(out, f, indent=1)
    except Exception:
        pass
