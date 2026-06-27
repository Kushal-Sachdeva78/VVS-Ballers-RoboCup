# Claude PCB connecting-beams v3 — rebuild per user feedback.
# Group A: clear Ø6 boss on each Main-PCB Teensy hole + Ø4.5 boss seating on the IR Cover,
#          symmetric outboard columns, Power PCB captured at front corners (Ø6).
# Group B: 4 posts that actually REACH the Ultrasonic plate (holes y143.6), identical
#          fixed-size blocks for a symmetric look; Ø5 at IR end, Ø6 at US end; resistor notches.
# Rear:    Power-PCB rear support — 2 legs per side from the top base (y61) up to the Power
#          underside (y85); corner legs carry the Ø6 screw.
import adsk.core, adsk.fusion, traceback

CM = 0.1
COMP = 'Claude Support Beams'

# --- Y planes (mm, Y up) ---
Y_TOPBASE = 61.05
Y_BASE_REAR = 50.05   # 'base' surface under the center-rear (top base is cut out here)
Y_MAIN    = 62.54
YB        = 81.2      # Power-capture band bottom (above motors @80.7)
YTHIN     = 83.0      # thin rail top (below terminal legs @83)
Y_PWRB    = 85.0      # Power PCB underside (boss tops)
Y_COVER   = 105.07
Y_IRB     = 116.3
Y_IRT     = 117.81
Y_USB     = 143.6      # Ultrasonic_PCB (original) mounting-plate underside
# IR Cover real Ø4.5 mounting holes (from cover-mesh boundary analysis)
COVER_L = (81.2, -29.2); COVER_R = (176.2, -29.2)

# --- verified current hole positions (world mm, after turret centering) ---
MAIN_L = (86.75, -22.97); MAIN_R = (171.75, -22.97)     # Ø6
PWR_FL = (73.0, -32.0);  PWR_FR = (184.5, -32.0)        # Ø6 front corners (Group A capture)
PWR_RL = (73.0, -89.0);  PWR_RR = (184.5, -88.5)        # Ø6 rear corners (rear legs)
# Group B IR (Ø5) and US (Ø6) holes
IR_RL=(96.67,-56.45); US_RL=(96.9,-61.13)
IR_RR=(157.58,-56.45); US_RR=(159.9,-61.13)
IR_FL=(96.62,1.55);   US_FL=(96.9,1.87)
IR_FR=(157.58,1.55);  US_FR=(159.9,1.87)
# vertical resistors crowding the IR holes (world AABB x0,x1,z0,z1; top y124.5)
RES = {'RL':(89.0,94.4,-65.0,-59.5),'RR':(158.7,164.2,-68.6,-63.1),
       'FL':(92.6,98.1,4.7,10.2),'FR':(162.3,167.8,1.1,6.6)}

# === temp-brep helpers ===
def mkbox(tbm,cx,cz,y0,y1,dx,dz):
    c=adsk.core.Point3D.create(cx*CM,(y0+y1)/2*CM,cz*CM)
    obb=adsk.core.OrientedBoundingBox3D.create(c,adsk.core.Vector3D.create(1,0,0),
        adsk.core.Vector3D.create(0,0,1),dx*CM,dz*CM,abs(y1-y0)*CM)
    return tbm.createBox(obb)
def mkcyl(tbm,x,z,y0,y1,dia):
    p1=adsk.core.Point3D.create(x*CM,y0*CM,z*CM); p2=adsk.core.Point3D.create(x*CM,y1*CM,z*CM)
    return tbm.createCylinderOrCone(p1,dia/2*CM,p2,dia/2*CM)
U=adsk.fusion.BooleanTypes.UnionBooleanType
DF=adsk.fusion.BooleanTypes.DifferenceBooleanType
def build_body(tbm,boxes,cuts,holes):
    base=None
    for bx in boxes:
        b=mkbox(tbm,*bx)
        if base is None:
            base=b
        else:
            tbm.booleanOperation(base,b,U)
    for cu in cuts:
        tbm.booleanOperation(base,mkbox(tbm,*cu),DF)
    for h in holes:
        tbm.booleanOperation(base,mkcyl(tbm,*h),DF)
    return base

# === beam specs ===  each returns (boxes, cuts, holes, name)
def groupA(side):
    # Restores the v2-style 4-corner Power-PCB capture (front + rear Ø6) plus the Main-PCB
    # Teensy hole (Ø6), and now lands the top on the IR Cover's REAL Ø4.5 hole. The column
    # stays in the clear motor band (z=-11.5); a top pad bridges over the Power terminal to
    # the cover hole.
    if side<0:
        mhx,mhz=MAIN_L; fx,fz=PWR_FL; rx,rz=PWR_RL; cvx,cvz=COVER_L; col=(71.0,-11.5); rail=63.0
        nm='Group A Beam L'
    else:
        mhx,mhz=MAIN_R; fx,fz=PWR_FR; rx,rz=PWR_RR; cvx,cvz=COVER_R; col=(183.0,-11.5); rail=195.5
        nm='Group A Beam R'
    cx,cz=col
    boxes=[]
    boxes.append((mhx,mhz,Y_MAIN,66.0,12,10))                          # Main-PCB boss (Ø6)
    ax0,ax1=sorted([mhx,cx]); az0,az1=sorted([mhz,cz])
    boxes.append(((ax0+ax1)/2,(az0+az1)/2,Y_MAIN,66.0,(ax1-ax0)+6,(az1-az0)+4)) # low arm
    boxes.append((cx,cz,Y_MAIN,Y_COVER,8,5))                           # column up to cover
    boxes.append(((cx+cvx)/2,(cz+cvz)/2,101.0,Y_COVER,abs(cx-cvx)+9,abs(cz-cvz)+9)) # cover bridge pad
    lx0,lx1=sorted([cx,fx]); lz0,lz1=sorted([cz,fz])
    boxes.append(((lx0+lx1)/2,(lz0+lz1)/2,YB,Y_PWRB,abs(lx1-lx0)+8,abs(lz1-lz0)+8)) # link->front boss
    boxes.append((fx,fz,YB,Y_PWRB,8,10))                               # front Power boss (Ø6)
    boxes.append((rail,(fz+rz)/2,YB,YTHIN,8,abs(fz-rz)+8))             # outboard thin rail front->rear
    cb0,cb1=sorted([rail,fx]); boxes.append(((cb0+cb1)/2,fz,YB,YTHIN,abs(rail-fx)+8,10))
    rb0,rb1=sorted([rail,rx]); boxes.append(((rb0+rb1)/2,rz,YB,YTHIN,abs(rail-rx)+8,10))
    boxes.append((rx,rz,YB,Y_PWRB,12,10))                              # rear Power boss (Ø6)
    holes=[(mhx,mhz,60.0,68.0,6.0),(cvx,cvz,100.0,107.0,4.5),
           (fx,fz,79.0,87.0,6.0),(rx,rz,79.0,87.0,6.0)]
    return boxes,[],holes,nm

def groupB(ir,us,res_key,nm):
    # two-tier post: narrow lower boss hugs the IR Ø5 hole (clears the crowded board parts),
    # widens above y125 (above all parts) and bridges up to the Ultrasonic plate (Ø6 @ y143.6).
    irx,irz=ir; usx,usz=us
    mx=(irx+usx)/2; mz=(irz+usz)/2
    boxes=[(irx,irz,Y_IRT,126.0,8.5,8.5),                       # lower boss at IR hole
           (mx,mz,125.0,Y_USB,11.0,12.0)]                      # upper bridge to US plate
    holes=[(irx,irz,115.0,128.0,5.0),(usx,usz,139.5,146.0,6.0)]
    x0,x1,z0,z1=RES[res_key]; M=0.8
    cuts=[((x0+x1)/2,(z0+z1)/2,Y_IRT-3.0,125.2,(x1-x0)+2*M,(z1-z0)+2*M)]
    return boxes,cuts,holes,nm

def rearLeg(x,z,nm):
    # rear Power-PCB support. The rear CORNERS are blocked by the drive gearmotors (y60-80),
    # so the legs stand in the clear central motor-gap (x 113-144). The top base is CUT OUT
    # here, so the legs land on the chassis 'base' plate at y50.05 (no longer floating).
    boxes=[(x,z,Y_BASE_REAR,Y_PWRB,9.0,10.0),(x,z,Y_BASE_REAR,Y_BASE_REAR+3,14,14)]  # leg + foot
    return boxes,[],[],nm

def rearBridge(x0,x1,z,nm):  # crossbar tying the two central legs, under the Power rear edge
    return [((x0+x1)/2,z,Y_PWRB-4,Y_PWRB,abs(x1-x0)+9,9.0)],[],[],nm

def specs():
    s=[groupA(-1),groupA(+1),
       groupB(IR_RL,US_RL,'RL','Group B Post RL'),
       groupB(IR_RR,US_RR,'RR','Group B Post RR'),
       groupB(IR_FL,US_FL,'FL','Group B Post FL'),
       groupB(IR_FR,US_FR,'FR','Group B Post FR'),
       rearLeg(122.0,-86.5,'Rear Leg L'),
       rearLeg(136.0,-86.5,'Rear Leg R'),
       rearBridge(122.0,136.0,-86.5,'Rear Bridge')]
    return s

# === clearance check (cut-aware AABB vs leaf bodies + USB envelope) ===
USB=[160.8,188.0,66.5,80.0,-31.0,-15.0]
EXCLUDE_TOKENS=['Ultrasonic_PCB 2.0']   # stale duplicate; posts intentionally pass it
def ov(a,b):
    ox=min(a[1],b[1])-max(a[0],b[0]); oy=min(a[3],b[3])-max(a[2],b[2]); oz=min(a[5],b[5])-max(a[4],b[4])
    if ox>0 and oy>0 and oz>0: return round(min(ox,oy,oz),3)
    return None
def baabb(bx):
    cx,cz,y0,y1,dx,dz=bx; return [cx-dx/2,cx+dx/2,min(y0,y1),max(y0,y1),cz-dz/2,cz+dz/2]
def covered(oa,cuts):
    for c in cuts:
        if oa[0]>=c[0]-0.05 and oa[1]<=c[1]+0.05 and oa[4]>=c[4]-0.05 and oa[5]<=c[5]+0.05 and oa[3]<=c[3]+0.05:
            return True
    return False

def run(_context):
    app=adsk.core.Application.get(); design=adsk.fusion.Design.cast(app.activeProduct); root=design.rootComponent
    if design.designType!=adsk.fusion.DesignTypes.ParametricDesignType:
        design.designType=adsk.fusion.DesignTypes.ParametricDesignType
    tbm=adsk.fusion.TemporaryBRepManager.get()
    for occ in list(root.occurrences):
        if occ.component.name==COMP: occ.deleteMe()
    sp=specs()
    newOcc=root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); comp=newOcc.component; comp.name=COMP
    bf=comp.features.baseFeatures.add(); bf.startEdit()
    made=[]
    for boxes,cuts,holes,nm in sp:
        body=build_body(tbm,boxes,cuts,holes); nb=comp.bRepBodies.add(body,bf); nb.name=nm
        made.append((nm,boxes,cuts))
    bf.finishEdit()
    print('BODIES:')
    for occ in root.occurrences:
        if occ.component.name==COMP:
            for b in occ.bRepBodies: print('  %-16s vol=%.2f cm3'%(b.name,b.volume))
    # clearance
    bboxes=[]; allcuts=[]
    for nm,boxes,cuts in made:
        for bx in boxes: bboxes.append((nm,baabb(bx)))
        for cu in cuts: allcuts.append(baabb(cu))
    cols=[]
    for occ in root.allOccurrences:
        if occ.component.name==COMP: continue
        if occ.childOccurrences.count>0: continue
        if occ.bRepBodies.count==0: continue
        fp=occ.fullPathName
        if any(t in fp for t in EXCLUDE_TOKENS): continue
        try:
            bb=occ.boundingBox
            oa=[bb.minPoint.x*10,bb.maxPoint.x*10,bb.minPoint.y*10,bb.maxPoint.y*10,bb.minPoint.z*10,bb.maxPoint.z*10]
        except: continue
        for bn,ab in bboxes:
            r=ov(ab,oa)
            if r and r>0.5 and not covered(oa,allcuts):
                cols.append((bn,occ.component.name,round(r,2)))
    for bn,ab in bboxes:
        r=ov(ab,USB)
        if r and r>0.0: cols.append((bn,'USB_ENVELOPE',round(r,2)))
    seen={}
    for c in cols:
        k=(c[0],c[1])
        if k not in seen or c[2]>seen[k]: seen[k]=c[2]
    print('COLLISIONS:',len(seen))
    for k,v in sorted(seen.items(),key=lambda x:-x[1]): print('  ',k[0],'->',k[1],'pen',v,'mm')
