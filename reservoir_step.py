"""
1 L water-drop reservoir, plug valve + lid lever + MG995 mounted on the tank's rear wall.
CadQuery B-rep model -> STEP (assembly + printable parts) and STL.

Frame (mm): origin = centre of the outlet at the bottom of the feet.
 +X = toward the servo end (aft in the aircraft), +Y = left/right width, +Z = up.
"""
import math
import cadquery as cq

# =========================== PARAMETERS ===========================
L, W, H      = 144.0, 92.0, 95.0      # tank envelope: feet bottom -> lid top
T            = 1.2                    # wall / floor / lid thickness
R_OUT        = 6.0                    # outer vertical corner radius
FEET_H       = 4.0                    # floor underside height (ribs + skirt stand on z = 0)
LID_T        = T
GASKET_T     = 1.0                    # silicone / EVA gasket between rim and lid
BODY_H       = H - LID_T - GASKET_T   # 92.8 rim height (lid top stays at 95)
LID_Z0       = BODY_H + GASKET_T      # lid underside
LEDGE_W, LEDGE_T, LEDGE_CH = 4.0, 1.6, 4.0   # inner rim ledge: width, flat thickness, 45 deg chamfer below
SCREWS       = [(-67.0, -41.0), (-67.0, 41.0), (67.0, -41.0), (67.0, 41.0),
                (-35.0, -41.6), (-35.0, 41.6), (35.0, -41.6), (35.0, 41.6)]   # 8 x M2
BOSS_D, BOSS_DEPTH, PILOT_D, PILOT_DEPTH = 6.0, 12.0, 1.7, 9.0   # pilot for M2 self-tap (3.2 for heat-set insert)
LID_HOLE_D   = 2.4
RIB_W, RIB_H = 6.0, 1.5               # stiffening rib on the lid edge (screw heads sit on it)
FILL_LINE_Z  = 88.8
FLOOR_Z      = FEET_H + T             # 5.2 flat inner floor = valve seat
# outlet / plug valve
OUT_D        = 20.0
SPOUT_OD, SPOUT_Z = 24.0, 0.5
PLUG_D, PLUG_T = 32.0, 5.0
ORING_PLUG   = (22.0, 2.0)            # ID x CS  (plug face seal)
GROOVE_D     = 1.5
LIFT         = 7.0                    # plug lift (gap area 440 mm2 = 1.4 x hole)
ROD_D        = 2.0
GUIDE_OD, GUIDE_ID = 6.0, 2.6
# lever on the lid
PIVOT_X      = 68.0                   # pivot bracket near the rear edge of the lid
LEVER_Z      = 98.0                   # lever centre-line height (lid top = 95)
LEVER_T, LEVER_HT = 3.0, 3.0          # lever thickness (Y) and height (Z)
ROD_X        = 0.0                    # rod sits over the outlet
# fill port
FILL_X, FILL_BORE, FILL_COLLAR_OD, FILL_COLLAR_H = -40.0, 24.0, 30.0, 3.0
ORING_CAP    = (20.0, 2.0)
# vent (low Z-labyrinth)
VENT_X, VENT_Y, VENT_OD, VENT_ID = -52.0, 30.0, 5.0, 3.0
# MG995 (manufacturer size 40.7 x 19.7 x 42.9; tab details = standard-size servo, verify on yours)
SV_L, SV_W, SV_CASE_H, SV_TOTAL_H = 40.7, 19.7, 37.0, 42.9
SV_TAB_SPAN, SV_TAB_T, SV_TAB_FROM_BOTTOM = 54.0, 2.5, 26.7
SV_HOLE_PITCH_L, SV_HOLE_PITCH_W, SV_HOLE_D = 49.5, 10.0, 4.5
SV_SHAFT_FROM_END = 10.2
HORN_R       = 14.0
SV_GAP       = 2.0                    # gap tank wall -> servo case
SV_HORN_Y    = 6.0                    # horn plane
# ==================================================================

IL, IW, IR = L - 2 * T, W - 2 * T, R_OUT - T
SV_X0 = L / 2 + SV_GAP                         # servo case, X from SV_X0 to SV_X0 + SV_W
SV_XC = SV_X0 + SV_W / 2
SV_Z0 = SV_TAB_SPAN / 2 - SV_L / 2 + 2.5       # case bottom (tabs clear the 2 mm cradle base by 0.5)
SV_ZC = SV_Z0 + SV_L / 2
SHAFT_Z = SV_Z0 + SV_L - SV_SHAFT_FROM_END
SV_YB = SV_HORN_Y - (SV_TOTAL_H - 1.5)         # case bottom face (Y)
TAB_Y0 = SV_YB + SV_TAB_FROM_BOTTOM
HORN_TIP_X = SV_XC + HORN_R
REAR_X = HORN_TIP_X                             # lever rear hole over the horn tip
PLUG_TOP = FLOOR_Z + PLUG_T + 4.0               # top of rod boss (closed)
GUIDE_BOT = PLUG_TOP + LIFT + 3.0
FRONT_RATIO = (PIVOT_X - ROD_X) / (REAR_X - PIVOT_X)
PUSHROD_TRAVEL = LIFT / FRONT_RATIO
HORN_SWING = math.degrees(math.asin(PUSHROD_TRAVEL / 2 / HORN_R)) * 2


def rbox(lx, ly, r, z0, z1, x=0, y=0):
    s = cq.Workplane("XY").box(lx, ly, z1 - z0, centered=(True, True, False))
    if r > 0:
        s = s.edges("|Z").fillet(r)
    return s.translate((x, y, z0))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl(d, z0, z1, x=0, y=0):
    return cq.Workplane("XY").circle(d / 2).extrude(z1 - z0).translate((x, y, z0))


def ycyl(d, y0, y1, x, z):
    return cq.Workplane("XZ").circle(d / 2).extrude(-(y1 - y0)).translate((x, y0, z))


def torus(R, r, x, y, z):
    return cq.Workplane("XZ").center(R, 0).circle(r).revolve(360, (-R, 0, 0), (-R, 1, 0)).translate((x, y, z))


def tube_between(p, q, d):
    import cadquery as _cq
    v = _cq.Vector(*q) - _cq.Vector(*p)
    s = _cq.Solid.makeCylinder(d / 2, v.Length, _cq.Vector(*p), v.normalized())
    return cq.Workplane("XY").add(s)


# ============================ BODY ============================
def make_body():
    b = rbox(L, W, R_OUT, 0, BODY_H)
    b = b.cut(rbox(IL, IW, IR, FLOOR_Z, BODY_H + 1))           # water cavity, flat floor
    b = b.cut(rbox(IL, IW, IR, -1, FEET_H))                    # underside -> skirt 1.2 mm
    # under-floor ribs (stiffen the 1.2 mm floor, stand on the bay floor)
    for y in (-22.0, 22.0):
        b = b.union(box(-IL / 2, IL / 2, y - T / 2, y + T / 2, 0, FEET_H + 0.01))
    for x in (-36.0, 36.0):
        b = b.union(box(x - T / 2, x + T / 2, -IW / 2, IW / 2, 0, FEET_H + 0.01))
    # spout + outlet hole
    b = b.union(cyl(SPOUT_OD, SPOUT_Z, FEET_H + 0.01))
    b = b.cut(cyl(OUT_D, -1, FLOOR_Z + 1))
    # drain/air notches in the skirt (middle of each side)
    b = b.cut(box(-15, 15, -W, W, -1, 2.5)).cut(box(-L, L, -12, 12, -1, 2.5))
    # ---- MG995 cradle on the rear wall
    py0, py1 = TAB_Y0 + SV_TAB_T, TAB_Y0 + SV_TAB_T + 2.5     # plate behind the tabs (shaft side)
    px0, px1 = L / 2 - 1.0, SV_X0 + SV_W + 4.0
    plate = box(px0, px1, py0, py1, 0, SV_Z0 + SV_L + 8.0)
    plate = plate.cut(box(SV_X0 - 0.2, SV_X0 + SV_W + 0.2, py0 - 1, py1 + 1, SV_Z0 - 0.2, SV_Z0 + SV_L + 0.2))
    for dz in (-SV_HOLE_PITCH_L / 2, SV_HOLE_PITCH_L / 2):
        for dx in (-SV_HOLE_PITCH_W / 2, SV_HOLE_PITCH_W / 2):
            plate = plate.cut(ycyl(2.5, py0 - 1, py1 + 1, SV_XC + dx, SV_ZC + dz))   # M3 self-tap pilots
    base = box(L / 2 - 1.0, px1, SV_YB - 1.0, py1, 0, 2.0)
    gusset = box(px1 - 2.0, px1, SV_YB - 1.0, py0, 0, 30.0)
    wall_pad = box(L / 2 - 1.0, L / 2 + 0.5, SV_YB - 1.0, py1, 0, SV_Z0 + SV_L + 8.0)
    b = b.union(plate).union(base).union(gusset).union(wall_pad)
    b = b.cut(rbox(IL, IW, IR, FLOOR_Z, BODY_H + 1))          # keep the cavity clean
    # inner rim ledge (gasket + screw face), 45 deg chamfer underneath -> prints without support
    zt = BODY_H - LEDGE_T
    flat = rbox(IL, IW, IR, zt, BODY_H).cut(rbox(IL - 2 * LEDGE_W, IW - 2 * LEDGE_W, 1.0, zt - 1, BODY_H + 1))
    frustum = (cq.Workplane("XY").workplane(offset=zt - LEDGE_CH - 1).rect(IL + 2, IW + 2)
               .workplane(offset=LEDGE_CH + 2).rect(IL - 2 * LEDGE_W - 2, IW - 2 * LEDGE_W - 2).loft())   # 45 deg, oversize both ends
    chamfer = rbox(IL, IW, IR, zt - LEDGE_CH, zt).cut(frustum)
    b = b.union(flat).union(chamfer)
    # screw bosses with 45 deg cone underneath
    for x, y in SCREWS:
        boss = cyl(BOSS_D, BODY_H - BOSS_DEPTH, BODY_H, x, y)
        cone = cq.Workplane("XY").circle(BOSS_D / 2).workplane(offset=-BOSS_D / 2).circle(0.3).loft().translate((x, y, BODY_H - BOSS_DEPTH))
        b = b.union(boss.union(cone).intersect(rbox(IL + 0.5, IW + 0.5, IR, 0, BODY_H)))
        b = b.cut(cyl(PILOT_D, BODY_H - PILOT_DEPTH, BODY_H + 1, x, y))
    return b


# ============================ LID ============================
def make_lid():
    z0 = LID_Z0
    lid = rbox(L, W, R_OUT, z0, H)
    rib = rbox(L, W, R_OUT, H - 0.01, H + RIB_H).cut(rbox(L - 2 * RIB_W, W - 2 * RIB_W, 1.0, H - 1, H + RIB_H + 1))
    rib = rib.cut(box(L / 2 - RIB_W - 1, L / 2 + 1, -6, 6, H - 0.5, H + RIB_H + 1))   # clearance under the lever
    lid = lid.union(rib)
    for x, y in SCREWS:
        lid = lid.cut(cyl(LID_HOLE_D, z0 - 1, H + RIB_H + 1, x, y))
    # rod guide tube
    lid = lid.union(cyl(GUIDE_OD, GUIDE_BOT, H, ROD_X, 0)).cut(cyl(GUIDE_ID, GUIDE_BOT - 1, H + 1, ROD_X, 0))
    # pivot bracket (two cheeks, M2 pin hole)
    for s in (-1, 1):
        y0 = s * (LEVER_T / 2 + 0.3)
        y1 = s * (LEVER_T / 2 + 0.3 + 2.0)
        cheek = box(PIVOT_X - 4, PIVOT_X + 4, min(y0, y1), max(y0, y1), H - 0.01, LEVER_Z + 3.0)
        lid = lid.union(cheek)
    lid = lid.cut(ycyl(2.1, -6, 6, PIVOT_X, LEVER_Z))
    # fill port collar + bore
    lid = lid.union(cyl(FILL_COLLAR_OD, H - 0.01, H + FILL_COLLAR_H, FILL_X, 0))
    lid = lid.union(cyl(FILL_BORE + 2 * T, z0 - 3.0, z0 + 0.01, FILL_X, 0))
    lid = lid.cut(cyl(FILL_BORE, z0 - 4, H + FILL_COLLAR_H + 1, FILL_X, 0))
    # low Z-vent: riser -> bottom bar -> diagonal -> top bar (opening faces -X)
    zb, zt, sp = H + 2.2, H + 6.2, 5.0     # bottom bar sinks 0.3 mm into the lid (no tangent faces)
    pts = [(VENT_X, VENT_Y, z0 - 4.0), (VENT_X, VENT_Y, zb), (VENT_X + sp, VENT_Y, zb), (VENT_X, VENT_Y, zt), (VENT_X - 3.0, VENT_Y, zt)]
    for p, q in zip(pts[:-1], pts[1:]):
        lid = lid.union(tube_between(p, q, VENT_OD))
        lid = lid.union(cq.Workplane("XY").sphere(VENT_OD / 2).translate(q))
    lid = lid.union(cq.Workplane("XY").sphere(VENT_OD / 2).translate(pts[0]))
    bore = [(VENT_X, VENT_Y, z0 - 6.0)] + pts[1:-1] + [(VENT_X - 8.0, VENT_Y, zt)]
    for p, q in zip(bore[:-1], bore[1:]):
        lid = lid.cut(tube_between(p, q, VENT_ID))
        lid = lid.cut(cq.Workplane("XY").sphere(VENT_ID / 2).translate(q))
    lid = lid.cut(box(VENT_X - 12, VENT_X - 3.0, VENT_Y - 5, VENT_Y + 5, zt - 5, zt + 5))   # open the mouth
    lid = lid.cut(cyl(10, z0 - 20, z0 - 4.0, VENT_X, VENT_Y))                              # open the tip
    # finger notch on the front edge
    lid = lid.cut(box(-L / 2 - 1, -L / 2 + 4, -8, 8, H - 0.6, H + 1))
    return lid


# ============================ SMALL PARTS ============================
def make_plug():
    p = cyl(PLUG_D, FLOOR_Z, FLOOR_Z + PLUG_T)
    oid, cs = ORING_PLUG
    p = p.cut(cyl(oid + 2 * cs, FLOOR_Z - 1, FLOOR_Z + GROOVE_D).cut(cyl(oid, FLOOR_Z - 2, FLOOR_Z + GROOVE_D + 1)))
    nose = cq.Workplane("XY").circle((OUT_D - 0.8) / 2).workplane(offset=-4.0).circle((OUT_D - 6) / 2).loft().translate((0, 0, FLOOR_Z))
    p = p.union(nose)
    p = p.union(cyl(8, FLOOR_Z + PLUG_T - 0.01, PLUG_TOP))
    p = p.cut(cyl(1.7, PLUG_TOP - 8, PLUG_TOP + 1))       # tap M2 or epoxy 2 mm rod
    return p


def make_lever():
    x0, x1 = ROD_X - 4.0, REAR_X + 4.0
    lv = box(x0, x1, -LEVER_T / 2, LEVER_T / 2, LEVER_Z - LEVER_HT / 2, LEVER_Z + LEVER_HT / 2)
    lv = lv.edges("|Y").fillet(1.2)
    lv = lv.union(ycyl(5.4, -LEVER_T / 2, LEVER_T / 2, PIVOT_X, LEVER_Z))
    for x in (ROD_X, PIVOT_X, REAR_X):
        lv = lv.cut(ycyl(2.1, -5, 5, x, LEVER_Z))
    return lv


def make_cap():
    oid, cs = ORING_CAP
    top_z = H + FILL_COLLAR_H
    c = cyl(FILL_COLLAR_OD, top_z, top_z + 1.6, FILL_X, 0)
    c = c.union(cyl(FILL_BORE - 0.4, top_z - 6.0, top_z + 0.01, FILL_X, 0))
    c = c.cut(cyl(FILL_BORE + 1, top_z - 4.6, top_z - 2.2, FILL_X, 0).cut(cyl(oid + 0.6, top_z - 5, top_z - 2, FILL_X, 0)))
    c = c.union(box(FILL_X - 2, FILL_X + 2, -10, 10, top_z + 1.6, top_z + 5.0))      # grip tab
    c = c.cut(cyl(FILL_BORE - 3.6, top_z - 7, top_z - 1.0, FILL_X, 0))                 # hollow
    return c


def make_gasket():
    g = rbox(L, W, R_OUT, BODY_H, BODY_H + GASKET_T).cut(rbox(IL - 2 * LEDGE_W, IW - 2 * LEDGE_W, 1.0, BODY_H - 1, BODY_H + 2))
    for x, y in SCREWS:
        g = g.cut(cyl(LID_HOLE_D, BODY_H - 1, BODY_H + 2, x, y))
    return g


def make_screws():
    out = None
    for x, y in SCREWS:
        sc = cyl(2.0, BODY_H - 6.0, H + RIB_H, x, y).union(cyl(3.8, H + RIB_H, H + RIB_H + 1.6, x, y))
        out = sc if out is None else out.union(sc)
    return out


def make_servo():
    case = box(SV_X0, SV_X0 + SV_W, SV_YB, SV_YB + SV_CASE_H, SV_Z0, SV_Z0 + SV_L)
    tabs = box(SV_X0, SV_X0 + SV_W, TAB_Y0, TAB_Y0 + SV_TAB_T, SV_ZC - SV_TAB_SPAN / 2, SV_ZC + SV_TAB_SPAN / 2)
    for dz in (-SV_HOLE_PITCH_L / 2, SV_HOLE_PITCH_L / 2):
        for dx in (-SV_HOLE_PITCH_W / 2, SV_HOLE_PITCH_W / 2):
            tabs = tabs.cut(ycyl(SV_HOLE_D, TAB_Y0 - 1, TAB_Y0 + SV_TAB_T + 1, SV_XC + dx, SV_ZC + dz))
    boss = ycyl(12.0, SV_YB + SV_CASE_H - 0.01, SV_YB + SV_CASE_H + 2.5, SV_XC, SHAFT_Z)
    spline = ycyl(6.0, SV_YB + SV_CASE_H, SV_YB + SV_TOTAL_H, SV_XC, SHAFT_Z)
    return case.union(tabs).union(boss).union(spline)


def make_horn():
    y0, y1 = SV_HORN_Y - 1.5, SV_HORN_Y + 1.5
    hub = ycyl(9.0, y0, y1, SV_XC, SHAFT_Z)
    arm = box(SV_XC, HORN_TIP_X + 3, y0, y1, SHAFT_Z - 3, SHAFT_Z + 3)
    return hub.union(arm).cut(ycyl(1.6, y0 - 1, y1 + 1, HORN_TIP_X, SHAFT_Z))


def make_rods():
    rod = cyl(ROD_D, PLUG_TOP - 8, LEVER_Z, ROD_X, 0)
    push = tube_between((REAR_X, SV_HORN_Y, SHAFT_Z), (REAR_X, 0.0, LEVER_Z), 1.5)
    pin = ycyl(2.0, -(LEVER_T / 2 + 2.6), LEVER_T / 2 + 2.6, PIVOT_X, LEVER_Z)
    return rod, push, pin


def make_orings():
    oid, cs = ORING_PLUG
    o1 = torus(oid / 2 + cs / 2, cs / 2, 0, 0, FLOOR_Z + GROOVE_D - cs / 2)
    oid2, cs2 = ORING_CAP
    o2 = torus(oid2 / 2 + 0.3 + cs2 / 2, cs2 / 2, FILL_X, 0, H + FILL_COLLAR_H - 3.4)
    return o1, o2


if __name__ == "__main__":
    import json
    parts = {
        "tank_body": (make_body(), (0.35, 0.70, 0.90)),
        "lid": (make_lid(), (0.60, 0.80, 0.95)),
        "plug": (make_plug(), (0.94, 0.51, 0.0)),
        "lever": (make_lever(), (0.94, 0.51, 0.0)),
        "fill_cap": (make_cap(), (0.94, 0.51, 0.0)),
        "gasket_1mm": (make_gasket(), (0.85, 0.2, 0.2)),
        "screws_8x_M2x10": (make_screws(), (0.6, 0.6, 0.6)),
        "MG995_servo": (make_servo(), (0.15, 0.15, 0.18)),
        "servo_horn": (make_horn(), (0.95, 0.95, 0.95)),
    }
    rod, push, pin = make_rods()
    o1, o2 = make_orings()
    parts.update({"rod_2mm_steel": (rod, (0.6, 0.6, 0.6)), "pushrod_1.5mm_steel": (push, (0.6, 0.6, 0.6)),
                  "pivot_pin_M2": (pin, (0.6, 0.6, 0.6)), "oring_plug_22x2": (o1, (0.1, 0.1, 0.1)),
                  "oring_cap_20x2": (o2, (0.1, 0.1, 0.1))})
    asm = cq.Assembly(name="water_reservoir_1L")
    for n, (s, c) in parts.items():
        asm.add(s, name=n, color=cq.Color(*c))
    asm.save("reservoir_MG995_assembly.step")
    for n in ("tank_body", "lid", "plug", "lever", "fill_cap", "gasket_1mm"):
        s = parts[n][0]
        cq.exporters.export(s, f"print_{n}.step")
        cq.exporters.export(s, f"print_{n}.stl", tolerance=0.05, angularTolerance=0.1)
        import trimesh as _tm                                   # drop zero-area pole triangles from the mesh
        _m = _tm.load(f"print_{n}.stl"); _m.merge_vertices(digits_vertex=4)
        _m.update_faces(_m.nondegenerate_faces()); _m.remove_unreferenced_vertices(); _m.export(f"print_{n}.stl")
    for n, (s, c) in parts.items():
        cq.exporters.export(s, f"_mesh_{n}.stl", tolerance=0.05, angularTolerance=0.15)
    info = {n: dict(volume_cm3=round(s.val().Volume() / 1000, 2),
                    bbox=[round(v, 2) for v in (s.val().BoundingBox().xmin, s.val().BoundingBox().xmax,
                                                 s.val().BoundingBox().ymin, s.val().BoundingBox().ymax,
                                                 s.val().BoundingBox().zmin, s.val().BoundingBox().zmax)])
            for n, (s, c) in parts.items()}
    # water capacity: cavity up to the bottom of the lid plug, minus everything that hangs into it
    fill_z = FILL_LINE_Z
    water = rbox(IL - 0.01, IW - 0.01, IR, FLOOR_Z, fill_z)
    for n in ("tank_body", "lid", "plug", "rod_2mm_steel"):
        water = water.cut(parts[n][0])
    info["_water_ml_to_lid_plug"] = round(water.val().Volume() / 1000, 1)
    info["_fill_line_z"] = fill_z
    info["_geometry"] = dict(lift=LIFT, lever_ratio_front_rear=round(FRONT_RATIO, 2), pushrod_travel=round(PUSHROD_TRAVEL, 2),
                             horn_swing_deg=round(HORN_SWING, 1), shaft_z=round(SHAFT_Z, 2), servo_case_x=(SV_X0, SV_X0 + SV_W),
                             servo_case_y=(round(SV_YB, 2), round(SV_YB + SV_CASE_H, 2)), servo_case_z=(round(SV_Z0, 2), round(SV_Z0 + SV_L, 2)),
                             guide_bottom_z=GUIDE_BOT, plug_top_z=PLUG_TOP, rear_hole_x=REAR_X,
                             lever_top_open_front=round(LEVER_Z + LEVER_HT / 2 + LIFT, 2))
    json.dump(info, open("reservoir_info.json", "w"), indent=1)
    print(json.dumps(info, indent=1))
