"""
RC water-drop reservoir v4 - PLUG (poppet) valve operated from the lid.
A servo in a dry well in the lid lifts a rod; the rod lifts a plug off the outlet.
Water pressure pushes the plug down onto an O-ring -> the fuller the tank, the tighter the seal.
Outputs: plug_body.stl, plug_lid.stl, plug_plug.stl, plug_assembly.stl
Units mm. X = length, Y = width, Z = up. Outlet at X=0, Y=0.
"""
import numpy as np
from manifold3d import Manifold, CrossSection, OpType

# ======================= PARAMETERS =======================
L, W, H   = 144.0, 94.0, 89.0    # overall box (feet bottom -> lid top). Vent tube rises above this.
WALL      = 2.0
R_CORNER  = 6.0
LID_T     = 2.0
LIP_H     = 6.0                  # lid plug depth into the body
FIT       = 0.25
OUTLET_D  = 20.0                 # drop hole
SEAT_D    = 34.0                 # flat seat around the hole (bottom of the sloped floor)
Z_CENTRE  = 6.0                  # floor height at the outlet (top of seat)
Z_EDGE    = 13.0                 # floor height at the walls (slope 6-9 deg)
NOZZLE_OD = 26.0
NOZZLE_Z  = 1.0                  # spout tip height (feet stand at z = 0)
ARCH_H    = 4.0                  # cut-outs between the 4 corner feet
FOOT      = 18.0
# --- plug ---
PLUG_D, PLUG_T = 32.0, 5.0
ORING_ID, ORING_CS = 22.0, 2.0   # O-ring 22 x 2 mm in a groove on the plug underside
GROOVE_D  = 1.5                  # -> ~25 % squeeze when the plug sits on the seat
LIFT      = 10.0                 # plug lift when open (flow area 2x the hole)
ROD_D     = 2.0                  # 2 mm steel rod / M2 threaded rod
# --- servo well in the lid (SG90 / MG90S lying on its side, shaft along Y) ---
SV_L, SV_W, SV_H = 22.8, 12.2, 22.5
SV_TAB_SPAN, SV_TAB_T, SV_TAB_FROM_FACE = 32.3, 2.5, 4.1
SV_HOLES, SV_SHAFT_OFF = 27.5, 5.9
HORN_R, HORN_SWING = 10.0, 30.0  # horn radius & +/- swing (lift = 2 R sin(swing))
WELL_DEPTH = 11.0                # well floor below the body rim = max fill level
WELL_WALL  = 1.6
# --- Z-vent ---
VENT_OD, VENT_ID = 6.0, 4.0
VENT_X, VENT_Y   = 45.0, 25.0
VENT_DROP        = 6.0
VENT_SPAN, VENT_RISE = 6.0, 6.0
SEG = 96
# ==========================================================
BODY_H = H - LID_T
il, iw, ir = L - 2 * WALL, W - 2 * WALL, R_CORNER - WALL
FILL_Z = BODY_H - WELL_DEPTH     # max water level (well floor / guide-tube top)


def rrect(lx, ly, r):
    r = max(r, 0.01); pts = []
    for k, (sx, sy) in enumerate([(1, 1), (-1, 1), (-1, -1), (1, -1)]):
        cx, cy = sx * (lx / 2 - r), sy * (ly / 2 - r)
        for a in np.linspace(0, np.pi / 2, 12):
            pts.append((cx + r * np.cos(a + k * np.pi / 2), cy + r * np.sin(a + k * np.pi / 2)))
    return CrossSection([pts])

def prism(lx, ly, r, z0, z1):
    return Manifold.extrude(rrect(lx, ly, r), z1 - z0).translate((0, 0, z0))

def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0), False).translate((x0, y0, z0))

def cyl(d, z0, z1, x=0, y=0):
    return Manifold.cylinder(z1 - z0, d / 2, d / 2, SEG).translate((x, y, z0))

def cone(d0, d1, z0, z1, x=0, y=0):
    return Manifold.cylinder(z1 - z0, d0 / 2, d1 / 2, SEG).translate((x, y, z0))

def funnel(lx, ly, r, z_edge, d_c, z_c):
    return Manifold.batch_hull([prism(lx, ly, r, z_edge, z_edge + 0.01), cyl(d_c, z_c, z_c + 0.01)])

def capsule(p, q, d):
    return Manifold.batch_hull([Manifold.sphere(d / 2, 32).translate(tuple(p)), Manifold.sphere(d / 2, 32).translate(tuple(q))])

def tube_path(pts, d):
    return Manifold.batch_boolean([capsule(pts[i], pts[i + 1], d) for i in range(len(pts) - 1)], OpType.Add)


# ================================ BODY ================================
FLOOR = WALL + 0.2
outer = prism(L, W, R_CORNER, 0, BODY_H)
cavity = funnel(il, iw, ir, Z_EDGE, SEAT_D, Z_CENTRE) + prism(il, iw, ir, Z_EDGE, BODY_H + 1)
under = prism(il, iw, ir, -1, Z_EDGE - FLOOR) - funnel(il, iw, ir, Z_EDGE - FLOOR, SEAT_D, Z_CENTRE - FLOOR)
body = outer - cavity - under
body = body + (cyl(NOZZLE_OD, NOZZLE_Z, Z_CENTRE) - cavity)          # short spout protects the hole edge
body = body - cyl(OUTLET_D, -1, Z_CENTRE + 1)
skirt = outer - prism(il, iw, ir, -2, BODY_H + 2)
arch_x = box(-(L - 2 * FOOT) / 2, (L - 2 * FOOT) / 2, -W, W, -1, ARCH_H)
arch_y = box(-L, L, -(W - 2 * FOOT) / 2, (W - 2 * FOOT) / 2, -1, ARCH_H)
body = body - (arch_x ^ skirt) - (arch_y ^ skirt)

# ================================ PLUG ================================
def make_plug():
    p = cyl(PLUG_D, 0, PLUG_T)
    p = p - (cyl(ORING_ID + 2 * ORING_CS, -1, GROOVE_D) - cyl(ORING_ID, -2, GROOVE_D + 1))   # O-ring groove
    p = p + cone(OUTLET_D - 0.8, OUTLET_D - 6, -4.0, 0.01)                                    # self-centring nose
    p = p + cyl(8, PLUG_T - 0.01, PLUG_T + 4)                                                 # rod boss
    p = p - cyl(1.7, PLUG_T - 4, PLUG_T + 5)                                                  # tap M2 / glue 2 mm rod
    return p
plug = make_plug()                      # local: z=0 is the sealing face
PLUG_CLOSED_Z = Z_CENTRE                # sealing face sits on the seat (O-ring squeezed into groove)

# ================================ LID ================================
# lid local frame: z=0 = underside of lid plate = body rim (absolute BODY_H)
lid = prism(L, W, R_CORNER, 0, LID_T)
pl, pw, pr = il - 2 * FIT, iw - 2 * FIT, ir - FIT
lid = lid + prism(pl, pw, pr, -LIP_H, 0.01) - prism(pl - 3.2, pw - 3.2, max(pr - 1.6, 0.5), -LIP_H - 1, 0.02)

# servo geometry (lid frame)
rod_x, rod_y = 0.0, 0.0
SX = rod_x - HORN_R * np.cos(np.radians(HORN_SWING))     # shaft X so the horn tip is over the rod
SZ = LID_T - SV_W / 2 - 0.1                              # shaft height: servo top flush with lid top
shaft_face_y = 3.5                                       # horn plane ~ y = 0, servo body toward +y
sv_x0, sv_x1 = SX - (SV_L - SV_SHAFT_OFF), SX + SV_SHAFT_OFF
sv_cx = (sv_x0 + sv_x1) / 2
tab_y0 = shaft_face_y + SV_TAB_FROM_FACE
well_floor = -WELL_DEPTH
wx0, wx1 = sv_cx - SV_TAB_SPAN / 2 - 1.0, rod_x + 4.5
wy0, wy1 = -5.0, shaft_face_y + SV_H + 1.0
well_in = box(wx0, wx1, wy0, wy1, well_floor, LID_T + 1)
well_out = box(wx0 - WELL_WALL, wx1 + WELL_WALL, wy0 - WELL_WALL, wy1 + WELL_WALL, well_floor - WELL_WALL, LID_T)
lid = lid + well_out - well_in
# servo mounting plate (tabs screw to it from the horn side), window open at the top so the servo drops in
plate = box(wx0, wx1, tab_y0 + SV_TAB_T, tab_y0 + SV_TAB_T + 2.5, well_floor, LID_T)
plate = plate - box(sv_x0 - 0.2, sv_x1 + 0.2, tab_y0, tab_y0 + 6, SZ - SV_W / 2 - 0.2, LID_T + 1)
for dx in (-SV_HOLES / 2, SV_HOLES / 2):
    plate = plate - Manifold.cylinder(12, 0.9, 0.9, 24).rotate((-90, 0, 0)).translate((sv_cx + dx, tab_y0 - 1, SZ))
lid = lid + plate
# rod guide tube: from the well floor down toward the plug (stops above the lifted plug)
guide_bot = (Z_CENTRE + PLUG_T + 4 + LIFT + 3) - BODY_H
lid = lid + cyl(6.5, guide_bot, well_floor + 0.01, rod_x, rod_y)
lid = lid - cyl(ROD_D + 0.6, guide_bot - 1, LID_T + 2, rod_x, rod_y)
# Z-shaped air vent
zb, zt = LID_T + 3.5, LID_T + 3.5 + VENT_RISE
P = [(VENT_X, VENT_Y, -VENT_DROP), (VENT_X, VENT_Y, zb), (VENT_X - VENT_SPAN, VENT_Y, zb), (VENT_X, VENT_Y, zt), (VENT_X - VENT_SPAN, VENT_Y, zt)]
Pi = list(P); Pi[0] = (VENT_X, VENT_Y, -VENT_DROP - 3); Pi[-1] = (VENT_X - VENT_SPAN - 6, VENT_Y, zt)
end_cut = box(VENT_X - VENT_SPAN - 10, VENT_X - VENT_SPAN, VENT_Y - 10, VENT_Y + 10, zt - 10, zt + 10)
lid = (lid + (tube_path(P, VENT_OD) - end_cut - cyl(20, -VENT_DROP - 10, -VENT_DROP, VENT_X, VENT_Y))) - tube_path(Pi, VENT_ID)
lid = lid - box(L / 2 - 4, L / 2 + 1, -8, 8, LID_T - 1.2, LID_T + 1)         # finger notch

# ================================ EXPORT ================================
if __name__ == "__main__":
    import trimesh

    def to_tm(m):
        mm = m.to_mesh()
        return trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts), process=True)

    lid_abs = lid.translate((0, 0, BODY_H))
    plug_abs = plug.translate((0, 0, PLUG_CLOSED_Z))
    for n, m in (("body", body), ("lid", lid), ("plug", plug)):
        t = to_tm(m)
        if n == "lid":
            t.apply_transform(trimesh.transformations.rotation_matrix(np.pi, (1, 0, 0)))   # print upside down: flat top on bed
        t.apply_translation((0, 0, -t.bounds[0][2]))
        t.export(f"plug_{n}.stl")
        print(f"{n:5s} watertight={t.is_watertight} size={np.round(t.bounds[1]-t.bounds[0],1)} vol={m.volume()/1000:.1f} cm3")
    asm = to_tm(body + lid_abs + plug_abs); asm.export("plug_assembly.stl")
    print("assembly size", np.round(asm.bounds[1] - asm.bounds[0], 1))
    water = (cavity ^ box(-L, L, -W, W, -5, FILL_Z)) - plug_abs - lid_abs
    print(f"water capacity to fill line (well floor, {FILL_Z:.0f} mm above feet) ~ {water.volume()/1000:.0f} ml")
