"""
RC plane water-drop reservoir — parametric model (manifold3d).
Outputs: reservoir_body.stl, reservoir_lid.stl, reservoir_assembly.stl
Units: mm.  X = length (90), Y = width (100), Z = height (110).
"""
import numpy as np
from manifold3d import Manifold, CrossSection

# ---------------- parameters ----------------
L, W, H   = 90.0, 100.0, 110.0   # overall box envelope (skirt bottom -> lid top)
WALL      = 2.0                  # tank wall thickness
R_CORNER  = 6.0                  # outer corner radius
LID_T     = 3.0                  # lid plate thickness
LIP_H     = 6.0                  # lid plug depth into body
FIT       = 0.25                 # clearance per side for the lid plug
OUTLET_D  = 20.0                 # circular drop opening (inner diameter)
NOZZLE_OD = 26.0
NOZZLE_Z  = 3.0                  # nozzle tip height above ground (stays clear of table)
Z_CENTRE  = 16.0                 # inner floor height at the centre (lowest point)
Z_EDGE    = 25.0                 # inner floor height at the walls  (~12 deg slope)
ARCH_H    = 10.0                 # skirt cut-out height (leaves 4 corner feet)
FOOT      = 16.0                 # corner foot width
TABS      = True                 # mounting tabs for removable fixing (M3)
TAB_W, TAB_OUT, TAB_T, TAB_HOLE = 22.0, 10.0, 3.0, 3.4
VENT_OD, VENT_ID = 8.0, 5.0
VENT_X, VENT_Y = 22.0, 30.0      # vent position on lid
VENT_DROP = 10.0                 # anti-slosh tube length below the lid
SEG = 96

BODY_H = H - LID_T               # body rim height
# --------------------------------------------


def rrect(lx, ly, r):
    r = max(r, 0.01)
    pts = []
    for sx, sy in [(1, 1), (-1, 1), (-1, -1), (1, -1)]:
        cx, cy = sx * (lx / 2 - r), sy * (ly / 2 - r)
        for a in np.linspace(0, np.pi / 2, 12):
            ang = a + {(1, 1): 0, (-1, 1): np.pi / 2, (-1, -1): np.pi, (1, -1): 1.5 * np.pi}[(sx, sy)]
            pts.append((cx + r * np.cos(ang), cy + r * np.sin(ang)))
    return CrossSection([pts])


def prism(lx, ly, r, z0, z1):
    return Manifold.extrude(rrect(lx, ly, r), z1 - z0).translate((0, 0, z0))


def cyl(d, z0, z1, x=0, y=0):
    return Manifold.cylinder(z1 - z0, d / 2, d / 2, SEG).translate((x, y, z0))


def funnel(lx, ly, r, z_edge, d_c, z_c):
    """Pyramid-like hull from a rounded rect at z_edge down to a circle at z_c."""
    top = prism(lx, ly, r, z_edge, z_edge + 0.01)
    bot = cyl(d_c, z_c, z_c + 0.01)
    return Manifold.batch_hull([top, bot])


def capsule(p, q, d):
    s1 = Manifold.sphere(d / 2, 32).translate(tuple(p))
    s2 = Manifold.sphere(d / 2, 32).translate(tuple(q))
    return Manifold.batch_hull([s1, s2])


def tube_path(pts, d):
    return Manifold.batch_boolean([capsule(pts[i], pts[i + 1], d) for i in range(len(pts) - 1)],
                                  __import__("manifold3d").OpType.Add)


# ---------------- BODY ----------------
il, iw, ir = L - 2 * WALL, W - 2 * WALL, R_CORNER - WALL
outer = prism(L, W, R_CORNER, 0, BODY_H)

# water cavity: sloped floor + straight walls
cavity = funnel(il, iw, ir, Z_EDGE, OUTLET_D, Z_CENTRE) + prism(il, iw, ir, Z_EDGE, BODY_H + 1)

# underside void (follows floor, keeps floor ~WALL thick) + skirt space
FLOOR = WALL + 0.2   # vertical floor thickness (same slope as the inner floor)
under = prism(il, iw, ir, -1, Z_EDGE - FLOOR) \
        - funnel(il, iw, ir, Z_EDGE - FLOOR, OUTLET_D, Z_CENTRE - FLOOR)

body = outer - cavity - under

# outlet nozzle
nozzle = cyl(NOZZLE_OD, NOZZLE_Z, Z_CENTRE + 0.5) - cyl(OUTLET_D, NOZZLE_Z - 1, Z_CENTRE + 2)
body = body + nozzle
body = body - cyl(OUTLET_D, NOZZLE_Z - 1, Z_CENTRE + 2)

# nozzle stiffening ribs from nozzle to the floor underside (4 webs)
for ang in (0, 90, 180, 270):
    rib = Manifold.cube((il / 2 - NOZZLE_OD / 2 + 1, 2.0, Z_EDGE - NOZZLE_Z), False) \
        .translate((NOZZLE_OD / 2 - 1, -1.0, NOZZLE_Z + 3))
    rib = rib.rotate((0, 0, ang))
    body = body + (rib - funnel(il, iw, ir, Z_EDGE, OUTLET_D, Z_CENTRE)
                   - prism(il, iw, ir, Z_EDGE, BODY_H + 1)
                   - cyl(OUTLET_D, 0, Z_CENTRE + 2))
# skirt arches -> 4 corner feet (the "support" so it stands up)
arch_x = Manifold.cube((L - 2 * FOOT, W + 10, ARCH_H), False).translate((-(L - 2 * FOOT) / 2, -(W + 10) / 2, -1))
arch_y = Manifold.cube((L + 10, W - 2 * FOOT, ARCH_H), False).translate((-(L + 10) / 2, -(W - 2 * FOOT) / 2, -1))
body = body - arch_x - arch_y
# put the nozzle back in case arches clipped it (they shouldn't: nozzle is centred & short)
body = body + (nozzle - cyl(OUTLET_D, NOZZLE_Z - 1, Z_CENTRE + 2))

# mounting tabs on both X ends, flush with the rim
if TABS:
    for s in (1, -1):
        tab = Manifold.cube((TAB_OUT + 4, TAB_W, TAB_T), False) \
            .translate((L / 2 - 4 if s > 0 else -L / 2 - TAB_OUT, -TAB_W / 2, BODY_H - TAB_T))
        gus = Manifold.batch_hull([
            Manifold.cube((0.01, 2, 0.01), False).translate((s * L / 2, -1, BODY_H - TAB_T - 10)),
            Manifold.cube((s * TAB_OUT * 0.8 if s > 0 else 0.01, 2, TAB_T), False)
            .translate((L / 2 if s > 0 else -L / 2 - TAB_OUT * 0.8, -1, BODY_H - TAB_T)),
        ])
        tab = tab + gus.translate((0, TAB_W / 2 - 1, 0)) + gus.translate((0, -TAB_W / 2 + 1, 0))
        hole = cyl(TAB_HOLE, BODY_H - TAB_T - 1, BODY_H + 1, x=s * (L / 2 + TAB_OUT / 2))
        body = (body + tab) - hole
    body = body - prism(il, iw, ir, Z_EDGE, BODY_H + 1)   # keep cavity clean

# ---------------- LID ----------------
lid_plate = prism(L, W, R_CORNER, 0, LID_T)
pl, pw, pr = il - 2 * FIT, iw - 2 * FIT, ir - FIT
plug = prism(pl, pw, pr, -LIP_H, 0.01) - prism(pl - 2 * 1.6, pw - 2 * 1.6, max(pr - 1.6, 0.5), -LIP_H - 1, 0.02)
lid = lid_plate + plug

# Z-shaped vent: riser -> bottom bar -> diagonal -> top bar (open end)
t = LID_T
zb, zt = t + 5.0, t + 17.0          # bar centre heights
span = 12.0
P = [
    (VENT_X, VENT_Y, -VENT_DROP),      # anti-slosh tube tip inside the tank
    (VENT_X, VENT_Y, zb),              # riser top
    (VENT_X - span, VENT_Y, zb),       # bottom bar
    (VENT_X, VENT_Y, zt),              # diagonal (≈45°, printable)
    (VENT_X - span, VENT_Y, zt),       # top bar -> open inlet
]
vent_outer = tube_path(P, VENT_OD)
# inner bore: same path, extended past both ends so it opens
Pi = list(P)
Pi[0] = (VENT_X, VENT_Y, -VENT_DROP - 3)
Pi[-1] = (VENT_X - span - 6, VENT_Y, zt)
vent_inner = tube_path(Pi, VENT_ID)
# cut the outer tube's rounded end at the inlet and at the tank tip
end_cut = Manifold.cube((10, 20, 20), False).translate((VENT_X - span - 10, VENT_Y - 10, zt - 10))
tip_cut = cyl(20, -VENT_DROP - 10, -VENT_DROP, VENT_X, VENT_Y)
lid = (lid + (vent_outer - end_cut - tip_cut)) - vent_inner

# finger pull tab for lid removal
pull = Manifold.batch_hull([cyl(14, 0, LID_T, x=-L / 2 + 2, y=0)]) - prism(L, W, R_CORNER, -1, LID_T + 1)
lid = lid + pull

# ---------------- export ----------------
import trimesh


def to_tm(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts), process=True)


body_tm = to_tm(body)
lid_tm = to_tm(lid)
lid_print = lid_tm.copy()                          # lid as printed: plate on bed? (vent up) -> keep orientation
lid_print.apply_translation((0, 0, LIP_H))         # plug sits on bed, rests on lip edge
assembly = to_tm(body + lid.translate((0, 0, BODY_H)))

body_tm.export("reservoir_body.stl")
lid_tm.export("reservoir_lid.stl")
assembly.export("reservoir_assembly.stl")

for n, m in [("body", body_tm), ("lid", lid_tm), ("assembly", assembly)]:
    b = m.bounds
    print(f"{n:9s} watertight={m.is_watertight} size={np.round(b[1]-b[0],1)} vol={m.volume/1000:.1f} cm3")

water = (prism(il, iw, ir, Z_EDGE, BODY_H - LIP_H) + funnel(il, iw, ir, Z_EDGE, OUTLET_D, Z_CENTRE)).volume()
print(f"water capacity ≈ {water/1000:.0f} ml")