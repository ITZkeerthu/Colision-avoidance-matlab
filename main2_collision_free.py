#!/usr/bin/env python3
"""
Indian Road Autonomous Driving Simulation — 10 Scenarios + GLB
"""

import pygame
import math
import random
import sys
import heapq
import os
import numpy as np
from typing import List, Tuple, Optional, Dict
from pygltflib import GLTF2

pygame.init()

# ── CONSTANTS ──────────────────────────────────────────────
SW, SH = 1400, 900
WW, WH = 5000, 4000
FPS = 60
GS = 25
GW, GH = WW // GS, WH // GS
CONN_DIST = 130
DET_R = 160

BLACK = (0, 0, 0); WHITE = (255, 255, 255); GRAY = (128, 128, 128)
DGRAY = (55, 55, 60); LGRAY = (190, 190, 190); RED = (220, 50, 50)
GREEN = (50, 180, 50); BLUE = (50, 100, 220); YELLOW = (230, 210, 50)
ORANGE = (220, 140, 40); BROWN = (139, 90, 43); DGREEN = (34, 100, 34)
CYAN = (0, 200, 255); PINK = (220, 100, 150)
GRASS = (52, 115, 38); EGO_GLOW = (0, 200, 255)
PATH_C = (0, 255, 200); GOAL_C = (255, 80, 80)

ROAD_C = {'highway': (75, 75, 80), 'urban': (70, 70, 75),
           'market': (65, 65, 68), 'village': (130, 115, 80)}
POTHOLE_C = (35, 35, 38); MARK_C = (200, 200, 200)

ESIZES = {
    'car': (20, 36), 'bus': (26, 62), 'truck': (28, 56),
    'auto_rickshaw': (16, 28), 'two_wheeler': (9, 20),
    'bicycle': (8, 20), 'pedestrian': (7, 7), 'pushcart': (14, 22),
    'animal': (16, 24), 'ego': (22, 42)
}
ECOLORS = {
    'car': [(50,100,200),(200,50,50),(50,180,50),(220,200,50),(200,100,50),(180,50,180)],
    'bus': [(220,140,30),(180,40,40),(40,80,160)],
    'truck': [(160,80,40),(80,80,90),(40,100,40)],
    'auto_rickshaw': [(50,200,50),(220,200,30),(220,140,40)],
    'two_wheeler': [(40,40,40),(160,40,40),(40,40,160)],
    'bicycle': [(150,80,40),(50,50,50)],
    'pedestrian': [(200,150,100),(160,110,70),(120,80,50)],
    'pushcart': [(139,90,43)],
    'animal': [(190,170,150),(130,110,90),(70,55,40)],
    'ego': [(0,180,240)]
}
ESPEEDS = {
    'car':(55,105),'bus':(35,65),'truck':(30,58),'auto_rickshaw':(42,78),
    'two_wheeler':(50,95),'bicycle':(12,32),'pedestrian':(5,12),
    'pushcart':(6,15),'animal':(3,10),'ego':(0,125)
}

GLB_PATHS = {
    'car': r"D:\sih\assets\car.glb",
    'animal': r"D:\sih\assets\cow.glb",
    'pedestrian': r"D:\sih\assets\pedestrian.glb",
    'pothole_1': r"D:\sih\assets\pothole_detroit.glb",
    'pothole_2': r"D:\sih\assets\pothole_real.glb",
    'manhole': r"D:\sih\assets\manhole.glb",
    'tree': r"D:\sih\assets\tree.glb",
    'road': r"D:\sih\assets\road.glb",
}

# ── 10 SCENARIOS ───────────────────────────────────────────
SCENARIOS = [
    # (ego_pos, ego_angle, goal, spawns, road_filter, name, flags)
    ((300,400),0,(4700,400),
     [('car',4),('truck',1),('bus',1),('auto_rickshaw',1),
      ('two_wheeler',3)],
     'highway',"Highway Chaos",{'unidirectional':True}),

    ((500,1100),0,(1800,2000),
     [('pedestrian',8),('pushcart',2),('auto_rickshaw',2),
      ('two_wheeler',3),('bicycle',1),('car',2)],
     'market',"Market Mayhem",{}),

    ((1200,2900),0,(3500,3600),
     [('animal',4),('car',2),('two_wheeler',2),('bicycle',1),
      ('pedestrian',2),('pushcart',1),('truck',1)],
     'village',"Village Navigate",{}),

    ((300,400),0,(2500,3600),
     [('car',3),('truck',1),('bus',1),('auto_rickshaw',2),
      ('two_wheeler',3),('bicycle',1),('pedestrian',3),
      ('pushcart',1),('animal',2)],
     None,"Full Journey",{}),

    ((2400,1350),0,(4200,1400),
     [('bus',2),('auto_rickshaw',3),('pedestrian',6),
      ('two_wheeler',2),('car',2),('pushcart',1)],
     'urban',"Bus Stand Chaos",{}),

    ((700,1300),0,(1500,1700),
     [('pedestrian',10),('bus',2),('auto_rickshaw',2),
      ('two_wheeler',2),('bicycle',3),('car',2)],
     'market',"School Zone",{}),

    ((300,400),0,(4700,400),
     [('two_wheeler',10),('auto_rickshaw',2),('car',2),('truck',1)],
     'highway',"Two-Wheeler Swarm",{'unidirectional':True}),

    ((1200,2900),0,(3500,2900),
     [('pedestrian',12),('animal',3),('pushcart',3),
      ('two_wheeler',2),('auto_rickshaw',2),('bicycle',1)],
     'village',"Temple Festival",{}),

    ((300,400),0,(4700,400),
     [('car',4),('truck',1),('bus',1),('auto_rickshaw',1),
      ('two_wheeler',3)],
     'highway',"Night Highway",{'night':True,'unidirectional':True}),

    ((300,400),0,(2500,3600),
     [('car',3),('truck',1),('bus',1),('auto_rickshaw',2),
      ('two_wheeler',3),('bicycle',1),('pedestrian',3),
      ('pushcart',1),('animal',1)],
     None,"Monsoon Madness",{'rain':True,'extra_potholes':True}),
]

# ── GLB LOADER ─────────────────────────────────────────────
class GLBModel:
    def __init__(self, path, render_size, fallback_color):
        self.path = path; self.render_size = render_size
        self.fallback_color = fallback_color
        self.sprites: Dict[int, pygame.Surface] = {}
        self.loaded = False; self.base_vertices = None; self.base_faces = None

    @staticmethod
    def _read_glb_bin(path):
        try:
            with open(path, 'rb') as f:
                magic = f.read(4)
                if magic != b'glTF': return None
                f.read(8)
                while True:
                    header = f.read(8)
                    if len(header) < 8: break
                    cl = int.from_bytes(header[0:4], 'little')
                    ct = header[4:8]
                    cd = f.read(cl)
                    pad = cl % 4
                    if pad: f.read(4 - pad)
                    if ct == b'BIN\x00': return cd
            return None
        except: return None

    def load(self):
        if not os.path.exists(self.path):
            print(f"[WARN] Not found: {self.path}"); return False
        try:
            bin_data = self._read_glb_bin(self.path)
            if bin_data is None:
                print(f"[WARN] No BIN chunk: {self.path}"); return False
            gltf = GLTF2().load(self.path)
            if not gltf.meshes:
                print(f"[WARN] No meshes: {self.path}"); return False
            av, af, off = [], [], 0
            for mesh in gltf.meshes:
                for prim in mesh.primitives:
                    if prim.attributes.POSITION is None: continue
                    pa = gltf.accessors[prim.attributes.POSITION]
                    pb = gltf.bufferViews[pa.bufferView]
                    bo = (pb.byteOffset or 0) + (pa.byteOffset or 0)
                    dt = np.dtype('<f4') if pa.componentType == 5126 else np.dtype('<u2')
                    vd = np.frombuffer(bin_data, dtype=dt, offset=bo, count=pa.count*3)
                    verts = vd.reshape(-1, 3).astype(np.float32); av.append(verts)
                    if prim.indices is not None:
                        ia = gltf.accessors[prim.indices]; ib = gltf.bufferViews[ia.bufferView]
                        io = (ib.byteOffset or 0) + (ia.byteOffset or 0)
                        idt = np.dtype('<u4') if ia.componentType == 5125 else np.dtype('<u2')
                        idx = np.frombuffer(bin_data, dtype=idt, offset=io, count=ia.count)
                        af.append(idx.astype(np.int32) + off); off += len(verts)
                    else:
                        if len(verts) >= 3:
                            af.append(np.arange(len(verts), dtype=np.int32).reshape(-1,3) + off)
                        off += len(verts)
            if av:
                self.base_vertices = np.vstack(av)
                if af: self.base_faces = np.vstack(af)
                self.loaded = True
                nf = len(self.base_faces) if self.base_faces is not None else 0
                print(f"[OK] {os.path.basename(self.path):25s} | {len(self.base_vertices):5d}v {nf:5d}f")
                return True
            return False
        except Exception as e:
            print(f"[ERR] {os.path.basename(self.path)}: {e}"); return False

    def generate_sprites(self, n=36):
        if not self.loaded or self.base_vertices is None: return
        v = self.base_vertices.copy()
        v -= (v.max(0) + v.min(0)) / 2
        me = np.abs(v).max(0)
        sx = self.render_size[0]/(2*me[0]) if me[0] > .001 else 1
        sz = self.render_size[1]/(2*me[2]) if me[2] > .001 else 1
        s = min(sx, sz) * .85; v[:,0] *= s; v[:,2] *= s
        ymn, ymx = v[:,1].min(), v[:,1].max()
        yr = ymx - ymn if ymx > ymn else 1
        w, h = self.render_size; fc = self.fallback_color
        for i in range(n):
            a = math.radians(i * 360 / n)
            ca, sa = math.cos(a), math.sin(a)
            rx = v[:,0]*ca - v[:,2]*sa; ry = v[:,0]*sa + v[:,2]*ca
            scx = (rx + w/2).astype(np.int32); scy = (ry + h/2).astype(np.int32)
            surf = pygame.Surface((w, h), pygame.SRCALPHA)
            if self.base_faces is not None and len(self.base_faces) > 0:
                fy = v[self.base_faces, 1].mean(1)
                for fi in np.argsort(-fy):
                    face = self.base_faces[fi]; pts = [(int(scx[vi]),int(scy[vi])) for vi in face]
                    sh = .4 + .6 * ((fy[fi]-ymn)/yr)
                    c = (int(min(255,fc[0]*sh)), int(min(255,fc[1]*sh)), int(min(255,fc[2]*sh)), 240)
                    try:
                        pygame.draw.polygon(surf, c, pts)
                        pygame.draw.polygon(surf, (min(255,c[0]+25),min(255,c[1]+25),min(255,c[2]+25),90), pts, 1)
                    except: pass
            else:
                for j in range(len(scx)):
                    sh = .5 + .5*((v[j,1]-ymn)/yr)
                    pygame.draw.circle(surf, (int(fc[0]*sh),int(fc[1]*sh),int(fc[2]*sh),240), (int(scx[j]),int(scy[j])), 2)
            self.sprites[i] = surf

    def get_sprite(self, angle_rad):
        if not self.sprites: return None
        d = math.degrees(angle_rad) % 360
        if d < 0: d += 360
        return self.sprites[int((d/360)*len(self.sprites)) % len(self.sprites)]


class GLBManager:
    def __init__(self):
        self.models: Dict[str, GLBModel] = {}
        self.fallbacks: Dict[str, Dict[tuple, pygame.Surface]] = {}

    def load_all(self):
        print("="*60); print("Loading GLB models..."); print("="*60)
        for et in ['car','animal','pedestrian']:
            p = GLB_PATHS.get(et)
            if p:
                ew, eh = ESIZES.get(et, (20, 20))
                m = GLBModel(p, (ew*4, eh*4), ECOLORS.get(et,[(180,180,180)])[0])
                if m.load(): m.generate_sprites(36); self.models[et] = m
                else: self._gfb(et)
        for pt in ['pothole_1','pothole_2']:
            p = GLB_PATHS.get(pt)
            if p:
                m = GLBModel(p, (80,80), (40,40,45))
                if m.load(): m.generate_sprites(1); self.models[pt] = m
        p = GLB_PATHS.get('manhole')
        if p:
            m = GLBModel(p, (50,50), (80,80,85))
            if m.load(): m.generate_sprites(1); self.models['manhole'] = m
        p = GLB_PATHS.get('tree')
        if p:
            m = GLBModel(p, (100,100), (30,120,30))
            if m.load(): m.generate_sprites(36); self.models['tree'] = m
        p = GLB_PATHS.get('road')
        if p:
            m = GLBModel(p, (200,200), (80,80,85))
            if m.load(): m.generate_sprites(1); self.models['road'] = m
        print("-"*60); print(f"Done. {len(self.models)}/8 loaded."); print("="*60)

    def _gfb(self, et):
        if et not in self.fallbacks: self.fallbacks[et] = {}
        for c in ECOLORS.get(et, [(180,180,180)]):
            w, h = ESIZES[et]; s = pygame.Surface((w,h), pygame.SRCALPHA)
            if et == 'pedestrian': pygame.draw.circle(s, c, (w//2,h//2), w//2)
            elif et == 'animal': pygame.draw.ellipse(s, c, (1,h//4,w-2,h*3//4))
            else: pygame.draw.rect(s, c, (0,0,w,h), border_radius=3)
            self.fallbacks[et][c] = s

    def get_entity_sprite(self, et, color, angle, w, h):
        m = self.models.get(et)
        if m and m.sprites:
            sp = m.get_sprite(angle)
            if sp: return pygame.transform.smoothscale(sp, (w*2, h*2))
        fb = self.fallbacks.get(et, {})
        s = fb.get(color)
        if not s:
            s = pygame.Surface((w,h), pygame.SRCALPHA)
            pygame.draw.rect(s, color, (0,0,w,h), border_radius=3)
        return pygame.transform.rotate(s, -math.degrees(angle)-90)

    def get_pothole_sprite(self, r):
        for pt in ['pothole_1','pothole_2']:
            m = self.models.get(pt)
            if m and m.sprites:
                return pygame.transform.smoothscale(list(m.sprites.values())[0], (r*2,r*2))
        return None

    def get_tree_sprite(self, angle=0, size=(60,60)):
        m = self.models.get('tree')
        if m and m.sprites:
            sp = m.get_sprite(angle)
            if sp: return pygame.transform.smoothscale(sp, size)
        return None

    def get_manhole_sprite(self, size=(30,30)):
        m = self.models.get('manhole')
        if m and m.sprites:
            return pygame.transform.smoothscale(list(m.sprites.values())[0], size)
        return None


# ── ROAD NETWORK ───────────────────────────────────────────
class Road:
    __slots__ = ('x1','y1','x2','y2','w','rtype','length','dx','dy','nx','ny','px','py')
    def __init__(self, x1, y1, x2, y2, w, rtype):
        self.x1,self.y1,self.x2,self.y2 = x1,y1,x2,y2
        self.w,self.rtype = w,rtype
        self.dx,self.dy = x2-x1, y2-y1
        self.length = math.hypot(self.dx, self.dy)
        self.nx,self.ny = (self.dx/self.length, self.dy/self.length) if self.length > 0 else (1,0)
        self.px,self.py = -self.ny, self.nx

ROADS = [
    Road(100,400,4900,400,150,'highway'),
    Road(2500,200,2500,3800,120,'urban'),
    Road(1200,400,1200,900,85,'urban'),
    Road(1200,900,1200,2100,72,'market'),
    Road(500,1100,1900,1100,62,'market'),
    Road(500,1400,1900,1400,62,'market'),
    Road(500,1700,1900,1700,62,'market'),
    Road(500,2000,1900,2000,62,'market'),
    Road(1900,1400,2500,1400,90,'urban'),
    Road(2500,1400,4500,1400,100,'urban'),
    Road(4500,400,4500,1400,100,'urban'),
    Road(2500,1400,2500,2300,100,'urban'),
    Road(2500,2300,2500,3700,82,'village'),
    Road(1200,2100,1200,2900,72,'village'),
    Road(1200,2900,3800,2900,72,'village'),
    Road(1200,3600,3800,3600,62,'village'),
    Road(1200,2900,1200,3600,62,'village'),
    Road(2500,2900,2500,3600,72,'village'),
    Road(3800,2900,3800,3600,62,'village'),
    Road(3800,1400,3800,2900,82,'village'),
    Road(700,900,700,2100,45,'market'),
    Road(1800,2900,1800,3600,50,'village'),
    Road(1800,2900,2500,2900,55,'village'),
    Road(4500,1400,4500,2900,70,'village'),
    Road(3800,2900,4500,2900,65,'village'),
]

ROAD_CONN: Dict[int, List[Tuple[int,int,float]]] = {}
for i, ra in enumerate(ROADS):
    cs = []
    for j, rb in enumerate(ROADS):
        if i == j: continue
        for ep in [(ra.x2,ra.y2),(ra.x1,ra.y1)]:
            for sp, tt in [((rb.x1,rb.y1),0),((rb.x2,rb.y2),1)]:
                d = math.hypot(ep[0]-sp[0], ep[1]-sp[1])
                if d < CONN_DIST: cs.append((j, tt, d))
    ROAD_CONN[i] = cs

# ── SCENERY ────────────────────────────────────────────────
class SceneryObj:
    __slots__ = ('x','y','stype','angle','cached_sprite','cached_shadow')
    def __init__(self, x, y, stype, angle=0):
        self.x,self.y,self.stype,self.angle=x,y,stype,angle
        self.cached_sprite=None; self.cached_shadow=None

def gen_scenery(roads):
    objs = []; rng = random.Random(42)
    for rd in roads:
        for _ in range(int(rd.length / 150)):
            t = rng.random(); side = rng.choice([-1,1])
            off = rd.w/2 + rng.uniform(15,80)
            x = rd.x1+rd.dx*t+rd.px*side*off; y = rd.y1+rd.dy*t+rd.py*side*off
            if 10<x<WW-10 and 10<y<WH-10:
                objs.append(SceneryObj(x, y, 'tree', rng.uniform(0,360)))
        if rd.rtype in ('urban','village','market'):
            for _ in range(max(0, int(rd.length/400))):
                t = rng.uniform(.1,.9); lat = rng.uniform(-rd.w*.3, rd.w*.3)
                objs.append(SceneryObj(rd.x1+rd.dx*t+rd.px*lat, rd.y1+rd.dy*t+rd.py*lat, 'manhole'))
    return objs

# ── POTHOLES ───────────────────────────────────────────────
class Pothole:
    __slots__ = ('x','y','r','mt')
    def __init__(self, x, y, r): self.x,self.y,self.r = x,y,r; self.mt = random.choice(['pothole_1','pothole_2'])

def gen_potholes(roads, density=0.0018):
    pots = []
    for rd in roads:
        n = max(0, int(rd.length*density))
        if rd.rtype=='highway': n = max(0, n//2)
        if rd.rtype=='village': n = int(n*1.25)
        for _ in range(n):
            t = random.random(); lat = random.uniform(-rd.w*.35, rd.w*.35)
            pots.append(Pothole(rd.x1+rd.dx*t+rd.px*lat, rd.y1+rd.dy*t+rd.py*lat, random.uniform(8,22)))
    return pots

# ── UTILITIES ──────────────────────────────────────────────
def dist(a,b): return math.hypot(a[0]-b[0], a[1]-b[1])
def ang_lerp(a,b,t):
    d=b-a
    while d>math.pi: d-=2*math.pi
    while d<-math.pi: d+=2*math.pi
    return a+d*t
def clamp(v,lo,hi): return max(lo, min(hi, v))
def pt_seg_dist(p,a,b):
    dx,dy = b[0]-a[0], b[1]-a[1]
    if dx==0 and dy==0: return dist(p,a), a
    t = clamp(((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy), 0, 1)
    cp = (a[0]+t*dx, a[1]+t*dy); return dist(p,cp), cp
def on_road(x,y,roads,margin=0):
    for rd in roads:
        d,_ = pt_seg_dist((x,y),(rd.x1,rd.y1),(rd.x2,rd.y2))
        if d < rd.w/2+margin: return True
    return False

# ── ENTITY ─────────────────────────────────────────────────
class Entity:
    def __init__(self, etype, x, y, angle=0):
        self.etype=etype; self.x,self.y,self.angle=float(x),float(y),float(angle)
        self.prev_x,self.prev_y=self.x,self.y
        self.speed=0.0; self.max_speed=random.uniform(*ESPEEDS[etype])
        self.w,self.h=ESIZES[etype]; self.color=random.choice(ECOLORS[etype])
        self.radius=max(self.w,self.h)*.55
        self.road_idx=-1; self.road_t=0.0; self.road_dir=1; self.lat_off=0.0
        self.beh_timer=random.uniform(1.5,8); self.stop_timer=0; self.stopped=False
        self.wrong_way=False; self.unidirectional=False; self.alive=True
        self.walk_angle=random.uniform(0,2*math.pi)
        self.label=etype.replace('_',' ').title()
        self.cached_sprite=None; self.cached_angle=None
        self.cached_label=None

    def place_on_road(self, roads, rf=None, unidirectional=False):
        self.unidirectional=bool(unidirectional)
        cs = [i for i,r in enumerate(roads) if rf is None or r.rtype in rf]
        if not cs: cs = list(range(len(roads)))
        self.road_idx = random.choice(cs); rd = roads[self.road_idx]
        # Highway scenarios pass unidirectional=True, so every traffic
        # vehicle starts in the road's forward direction.
        self.road_t = random.uniform(.05,.95); self.road_dir = 1 if unidirectional else random.choice([-1,1])
        if (not unidirectional) and self.etype=='two_wheeler' and random.random()<.15:
            self.wrong_way=True; self.road_dir*=-1
        self.lat_off = random.uniform(-rd.w*.32, rd.w*.32)
        self.x = rd.x1+rd.dx*self.road_t+rd.px*self.lat_off
        self.y = rd.y1+rd.dy*self.road_t+rd.py*self.lat_off
        self.prev_x,self.prev_y=self.x,self.y
        self.angle = math.atan2(rd.ny*self.road_dir, rd.nx*self.road_dir)

    def update(self, dt, roads):
        self.prev_x,self.prev_y=self.x,self.y
        self.beh_timer -= dt
        if self.beh_timer <= 0:
            self._rb(); self.beh_timer = random.uniform(2,10)
        if self.stopped:
            self.stop_timer -= dt; self.speed = max(0, self.speed-200*dt)
            if self.stop_timer <= 0: self.stopped = False
            else: return
        if self.road_idx >= 0: self._fr(dt, roads)
        else: self._wk(dt)
        self.x = clamp(self.x,30,WW-30); self.y = clamp(self.y,30,WH-30)

    def _fr(self, dt, roads):
        rd = roads[self.road_idx]
        if rd.length < 1: return
        spd = self.max_speed * (.6 if self.wrong_way else 1.)
        self.road_t += self.road_dir*spd*dt/rd.length
        if self.road_t > 1. or self.road_t < 0.: self._nr(roads); return
        cx = rd.x1+rd.dx*self.road_t+rd.px*self.lat_off
        cy = rd.y1+rd.dy*self.road_t+rd.py*self.lat_off
        bl = min(1., 6*dt)
        self.x += (cx-self.x)*bl; self.y += (cy-self.y)*bl
        ta = math.atan2(rd.ny*self.road_dir, rd.nx*self.road_dir)
        self.angle = ang_lerp(self.angle, ta, bl); self.speed = spd

    def _nr(self, roads):
        # Transition at a connected junction without teleporting.  The old
        # implementation changed road_t to .02/.98 but left x/y on the old
        # road; the next frame then snapped toward the new road centerline.
        rd = roads[self.road_idx]
        endpoint = (rd.x2, rd.y2) if self.road_t >= 1.0 else (rd.x1, rd.y1)
        cs = ROAD_CONN.get(self.road_idx, [])
        if cs:
            ranked = sorted(cs, key=lambda c: c[2])
            # On a one-way road, retain the forward direction and only use
            # connected roads whose direction can continue from this junction.
            if self.unidirectional:
                # Highway Chaos is a bounded one-way highway. Once a traffic
                # agent reaches the end of the highway it exits the scene; it
                # is never reversed, re-routed, or teleported back to the start.
                if self.road_idx == 0 and self.road_t >= 1.0:
                    self.x, self.y = endpoint
                    self.speed = 0.0
                    self.alive = False
                    return
            pk = random.choice(ranked[:min(3,len(ranked))])
            self.road_idx=pk[0]
            self.road_t=.02 if pk[1]==0 else .98
            self.road_dir=1 if pk[1]==0 else -1
            nrd=roads[self.road_idx]
            self.lat_off=random.uniform(-nrd.w*.3,nrd.w*.3)
            # Place exactly at the junction, then apply only the new road's
            # lateral offset. This is continuous rather than a teleport.
            self.x,self.y=endpoint
            # If the connected road is not perfectly coincident, project to
            # its selected endpoint plus the chosen lateral offset.
            anchor_t=self.road_t
            ax=nrd.x1+nrd.dx*anchor_t; ay=nrd.y1+nrd.dy*anchor_t
            ox=nrd.px*self.lat_off; oy=nrd.py*self.lat_off
            nx,ny=ax+ox,ay+oy
            gap=math.hypot(nx-self.x,ny-self.y)
            if gap <= CONN_DIST:
                # Keep the junction crossing continuous; lateral offset is
                # introduced progressively by _fr on subsequent frames.
                self.road_t=.02 if pk[1]==0 else .98
            ta=math.atan2(nrd.ny*self.road_dir,nrd.nx*self.road_dir)
            self.angle=ta
        else:
            self.road_dir*=-1
            self.road_t=clamp(self.road_t,.02,.98)
            self.x,self.y=endpoint

    def _wk(self, dt):
        # Walking/animal motion is deliberately slow and continuous.  The
        # entity can turn, but it cannot cover an implausible distance in a
        # single short interval.  This also prevents the visual "flying"
        # effect when the simulation is running quickly.
        if self.etype=='animal':
            self.walk_angle+=random.uniform(-1.0,1.0)*dt
            s=min(self.max_speed,10.0)
        else:
            self.walk_angle+=random.uniform(-0.8,0.8)*dt
            s=min(self.max_speed,12.0)
        step=min(s*dt, 12.0*dt)
        self.x+=math.cos(self.walk_angle)*step; self.y+=math.sin(self.walk_angle)*step
        self.angle=self.walk_angle; self.speed=s
        if not on_road(self.x,self.y,ROADS,30): self.walk_angle+=math.pi*.5

    def _rb(self):
        if self.etype in ('pedestrian','animal','pushcart'): return
        r = random.random()
        if r<.04 and self.etype in ('two_wheeler','auto_rickshaw') and not self.unidirectional:
            self.wrong_way=not self.wrong_way; self.road_dir*=-1
        elif r<.09:
            self.stopped=True; self.stop_timer=random.uniform(.5,3.5)
        elif r<.18:
            self.lat_off=random.uniform(-45,45)
        elif r<.22 and self.etype=='two_wheeler':
            self.lat_off=random.uniform(-55,55)

# ── PATH PLANNER ───────────────────────────────────────────
class PathPlanner:
    """
    Collision planner for the EGO vehicle only.

    The expensive global dynamic-obstacle A* from the original version has
    been replaced with:
      1) one static road A* route per goal,
      2) a small local A* around the ego vehicle,
      3) hard dynamic-obstacle cells built only from nearby vehicles,
      4) a hard collision gate in World.update().

    Other vehicles are never re-routed or stopped by this planner.
    """
    def __init__(self, roads, potholes, road_filter=None):
        self.roads=roads; self.potholes=potholes; self.road_filter=road_filter
        self.grid=[[100.]*GH for _ in range(GW)]
        self._bs()
        self.path=[]; self.path_idx=0; self.goal=None
        self.static_path=[]; self.static_idx=0
        self.state="IDLE"; self.forces={}; self.replan_timer=0.0
        self.last_plan_time=-999.0
        self.plan_interval=0.25
        self.local_radius_px=1000
        self.lookahead_px=700
        self.prediction_horizon=1.5
        self.prediction_step=0.30
        self.max_local_expansions=2600
        self.max_global_expansions=30000

    def _bs(self):
        for rd in self.roads:
            if self.road_filter and rd.rtype != self.road_filter: continue
            steps=max(1,int(rd.length/(GS*.5)))
            for i in range(steps+1):
                t=i/steps; cx=rd.x1+rd.dx*t; cy=rd.y1+rd.dy*t
                hw=rd.w/2+GS*.5
                for gx in range(max(0,int((cx-hw)/GS)),min(GW,int((cx+hw)/GS)+1)):
                    for gy in range(max(0,int((cy-hw)/GS)),min(GH,int((cy+hw)/GS)+1)):
                        px,py=gx*GS+GS//2,gy*GS+GS//2
                        d,_=pt_seg_dist((px,py),(rd.x1,rd.y1),(rd.x2,rd.y2))
                        if d<rd.w/2+GS*.3:
                            b={'highway':1.,'urban':1.5,'market':2.,'village':2.5}[rd.rtype]
                            self.grid[gx][gy]=min(self.grid[gx][gy],b)
        for p in self.potholes:
            gx,gy=int(p.x/GS),int(p.y/GS)
            for dx in range(-1,2):
                for dy in range(-1,2):
                    nx,ny=gx+dx,gy+dy
                    if 0<=nx<GW and 0<=ny<GH:
                        self.grid[nx][ny]+=30

    def set_goal(self,gx,gy):
        self.goal=(clamp(gx,0,WW),clamp(gy,0,WH))
        self.replan_timer=0.0
        self.last_plan_time=-999.0
        self.path=[]; self.path_idx=0
        self.static_path=[]; self.static_idx=0
        self.state="IDLE"

    def _free_cell(self, x, y, blocked=None):
        if not (0<=x<GW and 0<=y<GH): return False
        if self.grid[x][y] >= 99.0: return False
        if blocked is not None and (x,y) in blocked: return False
        return True

    def _nearest_free(self, x, y, blocked=None, radius=5):
        if self._free_cell(x,y,blocked): return (x,y)
        for r in range(1,radius+1):
            for dx in range(-r,r+1):
                for dy in (-r,r):
                    for xx,yy in ((x+dx,y+dy),(x+dx,y-dy)):
                        if self._free_cell(xx,yy,blocked):
                            return xx,yy
            for dy in range(-r+1,r):
                for dx in (-r,r):
                    for xx,yy in ((x+dx,y+dy),(x-dx,y+dy)):
                        if self._free_cell(xx,yy,blocked):
                            return xx,yy
        return None

    def _astar(self, start, goal, blocked=None, bounds=None, max_expansions=5000):
        sx,sy=start; gx,gy=goal
        if bounds is None:
            xmin,ymin,xmax,ymax=0,0,GW-1,GH-1
        else:
            xmin,ymin,xmax,ymax=bounds
        if not (xmin<=gx<=xmax and ymin<=gy<=ymax):
            gx=clamp(gx,xmin,xmax); gy=clamp(gy,ymin,ymax)
        s=self._nearest_free(sx,sy,blocked)
        g=self._nearest_free(gx,gy,blocked)
        if s is None or g is None: return []
        sx,sy=s; gx,gy=g
        if (sx,sy)==(gx,gy):
            return [(sx*GS+GS//2,sy*GS+GS//2)]

        heap=[(math.hypot(sx-gx,sy-gy),0.0,sx,sy)]
        came={}; gscore={(sx,sy):0.0}; visited=set()
        neighbors=((-1,0),(1,0),(0,-1),(0,1),
                   (-1,-1),(-1,1),(1,-1),(1,1))
        expansions=0
        while heap and expansions < max_expansions:
            _,cur_g,cx,cy=heapq.heappop(heap)
            if (cx,cy) in visited: continue
            visited.add((cx,cy)); expansions+=1
            if (cx,cy)==(gx,gy):
                out=[]; n=(gx,gy)
                while True:
                    out.append((n[0]*GS+GS//2,n[1]*GS+GS//2))
                    if n==(sx,sy): break
                    n=came[n]
                out.reverse()
                return out

            for dx,dy in neighbors:
                nx,ny=cx+dx,cy+dy
                if not (xmin<=nx<=xmax and ymin<=ny<=ymax): continue
                if (nx,ny) in visited: continue
                if not self._free_cell(nx,ny,blocked): continue
                # Prevent diagonal corner cutting through an occupied cell.
                if dx and dy:
                    if not self._free_cell(cx+dx,cy,blocked) or not self._free_cell(cx,cy+dy,blocked):
                        continue
                step=1.41421356 if dx and dy else 1.0
                ng=cur_g+self.grid[nx][ny]*step
                if ng < gscore.get((nx,ny),1e30):
                    gscore[(nx,ny)]=ng
                    h=math.hypot(nx-gx,ny-gy)
                    heapq.heappush(heap,(ng+h,ng,nx,ny))
                    came[(nx,ny)]=(cx,cy)
        return []

    def _compress(self,p,md=18):
        if len(p)<3: return p
        out=[p[0]]
        for pt in p[1:]:
            if dist(pt,out[-1])>=md: out.append(pt)
        if out[-1]!=p[-1]: out.append(p[-1])
        return out

    def _ensure_static_path(self, ex, ey):
        if self.goal is None: return False
        if self.static_path: return True
        sx,sy=clamp(int(ex/GS),0,GW-1),clamp(int(ey/GS),0,GH-1)
        gx,gy=clamp(int(self.goal[0]/GS),0,GW-1),clamp(int(self.goal[1]/GS),0,GH-1)
        p=self._astar((sx,sy),(gx,gy),None,None,self.max_global_expansions)
        if not p:
            self.state="NO_PATH"; return False
        self.static_path=self._compress(p,18)
        self.static_idx=0
        return True

    def _dynamic_blocked(self, ex, ey, ego_radius, ents):
        blocked=set()
        local_limit=self.local_radius_px+180
        h=self.prediction_horizon
        step=self.prediction_step
        steps=int(h/step)+1
        for e in ents:
            if not e.alive or e.etype=='ego': continue
            d=math.hypot(e.x-ex,e.y-ey)
            if d > local_limit + e.speed*h: continue

            # Predict the unchanged motion of the other agent.  Its route is
            # never edited; the EGO planner simply treats its swept tube as
            # forbidden space.
            vx=math.cos(e.angle)*e.speed
            vy=math.sin(e.angle)*e.speed
            tube=e.radius+ego_radius+18.0
            rc=int(math.ceil(tube/GS))+1
            for k in range(steps+1):
                t=min(h,k*step)
                px=e.x+vx*t
                py=e.y+vy*t
                cx=int(px/GS); cy=int(py/GS)
                for ox in range(-rc,rc+1):
                    for oy in range(-rc,rc+1):
                        nx,ny=cx+ox,cy+oy
                        if 0<=nx<GW and 0<=ny<GH:
                            qx=nx*GS+GS//2; qy=ny*GS+GS//2
                            if math.hypot(qx-px,qy-py)<=tube:
                                blocked.add((nx,ny))
        return blocked

    def _nearest_static_index(self, ex, ey):
        if not self.static_path: return 0
        start=max(0,min(self.static_idx,len(self.static_path)-1))
        best=start; bd=1e30
        # The path is ordered, so searching the remaining route is small.
        for i in range(start,min(len(self.static_path),start+120)):
            d=(self.static_path[i][0]-ex)**2+(self.static_path[i][1]-ey)**2
            if d<bd: bd=d; best=i
        self.static_idx=best
        return best

    def _route_threat(self, ex, ey, ego_radius, ents):
        if not self.static_path: return False
        start=max(0,self.static_idx)
        route=self.static_path[start:min(len(self.static_path),start+48)]
        if len(route)<2: return False

        # Only inspect the closest moving agents. Far agents are handled on
        # later replans and cannot affect the current local corridor.
        nearby=[]
        for e in ents:
            if not e.alive or e.etype=='ego': continue
            d=math.hypot(e.x-ex,e.y-ey)
            if d<950: nearby.append((d,e))
        nearby.sort(key=lambda z:z[0])
        for _,e in nearby[:20]:
            pred_speed=min(e.max_speed,max(e.speed,e.speed*1.5+20.0))
            for k in range(6):
                t=k*0.30
                px=e.x+math.cos(e.angle)*pred_speed*t
                py=e.y+math.sin(e.angle)*pred_speed*t
                # Check the predicted obstacle point against the route.
                for i in range(len(route)-1):
                    d,_=pt_seg_dist((px,py),route[i],route[i+1])
                    if d < ego_radius+e.radius+24:
                        return True
        return False

    def plan(self, ex, ey, ents, ego_radius=23.0, now=0.0, force=False):
        if self.goal is None: return
        if not force and now-self.last_plan_time < self.plan_interval:
            return
        self.last_plan_time=now

        if not self._ensure_static_path(ex,ey):
            self.path=[]; self.path_idx=0
            return

        self._nearest_static_index(ex,ey)
        route_threat=self._route_threat(ex,ey,ego_radius,ents)

        if not route_threat:
            # Fast path: no obstacle intersects the predicted base corridor.
            # No A* is executed in the normal case.
            end=min(len(self.static_path),self.static_idx+56)
            self.path=self.static_path[self.static_idx:end]
            self.path_idx=0
            self.state="CRUISING"
            return

        # Only when the base corridor is threatened do we build the dynamic
        # forbidden set and run a local A*.
        blocked=self._dynamic_blocked(ex,ey,ego_radius,ents)
        sx,sy=clamp(int(ex/GS),0,GW-1),clamp(int(ey/GS),0,GH-1)
        blocked.discard((sx,sy))

        r=int(self.local_radius_px/GS)
        xmin=max(0,sx-r); xmax=min(GW-1,sx+r)
        ymin=max(0,sy-r); ymax=min(GH-1,sy+r)
        bounds=(xmin,ymin,xmax,ymax)

        nearest=self.static_idx
        candidates=[]
        for ahead in (12,24,36,48,60):
            idx=min(len(self.static_path)-1,nearest+ahead)
            pt=self.static_path[idx]
            tx,ty=int(pt[0]/GS),int(pt[1]/GS)
            if xmin<=tx<=xmax and ymin<=ty<=ymax:
                candidates.append((tx,ty))

        if not candidates:
            pt=self.static_path[-1]
            candidates=[(clamp(int(pt[0]/GS),xmin,xmax),
                         clamp(int(pt[1]/GS),ymin,ymax))]

        new_path=[]
        for target in candidates:
            p=self._astar((sx,sy),target,blocked,bounds,self.max_local_expansions)
            if p:
                new_path=self._compress(p,18)
                break

        if new_path:
            self.path=new_path
            self.path_idx=0
            self.state="AVOIDING"
        else:
            # No safe local route. Stop only the EGO; never modify traffic.
            self.path=[]; self.path_idx=0
            self.state="EMERGENCY"

    def control(self, ex, ey, ego_radius, ents, max_speed, ego_angle=0.0):
        self.forces={}
        if not self.path or self.path_idx>=len(self.path):
            self.forces['goal']=(0.0,0.0)
            self.forces['obstacle']=(0.0,0.0)
            self.forces['threat']=1.0
            self.state="EMERGENCY"
            return 0.0,0.0,0.0,1.0

        # Consume passed waypoints.
        while self.path_idx < len(self.path)-1 and dist((ex,ey),self.path[self.path_idx])<45:
            self.path_idx+=1
        # Follow the A* path progressively.  Never select a waypoint that
        # would make the EGO command a backwards movement relative to its
        # current heading.  This prevents path replanning from producing
        # visually surprising reversals or jumps.
        fx,fy=math.cos(ego_angle),math.sin(ego_angle)
        candidate=self.path_idx
        for i in range(self.path_idx, min(len(self.path), self.path_idx+10)):
            qx,qy=self.path[i]
            qdx,qdy=qx-ex,qy-ey
            qd=math.hypot(qdx,qdy)
            if qd < 1.0:
                continue
            forward_dot=(qdx*fx+qdy*fy)/qd
            if forward_dot >= -0.15:
                candidate=i
                break
        self.path_idx=max(self.path_idx,candidate)
        wp=self.path[self.path_idx]
        dx,dy=wp[0]-ex,wp[1]-ey
        d=math.hypot(dx,dy)
        if d<1:
            return 0.0,0.0,0.0,0.0
        ux,uy=dx/d,dy/d

        # A* supplies the geometric route; the controller follows it with
        # bounded steering rather than snapping the vehicle onto a waypoint.
        # The EGO therefore always progresses through continuous world
        # coordinates, even when A* replans around a moving obstacle.

        # Threat is based on the actual current clearance.  This is a speed
        # governor; it does not modify any other vehicle.
        min_clear=1e9
        ox=oy=0.0
        for e in ents:
            if not e.alive or e.etype=='ego': continue
            dd=math.hypot(ex-e.x,ey-e.y)
            clear=dd-(ego_radius+e.radius)
            if clear<min_clear: min_clear=clear
            if dd<DET_R and dd>0.1:
                w=max(0.0,1.0-(dd/(DET_R)))
                ox+=(ex-e.x)/dd*w
                oy+=(ey-e.y)/dd*w

        threat=clamp(1.0-min_clear/140.0,0.0,1.0)
        if min_clear<35:
            target_speed=0.0
            self.state="EMERGENCY"
        elif min_clear<140:
            target_speed=max(0.0,max_speed*(min_clear/140.0))
            self.state="AVOIDING"
        else:
            target_speed=max_speed
            if self.state not in ("EMERGENCY","NO_PATH"): self.state="CRUISING"

        # Keep the force display meaningful without using force integration
        # to drive the vehicle.  The actual EGO direction comes only from path.
        self.forces['goal']=(ux*target_speed,uy*target_speed)
        self.forces['obstacle']=(ox*max_speed*0.15,oy*max_speed*0.15)
        self.forces['threat']=threat
        return ux,uy,target_speed,threat

# ── WORLD ──────────────────────────────────────────────────
class World:
    def __init__(self, glb: GLBManager):
        self.roads=ROADS; self.glb=glb; self.potholes=[]; self.scenery=[]
        self.entities: List[Entity]=[]; self.ego: Optional[Entity]=None
        self.planner: Optional[PathPlanner]=None; self.auto_mode=True
        # Rear-approach evasive maneuver state. This only affects the EGO.
        self.rear_evasion_until=0.0
        self.rear_evasion_side=0
        self.rear_evasion_target=None
        self.cam_x,self.cam_y=0,0; self.time=0; self.sim_speed=1.
        self.paused=False; self.scenario=0; self.trail=[]
        self.collision_count=0
        self.flags: Dict[str,bool] = {}

    def _spawn_safe(self, et, rf, flags):
        best=None; best_clear=-1.0
        for _ in range(160):
            e=Entity(et,0,0)
            e.place_on_road(self.roads, [rf] if rf else None, flags.get('unidirectional', False))
            if flags.get('rain'): e.max_speed *= 0.65
            if flags.get('night'): e.max_speed *= 0.8
            clear=min(
                (dist((e.x,e.y),(o.x,o.y))-(e.radius+o.radius)
                 for o in self.entities if o.etype!='ego'),
                default=1e9
            )
            ego_clear=dist((e.x,e.y),(self.ego.x,self.ego.y))-(e.radius+self.ego.radius)
            clear=min(clear,ego_clear)
            if clear>best_clear:
                best=e; best_clear=clear
            if clear>=25:
                return e
        return best

    def load_scenario(self, idx):
        self.scenario=idx; self.entities.clear(); self.trail.clear()
        self.collision_count=0
        sc = SCENARIOS[idx % len(SCENARIOS)]
        ep, ea, goal, spawns, rf, name, flags = sc
        self.flags = flags

        # Highway Chaos is intentionally a clean highway presentation:
        # no potholes and no trees in the UI. Other scenarios keep their
        # original scenery/pothole generation unchanged.
        if name == 'Highway Chaos':
            self.potholes = []
            self.scenery = [s for s in gen_scenery(ROADS) if s.stype != 'tree']
        elif flags.get('extra_potholes'):
            self.potholes = gen_potholes(ROADS, 0.003)
            self.scenery = gen_scenery(ROADS)
        else:
            self.potholes = gen_potholes(ROADS)
            self.scenery = gen_scenery(ROADS)
        self.planner = PathPlanner(self.roads, self.potholes, rf)

        self.ego = Entity('ego', ep[0], ep[1], ea)
        self.ego.max_speed = ESPEEDS['ego'][1]
        if flags.get('night'): self.ego.max_speed *= 0.75
        if flags.get('rain'): self.ego.max_speed *= 0.7
        self.entities.append(self.ego)
        self.planner.set_goal(goal[0], goal[1])

        for et, cnt in spawns:
            for _ in range(cnt):
                e=self._spawn_safe(et,rf,flags)
                if e is not None:
                    self.entities.append(e)

    @staticmethod
    def _collision_free_fraction(start, end, e, clearance):
        """
        Return the largest normalized fraction of the EGO move that is safe.
        The other agent follows its already-generated prev->current segment;
        its path is never changed.
        """
        ep0=(e.prev_x,e.prev_y); ep1=(e.x,e.y)
        mvx=end[0]-start[0]; mvy=end[1]-start[1]
        omx=ep1[0]-ep0[0]; omy=ep1[1]-ep0[1]

        def collides(t):
            ax=start[0]+mvx*t; ay=start[1]+mvy*t
            bx=ep0[0]+omx*t; by=ep0[1]+omy*t
            dx=ax-bx; dy=ay-by
            return dx*dx+dy*dy <= clearance*clearance

        if collides(0.0):
            return 0.0

        # Relative motion is continuous.  At the simulation speeds used here,
        # 16 samples per frame bound the missed interval tightly.
        prev_t=0.0
        for i in range(1,17):
            t=i/16.0
            if collides(t):
                lo,hi=prev_t,t
                for _ in range(12):
                    mid=(lo+hi)*0.5
                    if collides(mid): hi=mid
                    else: lo=mid
                return max(0.0, hi-0.035)
            prev_t=t
        return 1.0

    def _rear_collision_response(self, ego, dt):
        """
        Detect a fast vehicle approaching the EGO from behind and give the
        EGO a physically reachable escape target ahead of it.  Only the EGO
        is steered/accelerated; the approaching vehicle keeps its original
        path and speed.

        The maneuver deliberately uses a short look-ahead and the normal
        steering/acceleration limits in World.update(), so it cannot teleport.
        """
        fx,fy=math.cos(ego.angle),math.sin(ego.angle)
        sx,sy=-fy,fx
        ego_vx=fx*ego.speed; ego_vy=fy*ego.speed

        threats=[]
        for e in self.entities:
            if e.etype=='ego' or not e.alive: continue
            rx=e.x-ego.x; ry=e.y-ego.y
            longitudinal=rx*fx+ry*fy
            lateral=abs(rx*sx+ry*sy)
            d=math.hypot(rx,ry)
            if longitudinal >= -25.0 or d > 300.0: continue

            if dt>1e-6:
                ovx=(e.x-e.prev_x)/dt; ovy=(e.y-e.prev_y)/dt
            else:
                ovx=math.cos(e.angle)*e.speed; ovy=math.sin(e.angle)*e.speed
            closing=-( (ovx-ego_vx)*fx + (ovy-ego_vy)*fy )

            # A vehicle that is genuinely closing from behind, especially
            # within the next ~2 seconds, is treated as a rear threat.
            ttc=(-longitudinal)/max(closing,1e-3) if closing>1.0 else 999.0
            if closing>12.0 and (ttc<2.6 or d<150.0):
                threats.append((ttc,d,e,lateral))

        if not threats:
            return None

        threats.sort(key=lambda z:(z[0],z[1]))
        _,_,threat,_=threats[0]

        # Keep one chosen side briefly so the EGO makes a smooth, human-like
        # lane-change instead of oscillating left/right every frame.
        if self.time < self.rear_evasion_until and self.rear_evasion_side:
            side=self.rear_evasion_side
        else:
            # Prefer the side with the largest immediate clearance.
            candidates=[]
            for side_try in (-1,1):
                # Shift laterally but also forward. On the 150 px highway this
                # corresponds roughly to moving toward an adjacent lane.
                ahead=170.0
                shift=48.0
                tx=ego.x+fx*ahead+sx*side_try*shift
                ty=ego.y+fy*ahead+sy*side_try*shift
                if not on_road(tx,ty,self.roads,ego.radius+4):
                    continue
                min_clear=1e9
                for e in self.entities:
                    if e.etype=='ego' or not e.alive: continue
                    min_clear=min(min_clear,dist((tx,ty),(e.x,e.y))-ego.radius-e.radius)
                # Also favor the side away from the threatening vehicle.
                threat_side=(threat.x-ego.x)*sx+(threat.y-ego.y)*sy
                away_bonus=30.0 if (threat_side*side_try)<0 else 0.0
                candidates.append((min_clear+away_bonus,side_try,tx,ty))

            if not candidates:
                return None
            candidates.sort(reverse=True)
            _,side,tx,ty=candidates[0]
            self.rear_evasion_side=side
            self.rear_evasion_until=self.time+1.15
            self.rear_evasion_target=(tx,ty)

        # Rebuild the target using the current EGO pose so the target never
        # causes a positional jump if the vehicle has moved meanwhile.
        ahead=max(150.0,ego.speed*1.15)
        shift=52.0
        tx=ego.x+fx*ahead+sx*side*shift
        ty=ego.y+fy*ahead+sy*side*shift
        if not on_road(tx,ty,self.roads,ego.radius+4):
            return None

        # Verify that the immediate escape target is not occupied.
        for e in self.entities:
            if e.etype=='ego' or not e.alive: continue
            if dist((tx,ty),(e.x,e.y)) < ego.radius+e.radius+18.0:
                return None

        dx,dy=tx-ego.x,ty-ego.y
        d=math.hypot(dx,dy)
        if d<1.0: return None

        # Preserve forward progress; do not command an instantaneous lateral
        # jump. World.update() applies the normal steering-rate limit.
        desired_speed=min(ego.max_speed,max(ego.speed+35.0,ego.max_speed*0.92))
        return dx/d,dy/d,desired_speed,True

    def _safe_ego_endpoint(self, start, desired_end, dt):
        """
        Predictive hard safety gate.  The EGO move is checked against the
        other agent's current->predicted position for this same time step.
        Other agents are not stopped, steered, or re-routed.
        """
        frac=1.0
        sx,sy=start; ex,ey=desired_end
        reach=math.hypot(ex-sx,ey-sy)

        for e in self.entities:
            if e.etype=='ego' or not e.alive: continue

            # Conservative short-horizon prediction of the unchanged agent
            # motion.  Random behavioral changes are handled by the next
            # planner cycle; the safety margin protects this frame.
            pred_speed=min(e.max_speed, max(e.speed, e.speed*1.5+20.0))
            obs0=(e.x,e.y)
            obs1=(e.x+math.cos(e.angle)*pred_speed*dt,
                  e.y+math.sin(e.angle)*pred_speed*dt)

            margin=e.radius+self.ego.radius+12.0
            d1,_=pt_seg_dist(obs0,start,desired_end)
            d2,_=pt_seg_dist(obs1,start,desired_end)
            if min(d1,d2) > margin + pred_speed*dt:
                continue

            # Reuse the continuous relative-motion test with the predicted
            # obstacle segment rather than modifying the Entity itself.
            ep0=(e.prev_x,e.prev_y); ep1=(e.x,e.y)
            e.prev_x,e.prev_y=obs0
            e.x,e.y=obs1
            f=self._collision_free_fraction(start,desired_end,e,margin)
            e.prev_x,e.prev_y=ep0
            e.x,e.y=ep1
            if f<frac: frac=f

        if frac>=0.999:
            return desired_end,True
        safe=(sx+(ex-sx)*frac, sy+(ey-sy)*frac)
        return safe,False

    def update(self, dt):
        if self.paused: return
        dt *= self.sim_speed
        dt=min(dt,0.12)
        self.time += dt

        ego=self.ego
        if not ego: return

        # IMPORTANT: non-EGO traffic is advanced first. Their paths are never
        # changed. This gives the EGO controller the exact prev->current
        # trajectory for every obstacle during this simulation step.
        for e in self.entities:
            if e.etype=='ego': continue
            e.update(dt,self.roads)

        if self.auto_mode and self.planner:
            self.planner.plan(
                ego.x,ego.y,self.entities,
                ego_radius=ego.radius,
                now=self.time
            )

            # A* is the authoritative path planner.  Static A* supplies the
            # normal forward route; when a moving obstacle threatens that
            # route, local dynamic A* computes a new collision-free route.
            # The low-level controller then follows that route continuously.
            ux,uy,target_speed,th=self.planner.control(
                ego.x,ego.y,ego.radius,self.entities,ego.max_speed,ego.angle
            )

            # If a traffic vehicle is closing rapidly from behind, do not let
            # the EGO simply stop in its lane. Give it a reachable point ahead
            # and slightly to one side, then let the normal steering dynamics
            # execute the maneuver. Other vehicles are never modified.
            rear_response=self._rear_collision_response(ego,dt)
            if rear_response is not None:
                ux,uy,target_speed,_=rear_response
                self.planner.state="AVOIDING"

            if target_speed < ego.speed:
                ego.speed=max(target_speed,ego.speed-520*dt)
            else:
                ego.speed=min(target_speed,ego.speed+180*dt)

            if target_speed <= 0.1:
                ego.speed=max(0.0,ego.speed-700*dt)

            if ego.speed>0.01 and (ux or uy):
                desired_angle=math.atan2(uy,ux)
                delta=ang_lerp(ego.angle,desired_angle,1.0)
                max_turn=5.5*dt
                ego.angle=ang_lerp(
                    ego.angle,desired_angle,
                    min(1.0,max_turn/max(0.001,abs(delta)))
                )

            ego.prev_x,ego.prev_y=ego.x,ego.y
            start=(ego.x,ego.y)
            desired=(
                start[0]+math.cos(ego.angle)*ego.speed*dt,
                start[1]+math.sin(ego.angle)*ego.speed*dt
            )
            desired=(clamp(desired[0],30,WW-30),clamp(desired[1],30,WH-30))

            # Hard continuous collision gate against the ACTUAL obstacle
            # movement for this frame. Only the EGO endpoint is shortened.
            safe_end=(desired[0],desired[1])
            safe=True
            for e in self.entities:
                if e.etype=='ego' or not e.alive: continue
                margin=ego.radius+e.radius+8.0
                f=self._collision_free_fraction(start,safe_end,e,margin)
                if f < 0.999:
                    safe=False
                    safe_end=(
                        start[0]+(safe_end[0]-start[0])*max(0.0,f),
                        start[1]+(safe_end[1]-start[1])*max(0.0,f)
                    )

            actual_dist=dist(start,safe_end)
            ego.x,ego.y=safe_end
            ego.speed=actual_dist/dt if dt>1e-6 else 0.0

            if not safe:
                # Never teleport the EGO. If there is no collision-free motion
                # in this frame, remain at the current position and wait for
                # the next controller cycle to find a safe EGO-only path.
                if actual_dist < 1.0:
                    ego.x,ego.y=start
                    ego.speed=0.0
                self.planner.state="EMERGENCY"
            elif actual_dist>0.05:
                ego.angle=math.atan2(ego.y-start[1],ego.x-start[0])

        else:
            ego.prev_x,ego.prev_y=ego.x,ego.y
            keys=pygame.key.get_pressed(); acc=200*dt; turn=3.*dt
            if keys[pygame.K_UP] or keys[pygame.K_w]: ego.speed=min(ego.max_speed,ego.speed+acc)
            if keys[pygame.K_DOWN] or keys[pygame.K_s]: ego.speed=max(-ego.max_speed*.3,ego.speed-acc)
            if keys[pygame.K_LEFT] or keys[pygame.K_a]: ego.angle-=turn*(ego.speed/max(ego.max_speed,1)+.3)
            if keys[pygame.K_RIGHT] or keys[pygame.K_d]: ego.angle+=turn*(ego.speed/max(ego.max_speed,1)+.3)
            if not(keys[pygame.K_UP] or keys[pygame.K_w] or keys[pygame.K_DOWN] or keys[pygame.K_s]):
                ego.speed*=max(0,1-2*dt)
            ego.x+=math.cos(ego.angle)*ego.speed*dt
            ego.y+=math.sin(ego.angle)*ego.speed*dt
            ego.x=clamp(ego.x,30,WW-30); ego.y=clamp(ego.y,30,WH-30)

        # A collision is now a diagnostic condition, never a recovery trigger.
        # There is deliberately no positional correction/teleport.
        for e in self.entities:
            if e.etype!='ego' and e.alive:
                if dist((ego.x,ego.y),(e.x,e.y)) < ego.radius+e.radius:
                    self.collision_count += 1
                    ego.speed=0.0
                    if self.planner:
                        self.planner.state="EMERGENCY"
                    break

        self.trail.append((ego.x,ego.y))
        if len(self.trail)>300: self.trail.pop(0)
        self.cam_x+=(ego.x-SW//2-self.cam_x)*min(1,8*dt)
        self.cam_y+=(ego.y-SH//2-self.cam_y)*min(1,8*dt)
        self.cam_x=clamp(self.cam_x,0,WW-SW); self.cam_y=clamp(self.cam_y,0,WH-SH)

# ── RENDERER ───────────────────────────────────────────────
class Renderer:
    def __init__(self, screen, glb: GLBManager):
        self.screen=screen; self.glb=glb
        self.font_sm=pygame.font.SysFont('consolas',13)
        self.font_md=pygame.font.SysFont('consolas',16,bold=True)
        self.font_lg=pygame.font.SysFont('consolas',20,bold=True)
        self.font_xl=pygame.font.SysFont('consolas',28,bold=True)
        self.show_minimap=True; self.show_forces=False
        self.road_surface=None; self.road_dirty=True
        # Night headlight pre-render
        self.headlight = pygame.Surface((500, 500), pygame.SRCALPHA)
        for r in range(250, 0, -3):
            a = int(200 * (1 - (r/250)**.6))
            pygame.draw.circle(self.headlight, (a//2, a//2, a//3, a), (250, 250), r)
        pygame.draw.circle(self.headlight, (120, 120, 100, 255), (250, 250), 35)
        # Rain particles
        self.rain_drops = [[random.randint(0, SW), random.randint(0, SH),
                            random.uniform(10, 18), random.uniform(0.7, 1.0)] for _ in range(240)]
        self.det_surface = pygame.Surface((DET_R*2,DET_R*2), pygame.SRCALPHA)
        pygame.draw.circle(self.det_surface,(0,200,255,25),(DET_R,DET_R),DET_R)
        pygame.draw.circle(self.det_surface,(0,200,255,60),(DET_R,DET_R),DET_R,1)
        self.ego_glow = pygame.Surface((66,126), pygame.SRCALPHA)
        pygame.draw.ellipse(self.ego_glow,(0,200,255,40),self.ego_glow.get_rect())
        self.rain_tint = pygame.Surface((SW,SH), pygame.SRCALPHA)
        self.rain_tint.fill((15,25,45,35))
        self.fog_edge = pygame.Surface((60,SH), pygame.SRCALPHA)
        self.fog_edge.fill((40,50,60,40))
        self.minimap_surface = None
        self.minimap_timer = 0.0
        self.minimap_period = 0.12

    def _build_road_surface(self, world):
        self.road_surface = pygame.Surface((WW, WH)); self.road_surface.fill(GRASS)
        for rd in world.roads:
            c = ROAD_C[rd.rtype]; ew = rd.w+8
            pygame.draw.line(self.road_surface, DGRAY,
                (int(rd.x1),int(rd.y1)),(int(rd.x2),int(rd.y2)),int(ew))
            pygame.draw.line(self.road_surface, c,
                (int(rd.x1),int(rd.y1)),(int(rd.x2),int(rd.y2)),int(rd.w))
            if rd.rtype in ('highway','urban') and rd.length > 100:
                d = 0
                while d < rd.length:
                    t1=d/rd.length; t2=min(1,(d+30)/rd.length)
                    p1=(int(rd.x1+rd.dx*t1),int(rd.y1+rd.dy*t1))
                    p2=(int(rd.x1+rd.dx*t2),int(rd.y1+rd.dy*t2))
                    pygame.draw.line(self.road_surface, MARK_C, p1, p2, 2); d+=50
            if rd.rtype == 'highway':
                for s in (-1,1):
                    ox=rd.px*s*(rd.w//2-3); oy=rd.py*s*(rd.w//2-3)
                    pygame.draw.line(self.road_surface,(180,180,180),
                        (int(rd.x1+ox),int(rd.y1+oy)),(int(rd.x2+ox),int(rd.y2+oy)),2)
        for p in world.potholes:
            sp = self.glb.get_pothole_sprite(int(p.r))
            if sp: self.road_surface.blit(sp, (int(p.x-sp.get_width()//2), int(p.y-sp.get_height()//2)))
            else:
                pygame.draw.circle(self.road_surface, POTHOLE_C, (int(p.x),int(p.y)), int(p.r))
                pygame.draw.circle(self.road_surface, (25,25,28), (int(p.x),int(p.y)), int(p.r*.6))
        # Monsoon: puddles on roads
        if world.flags.get('rain'):
            rng = random.Random(99)
            for rd in world.roads:
                for _ in range(int(rd.length / 80)):
                    t = rng.random(); lat = rng.uniform(-rd.w*.3, rd.w*.3)
                    px = int(rd.x1+rd.dx*t+rd.px*lat); py = int(rd.y1+rd.dy*t+rd.py*lat)
                    ps = pygame.Surface((rng.randint(20,50), rng.randint(15,35)), pygame.SRCALPHA)
                    ps.fill((60, 80, 120, 50))
                    pygame.draw.ellipse(ps, (80, 100, 140, 70), ps.get_rect(), 1)
                    self.road_surface.blit(ps, (px, py))
        for s in world.scenery:
            if s.stype == 'manhole':
                sp = self.glb.get_manhole_sprite((30,30))
                if sp: self.road_surface.blit(sp, (int(s.x-15),int(s.y-15)))
                else:
                    pygame.draw.circle(self.road_surface,(80,80,85),(int(s.x),int(s.y)),12)
                    pygame.draw.circle(self.road_surface,(60,60,65),(int(s.x),int(s.y)),12,2)
        self.road_dirty = False

    def draw(self, world):
        if self.road_dirty or self.road_surface is None: self._build_road_surface(world)
        self.screen.fill(GRASS)
        cx,cy = int(world.cam_x), int(world.cam_y)
        self.screen.blit(self.road_surface, (0,0), (cx,cy,SW,SH))

        # Trees
        vt = [s for s in world.scenery if s.stype=='tree' and cx-80<s.x<cx+SW+80 and cy-80<s.y<cy+SH+80]
        vt.sort(key=lambda s: s.y)
        for t in vt:
            sx,sy=int(t.x-cx),int(t.y-cy)
            if t.cached_sprite is None:
                t.cached_sprite=self.glb.get_tree_sprite(math.radians(t.angle),(60,60))
                if t.cached_sprite:
                    t.cached_shadow=t.cached_sprite.copy()
                    t.cached_shadow.set_alpha(50)
            sp=t.cached_sprite
            if sp:
                if t.cached_shadow: self.screen.blit(t.cached_shadow,(sx-sp.get_width()//2+4,sy-sp.get_height()//2+4))
                self.screen.blit(sp,(sx-sp.get_width()//2,sy-sp.get_height()//2))
            else:
                pygame.draw.circle(self.screen,(30,100,30),(sx,sy),20)
                pygame.draw.circle(self.screen,(40,130,40),(sx-3,sy-3),16)
                pygame.draw.rect(self.screen,BROWN,(sx-3,sy,6,12))

        # Trail
        if len(world.trail) > 1:
            pts = [(int(p[0]-cx),int(p[1]-cy)) for p in world.trail]
            for i in range(1, len(pts)):
                a = i/len(pts)
                pygame.draw.line(self.screen, (0,int(150*a),int(200*a)), pts[i-1], pts[i], 2)

        # Path
        if world.planner and world.planner.path and world.auto_mode:
            path = world.planner.path
            pts = [(int(p[0]-cx),int(p[1]-cy)) for p in path[world.planner.path_idx:]]
            if len(pts)>1: pygame.draw.lines(self.screen, PATH_C, False, pts, 3)
            if world.planner.goal:
                gx,gy = int(world.planner.goal[0]-cx), int(world.planner.goal[1]-cy)
                pygame.draw.circle(self.screen, GOAL_C, (gx,gy), 14, 3)
                pygame.draw.line(self.screen, GOAL_C, (gx-8,gy),(gx+8,gy), 2)
                pygame.draw.line(self.screen, GOAL_C, (gx,gy-8),(gx,gy+8), 2)

        # Detection radius
        if world.ego and world.auto_mode:
            ex,ey = int(world.ego.x-cx), int(world.ego.y-cy)
            self.screen.blit(self.det_surface,(ex-DET_R,ey-DET_R))

        # Entities
        for e in sorted(world.entities, key=lambda e: e.y):
            sx,sy = int(e.x-cx), int(e.y-cy)
            if sx<-80 or sx>SW+80 or sy<-80 or sy>SH+80: continue
            self._draw_ent(e, sx, sy, world)

        if self.show_forces and world.auto_mode and world.planner and world.planner.forces:
            self._draw_forces(world.ego, cx, cy, world.planner.forces)

        # ── POST-PROCESSING: Night overlay ──
        if world.flags.get('night') and world.ego:
            dark = pygame.Surface((SW, SH)); dark.fill((5, 5, 18)); dark.set_alpha(210)
            self.screen.blit(dark, (0, 0))
            esx, esy = int(world.ego.x-cx), int(world.ego.y-cy)
            rhl = pygame.transform.rotate(self.headlight, -math.degrees(world.ego.angle)-90)
            self.screen.blit(rhl, rhl.get_rect(center=(esx, esy)), special_flags=pygame.BLEND_RGB_ADD)
            # Tail lights for nearby vehicles
            for e in world.entities:
                if e.etype=='ego' or not e.alive: continue
                d=dist((world.ego.x,world.ego.y),(e.x,e.y))
                if d<200:
                    tsx,tsy=int(e.x-cx),int(e.y-cy)
                    bright=int(180*(1-d/200))
                    pygame.draw.circle(self.screen,(bright,0,0),(tsx,tsy),3)

        # ── POST-PROCESSING: Rain overlay ──
        if world.flags.get('rain'):
            for i,(rx,ry,rs,ra) in enumerate(self.rain_drops):
                pygame.draw.line(self.screen,(int(150*ra),int(170*ra),int(200*ra)),
                                 (int(rx),int(ry)),(int(rx-3),int(ry+rs)),1)
                self.rain_drops[i][0]=(rx-2)%SW
                self.rain_drops[i][1]=(ry+rs*3.5)%SH
            self.screen.blit(self.rain_tint,(0,0))
            self.screen.blit(self.fog_edge,(0,0))
            self.screen.blit(self.fog_edge,(SW-60,0))

        self._draw_hud(world)
        if self.show_minimap: self._draw_minimap(world)

    def _draw_ent(self, e, sx, sy, world):
        if not e.alive:
            return
        w,h=e.w,e.h
        if e.etype=='ego':
            self.screen.blit(self.ego_glow,
                             (sx-self.ego_glow.get_width()//2,
                              sy-self.ego_glow.get_height()//2))

        qa=round(e.angle/(math.pi/18))*(math.pi/18)
        if e.cached_angle!=qa or e.cached_sprite is None:
            e.cached_sprite=self.glb.get_entity_sprite(e.etype,e.color,e.angle,w,h)
            e.cached_angle=qa

        if e.cached_sprite:
            self.screen.blit(e.cached_sprite,e.cached_sprite.get_rect(center=(sx,sy)))
        else:
            sf=pygame.Surface((w,h),pygame.SRCALPHA)
            pygame.draw.rect(sf,e.color,(0,0,w,h),border_radius=3)
            if e.etype=='ego':
                pygame.draw.rect(sf,EGO_GLOW,(0,0,w,h),2,border_radius=3)
            rot=pygame.transform.rotate(sf,-math.degrees(e.angle)-90)
            self.screen.blit(rot,rot.get_rect(center=(sx,sy)))

        if e.etype=='ego':
            # Cheap orientation cue; no per-frame transparent Surface allocation.
            ex2=sx+int(math.cos(e.angle)*w)
            ey2=sy+int(math.sin(e.angle)*w)
            pygame.draw.line(self.screen,(150,220,255),(sx,sy),(ex2,ey2),2)

        if e.etype!='ego':
            if e.cached_label is None:
                e.cached_label=self.font_sm.render(e.label,True,WHITE)
                e.cached_label.set_alpha(180)
            lbl=e.cached_label
            self.screen.blit(lbl,(sx-lbl.get_width()//2,sy-e.h-8))

        if world.auto_mode and e.etype!='ego' and world.ego:
            d=dist((e.x,e.y),(world.ego.x,world.ego.y))
            if d<DET_R:
                th=1.-d/DET_R
                if th>.3:
                    r=int(e.radius+10)
                    pygame.draw.circle(self.screen,(255,int(255*(1-th)),0),(sx,sy),r,2)

    def _draw_forces(self, ego, cx, cy, forces):
        ex,ey = int(ego.x-cx), int(ego.y-cy); sc=.3
        cols = {'goal':GREEN,'obstacle':RED,'pothole':ORANGE,'road':YELLOW}
        for n,(fx,fy) in forces.items():
            if n=='threat': continue
            c=cols.get(n,WHITE)
            endx,endy = ex+int(fx*sc), ey+int(fy*sc)
            pygame.draw.line(self.screen,c,(ex,ey),(endx,endy),2)
            a=math.atan2(fy,fx)
            for da in [2.5,-2.5]:
                pygame.draw.line(self.screen,c,(endx,endy),
                    (endx-int(8*math.cos(a+da)),endy-int(8*math.sin(a+da))),2)

    def _draw_hud(self, world):
        ego = world.ego
        if not ego: return
        # Left panel
        ph = 260 + (30 if world.flags else 0)
        panel = pygame.Surface((340, ph), pygame.SRCALPHA); panel.fill((0,0,0,150))
        self.screen.blit(panel, (10,10))
        y = 15
        sc = SCENARIOS[world.scenario]
        name = sc[5]
        # Scenario name with effect tags
        tags = ""
        if world.flags.get('night'): tags += " [NIGHT]"
        if world.flags.get('rain'): tags += " [MONSOON]"
        self.screen.blit(self.font_lg.render(f"{world.scenario+1}. {name}{tags}", True, CYAN), (15,y)); y+=26
        mc = GREEN if world.auto_mode else YELLOW
        mt = "AUTONOMOUS" if world.auto_mode else "MANUAL"
        self.screen.blit(self.font_md.render(f"Mode: {mt} [SPACE]", True, mc), (15,y)); y+=22
        self.screen.blit(self.font_sm.render(f"Speed: {abs(ego.speed):.0f} px/s  |  Pos: ({ego.x:.0f}, {ego.y:.0f})", True, WHITE), (15,y)); y+=18
        loaded = len(world.glb.models)
        gc = GREEN if loaded>=6 else YELLOW if loaded>0 else RED
        self.screen.blit(self.font_sm.render(f"GLB Models: {loaded}/8 loaded", True, gc), (15,y)); y+=18

        if world.flags.get('night'):
            self.screen.blit(self.font_sm.render("Condition: Night — reduced visibility, headlight only", True, (100,100,200)), (15,y)); y+=16
        if world.flags.get('rain'):
            self.screen.blit(self.font_sm.render("Condition: Monsoon — wet roads, puddles, reduced grip", True, (100,150,200)), (15,y)); y+=16

        if world.auto_mode and world.planner:
            state = world.planner.state
            stc = {'CRUISING':GREEN,'AVOIDING':YELLOW,'EMERGENCY':RED,
                   'POTHOLE_AVOID':ORANGE,'WAITING':LGRAY,'OFF_ROAD':PINK,
                   'NO_PATH':RED,'IDLE':LGRAY}.get(state, WHITE)
            self.screen.blit(self.font_md.render(f"Planner: {state}", True, stc), (15,y)); y+=22
            threat = world.planner.forces.get('threat', 0)
            threat = clamp(float(threat), 0.0, 1.0)
            bw,bh = 220,14
            pygame.draw.rect(self.screen, DGRAY, (15,y,bw,bh))
            tc = (int(255*threat),int(255*(1-threat)),0)
            pygame.draw.rect(self.screen, tc, (15,y,int(bw*threat),bh))
            pygame.draw.rect(self.screen, WHITE, (15,y,bw,bh), 1)
            self.screen.blit(self.font_sm.render("Threat Level", True, WHITE), (240,y)); y+=20
            pl = len(world.planner.path)-world.planner.path_idx if world.planner.path else 0
            self.screen.blit(self.font_sm.render(f"Path waypoints: {pl}", True, WHITE), (15,y)); y+=18
        self.screen.blit(self.font_sm.render(f"Entities: {len(world.entities)}  |  Sim: {world.sim_speed:.1f}x", True, WHITE), (15,y)); y+=18

        # Right panel: scenario list
        rp = pygame.Surface((220, 230), pygame.SRCALPHA); rp.fill((0,0,0,140))
        self.screen.blit(rp, (SW-230, 10))
        self.screen.blit(self.font_md.render("Scenarios [1-9,0]:", True, WHITE), (SW-225, 15))
        for i, s in enumerate(SCENARIOS):
            clr = CYAN if i == world.scenario else LGRAY
            prefix = ">" if i == world.scenario else " "
            tag = ""
            if s[6].get('night'): tag += " *"
            if s[6].get('rain'): tag += " ~"
            self.screen.blit(self.font_sm.render(f"{prefix}{i+1 if i<9 else 0}. {s[5]}{tag}", True, clr),
                           (SW-225, 38 + i*19))

        # Bottom hints
        hints = "SPACE:Mode  M:Minimap  F:Forces  P:Pause  R:Reset  Click:SetGoal  +/-:Speed"
        self.screen.blit(self.font_sm.render(hints, True, (160,160,160)), (10, SH-22))
        if world.paused:
            pt = self.font_xl.render("PAUSED", True, YELLOW)
            self.screen.blit(pt, (SW//2-pt.get_width()//2, SH//2-20))

    def _draw_minimap(self, world):
        mw,mh=200,160; mx,my=SW-mw-10,SH-mh-10
        sx,sy=mw/WW,mh/WH
        self.minimap_timer += 1.0/FPS
        if self.minimap_surface is None or self.minimap_timer>=self.minimap_period:
            self.minimap_timer=0.0
            mm=pygame.Surface((mw,mh),pygame.SRCALPHA); mm.fill((0,0,0,160))
            for rd in world.roads:
                c={'highway':(100,100,110),'urban':(90,90,95),
                   'market':(80,80,85),'village':(110,95,65)}[rd.rtype]
                pygame.draw.line(mm,c,(int(rd.x1*sx),int(rd.y1*sy)),
                                 (int(rd.x2*sx),int(rd.y2*sy)),max(1,int(rd.w*sx)))
            if world.planner and world.planner.path:
                pts=[(int(p[0]*sx),int(p[1]*sy)) for p in world.planner.path]
                if len(pts)>1: pygame.draw.lines(mm,PATH_C,False,pts,1)
            for e in world.entities:
                if not e.alive: continue
                ex,ey=int(e.x*sx),int(e.y*sy)
                if e.etype=='ego':
                    pygame.draw.circle(mm,EGO_GLOW,(ex,ey),4)
                else:
                    c=RED if e.etype in ('pedestrian','animal') else YELLOW if e.etype=='two_wheeler' else LGRAY
                    pygame.draw.circle(mm,c,(ex,ey),2)
            vx,vy=int(world.cam_x*sx),int(world.cam_y*sy)
            pygame.draw.rect(mm,WHITE,(vx,vy,int(SW*sx),int(SH*sy)),1)
            self.minimap_surface=mm
        self.screen.blit(self.minimap_surface,(mx,my))
        pygame.draw.rect(self.screen,WHITE,(mx,my,mw,mh),1)

# ── MAIN ───────────────────────────────────────────────────
def main():
    screen = pygame.display.set_mode((SW, SH))
    pygame.display.set_caption("Indian Road AV Simulation — 10 Scenarios + GLB")
    clock = pygame.time.Clock()

    screen.fill((20,20,30))
    fl = pygame.font.SysFont('consolas', 20)
    screen.blit(fl.render("Loading GLB Models...", True, CYAN), (SW//2-120, SH//2-10))
    pygame.display.flip()

    glb = GLBManager(); glb.load_all()
    world = World(glb); renderer = Renderer(screen, glb)
    world.load_scenario(0)

    running = True
    while running:
        dt = min(clock.tick(FPS)/1000., .05)
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            elif event.type == pygame.KEYDOWN:
                k = event.key
                if k == pygame.K_ESCAPE: running = False
                elif k == pygame.K_SPACE:
                    world.auto_mode = not world.auto_mode
                    if world.auto_mode and world.planner: world.planner.last_plan_time = -999.0
                elif k == pygame.K_m: renderer.show_minimap = not renderer.show_minimap
                elif k == pygame.K_f: renderer.show_forces = not renderer.show_forces
                elif k == pygame.K_p: world.paused = not world.paused
                elif k == pygame.K_r: world.load_scenario(world.scenario); renderer.road_dirty = True
                elif k in (pygame.K_EQUALS, pygame.K_PLUS): world.sim_speed = min(4., world.sim_speed+.5)
                elif k == pygame.K_MINUS: world.sim_speed = max(.25, world.sim_speed-.5)
                elif k == pygame.K_1: world.load_scenario(0); renderer.road_dirty = True
                elif k == pygame.K_2: world.load_scenario(1); renderer.road_dirty = True
                elif k == pygame.K_3: world.load_scenario(2); renderer.road_dirty = True
                elif k == pygame.K_4: world.load_scenario(3); renderer.road_dirty = True
                elif k == pygame.K_5: world.load_scenario(4); renderer.road_dirty = True
                elif k == pygame.K_6: world.load_scenario(5); renderer.road_dirty = True
                elif k == pygame.K_7: world.load_scenario(6); renderer.road_dirty = True
                elif k == pygame.K_8: world.load_scenario(7); renderer.road_dirty = True
                elif k == pygame.K_9: world.load_scenario(8); renderer.road_dirty = True
                elif k == pygame.K_0: world.load_scenario(9); renderer.road_dirty = True
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if world.auto_mode and world.planner:
                    mx,my = event.pos
                    world.planner.set_goal(mx+world.cam_x, my+world.cam_y)
                    world.planner.path = []; world.planner.path_idx = 0
        world.update(dt)
        renderer.draw(world)
        pygame.display.flip()
    pygame.quit(); sys.exit()

if __name__ == "__main__":
    main()