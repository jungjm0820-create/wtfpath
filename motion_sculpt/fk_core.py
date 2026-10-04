"""fk_core — Stateless pure-FK F-Curve direct parsing (numpy).

No frame_set(), no depsgraph evaluation. Reads F-Curve data directly,
composes FK chain matrices: global = parent_global @ rest_rel @ offset.

offset = T(loc) @ R(rot) @ S(scale), rot from euler/quaternion F-Curves.
If C++ extension is built (cpp/ -> motion_core_cpp.pyd), heavy loops
(sample_trajectory, skin_deform) dispatch there. Otherwise numpy fallback.

Rest cache must be rebuilt when armature structure changes.
"""

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Euler, Vector

try:
    import motion_core_cpp as _cpp  # built .pyd next to addon or on sys.path
    HAS_CPP = True
except Exception:
    _cpp = None
    HAS_CPP = False

# ---------------------------------------------------------------- rest cache

_rest_cache = {}  # arm_name -> {"parent":{}, "rest_rel":{name:4x4 np}, "rest_global":{...}, "length":{...}}


def build_rest_cache(arm_obj):
    """Precompute rest relative matrices from data.bones (armature space)."""
    assert arm_obj.type == 'ARMATURE'
    parent = {}
    rest_global = {}
    rest_rel = {}
    length = {}
    for b in arm_obj.data.bones:
        parent[b.name] = b.parent.name if b.parent else None
        rest_global[b.name] = np.array(b.matrix_local, dtype=np.float64)
        length[b.name] = float(b.length)
    for name, mg in rest_global.items():
        p = parent[name]
        if p is None:
            rest_rel[name] = mg.copy()
        else:
            rest_rel[name] = np.linalg.inv(rest_global[p]) @ mg
    _rest_cache[arm_obj.name] = {
        "parent": parent, "rest_global": rest_global,
        "rest_rel": rest_rel, "length": length,
    }
    return _rest_cache[arm_obj.name]


def get_rest_cache(arm_obj):
    c = _rest_cache.get(arm_obj.name)
    if c is None:
        c = build_rest_cache(arm_obj)
    return c


def invalidate_cache(arm_name=None):
    if arm_name:
        _rest_cache.pop(arm_name, None)
    else:
        _rest_cache.clear()

# ---------------------------------------------------------------- fcurve helpers

_EULER_ORDER = {'XYZ': 'XYZ', 'XZY': 'XZY', 'YXZ': 'YXZ', 'YZX': 'YZX',
                'ZXY': 'ZXY', 'ZYX': 'ZYX'}


def _find_action(arm_obj):
    # Blender 4.4+: animation_data.action ; layered actions compatible path
    ad = getattr(arm_obj, "animation_data", None)
    if ad is None:
        return None
    return getattr(ad, "action", None)


def index_fcurves(arm_obj):
    """Map (bone_name, channel) -> list of 3-4 FCurves or None entries."""
    out = {}
    act = _find_action(arm_obj)
    if act is None:
        return out
    for fc in act.fcurves:
        dp = fc.data_path  # e.g. pose.bones["Hand"].rotation_euler
        if not dp.startswith('pose.bones['):
            continue
        try:
            bname = dp.split('"')[1]
            chan = dp.rsplit(".", 1)[1]
        except Exception:
            continue
        out.setdefault((bname, chan), {})[fc.array_index] = fc
    return out


def eval_fcurve(fc, frame):
    if fc is None:
        return None
    try:
        return float(fc.evaluate(frame))
    except Exception:
        pass
    # manual fallback: nearest key
    try:
        kps = fc.keyframe_points
        if len(kps) == 0:
            return None
        best = min(kps, key=lambda k: abs(k.co.x - frame))
        return float(best.co.y)
    except Exception:
        return None


def _bone_pose_offset(arm_obj, pose_bone, bname, fc_index, frame):
    """Return 4x4 numpy offset matrix T*R*S for one bone at frame."""
    # defaults = current pose values (so unkeyed channels still work)
    loc = list(pose_bone.location)
    scale = list(pose_bone.scale)
    rot_mode = pose_bone.rotation_mode
    if rot_mode == 'QUATERNION':
        quat = list(pose_bone.rotation_quaternion)
    else:
        eul = pose_bone.rotation_euler
        quat = None
        eul_vals = [eul.x, eul.y, eul.z]
        order = eul.order if hasattr(eul, 'order') else 'XYZ'

    for chan, idxmap in (
        ("location", None), ("rotation_euler", None),
        ("rotation_quaternion", None), ("scale", None),
    ):
        pass  # lookup below individually for speed/clarity

    def ch_vals(chan, n):
        idxmap = fc_index.get((bname, chan))
        if not idxmap:
            return None
        return [eval_fcurve(idxmap.get(i), frame) for i in range(n)]

    lv = ch_vals("location", 3)
    if lv and all(v is not None for v in lv):
        loc = lv
    sv = ch_vals("scale", 3)
    if sv and all(v is not None for v in sv):
        scale = sv

    if rot_mode == 'QUATERNION':
        qv = ch_vals("rotation_quaternion", 4)
        if qv and all(v is not None for v in qv):
            quat = qv
        M = Matrix.Translation(Vector(loc)) @ Quaternion(quat).to_matrix().to_4x4()
    else:
        ev = ch_vals("rotation_euler", 3)
        if ev and all(v is not None for v in ev):
            eul_vals = ev
        M = (Matrix.Translation(Vector(loc))
             @ Euler((eul_vals[0], eul_vals[1], eul_vals[2]), order).to_matrix().to_4x4())
    M = M @ Matrix.Scale(scale[0], 4, (1, 0, 0)) \
          @ Matrix.Scale(scale[1], 4, (0, 1, 0)) \
          @ Matrix.Scale(scale[2], 4, (0, 0, 1))
    return np.array(M, dtype=np.float64)

# ---------------------------------------------------------------- FK evaluation

def chain_to_root(cache, bone_name):
    chain = [bone_name]
    p = cache["parent"].get(bone_name)
    while p:
        chain.append(p)
        p = cache["parent"].get(p)
    chain.reverse()
    return chain


def evaluate_globals(arm_obj, frame, bones=None):
    """Return {bone: 4x4 np global head matrix} without touching timeline.

    bones: optional subset; parents are auto-included.
    """
    cache = get_rest_cache(arm_obj)
    fc_index = index_fcurves(arm_obj)
    pose = arm_obj.pose.bones

    wanted = set(bones) if bones else set(cache["parent"].keys())
    # include ancestors
    full = set(wanted)
    for b in list(wanted):
        full.update(chain_to_root(cache, b))

    # topo order: roots first (chain_to_root gives root-first; sort by depth)
    def depth(n):
        d, p = 0, cache["parent"].get(n)
        while p:
            d += 1
            p = cache["parent"].get(p)
        return d
    ordered = sorted(full, key=depth)

    globals_ = {}
    I = np.eye(4)
    for bname in ordered:
        pb = pose.get(bname)
        if pb is None:
            continue
        off = _bone_pose_offset(arm_obj, pb, bname, fc_index, frame)
        rel = cache["rest_rel"][bname]
        local = rel @ off  # note: Blender order is rest-relative then pose offset
        # Correction: pose offset applies in bone-local space AFTER rest.
        # Standard formula used by many FK tools: global = parent_global @ rest_rel @ offset
        p = cache["parent"].get(bname)
        pg = globals_.get(p, I) if p else I
        globals_[bname] = pg @ local
    return globals_


def head_pos(gmat):
    return gmat[:3, 3].copy()


def tail_pos(gmat, length):
    # bone-local tail = (0, length, 0)
    v = gmat @ np.array([0.0, length, 0.0, 1.0])
    return v[:3].copy()


def sample_trajectory(arm_obj, bone_name, frame_start, frame_end, use_tail=True):
    """Sample head/tail world positions over a frame range. Returns (N,3) float."""
    if HAS_CPP and hasattr(_cpp, "sample_trajectory"):
        try:
            return _cpp.sample_trajectory(arm_obj.name, bone_name, frame_start, frame_end)
        except Exception:
            pass
    cache = get_rest_cache(arm_obj)
    length = cache["length"].get(bone_name, 1.0)
    pts = []
    for f in range(frame_start, frame_end + 1):
        g = evaluate_globals(arm_obj, float(f), bones=[bone_name])
        m = g.get(bone_name)
        if m is None:
            pts.append(np.zeros(3))
        else:
            pts.append(tail_pos(m, length) if use_tail else head_pos(m))
    return np.array(pts, dtype=np.float64)


def write_pose_rotations(arm_obj, rot_dict, frame, key=True):
    """rot_dict: bone -> Quaternion (local pose rotation). Writes + optional keying."""
    scn = bpy.context.scene
    for bname, q in rot_dict.items():
        pb = arm_obj.pose.bones.get(bname)
        if pb is None:
            continue
        if pb.rotation_mode == 'QUATERNION':
            pb.rotation_quaternion = q
            if key:
                pb.keyframe_insert("rotation_quaternion", frame=frame)
        else:
            e = q.to_euler(pb.rotation_mode if pb.rotation_mode in _EULER_ORDER else 'XYZ')
            pb.rotation_euler = e
            if key:
                pb.keyframe_insert("rotation_euler", frame=frame)
    try:
        scn.frame_set(scn.frame_current)  # refresh depsgraph once after batch write
    except Exception:
        pass
