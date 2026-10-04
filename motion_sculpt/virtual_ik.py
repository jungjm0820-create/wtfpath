"""virtual_ik — FK-based virtual IK (CCD solver).

Data stays pure FK rotations. When the animator drags a mesh point or a
motion-path point, we run a momentary CCD solve over the FK chain and write
the result back as FK rotation keyframes. No IK constraints needed, so Mixamo
/ Auto-Rig-Pro-in-FK-mode rigs all work identically.

CCD in world space, corrections converted to bone-local quaternions.
"""

import numpy as np
from mathutils import Vector, Quaternion, Matrix

from . import fk_core


def _quat_between(a: Vector, b: Vector):
    a = a.normalized()
    b = b.normalized()
    d = max(-1.0, min(1.0, a.dot(b)))
    if d > 0.99999:
        return Quaternion((1, 0, 0, 0))
    if d < -0.99999:
        # 180deg: pick any orthogonal axis
        axis = Vector((1, 0, 0)).cross(a)
        if axis.length < 1e-6:
            axis = Vector((0, 1, 0)).cross(a)
        return Quaternion(axis.normalized(), np.pi)
    axis = a.cross(b)
    return Quaternion(axis, np.arccos(d))


def solve_ccd(arm_obj, chain, target_world, frame, iterations=12, damping=1.0,
              end_bone_is_tip=True):
    """Solve FK rotations so chain tip reaches target_world.

    chain: [root ... tip] bone names (tip = controlled bone).
    Returns {bone: Quaternion local rotation} (does NOT write).
    """
    import bpy
    cache = fk_core.get_rest_cache(arm_obj)
    g = fk_core.evaluate_globals(arm_obj, float(frame), bones=[chain[-1]])
    length = cache["length"]

    # current local rotations as quats
    local_q = {}
    for b in chain:
        pb = arm_obj.pose.bones.get(b)
        if pb is None:
            continue
        if pb.rotation_mode == 'QUATERNION':
            local_q[b] = Quaternion(pb.rotation_quaternion)
        else:
            local_q[b] = pb.rotation_euler.to_quaternion()

    target = Vector(target_world)

    def rebuild():
        # rebuild globals from local_q + rest + loc/scale offsets (loc frozen)
        fc_index = fk_core.index_fcurves(arm_obj)
        pose = arm_obj.pose.bones
        # need per-bone offset with overridden rotation
        g2 = {}
        I = np.eye(4)
        ordered = chain  # already root-first expected
        # include true parents above root for correct world transform
        root_parent_chain = fk_core.chain_to_root(cache, chain[0])[:-1]
        full_order = root_parent_chain + list(ordered)
        tmp = fk_core.evaluate_globals(arm_obj, float(frame),
                                       bones=[full_order[-1]] if full_order else None)
        for p in root_parent_chain:
            g2[p] = tmp.get(p, np.eye(4))
        for b in ordered:
            pb = pose.get(b)
            loc = Vector(pb.location) if pb else Vector((0, 0, 0))
            off = (Matrix.Translation(loc)
                   @ local_q[b].to_matrix().to_4x4())
            off = np.array(off, dtype=np.float64)
            parent = cache["parent"].get(b)
            pg = g2.get(parent, np.eye(4)) if parent else np.eye(4)
            g2[b] = pg @ cache["rest_rel"][b] @ off
        return g2

    gcur = rebuild()

    def head_of(b):
        return Vector(gcur[b][:3, 3])

    def tip_of(b):
        v = gcur[b] @ np.array([0.0, length.get(b, 1.0), 0.0, 1.0])
        return Vector(v[:3])

    for _ in range(iterations):
        tip = tip_of(chain[-1])
        if (tip - target).length < 1e-5:
            break
        for b in reversed(chain):
            tip = tip_of(chain[-1])
            head = head_of(b)
            to_tip = tip - head
            to_tgt = target - head
            if to_tip.length < 1e-9 or to_tgt.length < 1e-9:
                continue
            # world-space correction
            q_world = _quat_between(to_tip, to_tgt)
            if damping < 1.0:
                q_world = Quaternion.slerp(
                    Quaternion((1, 0, 0, 0)), q_world, damping)
            # to bone-local: q_local_delta = parent_world_rot^-1 * q_world * parent_world_rot
            # approximate using current global rotation of bone
            import mathutils
            gmat = Matrix(gcur[b].tolist())
            grow = gmat.to_quaternion()
            q_local_delta = grow.inverted() @ q_world @ grow
            local_q[b] = (q_local_delta @ local_q[b]).normalized()
            gcur = rebuild()
            if (tip_of(chain[-1]) - target).length < 1e-5:
                break
    return local_q


def _placeholder():
    pass


def apply_drag(arm_obj, chain, target_world, frame, key=True):
    """Solve + write FK rotations at frame. Returns solved dict."""
    q = solve_ccd(arm_obj, chain, target_world, frame)
    fk_core.write_pose_rotations(arm_obj, {b: q[b] for b in chain if b in q},
                                 frame, key=key)
    return q


def fk_chain_for_bone(arm_obj, bone_name, max_up=4):
    """Walk up at most max_up parents: returns root-first chain."""
    cache = fk_core.get_rest_cache(arm_obj)
    chain = [bone_name]
    p = cache["parent"].get(bone_name)
    n = 0
    while p and n < max_up - 1:
        chain.append(p)
        p = cache["parent"].get(p)
        n += 1
    chain.reverse()
    return chain
