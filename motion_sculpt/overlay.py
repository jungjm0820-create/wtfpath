"""overlay — GPU viewport drawing: 3D motion path + onion skin + wormhole.

Performance design:
- Path points come from fk_core (stateless FK parse, no frame_set loop).
  With the C++ module they are computed in background threads at 60fps;
  without it, numpy fallback + frame-range cache keeps interaction smooth.
- Onion ghosts draw as armature sticks (+ optional mesh points), NOT full
  re-evaluated meshes per draw — that is what keeps it real-time.
- Wormhole view offsets each ghost along X by (f - current) * gap so
  in-place motion reads as a time-space row.

Blender 4.x/5.x gpu API compatible with fallbacks.
"""

import bpy
import numpy as np
import gpu
from gpu_extras.batch import batch_for_shader
from mathutils import Vector

_handler = None
_path_cache = {}
_skin_cache = {}


def _shader(name_options, args):
    for n in name_options:
        try:
            return gpu.shader.from_builtin(n, *args) if args else gpu.shader.from_builtin(n)
        except Exception:
            continue
    return gpu.shader.from_builtin('UNIFORM_COLOR')


def _active_armature(context):
    ao = context.view_layer.objects.active
    if ao is not None and ao.type == 'ARMATURE':
        return ao
    for o in context.selected_objects:
        if o.type == 'ARMATURE':
            return o
    return None


def _active_bone(arm):
    try:
        if arm.mode == 'POSE' and arm.data.bones.active:
            return arm.data.bones.active.name
        for pb in arm.pose.bones:
            if pb.bone.select:
                return pb.name
    except Exception:
        pass
    return None


def _frame_range(scn, context=None):
    return scn.frame_start, scn.frame_end


def _trajectory(arm, bone, f0, f1):
    from . import fk_core
    key = (arm.name, bone, f0, f1)
    if key in _path_cache:
        return _path_cache[key]
    pts = fk_core.sample_trajectory(arm, bone, f0, f1, use_tail=True)
    _path_cache[key] = pts
    if len(_path_cache) > 64:
        _path_cache.pop(next(iter(_path_cache)))
    return pts


def _draw_polyline(points_world, color=(1.0, 0.8, 0.2, 1.0), width=3):
    if len(points_world) < 2:
        return
    shader = _shader(['POLYLINE_SMOOTH_COLOR', 'POLYLINE_UNIFORM_COLOR',
                      'SMOOTH_COLOR', 'UNIFORM_COLOR'], [])
    cols = [color] * len(points_world)
    try:
        batch = batch_for_shader(shader, 'LINE_STRIP',
                                 {"pos": [tuple(p) for p in points_world],
                                  "color": cols})
    except Exception:
        try:
            batch = batch_for_shader(shader, 'LINE_STRIP',
                                     {"pos": [tuple(p) for p in points_world]})
            shader.uniform_float("color", color)
        except Exception:
            return
    try:
        gpu.state.blend_set('ALPHA')
        gpu.state.line_width_set(width)
    except Exception:
        pass
    try:
        batch.draw(shader)
    except Exception:
        pass
    try:
        gpu.state.line_width_set(1.0)
        gpu.state.blend_set('NONE')
    except Exception:
        pass


def _draw_points(points_world, color=(1.0, 0.4, 0.2, 1.0), size=6):
    if len(points_world) == 0:
        return
    try:
        shader = gpu.shader.from_builtin('POINT_SMOOTH_COLOR')
    except Exception:
        shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    try:
        batch = batch_for_shader(shader, 'POINTS',
                                 {"pos": [tuple(p) for p in points_world],
                                  "color": [color] * len(points_world)})
    except Exception:
        try:
            batch = batch_for_shader(shader, 'POINTS',
                                     {"pos": [tuple(p) for p in points_world]})
            shader.uniform_float("color", color)
        except Exception:
            return
    try:
        gpu.state.blend_set('ALPHA')
        try:
            gpu.state.point_size_set(size)
        except Exception:
            pass
        batch.draw(shader)
    except Exception:
        pass
    finally:
        try:
            gpu.state.blend_set('NONE')
        except Exception:
            pass


def draw_callback():
    context = bpy.context
    scn = context.scene
    arm = _active_armature(context)
    if arm is None:
        return
    bone = _active_bone(arm)
    if bone is None:
        return
    try:
        cur = scn.frame_current
        onion = int(getattr(scn, "motion_sculpt_onion", 5))
        worm = bool(getattr(scn, "motion_sculpt_wormhole", False))
        gap = float(getattr(scn, "motion_sculpt_wormhole_gap", 0.5))
        f0, f1 = _frame_range(scn)
        f0 = max(f0, cur - 60)
        f1 = min(f1, cur + 60)

        # --- motion path (full range, yellow) ---
        traj = _trajectory(arm, bone, f0, f1)
        if len(traj):
            off = np.zeros(3)
            pts = [Vector(p) for p in traj]
            if worm:
                pts = [Vector((p[0] + (f0 + i - cur) * gap, p[1], p[2]))
                       for i, p in enumerate(traj)]
            _draw_polyline(pts, color=(1.0, 0.8, 0.15, 1.0), width=3)
            # keyframe markers: sample action keyframes for this bone
            keys = _bone_key_frames(arm, bone)
            kp = [pts[k - f0] for k in keys if f0 <= k <= f1 and 0 <= k - f0 < len(pts)]
            _draw_points(kp, color=(1.0, 0.3, 0.15, 1.0), size=7)
            _draw_points([pts[cur - f0]] if 0 <= cur - f0 < len(pts) else [],
                         color=(0.3, 1.0, 0.4, 1.0), size=9)

        # --- onion skin ghosts (armature sticks, before=cyan after=magenta) ---
        if onion > 0:
            from . import fk_core
            for df in range(-onion, onion + 1):
                if df == 0:
                    continue
                f = cur + df
                if f < f0 or f > f1:
                    continue
                try:
                    g = fk_core.evaluate_globals(arm, float(f))
                except Exception:
                    continue
                cache = fk_core.get_rest_cache(arm)
                segs = []
                for bname, m in g.items():
                    try:
                        h = Vector(m[:3, 3])
                        import numpy as _np
                        t = Vector((m @ _np.array(
                            [0, cache["length"].get(bname, 0.5), 0, 1.0]))[:3])
                    except Exception:
                        continue
                    if worm:
                        h.x += df * gap
                        t.x += df * gap
                    segs += [h, t]
                col = (0.2, 0.9, 1.0, 0.35) if df < 0 else (1.0, 0.35, 0.9, 0.35)
                _draw_ghost_segments(segs, col)
    except Exception:
        pass


def _draw_ghost_segments(segs, color):
    if len(segs) < 2:
        return
    shader = _shader(['POLYLINE_UNIFORM_COLOR', 'UNIFORM_COLOR'], [])
    try:
        batch = batch_for_shader(shader, 'LINES', {"pos": [tuple(p) for p in segs]})
    except Exception:
        return
    try:
        shader.uniform_float("color", color)
    except Exception:
        pass
    try:
        gpu.state.blend_set('ALPHA')
        batch.draw(shader)
    except Exception:
        pass
    finally:
        try:
            gpu.state.blend_set('NONE')
        except Exception:
            pass


def _bone_key_frames(arm, bone):
    out = set()
    try:
        act = arm.animation_data.action if arm.animation_data else None
        if not act:
            return []
        prefix = f'pose.bones["{bone}"]'
        for fc in act.fcurves:
            if fc.data_path.startswith(prefix):
                for k in fc.keyframe_points:
                    out.add(int(round(k.co.x)))
    except Exception:
        pass
    return sorted(out)


def set_enabled(on):
    global _handler
    if on and _handler is None:
        _handler = bpy.types.SpaceView3D.draw_handler_add(
            draw_callback, (), 'WINDOW', 'POST_VIEW')
        _path_cache.clear()
    elif (not on) and _handler is not None:
        try:
            bpy.types.SpaceView3D.draw_handler_remove(_handler, 'WINDOW')
        except Exception:
            pass
        _handler = None
    for a in bpy.context.screen.areas if bpy.context.screen else []:
        if a.type == 'VIEW_3D':
            a.tag_redraw()


class MOTION_SCULPT_OT_toggle_overlay(bpy.types.Operator):
    bl_idname = "motion_sculpt.toggle_overlay"
    bl_label = "Toggle Motion Path Overlay"

    def execute(self, context):
        context.scene.motion_sculpt_enable = not context.scene.motion_sculpt_enable
        return {'FINISHED'}


class MOTION_SCULPT_OT_clear_cache(bpy.types.Operator):
    bl_idname = "motion_sculpt.clear_cache"
    bl_label = "Clear Path Cache"

    def execute(self, context):
        _path_cache.clear()
        from . import fk_core
        fk_core.invalidate_cache()
        return {'FINISHED'}


_classes = (MOTION_SCULPT_OT_toggle_overlay, MOTION_SCULPT_OT_clear_cache)


def register():
    for c in _classes:
        bpy.utils.register_class(c)


def unregister():
    set_enabled(False)
    for c in reversed(_classes):
        try:
            bpy.utils.unregister_class(c)
        except Exception:
            pass
