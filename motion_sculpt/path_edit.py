"""path_edit — grab motion-path points + motion brush.

Grab: click near the 3D trajectory line, drag -> virtual-IK solves the FK
chain for that frame and keys it. This keeps the path <-> F-Curve round-trip:
path edit => CCD => FK rotations => F-Curve keys => bezier handles follow.

Brush: stroke over a path segment -> smooth/relax surrounding keyframe
positions by re-solving neighbouring frames toward the smoothed target
(sculpt-like feel, still pure FK data underneath).
"""

import bpy
import numpy as np
from mathutils import Vector
from bpy_extras import view3d_utils


def _arm_and_bone(context):
    ao = context.view_layer.objects.active
    if ao is None or ao.type != 'ARMATURE':
        for o in context.selected_objects:
            if o.type == 'ARMATURE':
                ao = o
                break
    if ao is None:
        return None, None
    bone = None
    try:
        if ao.data.bones.active:
            bone = ao.data.bones.active.name
    except Exception:
        pass
    return ao, bone


class MOTION_SCULPT_OT_grab_path(bpy.types.Operator):
    """Grab a motion-path point and drag it (virtual IK -> FK key)"""
    bl_idname = "motion_sculpt.grab_path"
    bl_label = "Grab Motion Path"
    bl_options = {'REGISTER', 'UNDO', 'BLOCKING', 'GRAB_CURSOR'}

    frame: bpy.props.IntProperty(default=-1)

    def invoke(self, context, event):
        from . import fk_core
        arm, bone = _arm_and_bone(context)
        if arm is None or bone is None:
            self.report({'WARNING'}, "Select an armature + active bone first")
            return {'CANCELLED'}
        scn = context.scene
        f0, f1 = scn.frame_start, scn.frame_end
        f0 = max(f0, scn.frame_current - 60)
        f1 = min(f1, scn.frame_current + 60)
        traj = fk_core.sample_trajectory(arm, bone, f0, f1)
        # find nearest path point to mouse ray
        region, rv3d = context.region, context.region_data
        origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, (event.mouse_region_x, event.mouse_region_y))
        vec = view3d_utils.region_2d_to_vector_3d(region, rv3d, (event.mouse_region_x, event.mouse_region_y))
        best_i, best_d = 0, 1e18
        for i, p in enumerate(traj):
            v = Vector(p) - origin
            t = v.dot(vec)
            closest = origin + vec * t
            d = (Vector(p) - closest).length
            if d < best_d:
                best_d, best_i = d, i
        self.f0 = f0
        self.frame = f0 + best_i
        scn.frame_set(self.frame)
        from . import virtual_ik
        self.arm = arm.name
        self.bone = bone
        self.chain = virtual_ik.fk_chain_for_bone(arm, bone)
        context.window_manager.modal_handler_add(self)
        context.area.header_text_set(f"Grab path f{self.frame} | move= sculpt  LMB=key+exit  RMB=cancel")
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type in {'RIGHTMOUSE', 'ESC'}:
            context.area.header_text_set(None)
            return {'CANCELLED'}
        arm = bpy.data.objects.get(getattr(self, "arm", ""))
        if arm is None:
            context.area.header_text_set(None)
            return {'CANCELLED'}
        if event.type == 'MOUSEMOVE':
            region, rv3d = context.region, context.region_data
            coord = (event.mouse_region_x, event.mouse_region_y)
            origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
            vec = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
            from . import fk_core, virtual_ik
            g = fk_core.evaluate_globals(arm, float(self.frame), bones=[self.bone])
            m = g.get(self.bone)
            if m is not None:
                L = fk_core.get_rest_cache(arm)["length"].get(self.bone, 1.0)
                tip = Vector((m @ np.array([0, L, 0, 1.0]))[:3])
                t = (tip - origin).dot(vec)
                target = origin + vec * t
                virtual_ik.apply_drag(arm, self.chain, target, float(self.frame), key=False)
                context.view_layer.update()
            return {'RUNNING_MODAL'}
        if event.type == 'LEFTMOUSE' and event.value == 'RELEASE':
            from . import virtual_ik, fk_core
            # commit: key current solved rotations
            g = fk_core.evaluate_globals(arm, float(self.frame), bones=[self.bone])
            for b in self.chain:
                pb = arm.pose.bones.get(b)
                if pb is None:
                    continue
                if pb.rotation_mode == 'QUATERNION':
                    try:
                        pb.keyframe_insert("rotation_quaternion", frame=self.frame)
                    except Exception:
                        pass
                else:
                    try:
                        pb.keyframe_insert("rotation_euler", frame=self.frame)
                    except Exception:
                        pass
            context.area.header_text_set(None)
            try:
                from .overlay import _path_cache
                _path_cache.clear()
            except Exception:
                pass
            return {'FINISHED'}
        return {'RUNNING_MODAL'}


class MOTION_SCULPT_OT_brush_smooth(bpy.types.Operator):
    """Motion brush: smooth trajectory around current frame (FK re-solve)"""
    bl_idname = "motion_sculpt.brush_smooth"
    bl_label = "Motion Brush (Smooth)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        from . import fk_core, virtual_ik
        arm, bone = _arm_and_bone(context)
        if arm is None or bone is None:
            self.report({'WARNING'}, "Select armature + bone")
            return {'CANCELLED'}
        scn = context.scene
        cur = scn.frame_current
        radius = int(getattr(scn, "motion_sculpt_brush_radius", 5))
        strength = float(getattr(scn, "motion_sculpt_brush_strength", 0.5))
        f0, f1 = max(scn.frame_start, cur - radius), min(scn.frame_end, cur + radius)
        traj = fk_core.sample_trajectory(arm, bone, f0, f1)
        if len(traj) == 0:
            return {'CANCELLED'}
        # box-smooth target positions
        sm = traj.copy()
        w = 3
        for i in range(len(traj)):
            a, b = max(0, i - w), min(len(traj), i + w + 1)
            sm[i] = traj[a:b].mean(axis=0)
        chain = virtual_ik.fk_chain_for_bone(arm, bone)
        for i, f in enumerate(range(f0, f1 + 1)):
            fall = max(0.0, 1.0 - abs(f - cur) / max(1, radius))
            target = traj[i] * (1 - strength * fall) + sm[i] * (strength * fall)
            virtual_ik.apply_drag(arm, chain, Vector(target), float(f), key=True)
        try:
            from .overlay import _path_cache
            _path_cache.clear()
        except Exception:
            pass
        return {'FINISHED'}


_classes = (MOTION_SCULPT_OT_grab_path, MOTION_SCULPT_OT_brush_smooth)


def register():
    for c in _classes:
        bpy.utils.register_class(c)


def unregister():
    for c in reversed(_classes):
        try:
            bpy.utils.unregister_class(c)
        except Exception:
            pass
