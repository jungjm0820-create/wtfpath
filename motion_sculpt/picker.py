"""picker — mesh-surface weight pick (rigless UX).

Hover the character mesh -> read vertex weights at the hit face -> pick the
dominant FK bone. Click selects it; drag sculpts it via virtual IK.
No controller-bone hunting.
"""

import bpy
import bmesh
from mathutils import Vector
from bpy_extras import view3d_utils


def _depsgraph_raycast(context, origin, direction):
    deps = context.evaluated_depsgraph_get()
    try:
        # Blender 4.x signature: scene.ray_cast(view_layer.depsgraph, origin, direction)
        return context.scene.ray_cast(deps, origin, direction)
    except Exception:
        try:
            return context.scene.ray_cast(
                view_layer=context.view_layer, origin=origin, direction=direction)
        except Exception:
            return (False, None, None, None, None, None)


def pick_bone_at_hit(mesh_obj, face_index):
    """Return (bone_name, weight) of dominant deform bone for a polygon."""
    mesh = mesh_obj.data
    if face_index is None or face_index < 0 or face_index >= len(mesh.polygons):
        return None, 0.0
    poly = mesh.polygons[face_index]
    vg = mesh_obj.vertex_groups
    accum = {}
    for vi in poly.vertices:
        v = mesh.vertices[vi]
        for g in v.groups:
            try:
                name = vg[g.group].name
            except Exception:
                continue
            accum[name] = accum.get(name, 0.0) + float(g.weight)
    if not accum:
        return None, 0.0
    # keep only real pose bones
    arm = _armature_of(mesh_obj)
    if arm is not None:
        pbnames = set(arm.pose.bones.keys())
        accum = {k: v for k, v in accum.items() if k in pbnames}
        if not accum:
            return None, 0.0
    best = max(accum.items(), key=lambda kv: kv[1])
    return best[0], best[1] / max(1, len(poly.vertices))


def _armature_of(mesh_obj):
    for mod in getattr(mesh_obj, "modifiers", []):
        if mod.type == 'ARMATURE' and mod.object is not None:
            return mod.object
    # fallback: selected armature
    for o in bpy.context.selected_objects:
        if o.type == 'ARMATURE':
            return o
    ao = bpy.context.view_layer.objects.active
    if ao is not None and ao.type == 'ARMATURE':
        return ao
    return None


def select_fk_bone(arm_obj, bone_name):
    try:
        bpy.ops.object.mode_set(mode='OBJECT')
    except Exception:
        pass
    bpy.context.view_layer.objects.active = arm_obj
    arm_obj.select_set(True)
    try:
        bpy.ops.object.mode_set(mode='POSE')
    except Exception:
        return False
    for pb in arm_obj.pose.bones:
        pb.bone.select = (pb.name == bone_name)
    arm_obj.data.bones.active = arm_obj.data.bones.get(bone_name)
    return True


class MOTION_SCULPT_OT_weight_pick(bpy.types.Operator):
    """Hover mesh to preview dominant bone, click to select, drag to sculpt (virtual IK)"""
    bl_idname = "motion_sculpt.weight_pick"
    bl_label = "Weight Pick / Sculpt"
    bl_options = {'REGISTER', 'UNDO', 'BLOCKING', 'GRAB_CURSOR'}

    _hover_bone = ""

    def invoke(self, context, event):
        context.window_manager.modal_handler_add(self)
        self._dragging = False
        self._chain = []
        self._arm = None
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        region = context.region
        rv3d = context.region_data
        if region is None or rv3d is None:
            return {'PASS_THROUGH'}
        coord = (event.mouse_region_x, event.mouse_region_y)

        if event.type in {'RIGHTMOUSE', 'ESC'}:
            return {'CANCELLED'}

        origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
        direction = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)

        if event.type == 'MOUSEMOVE':
            hit, loc, nrm, idx, obj, mtx = _depsgraph_raycast(context, origin, direction)
            if hit and obj is not None and getattr(obj, "type", "") == 'MESH':
                bone, w = pick_bone_at_hit(obj.evaluated_get(
                    context.evaluated_depsgraph_get()) if hasattr(obj, "evaluated_get") else obj,
                    idx if isinstance(idx, int) else -1)
                # evaluated mesh has no vertex groups; re-read from original
                orig = obj.original if hasattr(obj, "original") else obj
                if orig is not None and orig.type == 'MESH':
                    bone, w = pick_bone_at_hit(orig, idx if isinstance(idx, int) else -1)
                if bone:
                    self._hover_bone = f"{orig.name} -> {bone} ({w:.2f})"
                    context.area.header_text_set(f"MotionSculpt pick: {self._hover_bone}  | click=select  drag=sculpt")
                    self._hover = (orig, bone)
                else:
                    context.area.header_text_set(None)
            else:
                context.area.header_text_set(None)
            if getattr(self, "_dragging", False):
                self._sculpt_to(context, coord)
            return {'RUNNING_MODAL'}

        if event.type == 'LEFTMOUSE':
            if event.value == 'PRESS':
                hit, loc, nrm, idx, obj, mtx = _depsgraph_raycast(context, origin, direction)
                if hit and obj is not None:
                    orig = obj.original if hasattr(obj, "original") else obj
                    if orig.type == 'MESH':
                        bone, w = pick_bone_at_hit(orig, idx if isinstance(idx, int) else -1)
                        if bone:
                            arm = _armature_of(orig)
                            if arm is not None:
                                select_fk_bone(arm, bone)
                                from . import virtual_ik
                                self._arm = arm
                                self._chain = virtual_ik.fk_chain_for_bone(arm, bone)
                                self._dragging = True
                                self._plane = (loc, view3d_utils.region_2d_to_vector_3d(
                                    region, rv3d, coord))
                                return {'RUNNING_MODAL'}
                return {'PASS_THROUGH'}
            elif event.value == 'RELEASE':
                self._dragging = False
                context.area.header_text_set(None)
                return {'FINISHED'}
        return {'RUNNING_MODAL'}

    def _sculpt_to(self, context, coord):
        from . import virtual_ik
        from bpy_extras import view3d_utils as vutils
        region, rv3d = context.region, context.region_data
        origin = vutils.region_2d_to_origin_3d(region, rv3d, coord)
        vec = vutils.region_2d_to_vector_3d(region, rv3d, coord)
        # project onto view-perpendicular plane through current tip
        from . import fk_core
        g = fk_core.evaluate_globals(self._arm, float(context.scene.frame_current),
                                     bones=[self._chain[-1]])
        m = g.get(self._chain[-1])
        if m is None:
            return
        import numpy as np
        tip = Vector((m @ np.array([0, fk_core.get_rest_cache(self._arm)["length"].get(
            self._chain[-1], 1.0), 0, 1.0]))[:3])
        # closest point on mouse ray to tip -> drag target
        # solve: origin + t*vec closest to tip
        t = (tip - origin).dot(vec)
        target = origin + vec * t
        virtual_ik.apply_drag(self._arm, self._chain, target,
                              float(context.scene.frame_current), key=False)
        context.view_layer.update()


_classes = (MOTION_SCULPT_OT_weight_pick,)


def register():
    for c in _classes:
        bpy.utils.register_class(c)


def unregister():
    for c in reversed(_classes):
        try:
            bpy.utils.unregister_class(c)
        except Exception:
            pass
