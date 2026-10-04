# Motion Sculpt — FK-first Motion Path + Virtual IK + Weight Pick
# Blender 5.2+ / 4.x compatible. Single-folder addon: copy `motion_sculpt/` into
# Blender addons or install the repo root as .zip.
#
# Goal (from user request):
# 1) Motion Path is king — viewport 3D trajectory, directly grabbable.
# 2) FK is faster than IK for evaluation -> all prediction is pure FK F-Curve parsing.
# 3) Virtual IK (CCD): drag mesh/trajectory -> auto-solved back into FK rotations.
# 4) Rigless UX: click visible mesh -> dominant vertex-weight bone auto-selected.
# 5) C++ core on GitHub for 60fps; Python/numpy fallback so addon works uncompiled.

bl_info = {
    "name": "Motion Sculpt (FK Path + Virtual IK)",
    "author": "Motion Sculpt Lab",
    "version": (0, 1, 0),
    "blender": (4, 0, 0),
    "location": "View3D > Sidebar > Motion Sculpt",
    "description": "Lightweight FK motion-path sculpting: weight-pick, grabbable 3D path, virtual IK, onion skin",
    "category": "Animation",
}

import bpy

from . import fk_core, virtual_ik, picker, overlay, path_edit, ui


_modules = (fk_core, virtual_ik, picker, overlay, path_edit, ui)


def register():
    for m in _modules:
        if hasattr(m, "register"):
            m.register()
    # start persistent overlay handler lazily via scene property
    bpy.types.Scene.motion_sculpt_enable = bpy.props.BoolProperty(
        name="Overlay", default=False,
        update=lambda self, ctx: overlay.set_enabled(self.motion_sculpt_enable),
    )
    bpy.types.Scene.motion_sculpt_onion = bpy.props.IntProperty(
        name="Onion Frames", default=5, min=0, max=30)
    bpy.types.Scene.motion_sculpt_wormhole = bpy.props.BoolProperty(
        name="Wormhole View", default=False)
    bpy.types.Scene.motion_sculpt_wormhole_gap = bpy.props.FloatProperty(
        name="Gap", default=0.5, min=0.05, max=5.0)
    bpy.types.Scene.motion_sculpt_brush_radius = bpy.props.IntProperty(
        name="Brush Radius", default=5, min=1, max=30)
    bpy.types.Scene.motion_sculpt_brush_strength = bpy.props.FloatProperty(
        name="Brush Strength", default=0.5, min=0.0, max=1.0)


def unregister():
    try:
        overlay.set_enabled(False)
    except Exception:
        pass
    for m in reversed(_modules):
        if hasattr(m, "unregister"):
            try:
                m.unregister()
            except Exception:
                pass
    for p in ("motion_sculpt_enable", "motion_sculpt_onion",
              "motion_sculpt_wormhole", "motion_sculpt_wormhole_gap",
              "motion_sculpt_brush_radius", "motion_sculpt_brush_strength"):
        if hasattr(bpy.types.Scene, p):
            try:
                delattr(bpy.types.Scene, p)
            except Exception:
                pass


if __name__ == "__main__":
    register()
