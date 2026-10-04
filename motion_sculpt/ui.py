"""ui — N-panel + keymaps."""

import bpy


class MOTION_SCULPT_PT_panel(bpy.types.Panel):
    bl_label = "Motion Sculpt"
    bl_idname = "MOTION_SCULPT_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Motion Sculpt"

    def draw(self, context):
        scn = context.scene
        L = self.layout
        try:
            from .fk_core import HAS_CPP
            L.label(text=f"C++ core: {'ON' if HAS_CPP else 'numpy fallback'}")
        except Exception:
            pass
        L.prop(scn, "motion_sculpt_enable", text="Path + Onion Overlay")
        r = L.row()
        r.operator("motion_sculpt.toggle_overlay", text="Toggle")
        r.operator("motion_sculpt.clear_cache", text="Clear cache")
        L.prop(scn, "motion_sculpt_onion", text="Onion frames")
        r = L.row()
        r.prop(scn, "motion_sculpt_wormhole", text="Wormhole")
        r.prop(scn, "motion_sculpt_wormhole_gap", text="Gap")
        L.separator()
        L.label(text="Rigless sculpt:")
        L.operator("motion_sculpt.weight_pick", text="Weight Pick / Drag (mesh)")
        L.separator()
        L.label(text="Motion path:")
        L.operator("motion_sculpt.grab_path", text="Grab Path Point")
        r = L.row(align=True)
        r.prop(scn, "motion_sculpt_brush_radius", text="R")
        r.prop(scn, "motion_sculpt_brush_strength", text="S")
        L.operator("motion_sculpt.brush_smooth", text="Brush: Smooth Path")
        L.separator()
        L.label(text="Workflow: mesh click -> drag -> auto FK key")


_addon_keymaps = []


def register():
    bpy.utils.register_class(MOTION_SCULPT_PT_panel)
    # hotkeys: W = weight pick, G = grab path (in 3D view, non-conflicting w/ defaults? uses Alt)
    try:
        wm = bpy.context.window_manager
        kc = wm.keyconfigs.addon
        if kc:
            km = kc.keymaps.new(name='3D View', space_type='VIEW_3D')
            kmi = km.keymap_items.new("motion_sculpt.weight_pick", 'W', 'PRESS', alt=True)
            _addon_keymaps.append((km, kmi))
            kmi = km.keymap_items.new("motion_sculpt.grab_path", 'G', 'PRESS', alt=True)
            _addon_keymaps.append((km, kmi))
    except Exception:
        pass


def unregister():
    try:
        for km, kmi in _addon_keymaps:
            km.keymap_items.remove(kmi)
    except Exception:
        pass
    _addon_keymaps.clear()
    try:
        bpy.utils.unregister_class(MOTION_SCULPT_PT_panel)
    except Exception:
        pass
