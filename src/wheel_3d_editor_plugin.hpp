#ifndef WHEEL_3D_EDITOR_PLUGIN_HPP
#define WHEEL_3D_EDITOR_PLUGIN_HPP

#include "wheel_3d_gizmo_plugin.hpp"

#include <godot_cpp/classes/editor_plugin.hpp>

using namespace godot;

class Wheel3DEditorPlugin : public EditorPlugin
{
  GDCLASS(Wheel3DEditorPlugin, EditorPlugin)

private:
  Ref<Wheel3DGizmoPlugin> wheel_3d_gizmo_plugin;

private:
  static void _bind_methods() {}
  // Ref<Texture> get_plugin_icon();

  // EditorInterface *editor_interface = get_editor_interface();
  // Control *base_control = editor_interface->get_base_control();
  // Ref<Theme> theme = base_control->get_theme()->duplicate();
  // Ref<Texture2D> icon_wheel_3d = theme->get_icon("VehicleWheel3D", "EditorIcons");
  // Ref<Texture2D> icon_vehicle_3d = theme->get_icon("VehicleBody3D", "EditorIcons");
  // void add_node_3d_gizmo_plugin(const Ref<EditorNode3DGizmoPlugin> &plugin);
  // void remove_node_3d_gizmo_plugin(const Ref<EditorNode3DGizmoPlugin> &plugin);
  // Ref<Wheel3DGizmoPlugin> wheel_3d_gizmo_plugin;

public:
  void _enter_tree() override;
  void _exit_tree() override;
};

#endif // WHEEL_3D_EDITOR_PLUGIN_HPP
       // #endif // GDJ_CONFIG_EDITOR