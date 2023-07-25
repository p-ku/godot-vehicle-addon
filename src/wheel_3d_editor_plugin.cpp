#include "wheel_3d_editor_plugin.hpp"
// #include <godot_cpp/core/class_db.hpp>
// #include <godot_cpp/godot.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/theme.hpp>
#include <godot_cpp/classes/control.hpp>
// #ifdef GDJ_CONFIG_EDITOR
// Ref<Texture> Wheel3DEditorPlugin::get_plugin_icon()
// {
//   return get_editor_interface()->get_base_control()->get_icon("Node", "EditorIcons");
// }
using namespace godot;

void Wheel3DEditorPlugin::_enter_tree()
{
  EditorInterface *editor_interface = get_editor_interface();
  Control *base_control = editor_interface->get_base_control();

  // HACK(mihe): For whatever reason the editor startup time takes a significant hit with every
  // icon added unless we duplicate the theme first and manipulate that instead.
  Ref<Theme> theme = base_control->get_theme()->duplicate();

  //  Ref<Texture2D> icon_pin = theme->get_icon("PinJoint3D", "EditorIcons");
  //  Ref<Texture2D> icon_hinge = theme->get_icon("HingeJoint3D", "EditorIcons");
  //  Ref<Texture2D> icon_slider = theme->get_icon("SliderJoint3D", "EditorIcons");
  //  Ref<Texture2D> icon_cone_twist = theme->get_icon("ConeTwistJoint3D", "EditorIcons");
  //  Ref<Texture2D> icon_6dof = theme->get_icon("Generic6DOFJoint3D", "EditorIcons");

  // theme->set_icon("JoltPinJoint3D", "EditorIcons", icon_pin);
  // theme->set_icon("JoltHingeJoint3D", "EditorIcons", icon_hinge);
  // theme->set_icon("JoltSliderJoint3D", "EditorIcons", icon_slider);
  // theme->set_icon("JoltConeTwistJoint3D", "EditorIcons", icon_cone_twist);
  // theme->set_icon("JoltGeneric6DOFJoint3D", "EditorIcons", icon_6dof);
  // Ref<Texture2D> icon_pin = theme->get_icon("PinJoint3D", "EditorIcons");
  // Ref<Texture2D> icon_hinge = theme->get_icon("HingeJoint3D", "EditorIcons");

  Ref<Texture2D> icon_wheel_3d = theme->get_icon("VehicleWheel3D", "EditorIcons");
  Ref<Texture2D> icon_vehicle_3d = theme->get_icon("VehicleBody3D", "EditorIcons");

  theme->set_icon("Wheel3D", "EditorIcons", icon_wheel_3d);
  theme->set_icon("Vehicle3D", "EditorIcons", icon_vehicle_3d);
  // Ref<Texture2D> icon_pin = theme->get_icon("Wheel3D", "EditorIcons", icon_wheel_3d);
  // Ref<Texture2D> icon_hinge = theme->get_icon("Vehicle3D", "EditorIcons", icon_vehicle_3d);
  //  base_control->set_theme(theme);

  //  wheel_3d_gizmo_plugin.instantiate();
  //  add_node_3d_gizmo_plugin(wheel_3d_gizmo_plugin);
}

void Wheel3DEditorPlugin::_exit_tree()
{
  remove_node_3d_gizmo_plugin(wheel_3d_gizmo_plugin);
  wheel_3d_gizmo_plugin.unref();
}

// #endif // GDJ_CONFIG_EDITOR