#include "wheel_3d_gizmo_plugin.hpp"
#include "vehicle_3d.hpp"
// #include <godot_cpp/classes/ref.hpp>
// #include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/vehicle_wheel3d.hpp>
// #include <godot_cpp/core/class_db.hpp>
// #include <godot_cpp/classes/node_3d_editor_plugin.hpp>
// #include <godot_cpp/classes/editor_settings.hpp>
// #include <godot_cpp/classes/editor_node3d_gizmo_plugin.hpp>
// #include <godot_cpp/classes/editor_node3d_gizmo.hpp>
// #include <godot_cpp/classes/ref.hpp>
// #include <godot_cpp/variant/string_name.hpp>
// #include <godot_cpp/variant/variant.hpp>

// #include <godot_cpp/classes/editor_plugin.hpp>

// #include <godot_cpp/classes/editor_node3d_gizmo.hpp>
//  #include <godot_cpp/classes/editor_node3d_gizmo_plugin.hpp>

using namespace godot;

// Wheel3DGizmoPlugin::Wheel3DGizmoPlugin()
//{
//  Color gizmo_color = EDITOR_GET("editors/3d_gizmos/gizmo_colors/shape");
//  Color editSet = get_setting("editors/3d_gizmos/gizmo_colors/shape");
//  Color gizmo_color2 = EditorSettings::get("editors/3d_gizmos/gizmo_colors/shape");

//   // Color gizmo_color = Color();
//
//  create_material("shape_material", gizmo_color);
//}

bool Wheel3DGizmoPlugin::_has_gizmo(Node3D *p_spatial) const
{
  return Object::cast_to<Wheel3D>(p_spatial) != nullptr;
}

String Wheel3DGizmoPlugin::_get_gizmo_name() const
{
  return U"Wheel3D";
}

int Wheel3DGizmoPlugin::_get_priority() const
{
  return -1;
}

void Wheel3DGizmoPlugin::_redraw(const Ref<EditorNode3DGizmo> &p_gizmo)
{
  Wheel3D *car_wheel = Object::cast_to<Wheel3D>(p_gizmo->get_node_3d());

  p_gizmo->clear();

  // std::vector<Vector3> points;
  PackedVector3Array points;

  float r = car_wheel->get_tire_radius();
  const int skip = 10;
  for (int i = 0; i <= 360; i += skip)
  {
    float ra = Math::deg_to_rad((float)i);
    float rb = Math::deg_to_rad((float)i + skip);
    Point2 a = Vector2(sin(ra), cos(ra)) * r;
    Point2 b = Vector2(sin(rb), cos(rb)) * r;

    points.push_back(Vector3(0, a.x, a.y));
    points.push_back(Vector3(0, b.x, b.y));

    const int springsec = 4;

    for (int j = 0; j < springsec; j++)
    {
      float t = car_wheel->get_suspension_length() * 5;
      points.push_back(Vector3(a.x, i / 360.0 * t / springsec + j * (t / springsec), a.y) * 0.2);
      points.push_back(Vector3(b.x, (i + skip) / 360.0 * t / springsec + j * (t / springsec), b.y) * 0.2);
    }
  }

  // travel
  points.push_back(Vector3(0, 0, 0));
  points.push_back(Vector3(0, car_wheel->get_suspension_length(), 0));

  // axis
  points.push_back(Vector3(r * 0.2, car_wheel->get_suspension_length(), 0));
  points.push_back(Vector3(-r * 0.2, car_wheel->get_suspension_length(), 0));
  // axis
  points.push_back(Vector3(r * 0.2, 0, 0));
  points.push_back(Vector3(-r * 0.2, 0, 0));

  // forward line
  points.push_back(Vector3(0, -r, 0));
  points.push_back(Vector3(0, -r, r * 2));
  points.push_back(Vector3(0, -r, r * 2));
  points.push_back(Vector3(r * 2 * 0.2, -r, r * 2 * 0.8));
  points.push_back(Vector3(0, -r, r * 2));
  points.push_back(Vector3(-r * 2 * 0.2, -r, r * 2 * 0.8));
  if (!created_materials)
  {
    // HACK(mihe): Ideally we would do this in the constructor, but the documentation generation
    // will instantiate this too early in the program's flow, leading to a bunch of errors about
    // missing editor settings.

    // create_material(U"shape_material", Color(0.5f, 0.7f, 1.0f));
    create_material(U"shape_material", Color(0.5f, 0.7f, 1.0f));

    created_materials = true;
  }
  if (Object::cast_to<Wheel3D>(car_wheel) != nullptr)
  {
    Ref<Material> material_common = get_material(U"shape_material", p_gizmo);
    p_gizmo->add_collision_segments(points);
    p_gizmo->add_lines(points, material_common);
  }
}