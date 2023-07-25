#ifndef WHEEL_3D_GIZMO_PLUGIN_HPP
#define WHEEL_3D_GIZMO_PLUGIN_HPP

#include <godot_cpp/classes/editor_node3d_gizmo_plugin.hpp>

namespace godot
{
  class Wheel3DGizmoPlugin : public EditorNode3DGizmoPlugin
  {
    GDCLASS(Wheel3DGizmoPlugin, EditorNode3DGizmoPlugin);

  private:
    static void _bind_methods(){};

  public:
    bool _has_gizmo(Node3D *p_spatial) const override;
    String _get_gizmo_name() const override;
    int _get_priority() const override;
    void _redraw(const Ref<EditorNode3DGizmo> &p_gizmo) override;
    Variant get_setting(const String &name) const;

  private:
    bool created_materials = false;

    // Wheel3DGizmoPlugin();
  };
}
#endif // WHEEL_3D_GIZMO_PLUGIN_HPP