#include "register_types.hpp"

// #include <gdextension_interface.h>

#include "vehicle3d.hpp"
// #include "wheel_3d_gizmo_plugin.hpp"
// #include "wheel_3d_editor_plugin.hpp"

using namespace godot;

void initialize_vehicle_module(ModuleInitializationLevel p_level)
{
    if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE)
    {
        ClassDB::register_class<Wheel3D>();
        ClassDB::register_class<Vehicle3D>();
    }
    // else if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR)
    // {
    //   ClassDB::register_class<Wheel3DGizmoPlugin>();
    //   ClassDB::register_class<Wheel3DEditorPlugin>();
    //   EditorPlugins::add_by_type<Wheel3DEditorPlugin>();
    // }
    else
    {
        return;
    }
}

void uninitialize_vehicle_module(ModuleInitializationLevel p_level)
{
    if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR)
    {
        //    EditorPlugins::remove_by_type<Wheel3DEditorPlugin>();
    }
    else
    {
        return;
    }
}

extern "C"
{
    // Initialization.
    GDExtensionBool GDE_EXPORT vehicle_module_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, const GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization)
    {
        godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

        init_obj.register_initializer(initialize_vehicle_module);
        init_obj.register_terminator(uninitialize_vehicle_module);
        init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

        return init_obj.init();
    }
}