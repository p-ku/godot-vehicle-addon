#ifndef VEHICLE3D_HPP
#define VEHICLE3D_HPP

#include "macros.hpp"
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/slider_joint3d.hpp>
#include <godot_cpp/classes/shape_cast3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/capsule_mesh.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/ray_cast3d.hpp>
#include <godot_cpp/classes/curve.hpp>
#include <godot_cpp/classes/ref.hpp>

using namespace godot;
//  #include <godot_cpp/variant/typed_array.hpp>

// #include <godot_cpp/classes/physics_body3d.hpp>

class Vehicle3D;
typedef double real_d;

class Wheel3D : public RigidBody3D
{
    GDCLASS(Wheel3D, RigidBody3D);
    friend class Vehicle3D;

public:
    enum Sides
    {
        LEFT,
        CENTER,
        RIGHT
    };

private:
    NodePath opposite_wheel;
    real_d anti_roll = 1.0;
    Vehicle3D *body = 0;
    SliderJoint3D suspension = SliderJoint3D();
    ShapeCast3D tire_cast = ShapeCast3D();
    double spring_force;
    double damp_force;
    Transform3D global_xform;
    Transform3D local_xform;
    CollisionShape3D dummy_collider = CollisionShape3D();
    Vector3 relaxed_position;

    CapsuleMesh springDebugMesh = CapsuleMesh();
    MeshInstance3D springDebug = MeshInstance3D();
    RayCast3D springDebugRay = RayCast3D();
    real_d prev_compression = 0.0;
    // bool alignment_enabled;
    //  Vector3 m_chassisConnectionPointCS; // const
    //  Vector3 m_wheelDirectionCS;         // const
    //  Vector3 m_wheelAxleCS;              // const or modified by steering
    static inline constexpr const double half_pi = Math_PI / 2;
    static inline const Vector3 zeros = Vector3(0, 0, 0);
    static inline const Vector3 ones = Vector3(1, 1, 1);
    static inline const Vector3 steering_axis_reference = Vector3(0, 1, 0);
    static inline const Vector3 forward_axis_reference = Vector3(0, 0, -1);
    static inline const Vector3 right_axis_reference = Vector3(1, 0, 0);

    bool steers = false;

    real_d tire_radius = 0.5;
    real_d tire_width = 0.3;

    real_d hub_radius = 0.3;
    real_d spring_length = 1.0;
    real_d spring_stiffness = 20.0;
    real_d stiff_rel;
    // real_d bump_fast = 100.0;
    // real_d bump_slow = 100.0;
    // real_d bump_threshold = 0.5;
    real_d bump = 1.0;
    real_d bump_rel;
    // real_d rebound_fast = 100.0;
    // real_d rebound_slow = 100.0;
    // real_d rebound_threshold = 0.5;
    real_d rebound = 1.0;
    real_d rebound_rel;
    Ref<Curve> bump_curve;
    Ref<Curve> rebound_curve;
    //  double bump_curve_limit;
    //  double rebound_curve_limit;

    // template <bool has_curve>
    // void fun(...){...};

    Sides side = Sides::LEFT;
    int side_sign = Sides::LEFT - 1;
    bool powered = true;

    bool steered = false;
    real_d steering = 0.0;
    real_d engine_torque = 0.0;
    real_d brake_torque = 0.0;
    real_d camber_angle = 0.0;
    real_d toe_angle = 0.0;
    real_d steering_axis_inclination = 0.0;
    real_d caster_angle = 0.0;
    real_d wheel_offset = 0.0;
    real_d signed_wheel_offset = 0.0;
    real_d signed_camber_angle = 0.0;
    real_d signed_toe_angle = 0.0;
    real_d signed_steering = 0.0;
    real_d signed_steering_axis_inclination = 0.0;

    Vector3 body_connect_point = Vector3(0, spring_length, 0);
    // Vector3 global_body_connect_point;
    Vector3 wheel_connect_point = Vector3(0, 0, 0);

    Vector3 axle_axis_reference = right_axis_reference * side_sign;

    Vector3 steering_axis = steering_axis_reference;
    Vector3 axle_axis = axle_axis_reference;
    void _update_axle_axis()
    {
        axle_axis = axle_axis_reference.rotated(forward_axis_reference, signed_camber_angle);
        axle_axis = axle_axis.rotated(steering_axis_reference, signed_toe_angle).normalized();
    }
    void _update_steering_axis()
    {
        steering_axis = steering_axis_reference.rotated(forward_axis_reference, signed_steering_axis_inclination);
        steering_axis = steering_axis.rotated(right_axis_reference, caster_angle).normalized();
    }
    void _update_connect_points()
    {
        wheel_connect_point = -axle_axis * wheel_offset;
        body_connect_point = wheel_connect_point + spring_length * steering_axis;
    }
    // double yes_damp_factor(Ref<Curve> curve);
    // double no_damp_factor(Ref<Curve> curve) { return 1.0; };

    // double final_bump_factor(double (*func)(Ref<Curve> curve));
    // double (*)(Ref<godot::Curve> curve) doo;
    // double (*doo)(Ref<Curve> curve);
    // double doo(double (*func)(Ref<Curve> curve));
    std::function<double(Ref<Curve>)> doo2;
    //   double (Wheel3D::*doo)(Ref<Curve> curve);

protected:
    void _notification(int p_what);
    static void _bind_methods()
    {
        BIND_METHOD(Wheel3D, set_opposite_wheel, "opposite_wheel");
        BIND_METHOD(Wheel3D, get_opposite_wheel);

        BIND_METHOD(Wheel3D, set_tire_radius, "tire_radius");
        BIND_METHOD(Wheel3D, get_tire_radius);

        BIND_METHOD(Wheel3D, set_width, "width");
        BIND_METHOD(Wheel3D, get_width);

        BIND_METHOD(Wheel3D, set_hub_radius, "hub_radius");
        BIND_METHOD(Wheel3D, get_hub_radius);

        BIND_METHOD(Wheel3D, set_side, "side");
        BIND_METHOD(Wheel3D, get_side);

        BIND_METHOD(Wheel3D, set_powered, "powered");
        BIND_METHOD(Wheel3D, get_powered);

        BIND_METHOD(Wheel3D, set_steered, "steered");
        BIND_METHOD(Wheel3D, get_steered);

        BIND_METHOD(Wheel3D, set_steering, "steering");
        BIND_METHOD(Wheel3D, get_steering);

        BIND_METHOD(Wheel3D, set_engine_torque, "engine_torque");
        BIND_METHOD(Wheel3D, get_engine_torque);

        BIND_METHOD(Wheel3D, set_brake_torque, "brake_torque");
        BIND_METHOD(Wheel3D, get_brake_torque);

        BIND_METHOD(Wheel3D, set_vehicle_connection_point, "vehicle_connection_point");
        BIND_METHOD(Wheel3D, get_vehicle_connection_point);

        BIND_PROPERTY_HINTED("opposite_wheel", Variant::NODE_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Wheel3D");
        //  ClassDB::add_property(get_class_static(), m_property, m_setter, m_getter)
        ADD_PROPERTY(PropertyInfo(Variant::NIL, "tester", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_CATEGORY), "set_vehicle_connection_point", "get_vehicle_connection_point");
        BIND_PROPERTY_RANGED("tire_radius", Variant::FLOAT, "0.001,2,0.001,suffix:m");
        BIND_PROPERTY_RANGED("width", Variant::FLOAT, "0.001,2,0.001,suffix:m");
        BIND_PROPERTY_RANGED("hub_radius", Variant::FLOAT, "0.001,2,0.001,suffix:m");
        BIND_PROPERTY_ENUM("side", "Left,Center,Right");
        BIND_PROPERTY("powered", Variant::BOOL);
        BIND_PROPERTY("steered", Variant::BOOL);
        BIND_PROPERTY_RANGED("steering", Variant::FLOAT, "-180,180,0.01,radians");
        BIND_PROPERTY_RANGED("engine_torque", Variant::FLOAT, u"0,1000,0.01,or_greater,suffix:kg\u22C5m/s\u00B2 (N)");
        BIND_PROPERTY_RANGED("brake_torque", Variant::FLOAT, u"0,1000,0.01,or_greater,suffix:kg\u22C5m/s\u00B2 (N)");
        BIND_PROPERTY_RANGED("vehicle_connection_point", Variant::VECTOR3, "-99999,99999,0.001,or_greater,or_less,hide_slider,suffix:m");

        BIND_METHOD(Wheel3D, set_alignment_wheel_offset, "alignment_wheel_offset");
        BIND_METHOD(Wheel3D, get_alignment_wheel_offset);

        BIND_METHOD(Wheel3D, set_alignment_steering_axis_inclination, "alignment_steering_axis_inclination");
        BIND_METHOD(Wheel3D, get_alignment_steering_axis_inclination);

        BIND_METHOD(Wheel3D, set_alignment_camber_angle, "alignment_camber_angle");
        BIND_METHOD(Wheel3D, get_alignment_camber_angle);

        BIND_METHOD(Wheel3D, set_alignment_toe_angle, "alignment_toe_angle");
        BIND_METHOD(Wheel3D, get_alignment_toe_angle);

        BIND_METHOD(Wheel3D, set_alignment_caster_angle, "alignment_caster_angle");
        BIND_METHOD(Wheel3D, get_alignment_caster_angle);

        ADD_GROUP("Alignment", "alignment_");
        BIND_PROPERTY_RANGED("alignment_wheel_offset", Variant::FLOAT, "0.0,2,0.001,suffix:m");
        BIND_PROPERTY_RANGED("alignment_steering_axis_inclination", Variant::FLOAT, "-90,90,0.01,radians");
        BIND_PROPERTY_RANGED("alignment_camber_angle", Variant::FLOAT, "-90,90,0.01,radians");
        BIND_PROPERTY_RANGED("alignment_toe_angle", Variant::FLOAT, "-90,90,0.01,radians");
        BIND_PROPERTY_RANGED("alignment_caster_angle", Variant::FLOAT, "-90,90,0.01,radians");

        BIND_METHOD(Wheel3D, set_spring_length, "spring_length");
        BIND_METHOD(Wheel3D, get_spring_length);

        BIND_METHOD(Wheel3D, set_spring_stiffness, "spring_stiffness");
        BIND_METHOD(Wheel3D, get_spring_stiffness);

        BIND_METHOD(Wheel3D, set_spring_anti_roll, "spring_anti_roll");
        BIND_METHOD(Wheel3D, get_spring_anti_roll);

        ADD_GROUP("Springs", "spring_");
        BIND_PROPERTY_RANGED("spring_length", Variant::FLOAT, "0.001,10,0.001,suffix:m");
        BIND_PROPERTY_RANGED("spring_stiffness", Variant::FLOAT, "0,1000,0.01,or_greater,suffix:N/m");
        BIND_PROPERTY_RANGED("spring_anti_roll", Variant::FLOAT, "0,1000,0.01,or_greater,suffix:N/m");

        BIND_METHOD(Wheel3D, set_damping_bump, "damping_bump");
        BIND_METHOD(Wheel3D, get_damping_bump);

        BIND_METHOD(Wheel3D, set_damping_rebound, "damping_rebound");
        BIND_METHOD(Wheel3D, get_damping_rebound);

        BIND_METHOD(Wheel3D, set_damping_bump_curve, "damping_bump_curve");
        BIND_METHOD(Wheel3D, get_damping_bump_curve);
        //  BIND_METHOD(Wheel3D, set_damping_bump_curve_limit, "damping_bump_curve_limit");
        //  BIND_METHOD(Wheel3D, get_damping_bump_curve_limit);
        BIND_METHOD(Wheel3D, set_damping_rebound_curve, "damping_rebound_curve");
        BIND_METHOD(Wheel3D, get_damping_rebound_curve);
        //  BIND_METHOD(Wheel3D, set_damping_rebound_curve_limit, "damping_rebound_curve_limit");
        //  BIND_METHOD(Wheel3D, get_damping_rebound_curve_limit);
        ADD_GROUP("Damping", "damping_");
        BIND_PROPERTY_RANGED("damping_bump", Variant::FLOAT, u"0,1000,0.01,or_greater,suffix:N\u22C5s/m");
        BIND_PROPERTY_RANGED("damping_rebound", Variant::FLOAT, u"0,1000,0.01,or_greater,suffix:N\u22C5s/m");
        //    BIND_PROPERTY("damping_bump", Variant::FLOAT);
        //    BIND_PROPERTY("damping_rebound", Variant::FLOAT);

        //   BIND_PROPERTY_RANGED("damping_bump_curve_limit", Variant::FLOAT, u"0,100000,2,or_greater,suffix:m/s");
        //  BIND_PROPERTY_RANGED("damping_rebound_curve_limit", Variant::FLOAT, u"0,100000,2,or_greater,suffix:N\u22C5s/m");

        ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "damping_bump_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_damping_bump_curve", "get_damping_bump_curve");
        ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "damping_rebound_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_damping_rebound_curve", "get_damping_rebound_curve");
        // ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_curve", "get_curve");

        BIND_METHOD(Wheel3D, get_wheel_connect_point);

        // BIND_METHOD(Wheel3D, get_body_connect_point);

        BIND_METHOD(Wheel3D, get_steering_axis);

        BIND_METHOD(Wheel3D, get_axle_axis);

        BIND_METHOD(Wheel3D, get_spring_force);

        BIND_ENUM_CONSTANT(LEFT);
        BIND_ENUM_CONSTANT(CENTER);
        BIND_ENUM_CONSTANT(RIGHT);
    }

    void _validate_property(PropertyInfo &p_property) const
    {
        if (p_property.name == StringName("property_from_list"))
        {
            if (powered)
                p_property.usage = PROPERTY_USAGE_NO_EDITOR;
            else
                p_property.usage = PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_INTERNAL | PROPERTY_USAGE_EDITOR;
        }
    }
    void _get_property_list(List<PropertyInfo> *p_list) const
    {
        p_list->push_back(PropertyInfo(Variant::VECTOR3, "property_from_list", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR));
        for (PropertyInfo &E : *p_list)
            _validate_property(E);
    }

    bool _get(const StringName &p_name, Variant &r_ret) const
    {
        String name = p_name;
        if (name == "property_from_list")
        {
            //     r_ret = property_from_list;
            return true;
        }
        return false;
    }
    bool _set(const StringName &p_name, const Variant &p_value)
    {
        String name = p_name;
        if (name == "property_from_list")
        {
            return true;
        }
        return false;
    }

public:
    NodePath get_opposite_wheel() const { return opposite_wheel; }
    void set_opposite_wheel(const NodePath &p_node)
    {
        opposite_wheel = p_node;
        // dummy_collider.set_shape(cast_to<Shape3D>(&dummyShape));
        //    cast_to<Wheel3D>(p_node);
        //  RigidBody3D sib1 = this->get_node<RigidBody3D>(p_node);
        // add_sibling();
        //  NodePath nn = NodePath();
        // SceneTree *tree = (SceneTree *)Engine::get_singleton()->get_main_loop();

        // Object *node = get_root()->get_node<Object>(p_path);
        Wheel3D *sib = get_node<Wheel3D>(opposite_wheel);
        // sib = this;
        // sib->set_tire_radius(tire_radius);
        // sib->queue_free();
        // sib->tire_radius = tire_radius;
        // sib->add_to_group
        // Wheel3D sib = cast_to<Wheel3D>(get_node(&opposite_wheel));
    }

    real_d get_steering() const { return steering; }
    void set_steering(real_d p_angle) { steering = p_angle; }

    real_d get_engine_torque() const { return engine_torque; }
    void set_engine_torque(real_d p_torque) { engine_torque = p_torque; }

    real_d get_brake_torque() const { return brake_torque; }
    void set_brake_torque(real_d p_torque) { brake_torque = p_torque; }

    real_d get_tire_radius() const { return tire_radius; }
    void set_tire_radius(real_d p_radius)
    {
        tire_radius = p_radius;
        if (tire_radius < hub_radius)
        {
            hub_radius = tire_radius;
        }
        update_gizmos();
    }

    real_d get_width() const { return tire_width; }
    void set_width(real_d p_width)
    {
        tire_width = p_width;
        update_gizmos();
    }

    real_d get_hub_radius() const { return hub_radius; }
    void set_hub_radius(real_d p_radius)
    {
        hub_radius = p_radius;
        if (tire_radius < hub_radius)
        {
            tire_radius = hub_radius;
        }
        update_gizmos();
    }

    real_d get_spring_length() const { return spring_length; }
    void set_spring_length(real_d p_length)
    {
        spring_length = p_length;
        _update_connect_points();
        //   set_position(body_connect_point - steering_axis * spring_length - axle_axis * wheel_offset);

        update_gizmos();
    }

    real_d get_spring_stiffness() const { return spring_stiffness; }
    void set_spring_stiffness(real_d p_value) { spring_stiffness = p_value; }

    real_d get_spring_anti_roll() const { return anti_roll; }
    void set_spring_anti_roll(real_d p_value) { anti_roll = p_value; }

    real_d get_damping_bump() const { return bump; }
    void set_damping_bump(real_d p_value) { bump = p_value; }

    Ref<Curve> get_damping_bump_curve() const { return bump_curve; }
    void set_damping_bump_curve(Ref<Curve> p_curve) { bump_curve = p_curve; }

    real_d get_damping_rebound() const { return rebound; }
    void set_damping_rebound(real_d p_value) { rebound = p_value; }

    Ref<Curve> get_damping_rebound_curve() const { return rebound_curve; }
    void set_damping_rebound_curve(Ref<Curve> p_value) { rebound_curve = p_value; }

    Sides get_side() const { return side; }
    void set_side(Sides p_value)
    {
        // side_sign = side - 1;
        side = p_value;
        side_sign = side - 1;
        signed_wheel_offset = -side_sign * wheel_offset;
        signed_camber_angle = side_sign * camber_angle;
        signed_toe_angle = side_sign * toe_angle;
        signed_steering_axis_inclination = side_sign * steering_axis_inclination;
        axle_axis_reference = side_sign * right_axis_reference;
        _update_axle_axis();
        _update_steering_axis();
        //   set_position(body_connect_point - steering_axis * spring_length - axle_axis * wheel_offset);
        _update_connect_points();
        //   set_position(-body_connect_point);
        update_gizmos();
    }

    bool get_powered() const { return powered; }
    void set_powered(bool p_value)
    {
        powered = p_value;
        notify_property_list_changed();
        update_gizmos();
    }

    bool get_steered() const { return steered; }
    void set_steered(bool p_value) { steered = p_value; }

    real_d get_alignment_steering_axis_inclination() const { return steering_axis_inclination; }
    void set_alignment_steering_axis_inclination(real_d p_angle)
    {
        steering_axis_inclination = p_angle;
        signed_steering_axis_inclination = -side_sign * steering_axis_inclination;
        _update_axle_axis();
        _update_steering_axis();
        _update_connect_points();
        //  set_position(-body_connect_point);
        //  set_position(body_connect_point - steering_axis * spring_length - axle_axis * wheel_offset);

        update_gizmos();
    }

    real_d get_alignment_camber_angle() const { return camber_angle; }
    void set_alignment_camber_angle(real_d p_angle)
    {
        camber_angle = p_angle;
        signed_camber_angle = side_sign * camber_angle;
        _update_axle_axis();
        _update_steering_axis();
        _update_connect_points();
        //  set_position(-body_connect_point);
        //  set_position(body_connect_point - steering_axis * spring_length - axle_axis * wheel_offset);
        update_gizmos();
    }

    real_d get_alignment_toe_angle() const { return toe_angle; }
    void set_alignment_toe_angle(real_d p_angle)
    {
        toe_angle = p_angle;
        signed_toe_angle = side_sign * toe_angle;
        _update_axle_axis();
        _update_steering_axis();
        _update_connect_points();
        //  set_position(-body_connect_point);
        //   set_position(body_connect_point - steering_axis * spring_length - axle_axis * wheel_offset);

        update_gizmos();
    }

    real_d get_alignment_caster_angle() const { return caster_angle; }
    void set_alignment_caster_angle(real_d p_angle)
    {
        caster_angle = p_angle;
        _update_axle_axis();
        _update_steering_axis();
        _update_connect_points();
        //   set_position(-body_connect_point);
        //  body_connect_point = axle_axis * wheel_offset - spring_length * steering_axis;
        //   set_position(body_connect_point - steering_axis * spring_length - axle_axis * wheel_offset);

        // suspensionpoints[i] = suspensionpoints[i].rotated(Vector3.FORWARD, inclin).rotated(Vector3.RIGHT, caster);
        update_gizmos();
    }

    real_d get_alignment_wheel_offset() const { return wheel_offset; }
    void set_alignment_wheel_offset(real_d p_value)
    {
        // translate(axle_axis * (p_value - wheel_offset));

        wheel_offset = p_value;
        signed_wheel_offset = -side_sign * wheel_offset;

        _update_connect_points();
        // set_position(-body_connect_point);
        // set_position(body_connect_point - steering_axis * spring_length - axle_axis * wheel_offset);
        //    _update_steering_axis();
        //    steering_axis = steering_axis.move_toward(axle_axis, wheel_offset);
        //  steering_axis = steering_axis + axle_axis * wheel_offset;
        update_gizmos();
    }

    Vector3 get_wheel_connect_point() const { return wheel_connect_point; }
    Vector3 get_vehicle_connection_point() const { return body_connect_point; }
    void set_vehicle_connection_point(Vector3 p_value) { body_connect_point = p_value; }

    Vector3 get_steering_axis() const { return axle_axis; }
    Vector3 get_axle_axis() const { return axle_axis; }
    double get_spring_force() const { return spring_force; }

    void _ready() override;
    void _physics_process(double delta) override;
    PackedStringArray _get_configuration_warnings() const override;

    // std::function<void(double)> yes_body_physics_process;

    //  std::function<void(double)> no_body_physics_process = {};
    //  std::function<void(double)> wheel_physics_process;
    // void (Wheel3D::*wheel_physics_process)(double delta);
    // auto wheel_physics_process = no_body_physics_process;
    // bool yes_body_physics_process(double delta);
    // bool no_body_physics_process(double delta)
    // {
    //     std::cout << delta << '\n';
    //     return true;
    // }

    //  void wheel_physics_process();

    Wheel3D()
    {
    }
    ~Wheel3D() { queue_free(); }
    //   friend void yes_body_physics_process(double delta, Wheel3D &w);
};
// namespace funcy2
//{
//     using namespace godot;
//     void (Wheel3D::*Wheel3D::wheel_physics_process)(double delta);
//
//     //    get_type void (*Wheel3D::wheel_physics_process)(double delta);
// } // namespace funcy2

namespace
{
    //   void func(int a) { return; }
    //   void (*ptr)(int) = &func;
    //   int ddd = 5;
    //   void (*ptr)(&ddd);
    double yes_damp_factor(real_d damping, Ref<Curve> damp_curve, real_d compress_velocity)
    {
        return damping * compress_velocity * (damp_curve->sample_baked(compress_velocity));
    }
    double no_damp_factor(real_d damping, Ref<Curve> damp_curve, real_d compress_velocity)
    {
        return damping * compress_velocity;
    }
    auto bump_function = no_damp_factor;
    auto rebound_function = no_damp_factor;
    //  bool (Wheel3D::*wheel_physics_process)(double){no_body_physics_process}; // = &no_body_physics_process;
    void yes_body_physics_process() {}
    inline void no_body_physics_process() { return; }
    auto wheel_physics_process = no_body_physics_process;

    // auto wheel_physics_process = yes_body_physics_process;
}

VARIANT_ENUM_CAST(Wheel3D::Sides);

class Vehicle3D : public RigidBody3D
{
    GDCLASS(Vehicle3D, RigidBody3D);

    friend class Wheel3D;

private:
    Transform3D global_xform;
    Transform3D local_xform;
    Basis local_basis;
    Vector3 local_origin;

    std::set<Wheel3D *> wheels;

protected:
    static void _bind_methods()
    {
        // BIND_METHOD(Vehicle3D, get_wheels);
        // ClassDB::bind_method(D_METHOD("get_wheels"), &Vehicle3D::get_wheels);
    }

public:
    std::set<Wheel3D *> get_wheels() const { return wheels; }
    void _ready() override;
    void _physics_process(double delta) override;

    Vehicle3D()
    {
    }
    ~Vehicle3D() { queue_free(); }
};

//   void no_body_physics_process(double delta, Wheel3D &w) { std::cout << delta << '\n'; }

// auto wheel_physics_process = no_body_physics_process;
//  class Funcy
//{
//      friend class Wheel3D;
//      auto wheel_physics_process = Wheel3D::no_body_physics_process;
//  }
//   auto wheel_physics_process = Wheel3D::no_body_physics_process;

#endif // VEHICLE3D_HPP