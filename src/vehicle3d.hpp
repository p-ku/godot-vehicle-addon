#ifndef VEHICLE3D_HPP
#define VEHICLE3D_HPP

#include "macros.hpp"
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/slider_joint3d.hpp>
#include <godot_cpp/classes/shape_cast3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/cylinder_shape3d.hpp>
#include <godot_cpp/classes/capsule_mesh.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/ray_cast3d.hpp>
#include <godot_cpp/classes/curve.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/input_event_key.hpp>

// #include <godot_cpp/templates/cowdata.hpp>
#include <godot_cpp/templates/vector.hpp>

using namespace godot;
//  #include <godot_cpp/variant/typed_array.hpp>

// #include <godot_cpp/classes/physics_body3d.hpp>

class Vehicle3D;
// typedef double real_d;
using real_d = double;
namespace
{
    static inline const Vector3 zeros = Vector3(0, 0, 0);
    static inline const Vector3 ones = Vector3(1, 1, 1);
    static inline const Vector3 up_axis_reference = Vector3(0, 1, 0);
    static inline const Vector3 forward_axis_reference = Vector3(0, 0, -1);
    static inline const Vector3 right_axis_reference = Vector3(1, 0, 0);
    static inline constexpr double HALF_PI = Math_PI / 2.0;

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
}

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
    real_d ackermann_input = 0.0;
    real_d max_ackermann = 0.57;
    real_d ackermann_factor = 0.0;

    real_d ackermann_steering = 0.0;

    Vector3 w_to_v;
    Transform3D global_xform;
    Basis global_basis;
    Vector3 global_origin;
    Vector3 local_origin;

    Vehicle3D *body = nullptr;
    bool relative_transform = false;
    real_d corner_distance;
    real_d corner_angle;
    Wheel3D *sib;

    NodePath opposite_wheel;
    real_d anti_roll = 1.0;
    SliderJoint3D suspension = SliderJoint3D();
    ShapeCast3D tire_cast = ShapeCast3D();
    double spring_force;
    double damp_force;
    real_d compression = 0.0;
    SphereShape3D hub_shape = SphereShape3D();
    CollisionShape3D hub_collider = CollisionShape3D();
    CylinderShape3D tire_cast_shape = CylinderShape3D();
    Transform3D relaxed_transform;

    CapsuleMesh springDebugMeshA = CapsuleMesh();
    CapsuleMesh springDebugMeshB = CapsuleMesh();
    BoxMesh connectDebugMeshA = BoxMesh();
    BoxMesh connectDebugMeshB = BoxMesh();
    StandardMaterial3D matA = StandardMaterial3D();
    StandardMaterial3D matB = StandardMaterial3D();

    MeshInstance3D springDebugA = MeshInstance3D();
    MeshInstance3D springDebugB = MeshInstance3D();
    MeshInstance3D connectDebugA = MeshInstance3D();
    MeshInstance3D connectDebugB = MeshInstance3D();
    RayCast3D springDebugRay = RayCast3D();
    RayCast3D ray_center = RayCast3D();
    RayCast3D ray_front_left = RayCast3D();
    RayCast3D ray_front_right = RayCast3D();
    RayCast3D ray_back_left = RayCast3D();
    RayCast3D ray_back_right = RayCast3D();

    real_d prev_compression = 0.0;

    bool steers = false;

    real_d tire_radius = 0.5;
    real_d tire_width = 0.3;

    real_d hub_radius = tire_width / 2.0;
    real_d spring_travel = 1.0;
    real_d spring_stiffness = 20.0;
    real_d bump = 1.0;
    real_d rebound = 1.0;
    real_d bump_rel = bump * spring_travel;
    real_d rebound_rel = rebound * spring_travel;
    real_d stiff_rel = spring_stiffness * spring_travel;
    Ref<Curve> bump_curve;
    Ref<Curve> rebound_curve;

    Sides side = Sides::LEFT;
    int side_sign = side - 1;
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
    //  Vector3 v_connect_point = get_position() + Vector3(0, spring_travel, 0);

    //  Vector3 v_connect_point = get_position() + Vector3(0, spring_travel, 0);

    //   Vector3 axle_axis_reference = right_axis_reference * side_sign;

    Vector3 steering_axis = up_axis_reference;
    Vector3 axle_axis = side_sign * right_axis_reference;
    Vector3 w_connect_point = Vector3(0, 0, 0);
    Vector3 v_connect_point; // = get_basis().xform_inv(w_connect_point + spring_travel * steering_axis) + get_position();
    //  v_connect_point = get_basis().xform(v_connect_point);

    // void _update_axle_axis()
    // {
    //     axle_axis = axle_axis_reference.rotated(back_axis_reference, signed_camber_angle);
    //     axle_axis = axle_axis.rotated(up_axis_reference, signed_toe_angle).normalized();
    // }
    void _update_steering_axis()
    {
        steering_axis = up_axis_reference.rotated(forward_axis_reference, signed_steering_axis_inclination);
        steering_axis = steering_axis.rotated(right_axis_reference, caster_angle);
        steering_axis = steering_axis.rotated(get_basis().get_column(1), -signed_toe_angle);
        steering_axis = steering_axis.rotated(get_basis().get_column(1), ackermann_steering);
        steering_axis = steering_axis.rotated(get_basis().get_column(2), signed_camber_angle);
        steering_axis = steering_axis.normalized();

        //    steering_axis = v_connect_point - w_connect_point;
        //    steering_axis = steering_axis.normalized();
    }

    void _update_connect_points()
    {

        _update_steering_axis();
        w_connect_point = -axle_axis * wheel_offset;
        v_connect_point = get_basis().xform(w_connect_point + spring_travel * steering_axis) + get_position();
        // v_connect_point = get_basis().xform(v_connect_point);
    }
    void _update_suspension_transform()
    {

        suspension.set_rotation(Vector3(0, 0, HALF_PI));
        suspension.rotate(forward_axis_reference, signed_steering_axis_inclination);
        suspension.rotate(right_axis_reference, caster_angle);
        suspension.rotate(up_axis_reference, -ackermann_steering);

        suspension.rotate(get_basis().get_column(1), -signed_toe_angle);
        suspension.rotate(get_basis().get_column(2), signed_camber_angle);

        suspension.set_position(w_connect_point);

        springDebugA.set_position(Vector3(spring_travel, 0, 0));

        connectDebugA.set_position(w_connect_point + spring_travel * steering_axis);
        connectDebugB.set_position(w_connect_point);
        springDebugRay.set_position(w_connect_point + spring_travel * steering_axis);
    }
    void _update_wheel_dimensions()
    {

        hub_shape.set_radius(hub_radius);
        //  hub_shape.set_height(tire_width);
        //  hub_radius = tire_width / 2.0;

        tire_cast_shape.set_radius(tire_radius);
        tire_cast_shape.set_height(tire_width);

        //  tire_cast.set_shape(tire_cast_shape);
        corner_distance = Math::sqrt(tire_radius * tire_radius + 0.25 * tire_width * tire_width);
        corner_angle = HALF_PI - Math::atan(tire_radius / (0.5 * tire_width));
    }
    void _update_suspension_length()
    {
        bump_rel = bump * spring_travel;
        rebound_rel = rebound * spring_travel;
        stiff_rel = spring_stiffness * spring_travel;
    }

protected:
    static void _bind_methods()
    {
        BIND_METHOD(Wheel3D, get_ackermann_input);
        BIND_METHOD(Wheel3D, set_ackermann_input, "ackermann_input");

        BIND_METHOD(Wheel3D, get_relative_transform);
        BIND_METHOD(Wheel3D, set_relative_transform, "rel_transform");

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

        //    BIND_METHOD(Wheel3D, set_vehicle_connection_point, "vehicle_connect_point");
        //    BIND_METHOD(Wheel3D, get_vehicle_connection_point);
        BIND_PROPERTY_RANGED("ackermann_input", Variant::FLOAT, "0.0,1,0.001");
        BIND_PROPERTY_HINTED("opposite_wheel", Variant::NODE_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Wheel3D");
        //  ADD_PROPERTY(PropertyInfo(Variant::NIL, "tester", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_CATEGORY), "set_vehicle_connection_point", "get_vehicle_connection_point");
        BIND_PROPERTY_RANGED("tire_radius", Variant::FLOAT, "0.001,2,0.001,suffix:m");
        BIND_PROPERTY_RANGED("width", Variant::FLOAT, "0.001,2,0.001,suffix:m");
        BIND_PROPERTY_RANGED("hub_radius", Variant::FLOAT, "0.001,2,0.001,suffix:m");
        BIND_PROPERTY_ENUM("side", "Left,Center,Right");
        BIND_PROPERTY("relative_transform", Variant::BOOL);

        BIND_PROPERTY("powered", Variant::BOOL);
        BIND_PROPERTY("steered", Variant::BOOL);
        BIND_PROPERTY_RANGED("steering", Variant::FLOAT, "-60,60,0.01,radians");
        BIND_PROPERTY_RANGED("engine_torque", Variant::FLOAT, u"0,1000,0.01,or_greater,suffix:kg\u22C5m/s\u00B2 (N)");
        BIND_PROPERTY_RANGED("brake_torque", Variant::FLOAT, u"0,1000,0.01,or_greater,suffix:kg\u22C5m/s\u00B2 (N)");
        //   BIND_PROPERTY_RANGED("vehicle_connection_point", Variant::VECTOR3, "-99999,99999,0.001,or_greater,or_less,hide_slider,suffix:m");
        //   BIND_PROPERTY_RANGED("wheel_connection_point", Variant::VECTOR3, "-99999,99999,0.001,or_greater,or_less,hide_slider,suffix:m");

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

        BIND_METHOD(Wheel3D, set_spring_travel, "spring_travel");
        BIND_METHOD(Wheel3D, get_spring_travel);

        BIND_METHOD(Wheel3D, set_spring_stiffness, "spring_stiffness");
        BIND_METHOD(Wheel3D, get_spring_stiffness);

        BIND_METHOD(Wheel3D, set_spring_anti_roll, "spring_anti_roll");
        BIND_METHOD(Wheel3D, get_spring_anti_roll);

        ADD_GROUP("Springs", "spring_");
        BIND_PROPERTY_RANGED("spring_travel", Variant::FLOAT, "0.001,10,0.001,suffix:m");
        BIND_PROPERTY_RANGED("spring_stiffness", Variant::FLOAT, "0,1000,0.01,or_greater,suffix:N/m");
        BIND_PROPERTY_RANGED("spring_anti_roll", Variant::FLOAT, "0,1000,0.01,or_greater,suffix:N/m");

        BIND_METHOD(Wheel3D, set_damping_bump, "damping_bump");
        BIND_METHOD(Wheel3D, get_damping_bump);

        BIND_METHOD(Wheel3D, set_damping_rebound, "damping_rebound");
        BIND_METHOD(Wheel3D, get_damping_rebound);

        BIND_METHOD(Wheel3D, set_damping_bump_curve, "damping_bump_curve");
        BIND_METHOD(Wheel3D, get_damping_bump_curve);

        BIND_METHOD(Wheel3D, set_damping_rebound_curve, "damping_rebound_curve");
        BIND_METHOD(Wheel3D, get_damping_rebound_curve);

        ADD_GROUP("Damping", "damping_");
        BIND_PROPERTY_RANGED("damping_bump", Variant::FLOAT, u"0,1000,0.01,or_greater,suffix:N\u22C5s/m");
        BIND_PROPERTY_RANGED("damping_rebound", Variant::FLOAT, u"0,1000,0.01,or_greater,suffix:N\u22C5s/m");

        ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "damping_bump_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_damping_bump_curve", "get_damping_bump_curve");
        ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "damping_rebound_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_damping_rebound_curve", "get_damping_rebound_curve");

        //    BIND_METHOD(Wheel3D, get_wheel_connection_point);
        //    BIND_METHOD(Wheel3D, set_wheel_connection_point, "w_connect_point");

        //    BIND_METHOD(Wheel3D, get_vehicle_connection_point);
        //    BIND_METHOD(Wheel3D, set_vehicle_connection_point, "v_connect_point");
        BIND_METHOD(Wheel3D, get_ackermann_steering);

        BIND_METHOD(Wheel3D, get_steering_axis);

        BIND_METHOD(Wheel3D, get_axle_axis);

        BIND_METHOD(Wheel3D, get_spring_force);

        BIND_ENUM_CONSTANT(LEFT);
        BIND_ENUM_CONSTANT(CENTER);
        BIND_ENUM_CONSTANT(RIGHT);
    }

    void _validate_property(PropertyInfo &p_property) const
    {
        if (p_property.name == StringName("rotation"))
        {
            p_property.usage = PROPERTY_USAGE_NO_EDITOR;
        }
        if (p_property.name == StringName("wheel_connection_point"))
        {
            if (relative_transform)
                p_property.usage = PROPERTY_USAGE_NO_EDITOR;
            else
                p_property.usage = PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_INTERNAL | PROPERTY_USAGE_EDITOR;
        }
        if (p_property.name == StringName("vehicle_connection_point"))
        {
            if (relative_transform)
                p_property.usage = PROPERTY_USAGE_NO_EDITOR;
            else
                p_property.usage = PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_INTERNAL | PROPERTY_USAGE_EDITOR;
        }
    }
    void _get_property_list(List<PropertyInfo> *p_list) const
    {
        p_list->push_back(PropertyInfo(Variant::VECTOR3, "wheel_connection_point", PROPERTY_HINT_RANGE, "-99999,99999,0.001,or_greater,or_less,hide_slider,suffix:m"));
        p_list->push_back(PropertyInfo(Variant::VECTOR3, "vehicle_connection_point", PROPERTY_HINT_RANGE, "-99999,99999,0.001,or_greater,or_less,hide_slider,suffix:m"));

        for (PropertyInfo &E : *p_list)
            _validate_property(E);
    }
    bool _get(const StringName &p_name, Variant &r_ret) const
    {
        String name = p_name;
        //  if (name == "rotation")
        //  {
        //      return true;
        //  }
        if (name == "wheel_connection_point")
        {
            r_ret = w_connect_point;
            return true;
        }
        if (name == "vehicle_connection_point")
        {
            r_ret = v_connect_point;
            return true;
        }
        return false;
    }
    bool _set(const StringName &p_name, const Variant &p_value)
    {
        String name = p_name;
        if (name == "rotation")
        {
            return true;
        }
        //    if (name == "wheel_connection_point")
        //    {
        //        w_connect_point = p_value;
        //        steering_axis = get_basis().xform_inv(v_connect_point) - w_connect_point;
        //        steering_axis = steering_axis.normalized();
        //        _update_suspension_transform();
        //        update_gizmos();
        //        return true;
        //    }
        //    if (name == "vehicle_connection_point")
        //    {
        //        v_connect_point = p_value;
        //        steering_axis = get_basis().xform_inv(v_connect_point) - w_connect_point;
        //        steering_axis = steering_axis.normalized();
        //        _update_suspension_transform();
        //        update_gizmos();
        //        return true;
        //    }
        return false;
    }

public:
    void _enter_tree() override;
    void _exit_tree() override;
    void _ready() override;

    NodePath get_opposite_wheel() const { return opposite_wheel; }
    void set_opposite_wheel(const NodePath &p_node)
    {
        opposite_wheel = p_node;
        //      sib->opposite_wheel = this->get_path();

        //  Vehicle3D *cb = Object::cast_to<Vehicle3D>(get_parent());
        //         Vehicle3D *cb = Object::cast_to<Vehicle3D>(get_parent());

        // sib = Object::cast_to<Wheel3D>(get_node<Wheel3D>(opposite_wheel));
        //   sib = get_node<Wheel3D>(opposite_wheel);
        //     sib->set_opposite_wheel(this->get_path());
        //      sib->set_spring_anti_roll(anti_roll);
    }
    real_d get_ackermann_input() const { return ackermann_input; }
    void set_ackermann_input(real_d p_factor)
    {
        ackermann_input = p_factor;

        ackermann_factor = max_ackermann * ackermann_input;
        ackermann_steering = Math::atan(Math::tan(steering) / (1.0 + side_sign * ackermann_factor * Math::tan(steering)));

        update_gizmos();
    }

    real_d get_steering() const { return steering; }
    void set_steering(real_d p_angle)
    {
        steering = p_angle;
        // signed_steering = steering * side_sign;
        ackermann_steering = Math::atan(Math::tan(steering) / (1.0 + side_sign * ackermann_factor * Math::tan(steering)));
        set_basis(relaxed_transform.get_basis().rotated_local(steering_axis, ackermann_steering));
        // set_basis(relaxed_transform.get_basis());

        _update_steering_axis();
        update_gizmos();
    }

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

        _update_wheel_dimensions();
        update_gizmos();
    }

    real_d get_width() const { return tire_width; }
    void set_width(real_d p_width)
    {
        tire_width = p_width;
        _update_wheel_dimensions();
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
        _update_wheel_dimensions();
        update_gizmos();
    }

    real_d get_spring_travel() const { return spring_travel; }
    void set_spring_travel(real_d p_length)
    {
        spring_travel = p_length;
        _update_suspension_length();
        _update_connect_points();
        _update_suspension_transform();

        //   set_position(body_connect_point - steering_axis * spring_travel - axle_axis * wheel_offset);

        update_gizmos();
    }

    real_d get_spring_stiffness() const { return spring_stiffness; }
    void set_spring_stiffness(real_d p_value)
    {
        spring_stiffness = p_value;
        _update_suspension_length();
    }

    real_d get_spring_anti_roll() const { return anti_roll; }
    void set_spring_anti_roll(real_d p_value) { anti_roll = p_value; }

    real_d get_damping_bump() const { return bump; }
    void set_damping_bump(real_d p_value)
    {
        bump = p_value;
        _update_suspension_length();
    }

    Ref<Curve> get_damping_bump_curve() const { return bump_curve; }
    void set_damping_bump_curve(Ref<Curve> p_curve)
    {
        bump_curve = p_curve;
        if (bump_curve.is_valid())
            bump_function = yes_damp_factor;
    }

    real_d get_damping_rebound() const { return rebound; }
    void set_damping_rebound(real_d p_value)
    {
        rebound = p_value;
        _update_suspension_length();
    }

    Ref<Curve> get_damping_rebound_curve() const { return rebound_curve; }
    void set_damping_rebound_curve(Ref<Curve> p_value)
    {
        rebound_curve = p_value;
        if (rebound_curve.is_valid())
            rebound_function = yes_damp_factor;
    }

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
        //     axle_axis_reference = side_sign * right_axis_reference;
        //   _update_axle_axis();
        axle_axis = right_axis_reference * side_sign;
        //   _update_steering_axis();
        //   set_position(body_connect_point - steering_axis * spring_travel - axle_axis * wheel_offset);
        _update_connect_points();
        _update_suspension_transform();

        //   set_position(-body_connect_point);
        update_gizmos();
    }

    bool get_powered() const { return powered; }
    void set_powered(bool p_value)
    {
        powered = p_value;
        update_gizmos();
    }

    bool get_steered() const { return steered; }
    void set_steered(bool p_value) { steered = p_value; }

    bool get_relative_transform() const { return relative_transform; }
    void set_relative_transform(bool p_value)
    {
        relative_transform = p_value;
        notify_property_list_changed();
    }

    real_d get_alignment_steering_axis_inclination() const { return steering_axis_inclination; }
    void set_alignment_steering_axis_inclination(real_d p_angle)
    {
        steering_axis_inclination = p_angle;
        signed_steering_axis_inclination = -side_sign * steering_axis_inclination;
        //    _update_axle_axis();
        //   _update_steering_axis();
        _update_connect_points();
        _update_suspension_transform();

        //  set_position(-body_connect_point);
        //  set_position(body_connect_point - steering_axis * spring_travel - axle_axis * wheel_offset);

        update_gizmos();
    }

    real_d get_alignment_camber_angle() const { return camber_angle; }
    void set_alignment_camber_angle(real_d p_angle)
    {
        camber_angle = p_angle;
        signed_camber_angle = side_sign * camber_angle;
        ///   _update_axle_axis();
        //    _update_steering_axis();
        set_rotation(Vector3(0.0, signed_toe_angle, -signed_camber_angle));

        _update_connect_points();

        _update_suspension_transform();
        //  set_position(-body_connect_point);
        //  set_position(body_connect_point - steering_axis * spring_travel - axle_axis * wheel_offset);
        update_gizmos();
    }

    real_d get_alignment_toe_angle() const { return toe_angle; }
    void set_alignment_toe_angle(real_d p_angle)
    {
        toe_angle = p_angle;
        signed_toe_angle = side_sign * toe_angle;
        //   _update_axle_axis();
        //    _update_steering_axis();
        set_rotation(Vector3(0.0, signed_toe_angle, -signed_camber_angle));

        _update_connect_points();
        _update_suspension_transform();

        //  set_position(-body_connect_point);
        //   set_position(body_connect_point - steering_axis * spring_travel - axle_axis * wheel_offset);

        update_gizmos();
    }

    real_d get_alignment_caster_angle() const { return caster_angle; }
    void set_alignment_caster_angle(real_d p_angle)
    {
        caster_angle = p_angle;
        //      _update_axle_axis();
        //   _update_steering_axis();

        _update_connect_points();
        _update_suspension_transform();

        //   set_position(-body_connect_point);
        //  body_connect_point = axle_axis * wheel_offset - spring_travel * steering_axis;
        //   set_position(body_connect_point - steering_axis * spring_travel - axle_axis * wheel_offset);

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
        _update_suspension_transform();

        // set_position(-body_connect_point);
        // set_position(body_connect_point - steering_axis * spring_travel - axle_axis * wheel_offset);
        //    _update_steering_axis();
        //    steering_axis = steering_axis.move_toward(axle_axis, wheel_offset);
        //  steering_axis = steering_axis + axle_axis * wheel_offset;
        update_gizmos();
    }

    // Vector3 get_wheel_connection_point() const { return w_connect_point; }
    // void set_wheel_connection_point(Vector3 p_value) { w_connect_point = p_value; }

    // Vector3 get_vehicle_connection_point() const { return v_connect_point; }
    // void set_vehicle_connection_point(Vector3 p_value) { v_connect_point = p_value; }
    real_d get_ackermann_steering() const { return ackermann_steering; }
    Vector3 get_steering_axis() const { return steering_axis; }
    Vector3 get_axle_axis() const { return axle_axis; }
    double get_spring_force() const { return spring_force; }

    PackedStringArray _get_configuration_warnings() const override;

    Wheel3D()
    {
        // set_notify_local_transform(false);
        // set_notify_transform(false);

        _update_wheel_dimensions();
        //    _update_suspension_length();
        //    _update_suspension_transform();
        Vector3 wheel_child_rotation = Vector3(0, 0, HALF_PI);
        tire_cast.set_rotation(wheel_child_rotation);
        hub_collider.set_rotation(wheel_child_rotation);
        suspension.set_rotation(wheel_child_rotation);

        hub_collider.set_disabled(false);

        tire_cast.set_target_position(zeros);

        tire_cast.set_shape(cast_to<Shape3D>(&tire_cast_shape));
        hub_collider.set_shape(cast_to<Shape3D>(&hub_shape));

        suspension.set_visible(true);
        suspension.set_exclude_nodes_from_collision(true);
        suspension.set_param(SliderJoint3D::Param::PARAM_LINEAR_LIMIT_UPPER, spring_travel);
        suspension.set_param(SliderJoint3D::Param::PARAM_LINEAR_LIMIT_LOWER, 0.0);
        // Vector3 wheel_child_rotation = Vector3(0, 0, HALF_PI);
        tire_cast.set_rotation(wheel_child_rotation);
        hub_collider.set_rotation(wheel_child_rotation);
        suspension.set_rotation(wheel_child_rotation);

        springDebugA.set_position(Vector3(spring_travel, 0, 0));
        springDebugB.set_position(Vector3(0, 0, 0));

        suspension.set_position(w_connect_point);

        matA.set_albedo(Color(1, 1, 1));
        matB.set_albedo(Color(0, 0, 1));
        springDebugMeshA.set_material(cast_to<Material>(&matA));
        springDebugMeshB.set_material(cast_to<Material>(&matB));
        connectDebugMeshA.set_material(cast_to<Material>(&matA));
        connectDebugMeshB.set_material(cast_to<Material>(&matB));

        springDebugMeshA.set_radius(0.05);
        springDebugMeshA.set_height(0.1);
        connectDebugMeshA.set_size(Vector3(0.1, 0.1, 0.1));
        springDebugMeshA.set_radius(0.05);
        springDebugMeshA.set_height(0.1);
        connectDebugMeshA.set_size(Vector3(0.1, 0.1, 0.1));

        springDebugMeshB.set_radius(0.05);
        springDebugMeshB.set_height(0.1);
        connectDebugMeshB.set_size(Vector3(0.1, 0.1, 0.1));
        springDebugMeshB.set_radius(0.05);
        springDebugMeshB.set_height(0.1);
        connectDebugMeshB.set_size(Vector3(0.1, 0.1, 0.1));

        springDebugA.set_mesh(cast_to<Mesh>(&springDebugMeshA));
        springDebugB.set_mesh(cast_to<Mesh>(&springDebugMeshB));
        connectDebugA.set_mesh(cast_to<Mesh>(&connectDebugMeshA));
        connectDebugB.set_mesh(cast_to<Mesh>(&connectDebugMeshB));

        side_sign = side - 1;
    }
    ~Wheel3D() { queue_free(); }
    virtual void _input(const Ref<InputEvent> &event) override;
    void emit_custom_signal(const String &name, int value)
    {
        emit_signal("custom_signal", name, value);
    }
    void _notification(int p_what)
    {
        if (p_what == NOTIFICATION_LOCAL_TRANSFORM_CHANGED)
        {
            Vector3 wheel_rot = get_rotation();
            //   caster_angle = wheel_rot.x;
            signed_toe_angle = wheel_rot.y;
            toe_angle = signed_toe_angle * side_sign;
            signed_camber_angle = -wheel_rot.z;
            camber_angle = signed_camber_angle * side_sign;
            // set_rotation(Vector3(0.0, wheel_rot.y, wheel_rot.z));
            _update_connect_points();
            update_gizmos();
        }
    }
};

VARIANT_ENUM_CAST(Wheel3D::Sides);

class Vehicle3D : public RigidBody3D
{
    GDCLASS(Vehicle3D, RigidBody3D);

    friend class Wheel3D;

private:
    // Transform3D v_global_xform;
    // Transform3D v_local_xform;
    // Basis v_local_basis;
    // Vector3 v_local_origin;
    Vector<Wheel3D *> wheels;
    Transform3D global_xform;
    Basis global_basis;
    Vector3 global_origin;

protected:
    static void _bind_methods()
    {
    }

public:
    //  std::vector<Wheel3D *> get_wheels() const { return wheels; }
    void _enter_tree() override;
    void _exit_tree() override;
    void _ready() override;
    void _physics_process(double delta) override;

    // void _integrate_forces(PhysicsDirectBodyState3D *state) override;

    // void _notification(int p_what);

    Vehicle3D()
    {
    }
    ~Vehicle3D() { queue_free(); }
};
#endif // VEHICLE3D_HPP