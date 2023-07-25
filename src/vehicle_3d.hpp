#ifndef VEHICLE_3D_HPP
#define VEHICLE_3D_HPP

#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/slider_joint3d.hpp>
#include <godot_cpp/classes/shape_cast3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
// #include <godot_cpp/classes/vehicle_wheel3d.hpp>

// #include <godot_cpp/variant/typed_array.hpp>

// #include <godot_cpp/classes/physics_body3d.hpp>

using namespace godot;

class Vehicle3D;

class Wheel3D : public RigidBody3D
{
    GDCLASS(Wheel3D, RigidBody3D)

    friend class Vehicle3D;

public:
    enum Sides
    {
        LEFT,
        CENTER,
        RIGHT
    };

private:
    NodePath m_pair;
    Vehicle3D *body = nullptr;
    SliderJoint3D suspension = SliderJoint3D();
    ShapeCast3D tireCast = ShapeCast3D();

    Transform3D m_worldTransform;
    Transform3D local_xform;

    Vector3 m_chassisConnectionPointCS; // const
    Vector3 m_wheelDirectionCS;         // const
    Vector3 m_wheelAxleCS;              // const or modified by steering

    bool steers = false;

    real_t m_tireRadius = 0.5;
    real_t m_width = 0.3;

    real_t m_hubRadius = 0.3;
    real_t m_suspensionLength = 1.0;
    real_t m_springStiffness = 20.0;
    real_t m_bumpDamping = 1.0;
    real_t m_reboundDamping = 1.0;
    Sides m_side = Sides::LEFT;
    bool m_powered = false;
    bool m_steered = false;
    real_t m_steering = 0.0;
    real_t m_engine_torque = 0.0;
    real_t m_brake_torque = 0.0;
    real_t m_camber_angle = 0.0;
    real_t m_toe_angle = 0.0;
    real_t m_steering_axis_inclination = 0.0;
    real_t m_caster_angle = 0.0;
    real_t m_wheelOffset = 0.0;

    const Vector3 m_steeringAxisReference = Vector3(0, 1, 0);
    const Vector3 m_forwardAxisReference = Vector3(0, 0, -1);
    const Vector3 m_axleAxisReference = Vector3(1, 0, 0);

    Vector3 m_steeringAxis = m_steeringAxisReference;
    Vector3 m_axleAxis = m_axleAxisReference;

protected:
    void _notification(int p_what);
    static void _bind_methods();

public:
    void set_pair(const NodePath &p_node);
    NodePath get_pair() const;

    void set_steering(real_t p_angle);
    real_t get_steering() const;

    void set_engine_torque(real_t p_torque);
    real_t get_engine_torque() const;

    void set_brake_torque(real_t p_torque);
    real_t get_brake_torque() const;

    void set_tire_radius(real_t p_radius);
    real_t get_tire_radius() const;

    void set_width(real_t p_width);
    real_t get_width() const;

    void set_hub_radius(real_t p_radius);
    real_t get_hub_radius() const;

    void set_suspension_length(real_t p_length);
    real_t get_suspension_length() const;

    void set_suspension_spring_stiffness(real_t p_value);
    real_t get_suspension_spring_stiffness() const;

    void set_suspension_bump_damping(real_t p_value);
    real_t get_suspension_bump_damping() const;

    void set_suspension_rebound_damping(real_t p_value);
    real_t get_suspension_rebound_damping() const;

    void set_side(Sides p_value);
    Sides get_side() const;

    void set_powered(bool p_value);
    bool get_powered() const;

    void set_steered(bool p_value);
    bool get_steered() const;

    void set_alignment_steering_axis_inclination(real_t p_angle);
    real_t get_alignment_steering_axis_inclination() const;

    void set_alignment_camber_angle(real_t p_angle);
    real_t get_alignment_camber_angle() const;

    void set_alignment_toe_angle(real_t p_angle);
    real_t get_alignment_toe_angle() const;

    void set_alignment_caster_angle(real_t p_angle);
    real_t get_alignment_caster_angle() const;

    void set_alignment_wheel_offset(real_t p_value);
    real_t get_alignment_wheel_offset() const;

    Vector3 get_steering_axis() const;
    Vector3 get_axle_axis() const;

    // PackedStringArray _get_configuration_warnings() const override;

    Wheel3D();
    ~Wheel3D();

    // void set_amplitude(const double p_amplitude);
    // double get_amplitude() const;

    //       void _process(double delta) override;
    void _ready() override;
    void _physics_process(double delta) override;
};

VARIANT_ENUM_CAST(Wheel3D::Sides);

// class Wheel3DPair : public Wheel3D
//{
//     GDCLASS(Wheel3DPair, Wheel3D)
// private:
//     real_t m_track_width;
// };

class Vehicle3D : public RigidBody3D
{
    GDCLASS(Vehicle3D, RigidBody3D);

    friend class VehicleWheel3D;
    std::set<Wheel3D *> wheels;

protected:
    static void _bind_methods();

public:
    Vehicle3D();
    ~Vehicle3D();
};

#endif // VEHICLE_3D_HPP