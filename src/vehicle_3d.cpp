#include "vehicle_3d.hpp"
#include "macros.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/engine.hpp>

using namespace godot;

void Wheel3D::_bind_methods()
{
    BIND_METHOD(Wheel3D, set_pair, "pair");
    BIND_METHOD(Wheel3D, get_pair);

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

    BIND_METHOD(Wheel3D, get_steering_axis);

    BIND_METHOD(Wheel3D, get_axle_axis);
    //  ClassDB::bind_method(D_METHOD("set_pair", "pair"), &Wheel3D::set_pair);
    //  ClassDB::bind_method(D_METHOD("get_pair"), &Wheel3D::get_pair);

    //  ClassDB::bind_method(D_METHOD("set_tire_radius", "tire_radius"), &Wheel3D::set_tire_radius);
    // ClassDB::bind_method(D_METHOD("get_tire_radius"), &Wheel3D::get_tire_radius);

    //    ClassDB::bind_method(D_METHOD("set_width", "tire_width"), &Wheel3D::set_width);
    //    ClassDB::bind_method(D_METHOD("get_width"), &Wheel3D::get_width);
    //
    //    ClassDB::bind_method(D_METHOD("set_hub_radius", "hub_radius"), &Wheel3D::set_hub_radius);
    //    ClassDB::bind_method(D_METHOD("get_hub_radius"), &Wheel3D::get_hub_radius);
    //
    //    ClassDB::bind_method(D_METHOD("set_side", "side"), &Wheel3D::set_side);
    //    ClassDB::bind_method(D_METHOD("get_side"), &Wheel3D::get_side);
    //
    //    ClassDB::bind_method(D_METHOD("set_powered", "powered"), &Wheel3D::set_powered);
    //    ClassDB::bind_method(D_METHOD("get_powered"), &Wheel3D::get_powered);
    //
    //    ClassDB::bind_method(D_METHOD("set_steered", "steered"), &Wheel3D::set_steered);
    //    ClassDB::bind_method(D_METHOD("get_steered"), &Wheel3D::get_steered);
    //
    //    ClassDB::bind_method(D_METHOD("set_steering", "steering"), &Wheel3D::set_steering);
    //    ClassDB::bind_method(D_METHOD("get_steering"), &Wheel3D::get_steering);
    //
    //    ClassDB::bind_method(D_METHOD("set_engine_torque", "engine_torque"), &Wheel3D::set_engine_torque);
    //    ClassDB::bind_method(D_METHOD("get_engine_torque"), &Wheel3D::get_engine_torque);
    //
    //    ClassDB::bind_method(D_METHOD("set_brake_torque", "brake_torque"), &Wheel3D::set_brake_torque);
    //    ClassDB::bind_method(D_METHOD("get_brake_torque"), &Wheel3D::get_brake_torque);
    //
    //    ClassDB::bind_method(D_METHOD("get_steering_axis"), &Wheel3D::get_steering_axis);
    //    ClassDB::bind_method(D_METHOD("get_axle_axis"), &Wheel3D::get_axle_axis);

    // ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "pair", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Wheel3D"), "set_pair", "get_pair");
    //    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tire_radius", PROPERTY_HINT_RANGE, "0.001,5,0.001,suffix:m"), "set_tire_radius", "get_tire_radius");
    //    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "width", PROPERTY_HINT_RANGE, "0.001,5,0.001,suffix:m"), "set_width", "get_width");
    //    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "hub_radius", PROPERTY_HINT_RANGE, "0.001,5,0.001,suffix:m"), "set_hub_radius", "get_hub_radius");
    //    ADD_PROPERTY(PropertyInfo(Variant::INT, "side", PROPERTY_HINT_ENUM, "Left,Center,Right"), "set_side", "get_side");
    //    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "powered"), "set_powered", "get_powered");
    //    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "steered"), "set_steered", "get_steered");
    //    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "steering", PROPERTY_HINT_RANGE, "-180,180,0.01,radians"), "set_steering", "get_steering");
    //    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "engine_torque", PROPERTY_HINT_RANGE, u"0,1000,0.01,or_greater,suffix:kg\u22C5m/s\u00B2 (N)"), "set_engine_torque", "get_engine_torque");
    //    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "brake_torque", PROPERTY_HINT_RANGE, u"0,1000,0.01,or_greater,suffix:kg\u22C5m/s\u00B2 (N)"), "set_brake_torque", "get_brake_torque");

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

    // ClassDB::bind_method(D_METHOD("set_wheel_offset", "alignment_wheel_offset"), &Wheel3D::set_wheel_offset);
    // ClassDB::bind_method(D_METHOD("get_wheel_offset"), &Wheel3D::get_wheel_offset);
    //
    // ClassDB::bind_method(D_METHOD("set_steering_axis_inclination", "alignment_steering_axis_inclination"), &Wheel3D::set_steering_axis_inclination);
    // ClassDB::bind_method(D_METHOD("get_steering_axis_inclination"), &Wheel3D::get_steering_axis_inclination);
    //
    // ClassDB::bind_method(D_METHOD("set_camber_angle", "alignment_camber_angle"), &Wheel3D::set_camber_angle);
    // ClassDB::bind_method(D_METHOD("get_camber_angle"), &Wheel3D::get_camber_angle);
    //
    // ClassDB::bind_method(D_METHOD("set_toe_angle", "alignment_toe_angle"), &Wheel3D::set_toe_angle);
    // ClassDB::bind_method(D_METHOD("get_toe_angle"), &Wheel3D::get_toe_angle);
    //
    // ClassDB::bind_method(D_METHOD("set_caster_angle", "alignment_caster_angle"), &Wheel3D::set_caster_angle);
    // ClassDB::bind_method(D_METHOD("get_caster_angle"), &Wheel3D::get_caster_angle);

    // ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "alignment_wheel_offset", PROPERTY_HINT_RANGE, "0.0,5,0.001,suffix:m"), "set_wheel_offset", "get_wheel_offset");
    // ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "alignment_steering_axis_inclination", PROPERTY_HINT_RANGE, "-90,90,0.01,radians"), "set_steering_axis_inclination", "get_steering_axis_inclination");
    // ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "alignment_camber_angle", PROPERTY_HINT_RANGE, "-90,90,0.01,radians"), "set_camber_angle", "get_camber_angle");
    // ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "alignment_toe_angle", PROPERTY_HINT_RANGE, "-90,90,0.01,radians"), "set_toe_angle", "get_toe_angle");
    // ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "alignment_caster_angle", PROPERTY_HINT_RANGE, "-90,90,0.01,radians"), "set_caster_angle", "get_caster_angle");

    BIND_METHOD(Wheel3D, set_suspension_length, "suspension_length");
    BIND_METHOD(Wheel3D, get_suspension_length);

    BIND_METHOD(Wheel3D, set_suspension_spring_stiffness, "suspension_spring_stiffness");
    BIND_METHOD(Wheel3D, get_suspension_spring_stiffness);

    BIND_METHOD(Wheel3D, set_suspension_bump_damping, "suspension_bump_damping");
    BIND_METHOD(Wheel3D, get_suspension_bump_damping);

    BIND_METHOD(Wheel3D, set_suspension_rebound_damping, "suspension_rebound_damping");
    BIND_METHOD(Wheel3D, get_suspension_rebound_damping);

    // ClassDB::bind_method(D_METHOD("set_suspension_length", "suspension_length"), &Wheel3D::set_suspension_length);
    // ClassDB::bind_method(D_METHOD("get_suspension_length"), &Wheel3D::get_suspension_length);

    // ClassDB::bind_method(D_METHOD("set_spring_stiffness", "suspension_spring_stiffness"), &Wheel3D::set_spring_stiffness);
    // ClassDB::bind_method(D_METHOD("get_spring_stiffness"), &Wheel3D::get_spring_stiffness);

    // ClassDB::bind_method(D_METHOD("set_bump_damping", "suspension_bump_damping"), &Wheel3D::set_bump_damping);
    // ClassDB::bind_method(D_METHOD("get_bump_damping"), &Wheel3D::get_bump_damping);

    // ClassDB::bind_method(D_METHOD("set_rebound_damping", "suspension_rebound_damping"), &Wheel3D::set_bump_damping);
    // ClassDB::bind_method(D_METHOD("get_rebound_damping"), &Wheel3D::get_rebound_damping);
    BIND_PROPERTY_HINTED("pair", Variant::NODE_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Wheel3D");
    BIND_PROPERTY_RANGED("tire_radius", Variant::FLOAT, "0.001,5,0.001,suffix:m");
    BIND_PROPERTY_RANGED("width", Variant::FLOAT, "0.001,5,0.001,suffix:m");
    BIND_PROPERTY_RANGED("hub_radius", Variant::FLOAT, "0.001,5,0.001,suffix:m");
    BIND_PROPERTY_ENUM("side", "Left,Center,Right");
    BIND_PROPERTY("powered", Variant::BOOL);
    BIND_PROPERTY("steered", Variant::BOOL);
    BIND_PROPERTY_RANGED("steering", Variant::FLOAT, "-180,180,0.01,radians");
    BIND_PROPERTY_RANGED("engine_torque", Variant::FLOAT, u"0,1000,0.01,or_greater,suffix:kg\u22C5m/s\u00B2 (N)");
    BIND_PROPERTY_RANGED("brake_torque", Variant::FLOAT, u"0,1000,0.01,or_greater,suffix:kg\u22C5m/s\u00B2 (N)");

    ADD_GROUP("Alignment", "alignment_");

    BIND_PROPERTY_RANGED("alignment_wheel_offset", Variant::FLOAT, "0.0,5,0.001,suffix:m");
    BIND_PROPERTY_RANGED("alignment_steering_axis_inclination", Variant::FLOAT, "-90,90,0.01,radians");
    BIND_PROPERTY_RANGED("alignment_camber_angle", Variant::FLOAT, "-90,90,0.01,radians");
    BIND_PROPERTY_RANGED("alignment_toe_angle", Variant::FLOAT, "-90,90,0.01,radians");
    BIND_PROPERTY_RANGED("alignment_caster_angle", Variant::FLOAT, "-90,90,0.01,radians");

    ADD_GROUP("Suspension", "suspension_");

    BIND_PROPERTY_RANGED("suspension_length", Variant::FLOAT, "0.001,10,0.001,suffix:m");
    BIND_PROPERTY_RANGED("suspension_spring_stiffness", Variant::FLOAT, "0,1000,0.01,suffix:m");
    BIND_PROPERTY_RANGED("suspension_bump_damping", Variant::FLOAT, u"0,1000,0.01,or_greater,suffix:N\u22C5s/m");
    BIND_PROPERTY_RANGED("suspension_rebound_damping", Variant::FLOAT, u"0,1000,0.01,or_greater,suffix:N\u22C5s/m");
    //  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_length", PROPERTY_HINT_RANGE, "0.001,10,0.001,suffix:m"), "set_suspension_length", "get_suspension_length");
    //  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_spring_stiffness", PROPERTY_HINT_RANGE, "0,1000,0.01,or_greater,suffix:m"), "set_spring_stiffness", "get_spring_stiffness");
    //  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_bump_damping", PROPERTY_HINT_RANGE, u"0,1000,0.01,or_greater,suffix:N\u22C5s/m"), "set_bump_damping", "get_bump_damping");
    //  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_rebound_damping", PROPERTY_HINT_RANGE, u"0,1000,0.01,or_greater,suffix:N\u22C5s/m"), "set_bump_damping", "get_rebound_damping");

    BIND_ENUM_CONSTANT(LEFT);
    BIND_ENUM_CONSTANT(CENTER);
    BIND_ENUM_CONSTANT(RIGHT);
}

Wheel3D::Wheel3D()
{

    // Initialize any variables here.
    // time_passed = 0.0;
    // amplitude = 10.0;
    //  suspension = Generic6DOFJoint3D();
    //   suspension.set_param_x(Generic6DOFJoint3D::Param::PARAM_LINEAR_RESTITUTION, 0);
    CollisionShape3D dummyCollider = CollisionShape3D();
    SphereShape3D dummyShape = SphereShape3D();
    dummyShape.set_radius(m_tireRadius / 2);
    dummyCollider.set_shape(cast_to<Shape3D>(&dummyShape));
    dummyCollider.set_disabled(true);
    // add_child(&dummyCollider);

    set_inertia(0.4 * get_mass() * m_tireRadius * m_tireRadius * Vector3(real_t(1), real_t(1), real_t(1)));
}

Wheel3D::~Wheel3D()
{
    // Add your cleanup here.
    queue_free();
}

// void Wheel::_process(double delta)
//{
//     time_passed += delta;
// }
void Wheel3D::_ready()
{
    SphereShape3D castShape = SphereShape3D();
    castShape.set_radius(m_tireRadius);
    tireCast.set_shape(cast_to<Shape3D>(&castShape));
    tireCast.set_target_position(Vector3(real_t(0), real_t(0), real_t(0)));

    suspension.set_rotation(Vector3(real_t(0), real_t(0), Math_PI * 0.5));
    suspension.set_param(SliderJoint3D::Param::PARAM_LINEAR_LIMIT_UPPER, m_suspensionLength);
    suspension.set_param(SliderJoint3D::Param::PARAM_LINEAR_LIMIT_LOWER, 0.0);
    suspension.set_exclude_nodes_from_collision(true);
    suspension.set_node_b(this->get_path());
    if (body != nullptr)
    {
        suspension.set_node_a(body->get_path());
    }

    // add_child(&tireCast);
    // add_child(&suspension);
}
void Wheel3D::_physics_process(double delta)
{
    if (tireCast.is_colliding())
    {
        Vector3 collisionPointGlobal = tireCast.get_collision_point(0);
        Vector3 collisionNormal = tireCast.get_collision_normal(0);
        float tireSqueeze = 1.0 - m_tireRadius / collisionPointGlobal.distance_to(get_global_position());
        //     apply_central_force(-20.0 * collisionNormal * tireSqueeze);
    }

    float springForce = std::clamp((m_suspensionLength - get_position().y) * m_springStiffness, real_t(0), m_suspensionLength * m_springStiffness);
    Vector3 suspensionForceVec = get_transform().basis.get_column(Vector3::AXIS_Y).normalized() * (springForce); // * (1f - Mathf.Sin(Vector3.Up.AngleTo(GlobalTransform.Basis * suspensionUp)));
    if (body != nullptr)
    {
        body->apply_force(get_global_transform().get_basis().xform(suspensionForceVec), get_global_position() - body->get_global_position());
    }
    apply_central_force(get_global_transform().get_basis().xform(-suspensionForceVec));

    //  ERR_PRINT_ONCE("DSKLJF");
    //   std::cout << "test";
    //  ERR_PRINT("\u00A5");
    //   WARN_PRINT(std::to_string(0));
    //    printf("Characters: %c %c \n", 'a', 65);
}
void Wheel3D::_notification(int p_what)
{
    switch (p_what)
    {
    case NOTIFICATION_ENTER_TREE:
    {
        //  VehicleWheel3D dummyWheel = VehicleWheel3D();
        //  add_child(&dummyWheel);
        Vehicle3D *cb = Object::cast_to<Vehicle3D>(get_parent());
        if (!cb)
        {
            body = nullptr;
            return;
        }
        body = cb;
        local_xform = get_transform();
        //     cb->wheels.push_back(this);
        cb->wheels.emplace(this);

        m_chassisConnectionPointCS = get_transform().origin;
        m_wheelDirectionCS = -get_transform().basis.get_column(Vector3::AXIS_Y).normalized();
        m_wheelAxleCS = get_transform().basis.get_column(Vector3::AXIS_X).normalized();
    }
    break;

    case NOTIFICATION_EXIT_TREE:
    {
        Vehicle3D *cb = Object::cast_to<Vehicle3D>(get_parent());
        //  String mujclass = get_parent().get_class();
        if (!cb)
        {
            body = nullptr;
            return;
        }
        // cb->wheels.erase delete[] &cb->wheels;
        // cb->wheels.erase(cb->wheels.begin() + (this - cb->wheels.front()));
        cb->wheels.erase(this);
        body = nullptr;
    }
    break;
    }
}

// PackedStringArray Wheel3D::_get_configuration_warnings() const
//{
//     PackedStringArray warnings = Node::_get_configuration_warnings();
//
//     if (Object::cast_to<Vehicle3D>(get_parent()) == nullptr)
//     {
//         warnings.push_back("VehicleWheel3D serves to provide a wheel system to a VehicleBody3D. Please use it as a child of a VehicleBody3D.");
//     }
//
//     return warnings;
// }

void Wheel3D::set_pair(const NodePath &p_node)
{
    m_pair = p_node;
    // dummyCollider.set_shape(cast_to<Shape3D>(&dummyShape));
    //    cast_to<Wheel3D>(p_node);
    //  RigidBody3D sib1 = this->get_node<RigidBody3D>(p_node);
    // add_sibling();
    //  NodePath nn = NodePath();
    // SceneTree *tree = (SceneTree *)Engine::get_singleton()->get_main_loop();

    // Object *node = get_root()->get_node<Object>(p_path);
    Wheel3D *sib = get_node<Wheel3D>(m_pair);
    // sib = this;
    // sib->set_tire_radius(m_tireRadius);
    // sib->queue_free();
    // sib->m_tireRadius = m_tireRadius;
    // sib->add_to_group
    // Wheel3D sib = cast_to<Wheel3D>(get_node(&m_pair));
}

NodePath Wheel3D::get_pair() const
{
    return m_pair;
}
void Wheel3D::set_tire_radius(real_t p_radius)
{
    m_tireRadius = p_radius;
    if (m_tireRadius < m_hubRadius)
    {
        m_hubRadius = m_tireRadius;
    }
    update_gizmos();
}

real_t Wheel3D::get_tire_radius() const
{
    return m_tireRadius;
}
void Wheel3D::set_width(real_t p_width)
{
    m_width = p_width;
    update_gizmos();
}

real_t Wheel3D::get_width() const
{
    return m_width;
}

void Wheel3D::set_hub_radius(real_t p_radius)
{
    m_hubRadius = p_radius;
    if (m_tireRadius < m_hubRadius)
    {
        m_tireRadius = m_hubRadius;
    }
    update_gizmos();
}

real_t Wheel3D::get_hub_radius() const
{
    return m_hubRadius;
}

void Wheel3D::set_suspension_length(real_t p_length)
{
    m_suspensionLength = p_length;
    update_gizmos();
}

real_t Wheel3D::get_suspension_length() const
{
    return m_suspensionLength;
}

void Wheel3D::set_suspension_spring_stiffness(real_t p_value)
{
    m_springStiffness = p_value;
}

real_t Wheel3D::get_suspension_spring_stiffness() const
{
    return m_springStiffness;
}

void Wheel3D::set_suspension_bump_damping(real_t p_value)
{
    m_bumpDamping = p_value;
}

real_t Wheel3D::get_suspension_bump_damping() const
{
    return m_bumpDamping;
}

void Wheel3D::set_suspension_rebound_damping(real_t p_value)
{
    m_reboundDamping = p_value;
}

real_t Wheel3D::get_suspension_rebound_damping() const
{
    return m_reboundDamping;
}
void Wheel3D::set_side(Sides p_value)
{
    m_side = p_value;
    update_gizmos();
}

Wheel3D::Sides Wheel3D::get_side() const
{
    return m_side;
}
void Wheel3D::set_powered(bool p_value)
{
    m_powered = p_value;
    update_gizmos();
}

bool Wheel3D::get_powered() const
{
    return m_powered;
}
void Wheel3D::set_steered(bool p_value)
{
    m_steered = p_value;
    update_gizmos();
}

bool Wheel3D::get_steered() const
{
    return m_steered;
}
void Wheel3D::set_steering(real_t p_angle)
{
    m_steering = p_angle;
    update_gizmos();
}

real_t Wheel3D::get_steering() const
{
    return m_steering;
}
void Wheel3D::set_engine_torque(real_t p_torque)
{
    m_engine_torque = p_torque;
    update_gizmos();
}

real_t Wheel3D::get_engine_torque() const
{
    return m_engine_torque;
}
void Wheel3D::set_brake_torque(real_t p_torque)
{
    m_brake_torque = p_torque;
}

real_t Wheel3D::get_brake_torque() const
{
    return m_brake_torque;
}
void Wheel3D::set_alignment_wheel_offset(real_t p_value)
{
    m_wheelOffset = p_value;
    update_gizmos();
}

real_t Wheel3D::get_alignment_wheel_offset() const
{
    return m_wheelOffset;
}

void Wheel3D::set_alignment_steering_axis_inclination(real_t p_angle)
{
    m_steering_axis_inclination = p_angle;
    m_steeringAxis = m_steeringAxisReference.rotated(m_forwardAxisReference, -m_steering_axis_inclination);
    m_steeringAxis = m_steeringAxis.rotated(m_axleAxisReference, m_caster_angle).normalized();

    update_gizmos();
}

real_t Wheel3D::get_alignment_steering_axis_inclination() const
{
    return m_steering_axis_inclination;
}
void Wheel3D::set_alignment_camber_angle(real_t p_angle)
{
    m_camber_angle = p_angle;
    m_axleAxis = m_axleAxisReference.rotated(m_forwardAxisReference, m_camber_angle);
    m_axleAxis = m_axleAxis.rotated(m_steeringAxisReference, m_toe_angle).normalized();
    update_gizmos();
}

real_t Wheel3D::get_alignment_camber_angle() const
{
    return m_camber_angle;
}
void Wheel3D::set_alignment_toe_angle(real_t p_angle)
{
    m_toe_angle = p_angle;
    m_axleAxis = m_axleAxisReference.rotated(m_forwardAxisReference, m_camber_angle);
    m_axleAxis = m_axleAxis.rotated(m_steeringAxisReference, m_toe_angle).normalized();
    update_gizmos();
}

real_t Wheel3D::get_alignment_toe_angle() const
{
    return m_toe_angle;
}
void Wheel3D::set_alignment_caster_angle(real_t p_angle)
{
    m_caster_angle = p_angle;

    m_steeringAxis = m_steeringAxisReference.rotated(m_forwardAxisReference, -m_steering_axis_inclination);
    m_steeringAxis = m_steeringAxis.rotated(m_axleAxisReference, m_caster_angle).normalized();
    // suspensionpoints[i] = suspensionpoints[i].rotated(Vector3.FORWARD, inclin).rotated(Vector3.RIGHT, caster);
    update_gizmos();
}
real_t Wheel3D::get_alignment_caster_angle() const
{
    return m_caster_angle;
}
Vector3 Wheel3D::get_steering_axis() const
{
    return m_steeringAxis;
}
Vector3 Wheel3D::get_axle_axis() const
{
    return m_axleAxis;
}
// void Wheel::set_amplitude(const double p_amplitude)
//{
//     amplitude = p_amplitude;
// }

// double Wheel::get_amplitude() const
//{
//     return amplitude;
// }
void Vehicle3D::_bind_methods()
{
}

Vehicle3D::Vehicle3D()
{
}

Vehicle3D::~Vehicle3D()
{
    queue_free();
    //  delete[] &wheels;
    //   Vehicle3D::cast_to<Node>(this).free();
    //   Object::free
}