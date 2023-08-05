#include "vehicle3d.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/physics_direct_body_state3d.hpp>

using namespace godot;

void Vehicle3D::_enter_tree()
{
}
void Vehicle3D::_exit_tree()
{
}
void Wheel3D::_enter_tree()
{

    suspension.set_node_b(get_path());
    Vehicle3D *cb = Object::cast_to<Vehicle3D>(get_parent());
    if (!cb)
        return;
    body = cb;
    cb->wheels.push_back(this);
    suspension.set_node_a(body->get_path());
    suspension.set_node_a("../..");
}
void Wheel3D::_exit_tree()
{
    suspension.set_node_a("");
    Vehicle3D *cb = Object::cast_to<Vehicle3D>(get_parent());
    if (!cb)
        return;
    body = nullptr;
    cb->wheels.erase(this);
    sib = get_node<Wheel3D>(get_path());
}

void Wheel3D::_ready()
{
    if (opposite_wheel.is_empty())
        sib = get_node<Wheel3D>(get_path());
    else
        sib = get_node<Wheel3D>(opposite_wheel);
    add_child(&tire_cast);
    add_child(&hub_collider);
    add_child(&suspension);

    _update_connect_points();
    suspension.add_child(&springDebugA);
    suspension.add_child(&springDebugB);

    relaxed_transform = get_transform();
}
void Vehicle3D::_ready()
{

    for (Wheel3D *w : wheels)
    {
        add_child(&w->connectDebugA);
        add_child(&w->connectDebugB);
        add_child(&w->springDebugRay);

        //  add_child(&w->suspension);
        w->connectDebugA.set_position(w->v_connect_point);
        w->connectDebugB.set_position(get_basis().xform(w->w_connect_point));
        w->springDebugRay.set_position(w->v_connect_point);

        //   w->suspension.set_node_a(get_path());
    }
}

void Vehicle3D::_physics_process(double delta)
{
    global_xform = get_global_transform();
    global_basis = global_xform.get_basis();
    global_origin = global_xform.get_origin();
    for (Wheel3D *w : wheels)
    {
        w->global_xform = w->get_global_transform();
        w->global_basis = w->global_xform.get_basis();
        w->global_origin = w->global_xform.get_origin();
        w->local_origin = w->get_position();
        //    w->tireRay.set_target_position(w->get_transform().get_basis().xform(Vector3(0, -w->tire_radius, 0)));
        //   w->tireRay.set_target_position(Vector3(0, -w->tire_radius, 0));

        if (w->tire_cast.is_colliding())
        {

            Vector3 collisionPointGlobal = w->tire_cast.get_collision_point(0);
            Vector3 collisionNormal = w->tire_cast.get_collision_normal(0);
            Plane collisionPlane = Plane(collisionNormal);
            real_d collision_angle = w->axle_axis.angle_to(collisionNormal);

            //     Vector3 ray_place = collisionNormal.rotated();

            real_d d_max = w->corner_distance * Math::cos(w->corner_angle + collision_angle);
            //   w->springDebug.set_global_position(collisionPointGlobal);
            real_d chord_distance = collisionPointGlobal.distance_to(w->global_origin);
            real_d chord1_length = 2.0 * Math::sqrt(w->tire_radius * w->tire_radius);
            //  real_d tireSqueeze = collisionPointGlobal.distance_to(w_global_origin);
            real_d tireSqueeze = 1.0 - collisionPointGlobal.distance_to(global_origin) / w->tire_radius;
            //    real_d tireSqueeze = 1.0 - w->tireRay.get_collision_point().distance_to(w_global_origin) / w->tire_radius;

            // apply_central_force(-20.0 * collisionNormal * tireSqueeze);
            //   w->apply_force(5000.0 * collisionNormal * tireSqueeze * tireSqueeze, collisionPointGlobal);
            //   w->apply_central_force(220632.0 * Vector3(0, 1, 0) / tireSqueeze * tireSqueeze);
        }
        //       w->compression = w->relaxed_position.distance_to(w_local_origin) / w->spring_travel;
        //   double precomp = w->to_global(w->w_connect_point).distance_to(to_global(w->v_connect_point)) / w->spring_travel;
        w->w_to_v = to_global(w->v_connect_point) - w->to_global(w->w_connect_point);
        //    real_d v_to_w_distance = w->v_to_w.length();

        //  w->compression = 1.0 - w->to_global(w->w_connect_point).distance_to(to_global(w->v_connect_point)) /
        //                             w->spring_travel;
        w->compression = 1.0 - w->w_to_v.length() / w->spring_travel;
        //  w->w_to_v = (w->to_global(w->w_connect_point)).direction_to(to_global(w->v_connect_point));
    }
    for (Wheel3D *w : wheels)
    {

        //   real_d compression = 0.0;
        // cout << relaxed_position.x << ", " << relaxed_position.y << ", " << relaxed_position.z << "\n";
        // real_d max_force = spring_travel * spring_stiffness;
        real_d opposite_diff = w->compression - w->sib->compression;
        w->spring_force = w->stiff_rel * w->compression;
        real_d compress_velocity = (w->compression - w->prev_compression) / delta;
        if (compress_velocity > 0)
            w->damp_force = bump_function(w->bump_rel, w->bump_curve, compress_velocity);
        else
            w->damp_force = rebound_function(w->rebound_rel, w->rebound_curve, compress_velocity);

        real_d anti_roll_force = w->anti_roll * opposite_diff;
        Vector3 global_steering_axis = w->global_basis.xform(w->steering_axis);
        Vector3 w_local_suspension_force = w->steering_axis * (w->spring_force + w->damp_force + anti_roll_force);
        Vector3 global_suspension_force_old = w->global_basis.xform(w_local_suspension_force);
        global_suspension_force_old = global_steering_axis * (w->spring_force + w->damp_force + anti_roll_force);
        //  double steer_to_actual = global_steering_axis.angle_to(w->w_to_v);
        //    global_suspension_force_old *= Math::cos(steer_to_actual);
        //  Vector3 w_local_suspension_force = w->get_basis().xform(local_suspension_force);

        //     Vector3 v_local_suspension_force = w->get_basis().xform(w_local_suspension_force);
        // v_local_suspension_force = Vector3(0, 0, 0);
        // Vector3 w_global_suspension_force = w_global_basis.xform(w_local_suspension_force);
        //   Vector3 v_global_suspension_force = v_global_basis.xform(v_local_suspension_force);
        //  Vector3 global_suspension_force = w->global_basis.xform(w_local_suspension_force);
        Vector3 global_suspension_force = w->w_to_v.normalized() * (w->spring_force); //- w->damp_force + anti_roll_force);

        //   GPRINTVEC(w->global_basis.xform_inv(w->w_to_v.normalized()));

        //   Vector3 global_body_to_connection = v_global_basis.xform(w->v_connect_point);
        //  Vector3 global_body_connect_point = to_global(w->v_connect_point);

        //   Vector3 global_body_to_connection = global_body_connect_point - v_global_origin;

        //  apply_force(global_suspension_force, global_basis.xform(w->v_connect_point));
        //  w->apply_force(-global_suspension_force, w->global_basis.xform(w->w_connect_point));

        apply_force(global_suspension_force_old, global_basis.xform(w->v_connect_point));
        w->apply_force(-global_suspension_force_old, w->global_basis.xform(w->w_connect_point));
        w->springDebugRay.set_target_position(w->get_basis().xform(w->w_connect_point) + w->get_position() - w->v_connect_point);
        w->prev_compression = w->compression;
        GPRINT(is_transform_notification_enabled());
    }
    RID riddy = get_rid();
    PhysicsServer3D *physics_server = PhysicsServer3D::get_singleton();
    PhysicsDirectBodyState3D *state = physics_server->body_get_direct_state(riddy);
    //  GPRINTVEC(state->get_inverse_inertia().inverse());

    //   GPRINTVEC(get_inertia());
    //  GPRINTVEC(state->get_center_of_mass());
    // GPRINTVEC(state->get_center_of_mass_local());

    // GPRINTVEC(get_center_of_mass());

    // GPRINTVEC(get_inertia());
    //   GPRINTVEC(get_global_position());
}
// void _integrate_forces(PhysicsDirectBodyState3D *state)
//{
//     //    state->get_center_of_mass_local();
// }

PackedStringArray Wheel3D::_get_configuration_warnings() const
{
    PackedStringArray warnings = Node::_get_configuration_warnings();
    if (cast_to<Vehicle3D>(get_parent()) == nullptr)
        warnings.push_back("Wheel3D must be a child of Vehicle3D to function as intended.");
    return warnings;
}
void Wheel3D::_input(const Ref<InputEvent> &event)
{
    const InputEventKey *key_event = Object::cast_to<const InputEventKey>(*event);
    if (key_event)
    {
        emit_custom_signal(String("_input: ") + key_event->get_key_label(), key_event->get_unicode());
    }
}