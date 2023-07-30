#include "vehicle3d.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;
using Function = std::function<int(int, int)>;

void Wheel3D::_notification(int p_what)
{
    switch (p_what)
    {
    case NOTIFICATION_PARENTED:
    {
        Vehicle3D *cb = Object::cast_to<Vehicle3D>(get_parent());
        if (!cb)
            return;
        body = cb;
        local_xform = get_transform();
        cb->wheels.emplace(this);
        //  std::function<void(double)> wheel_physics_process = yes_body_physics_process;
        //     void (Wheel3D::*wheel_physics_process)(double){no_body_physics_process};

        wheel_physics_process = no_body_physics_process;
        //  wheel_physics_process = yes_body_physics_process;
        //        char *boo = std::type_info->typeid(wheel_offset).name();
    }
    break;

    case NOTIFICATION_UNPARENTED:
    {
        Vehicle3D *cb = Object::cast_to<Vehicle3D>(get_parent());
        if (!cb)
            return;
        //   std::function<void(double)> wheel_physics_process = no_body_physics_process;
        //     void (Wheel3D::*wheel_physics_process)(double){yes_body_physics_process};

        wheel_physics_process = yes_body_physics_process;

        //    wheel_physics_process = no_body_physics_process;
        cb->wheels.erase(this);
        body = nullptr;
    }
    break;
    }
}
void Wheel3D::_ready()
{
    SphereShape3D dummyShape = SphereShape3D();
    dummyShape.set_radius(hub_radius);
    dummy_collider.set_shape(cast_to<Shape3D>(&dummyShape));
    dummy_collider.set_disabled(false);
    add_child(&dummy_collider);

    set_inertia(0.4 * get_mass() * tire_radius * tire_radius * ones);
    _update_axle_axis();
    _update_steering_axis();
    _update_connect_points();
    SphereShape3D castShape = SphereShape3D();
    castShape.set_radius(tire_radius);
    tire_cast.set_shape(cast_to<Shape3D>(&castShape));
    tire_cast.set_target_position(zeros);

    //   springDebug.set_rotation(suspension.get_rotation());
    // springDebug.set_position(Vector3(0, spring_length / 2, 0));

    // springDebugRay.set_position(suspension.get_position());

    //    springDebugRay.set_rotation(Vector3(0, 0, -half_pi));

    // set_position(axle_axis * wheel_offset + spring_length * steering_axis);
    if (body != nullptr)
    {
        if (bump_curve.is_valid())
            bump_function = yes_damp_factor;
        if (rebound_curve.is_valid())
            rebound_function = yes_damp_factor;
        bump_rel = bump * spring_length;
        rebound_rel = rebound * spring_length;
        stiff_rel = spring_stiffness * spring_length;
        add_child(&suspension);
        suspension.add_child(&springDebugRay);
        //   suspension.add_child(&springDebug);

        suspension.set_param(SliderJoint3D::Param::PARAM_LINEAR_LIMIT_UPPER, spring_length);
        suspension.set_param(SliderJoint3D::Param::PARAM_LINEAR_LIMIT_LOWER, 0.0);

        suspension.set_rotation(Vector3(caster_angle, 0, half_pi + side_sign * steering_axis_inclination));
        suspension.set_position(-wheel_offset * axle_axis);
        suspension.set_visible(true);
        suspension.set_exclude_nodes_from_collision(true);
        springDebugRay.set_target_position(right_axis_reference * spring_length);

        //   springDebugRay.set_rotation(Vector3(caster_angle, 0, -half_pi + side_sign * steering_axis_inclination));
        springDebugMesh.set_radius(0.05);
        springDebugMesh.set_height(0.1);
        springDebug.set_mesh(cast_to<Mesh>(&springDebugMesh));
        add_child(&springDebug);
        suspension.set_node_a(body->get_path());
        suspension.set_node_b(this->get_path());
    }
    else
        set_sleeping(true);
    relaxed_position = get_position();
    add_child(&tire_cast);
    side_sign = side - 1;
}

// void yes_body_physics_process(double delta, Wheel3D &w)
//{
// }
/* void yes_body_physics_process(double delta, Wheel3D &w)
{
    // using namespace godot;
    if (w.tire_cast.is_colliding())
    {
        Vector3 collisionPointGlobal = w.tire_cast.get_collision_point(0);
        Vector3 collisionNormal = w.tire_cast.get_collision_normal(0);
        real_d tireSqueeze = 1.0 - w.tire_radius / collisionPointGlobal.distance_to(w.get_global_position());
        //     apply_central_force(-20.0 * collisionNormal * tireSqueeze);
    }
    real_d compression = w.relaxed_position.distance_to(w.get_position()) / w.spring_length;

    // cout << relaxed_position.x << ", " << relaxed_position.y << ", " << relaxed_position.z << "\n";
    // real_d max_force = spring_length * spring_stiffness;

    w.spring_force = w.stiff_rel * compression;
    //   real_d compress_delta = compression - prev_compression;
    real_d compress_velocity = (compression - w.prev_compression) / delta;

    if (compress_velocity > 0)
        w.damp_force = bump_function(w.bump_rel, w.bump_curve, compress_velocity);
    else
        w.damp_force = rebound_function(w.rebound_rel, w.rebound_curve, compress_velocity);

    //  damp_force = (*bump_function)(rebound_rel, rebound_curve, compress_velocity);
    Vector3 suspensionForceVec = w.steering_axis * (w.spring_force + w.damp_force);
    Transform3D body_transform = w.body->get_transform().get_basis();
    Basis body_basis = body_transform.get_basis();
    Vector3 global_body_connect_point = w.to_global(w.body_connect_point);
    Vector3 global_body_to_connection = global_body_connect_point - w.body->get_global_position();

    w.body->apply_force(body_basis.xform(suspensionForceVec), global_body_to_connection);
    //
    w.apply_force(-suspensionForceVec, w.get_global_transform().get_basis().xform(w.wheel_connect_point));

    w.prev_compression = compression;
} */
/* bool Wheel3D::yes_body_physics_process(double delta)
{
    using namespace godot;
    if (tire_cast.is_colliding())
    {
        Vector3 collisionPointGlobal = tire_cast.get_collision_point(0);
        Vector3 collisionNormal = tire_cast.get_collision_normal(0);
        real_d tireSqueeze = 1.0 - tire_radius / collisionPointGlobal.distance_to(get_global_position());
        //     apply_central_force(-20.0 * collisionNormal * tireSqueeze);
    }
    real_d compression = relaxed_position.distance_to(get_position()) / spring_length;

    // cout << relaxed_position.x << ", " << relaxed_position.y << ", " << relaxed_position.z << "\n";
    // real_d max_force = spring_length * spring_stiffness;

    spring_force = stiff_rel * compression;
    //   real_d compress_delta = compression - prev_compression;
    real_d compress_velocity = (compression - prev_compression) / delta;
    if (compress_velocity > 0)
        damp_force = bump_function(bump_rel, bump_curve, compress_velocity);
    else
        damp_force = rebound_function(rebound_rel, rebound_curve, compress_velocity);
    //  damp_force = 1.0;
    //   damp_force = (*bump_function)(rebound_rel, rebound_curve, compress_velocity);
    Vector3 suspensionForceVec = steering_axis * (spring_force + damp_force);
    Transform3D body_transform = body->get_transform().get_basis();
    Basis body_basis = body_transform.get_basis();
    Vector3 global_body_connect_point = to_global(body_connect_point);
    Vector3 global_body_to_connection = global_body_connect_point - body->get_global_position();

    body->apply_force(body_basis.xform(suspensionForceVec), global_body_to_connection);
    //
    apply_force(-suspensionForceVec, get_global_transform().get_basis().xform(wheel_connect_point));

    prev_compression = compression;
    return true;
} */
void Wheel3D::_physics_process(double delta)
{
    /*     wheel_physics_process();
        using namespace godot;
        if (tire_cast.is_colliding())
        {
            Vector3 collisionPointGlobal = tire_cast.get_collision_point(0);
            Vector3 collisionNormal = tire_cast.get_collision_normal(0);
            real_d tireSqueeze = 1.0 - tire_radius / collisionPointGlobal.distance_to(get_global_position());
            //     apply_central_force(-20.0 * collisionNormal * tireSqueeze);
        }
        real_d compression = relaxed_position.distance_to(get_position()) / spring_length;

        // cout << relaxed_position.x << ", " << relaxed_position.y << ", " << relaxed_position.z << "\n";
        // real_d max_force = spring_length * spring_stiffness;

        spring_force = stiff_rel * compression;
        //   real_d compress_delta = compression - prev_compression;
        real_d compress_velocity = (compression - prev_compression) / delta;
        if (compress_velocity > 0)
            damp_force = bump_function(bump_rel, bump_curve, compress_velocity);
        else
            damp_force = rebound_function(rebound_rel, rebound_curve, compress_velocity);
        //  damp_force = 1.0;
        //   damp_force = (*bump_function)(rebound_rel, rebound_curve, compress_velocity);
        Vector3 suspensionForceVec = steering_axis * (spring_force + damp_force);
        Transform3D body_transform = body->get_transform().get_basis();
        Basis body_basis = body_transform.get_basis();
        Vector3 global_body_connect_point = to_global(body_connect_point);
        Vector3 global_body_to_connection = global_body_connect_point - body->get_global_position();

        body->apply_force(body_basis.xform(suspensionForceVec), global_body_to_connection);
        //
        apply_force(-suspensionForceVec, get_global_transform().get_basis().xform(wheel_connect_point));

        prev_compression = compression; */
}
void Vehicle3D::_physics_process(double delta)
{
    //   for (int w_it = 0; w_it < wheels.size(); w_it++)
    //   {
    //       wheels[w_it].add_child();
    //   }
    Transform3D global_transform = get_global_transform();
    Basis global_basis = global_transform.get_basis();
    Vector3 global_position = global_transform.get_origin();
    for (Wheel3D *w : wheels)
    {
        //   wheel_physics_process();
        //   using namespace godot;
        if (w->tire_cast.is_colliding())
        {
            Vector3 collisionPointGlobal = w->tire_cast.get_collision_point(0);
            Vector3 collisionNormal = w->tire_cast.get_collision_normal(0);
            real_d tireSqueeze = 1.0 - w->tire_radius / collisionPointGlobal.distance_to(w->get_global_position());
            //     apply_central_force(-20.0 * collisionNormal * tireSqueeze);
        }
        real_d compression = w->relaxed_position.distance_to(w->get_position()) / w->spring_length;

        // cout << relaxed_position.x << ", " << relaxed_position.y << ", " << relaxed_position.z << "\n";
        // real_d max_force = spring_length * spring_stiffness;

        w->spring_force = w->stiff_rel * compression;
        //   real_d compress_delta = compression - prev_compression;
        real_d compress_velocity = (compression - w->prev_compression) / delta;
        if (compress_velocity > 0)
            w->damp_force = bump_function(w->bump_rel, w->bump_curve, compress_velocity);
        else
            w->damp_force = rebound_function(w->rebound_rel, w->rebound_curve, compress_velocity);
        //  damp_force = 1.0;
        //   damp_force = (*bump_function)(rebound_rel, rebound_curve, compress_velocity);
        Vector3 suspensionForceVec = w->steering_axis * (w->spring_force + w->damp_force);
        // Transform3D body_transform = get_transform().get_basis();
        //  Basis body_basis = body_transform.get_basis();

        Vector3 global_body_connect_point = w->to_global(w->body_connect_point);
        Vector3 global_body_to_connection = global_body_connect_point - global_position;

        apply_force(local_basis.xform(suspensionForceVec), global_body_to_connection);
        w->apply_force(-suspensionForceVec, global_basis.xform(w->wheel_connect_point));

        w->prev_compression = compression;
    }
}
PackedStringArray Wheel3D::_get_configuration_warnings() const
{
    PackedStringArray warnings = Node::_get_configuration_warnings();
    if (Object::cast_to<Vehicle3D>(get_parent()) == nullptr)
        warnings.push_back("Wheel3D must be a child of Vehicle3D to function as intended.");
    return warnings;
}

void Vehicle3D::_ready()
{
    local_xform = get_transform();
    local_basis = local_xform.get_basis();
    local_origin = local_xform.get_origin();
    // for (int w_it = 0; w_it < wheels.size(); w_it++)
    // {
    //     wheels[w_it].add_child();
    // }
    //  for (Wheel3D *wheel : wheels)
    //  {
    //      add_sibling();
    //  }
}