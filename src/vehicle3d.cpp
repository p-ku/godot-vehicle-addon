#include "vehicle3d.hpp"

#include <godot_cpp/classes/physics_direct_body_state3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// void Wheel3D::_on_wheel_rotated() {
//	set_rotation(centered_rotation);
//	rotate_object_local(steering_axis, ackermann_steering);
// }

// void Wheel3D::_on_wheel_rotated() {
//_update_steering_axis();
//_update_axle_axis();

//	centered_rotation = Vector3(0.0, signed_toe_angle, -signed_camber_angle);
//	centered_position = get_basis().xform(
//		local_w_connect_point -
//		local_w_connect_point.rotated(steering_axis, ackermann_steering -
// prev_ackermann_steering)
//	);-axle_axis * wheel_offset
//	centered_position = w_connect_point - Vector3(signed_wheel_offset, 0.0, 0.0);
// set_rotation(Vector3(0.0, signed_toe_angle + ackermann_steering, -signed_camber_angle));
// set_rotation(centered_rotation);
//	rotate_object_local(steering_axis, ackermann_steering);
//	_on_wheel_rotated();
//}

// void Wheel3D::_on_suspension_rotated() {
//	_update_steering_axis();
// }

void Wheel3D::_on_steering_changed() {
	ackermann_factor = max_ackermann * ackermann_input;
	ackermann_steering = Math::atan(
		Math::tan(steering) / (1.0 + side_sign * ackermann_factor * Math::tan(steering))
	);
	//	set_rotation(Vector3(0.0, signed_toe_angle + ackermann_steering, -signed_camber_angle));

	//	_on_wheel_rotated();
	//	set_position(
	//		v_connect_point +
	//		get_basis().xform((-steering_axis * suspension_travel + wheel_offset * axle_axis)
	//							  .rotated(steering_axis, ackermann_steering))
	//	);
	//	set_rotation(Vector3(0.0, signed_toe_angle + ackermann_steering, -signed_camber_angle));
	//	set_rotation(centered_rotation);
	//	rotate_object_local(steering_axis, ackermann_steering);
	// set_position(get_basis().xform(
	//	-local_w_connect_point +
	//	local_w_connect_point.rotated(steering_axis, -ackermann_steering +
	// prev_ackermann_steering)
	//));
	//	translate_object_local(
	//		-local_w_connect_point +
	//		local_w_connect_point.rotated(steering_axis, ackermann_steering -
	// prev_ackermann_steering)
	//	);
	// set_position(get_basis().xform(
	//	dynamic_local_w_connect_point -
	//	dynamic_local_w_connect_point.rotated(steering_axis, ackermann_steering)
	//));
	_update_axle_axis();
	_update_dynamic_connect_points();
	//	dynamic_local_w_connect_point = -wheel_offset * axle_axis;
	// dynamic_local_v_connect_point = local_w_connect_point +
	//	suspension_travel * steering_axis * uncompression;
	set_position(v_connect_point_max - steering_axis * uncompression + wheel_offset * axle_axis);
	// translate_object_local(
	//	-local_w_connect_point +
	//	local_w_connect_point.rotated(steering_axis, ackermann_steering - prev_ackermann_steering)
	//);

	//		set_position(centered_position);
	prev_ackermann_steering = ackermann_steering;
	// set_position(get_basis().xform(local_w_connect_point - local_w_connect_point));
	_update_connect_points();
	_update_suspension_transform();
	update_gizmos();
}

void Wheel3D::_on_wheel_dimensions_changed() {
	hub_shape.set_radius(hub_radius);
	// hub_shape.set_height(tire_width);
	// hub_radius = tire_width / 2.0;

	tire_cast_shape.set_radius(tire_radius);
	tire_cast_shape.set_height(tire_width);

	// tire_cast.set_shape(tire_cast_shape);
	corner_distance = Math::sqrt(tire_radius * tire_radius + 0.25 * tire_width * tire_width);
	corner_angle = HALF_PI - Math::atan(tire_radius / (0.5 * tire_width));
	update_gizmos();
}

template<typename Function>
void Wheel3D::_curve_temp(Ref<Curve>& curve, Function& func) {
	if (curve.is_valid()) {
		func = yes_curve;
	} else {
		func = no_curve;
	}
}

void Wheel3D::_update_debug_shapes() {
	suspension.set_position(local_w_connect_point);

	springDebugB.set_position(Vector3(0, 0, 0));
	axisDebug0.set_position(ZEROS);

	springDebugA.set_position(Vector3(suspension_travel, 0, 0));
	connectDebugA.set_position(local_w_connect_point + suspension_travel * steering_axis);
	connectDebugB.set_position(local_w_connect_point);
	axisDebugA.set_position(axle_axis);
	axisDebugS.set_position(steering_axis + local_w_connect_point);
	springDebugRay.set_position(local_w_connect_point + suspension_travel * steering_axis);
	//	springDebugRay2.set_position(ZEROS);
	//	springDebugRay2.set_target_position(axle_axis);
}

void Wheel3D::_add_debug_ray(const Vector3& position, const Vector3& target) {
	RayCast3D ray;
	ray.set_position(position);
	ray.set_target_position(target);
	add_child(&ray);
}

void Wheel3D::_update_steering_axis() {
	// steering_axis = Vector3(Math::tan(signed_inclination), 1.0, Math::tan(caster_angle))
	//					.normalized();
	steering_axis.x = Math::tan(signed_inclination);
	steering_axis.y = 1.0;
	steering_axis.z = Math::tan(caster_angle);
	steering_axis = steering_axis.normalized();
	_update_debug_shapes();
};

void Wheel3D::_update_axle_axis() {
	axle_axis.x = side_sign;
	axle_axis.y = -Math::tan(camber_angle);
	axle_axis.z = -Math::tan(toe_angle);
	axle_axis = axle_axis.normalized();
	_update_debug_shapes();
}

void Wheel3D::_update_suspension_transform() {
	suspension.set_rotation(Vector3(0, 0, HALF_PI));

	suspension.set_quaternion(Quaternion(RIGHT_AXIS, steering_axis).normalized());
	_update_debug_shapes();
	//	suspension.look_at(get_global_transform().get_basis().xform(steering_axis),
	//-UP_AXIS);
	//	suspension.rotate(UP_AXIS, HALF_PI);
	//	suspension.rotate(FRONT_AXIS_REFERENCE, signed_inclination);
	//	suspension.rotate(RIGHT_AXIS, caster_angle);
}

void Wheel3D::_update_connect_points() {
	//	local_v_connect_point_max = local_w_connect_point + suspension_travel * steering_axis;
	local_w_connect_point = -wheel_offset * axle_axis;
	local_v_connect_point_max = local_w_connect_point + suspension_travel * steering_axis;

	// local_v_connect_point_max = get_position() +
	v_connect_point_max = get_basis().xform(local_v_connect_point_max) + get_position();
	w_connect_point = get_basis().xform(local_w_connect_point) + get_position();
	// relaxed_wheel_connect_point = get_basis().xform(local_w_connect_point);
	// relaxed_wheel_connect_point = get_basis().xform(local_w_connect_point);
	_update_debug_shapes();
}

void Wheel3D::_update_dynamic_connect_points() {
	dynamic_local_w_connect_point = -wheel_offset * axle_axis;
	dynamic_local_v_connect_point = local_w_connect_point +
		suspension_travel * steering_axis * uncompression;
	_update_debug_shapes();
}

// void Wheel3D::_update_suspension_length() {
//	bump_rel = bump * suspension_travel;
//	rebound_rel = rebound * suspension_travel;
//	stiff_rel = spring_stiffness * suspension_travel;
// }

void Wheel3D::_enter_tree() {
	suspension.set_node_b(get_path());
	Vehicle3D* cb = cast_to<Vehicle3D>(get_parent());
	if (!cb) {
		return;
	}
	body = cb;
	cb->wheels.push_back(this);
	suspension.set_node_a(body->get_path());
	suspension.set_node_a("../..");
}

void Wheel3D::_exit_tree() {
	suspension.set_node_a("");
	Vehicle3D* cb = cast_to<Vehicle3D>(get_parent());
	if (!cb) {
		return;
	}
	body = nullptr;
	cb->wheels.erase(this);
	sib = get_node<Wheel3D>(get_path());
}

void Wheel3D::_ready() {
	if (opposite_wheel.is_empty()) {
		sib = get_node<Wheel3D>(get_path());
	} else {
		sib = get_node<Wheel3D>(opposite_wheel);
	}
	add_child(&tire_cast);
	add_child(&hub_collider);
	add_child(&suspension);
	add_child(&axisDebug0);
	add_child(&axisDebugS);
	_add_debug_ray(ZEROS, axle_axis);

	_update_connect_points();
	suspension.add_child(&springDebugA);
	suspension.add_child(&springDebugB);

	relaxed_transform = get_transform();
	// relaxed_position = get_position();
}

// void Wheel3D::_notification(int p_what) {
//	if (p_what == NOTIFICATION_LOCAL_TRANSFORM_CHANGED || NOTIFICATION_PARENTED) {
//		//	_update_connect_points();
//		//	Vector3 wheel_rot = get_rotation();
//		//	signed_toe_angle = wheel_rot.y;
//		//	toe_angle = signed_toe_angle * side_sign;
//		//	signed_camber_angle = -wheel_rot.z;
//		//	camber_angle = signed_camber_angle * side_sign;
//		// set_rotation(Vector3(0.0, wheel_rot.y, wheel_rot.z));
//		//	_on_connect_point_changed();
//		//	relaxed_transform = get_transform();
//	}
// }

void Vehicle3D::_ready() {
	for (Wheel3D* w : wheels) {
		add_child(&w->connectDebugA);
		add_child(&w->connectDebugB);
		add_child(&w->springDebugRay);

		// add_child(&w->suspension);
		w->connectDebugA.set_position(get_basis().xform(w->local_v_connect_point_max));
		w->connectDebugB.set_position(get_basis().xform(w->local_w_connect_point));
		w->springDebugRay.set_position(get_basis().xform(w->local_v_connect_point_max));

		// w->suspension.set_node_a(get_path());
	}
}

void Vehicle3D::_physics_process(double delta) {
	globalTransform = get_global_transform();
	globalBasis = globalTransform.get_basis();
	globalOrigin = globalTransform.get_origin();
	for (Wheel3D* w : wheels) {
		w->globalTransform = w->get_global_transform();
		w->globalBasis = w->globalTransform.get_basis();
		w->globalOrigin = w->globalTransform.get_origin();
		w->local_origin = w->get_position();
		// w->tireRay.set_target_position(w->get_transform().get_basis().xform(Vector3(0,
		// -w->tire_radius, 0)));
		// w->tireRay.set_target_position(Vector3(0, -w->tire_radius, 0));

		if (w->tire_cast.is_colliding()) {
			Vector3 collisionPointGlobal = w->tire_cast.get_collision_point(0);
			Vector3 collisionNormal = w->tire_cast.get_collision_normal(0);
			Plane collisionPlane = Plane(collisionNormal);
			double collision_angle = w->axle_axis.angle_to(collisionNormal);

			// Vector3 ray_place = collisionNormal.rotated();

			double d_max = w->corner_distance * Math::cos(w->corner_angle + collision_angle);
			// w->springDebug.set_global_position(collisionPointGlobal);
			double chord_distance = collisionPointGlobal.distance_to(w->globalOrigin);
			double chord1_length = 2.0 * Math::sqrt(w->tire_radius * w->tire_radius);
			// double tireSqueeze = collisionPointGlobal.distance_to(w_globalOrigin);
			double tireSqueeze = 1.0 -
				collisionPointGlobal.distance_to(globalOrigin) / w->tire_radius;
			// double tireSqueeze = 1.0 -
			// w->tireRay.get_collision_point().distance_to(w_globalOrigin) / w->tire_radius;

			// apply_central_force(-20.0 * collisionNormal * tireSqueeze);
			//   w->apply_force(5000.0 * collisionNormal * tireSqueeze * tireSqueeze,
			//   collisionPointGlobal); w->apply_central_force(220632.0 * Vector3(0, 1, 0) /
			//   tireSqueeze * tireSqueeze);
		}
		// w->compression = w->relaxed_position.distance_to(w_local_origin) /
		// w->suspension_travel; double precomp =
		// w->to_global(w->local_w_connect_point).distance_to(to_global(w->v_connect_point)) /
		// w->suspension_travel;
		w->w_to_v = to_global(w->v_connect_point_max) - w->to_global(w->local_w_connect_point);
		//	w->w_to_v = w->local_v_connect_point_max - w->local_w_connect_point;
		//	w->w_to_v = w->local_v_connect_point_max - w->local_w_connect_point;

		// double v_to_w_distance = w->v_to_w.length();

		// w->compression = 1.0 -
		// w->to_global(w->local_w_connect_point).distance_to(to_global(w->v_connect_point)) /
		//                            w->suspension_travel;
		Vector3 wheel_connect_p = w->get_basis().xform(w->local_w_connect_point);
		w->uncompression = w->w_to_v.length() / w->suspension_travel;

		w->compression = 1.0 - w->uncompression;
		//	w->compression = 1.0 -
		//		(wheel_connect_p - w->relaxed_wheel_connect_point).length() /
		// w->suspension_travel;

		// w->w_to_v =
		// (w->to_global(w->local_w_connect_point)).direction_to(to_global(w->v_connect_point));
	}
	for (Wheel3D* w : wheels) {
		// double compression = 0.0;
		// cout << relaxed_position.x << ", " << relaxed_position.y << ", " <<
		// relaxed_position.z <<
		// "\n"; double max_force = suspension_travel * spring_stiffness;
		double opposite_diff = w->compression - w->sib->compression;
		w->spring_force = w->stiff_rel * w->compression;
		double compress_velocity = (w->compression - w->prev_compression) / delta;
		if (compress_velocity > 0) {
			w->damp_force = bump_function(w->bump_rel, compress_velocity, w->bump_curve);
		} else {
			w->damp_force = rebound_function(w->rebound_rel, compress_velocity, w->rebound_curve);
		}

		double anti_roll_force = w->anti_roll * opposite_diff;
		Vector3 global_steering_axis = w->globalBasis.xform(w->steering_axis);
		Vector3 w_local_suspension_force = w->steering_axis *
			(w->spring_force + w->damp_force + anti_roll_force);
		Vector3 global_suspension_force_old = w->globalBasis.xform(w_local_suspension_force);
		global_suspension_force_old = global_steering_axis *
			(w->spring_force + w->damp_force + anti_roll_force);
		// double steer_to_actual = global_steering_axis.angle_to(w->w_to_v);
		//   global_suspension_force_old *= Math::cos(steer_to_actual);
		// Vector3 w_local_suspension_force = w->get_basis().xform(local_suspension_force);

		// Vector3 v_local_suspension_force = w->get_basis().xform(w_local_suspension_force);
		// v_local_suspension_force = Vector3(0, 0, 0);
		// Vector3 w_global_suspension_force = w_globalBasis.xform(w_local_suspension_force);
		// Vector3 v_global_suspension_force = v_globalBasis.xform(v_local_suspension_force);
		// Vector3 global_suspension_force = w->globalBasis.xform(w_local_suspension_force);
		Vector3 global_suspension_force = w->w_to_v.normalized() *
			(w->spring_force); //- w->damp_force +
							   // anti_roll_force);

		// GPRINTVEC(w->globalBasis.xform_inv(w->w_to_v.normalized()));

		// Vector3 global_body_to_connection = v_globalBasis.xform(w->v_connect_point);
		// Vector3 global_body_connect_point = to_global(w->v_connect_point);

		// Vector3 global_body_to_connection = global_body_connect_point - v_globalOrigin;

		// apply_force(global_suspension_force, globalBasis.xform(w->v_connect_point));
		// w->apply_force(-global_suspension_force,
		// w->globalBasis.xform(w->local_w_connect_point));

		apply_force(global_suspension_force_old, globalBasis.xform(w->v_connect_point_max));
		w->apply_force(
			-global_suspension_force_old,
			w->globalBasis.xform(w->local_w_connect_point)
		);
		w->springDebugRay.set_target_position(
			w->get_basis().xform(w->local_w_connect_point) + w->get_position() -
			w->v_connect_point_max
		);
		w->prev_compression = w->compression;
		w->prev_uncompression = w->uncompression;
	}

	RID riddy = get_rid();
	PhysicsServer3D* physics_server = PhysicsServer3D::get_singleton();
	PhysicsDirectBodyState3D* state = physics_server->body_get_direct_state(riddy);
}

PackedStringArray Wheel3D::_get_configuration_warnings() const {
	PackedStringArray warnings = Node::_get_configuration_warnings();
	if (cast_to<Vehicle3D>(get_parent()) == nullptr) {
		warnings.push_back("Wheel3D must be a child of Vehicle3D to function as intended.");
	}
	return warnings;
}

void Wheel3D::_input(const Ref<InputEvent>& event) {
	const InputEventKey* key_event = Object::cast_to<const InputEventKey>(*event);
	if (key_event) {
		emit_custom_signal(
			String("_input: ") + key_event->get_key_label(),
			key_event->get_unicode()
		);
	}
}

NodePath Wheel3D::get_opposite_wheel() const {
	return opposite_wheel;
}

void Wheel3D::set_opposite_wheel(const NodePath& p_node) {
	opposite_wheel = p_node;
	// sib->opposite_wheel = this->get_path();

	// Vehicle3D *cb = Object::cast_to<Vehicle3D>(get_parent());
	//        Vehicle3D *cb = Object::cast_to<Vehicle3D>(get_parent());

	// sib = Object::cast_to<Wheel3D>(get_node<Wheel3D>(opposite_wheel));
	//   sib = get_node<Wheel3D>(opposite_wheel);
	//     sib->set_opposite_wheel(this->get_path());
	//      sib->set_spring_anti_roll(anti_roll);
}

double Wheel3D::get_ackermann_input() const {
	return ackermann_input;
}

void Wheel3D::set_ackermann_input(const double& p_factor) {
	ackermann_input = p_factor;
	_on_steering_changed();
}

double Wheel3D::get_steering() const {
	return steering;
}

void Wheel3D::set_steering(const double& p_angle) {
	steering = p_angle;
	_on_steering_changed();
}

double Wheel3D::get_engine_torque() const {
	return engine_torque;
}

void Wheel3D::set_engine_torque(const double& p_torque) {
	engine_torque = p_torque;
}

double Wheel3D::get_brake_torque() const {
	return brake_torque;
}

void Wheel3D::set_brake_torque(const double& p_torque) {
	brake_torque = p_torque;
}

double Wheel3D::get_tire_radius() const {
	return tire_radius;
}

void Wheel3D::set_tire_radius(const double& p_radius) {
	tire_radius = p_radius;
	if (tire_radius < hub_radius) {
		hub_radius = tire_radius;
	}

	_on_wheel_dimensions_changed();
}

double Wheel3D::get_width() const {
	return tire_width;
}

void Wheel3D::set_width(const double& p_width) {
	tire_width = p_width;
	_on_wheel_dimensions_changed();
}

double Wheel3D::get_hub_radius() const {
	return hub_radius;
}

void Wheel3D::set_hub_radius(const double& p_radius) {
	hub_radius = p_radius;
	if (tire_radius < hub_radius) {
		tire_radius = hub_radius;
	}
	_on_wheel_dimensions_changed();
}

double Wheel3D::get_suspension_travel() const {
	return suspension_travel;
}

void Wheel3D::set_suspension_travel(const double& p_length) {
	suspension_travel = p_length;
	_update_connect_points();

	bump_rel = bump * suspension_travel;
	rebound_rel = rebound * suspension_travel;
	stiff_rel = spring_stiffness * suspension_travel;
	anti_roll_rel = anti_roll * suspension_travel;
	suspension.set_param(SliderJoint3D::Param::PARAM_LINEAR_LIMIT_UPPER, suspension_travel);

	update_gizmos();
}

double Wheel3D::get_spring_stiffness() const {
	return spring_stiffness;
}

void Wheel3D::set_spring_stiffness(const double& p_value) {
	spring_stiffness = p_value;
	stiff_rel = spring_stiffness * suspension_travel;
}

double Wheel3D::get_spring_anti_roll() const {
	return anti_roll;
}

void Wheel3D::set_spring_anti_roll(const double& p_value) {
	anti_roll = p_value;
	anti_roll_rel = anti_roll * suspension_travel;
}

double Wheel3D::get_damping_bump() const {
	return bump;
}

void Wheel3D::set_damping_bump(const double& p_value) {
	bump = p_value;
	bump_rel = bump * suspension_travel;
}

Ref<Curve> Wheel3D::get_damping_bump_curve() const {
	return bump_curve;
}

void Wheel3D::set_damping_bump_curve(const Ref<Curve>& p_curve) {
	bump_curve = p_curve;
	_curve_temp(bump_curve, bump_function);
}

double Wheel3D::get_damping_rebound() const {
	return rebound;
}

void Wheel3D::set_damping_rebound(const double& p_value) {
	rebound = p_value;
	rebound_rel = rebound * suspension_travel;
}

Ref<Curve> Wheel3D::get_damping_rebound_curve() const {
	return rebound_curve;
}

void Wheel3D::set_damping_rebound_curve(const Ref<Curve>& p_curve) {
	rebound_curve = p_curve;
	_curve_temp(rebound_curve, rebound_function);
}

Wheel3D::Sides Wheel3D::get_side() const {
	return side;
}

void Wheel3D::set_side(const Sides p_value) {
	// side_sign = side - 1;
	side = p_value;
	side_sign = side - 1;
	// signed_wheel_offset = -side_sign * wheel_offset;
	signed_camber_angle = side_sign * camber_angle;
	signed_toe_angle = side_sign * toe_angle;
	signed_inclination = -side_sign * inclination;
	// axle_axis_reference = side_sign * RIGHT_AXIS;
	// _update_axle_axis();
	axle_axis_reference = RIGHT_AXIS * side_sign;
	// _update_steering_axis();
	// set_position(body_connect_point - steering_axis * suspension_travel - axle_axis *
	// wheel_offset);
	_update_steering_axis();
	_update_axle_axis();
	_update_suspension_transform();
	_update_connect_points();
	update_gizmos();
}

bool Wheel3D::get_powered() const {
	return powered;
}

void Wheel3D::set_powered(const bool& p_value) {
	powered = p_value;
	update_gizmos();
}

bool Wheel3D::get_steered() const {
	return steered;
}

void Wheel3D::set_steered(const bool& p_value) {
	steered = p_value;
}

// bool Wheel3D::get_relative_transform() const {
//	return relative_transform;
// }
//
// void Wheel3D::set_relative_transform(bool p_value) {
//	relative_transform = p_value;
//	notify_property_list_changed();
// }

double Wheel3D::get_alignment_steering_axis_inclination() const {
	return inclination;
}

void Wheel3D::set_alignment_steering_axis_inclination(const double& p_angle) {
	inclination = p_angle;
	signed_inclination = -side_sign * inclination;
	_update_steering_axis();
	_update_axle_axis();
	_update_suspension_transform();
	_update_connect_points();
	// set_position(w_connect_point + axle_axis * wheel_offset);
	update_gizmos();
}

double Wheel3D::get_alignment_caster_angle() const {
	return caster_angle;
}

void Wheel3D::set_alignment_caster_angle(const double& p_angle) {
	caster_angle = p_angle;
	_update_steering_axis();
	_update_axle_axis();
	_update_suspension_transform();
	_update_connect_points();
	//	set_position(w_connect_point + axle_axis * wheel_offset);
	update_gizmos();
}

double Wheel3D::get_alignment_camber_angle() const {
	return camber_angle;
}

void Wheel3D::set_alignment_camber_angle(const double& p_angle) {
	camber_angle = p_angle;
	signed_camber_angle = side_sign * camber_angle;
	//	_on_centered_wheel_rotated();
	_update_axle_axis();
	_update_connect_points();
	update_gizmos();
}

double Wheel3D::get_alignment_toe_angle() const {
	return toe_angle;
}

void Wheel3D::set_alignment_toe_angle(const double& p_angle) {
	toe_angle = p_angle;
	signed_toe_angle = side_sign * toe_angle;
	//	_on_centered_wheel_rotated();
	_update_axle_axis();
	_update_connect_points();
	update_gizmos();
}

double Wheel3D::get_alignment_wheel_offset() const {
	return wheel_offset;
}

void Wheel3D::set_alignment_wheel_offset(const double& p_value) {
	// translate(axle_axis * (p_value - wheel_offset));

	wheel_offset = p_value;
	// signed_wheel_offset = -side_sign * wheel_offset;
	//	local_w_connect_point = -axle_axis * wheel_offset;
	//	w_connect_point = get_basis().xform(local_w_connect_point);
	//  set_position(axle_axis * wheel_offset);
	//  set_position(v_connect_point_max - steering_axis * uncompression + wheel_offset *
	//  axle_axis);
	// set_position(w_connect_point + axle_axis * wheel_offset);
	//_update_connect_points();
	_update_suspension_transform();
	_update_connect_points();
	_update_debug_shapes();
	update_gizmos();
}

// Vector3 get_wheel_connection_point() const { return local_w_connect_point; }
// void set_wheel_connection_point(Vector3 p_value) { local_w_connect_point = p_value; }

// Vector3 get_vehicle_connection_point() const { return v_connect_point; }
// void set_vehicle_connection_point(Vector3 p_value) { v_connect_point = p_value; }
double Wheel3D::get_ackermann_steering() const {
	return ackermann_steering;
}

Vector3 Wheel3D::get_steering_axis() const {
	return steering_axis;
}

Vector3 Wheel3D::get_axle_axis() const {
	return axle_axis;
}

double Wheel3D::get_spring_force() const {
	return spring_force;
}

Wheel3D::Wheel3D() {
	_on_wheel_dimensions_changed();
	Vector3 wheel_child_rotation = Vector3(0, 0, HALF_PI);
	tire_cast.set_rotation(wheel_child_rotation);
	hub_collider.set_rotation(wheel_child_rotation);
	suspension.set_rotation(wheel_child_rotation);

	hub_collider.set_disabled(false);

	tire_cast.set_target_position(ZEROS);

	tire_cast.set_shape(cast_to<Shape3D>(&tire_cast_shape));
	hub_collider.set_shape(cast_to<Shape3D>(&hub_shape));

	suspension.set_visible(true);
	suspension.set_exclude_nodes_from_collision(true);
	suspension.set_param(SliderJoint3D::Param::PARAM_LINEAR_LIMIT_UPPER, suspension_travel);
	suspension.set_param(SliderJoint3D::Param::PARAM_LINEAR_LIMIT_LOWER, 0.0);
	tire_cast.set_rotation(wheel_child_rotation);
	hub_collider.set_rotation(wheel_child_rotation);
	suspension.set_rotation(wheel_child_rotation);
	_update_debug_shapes();

	genMat.set_albedo(Color(1, 1, 1));
	matA.set_albedo(Color(1, 1, 1));
	matB.set_albedo(Color(0, 0, 1));
	genericMesh.set_material(cast_to<Material>(&genMat));
	springDebugMeshA.set_material(cast_to<Material>(&matA));
	springDebugMeshB.set_material(cast_to<Material>(&matB));
	connectDebugMeshA.set_material(cast_to<Material>(&matA));
	connectDebugMeshB.set_material(cast_to<Material>(&matB));

	genericMesh.set_radius(0.05);
	genericMesh.set_height(0.1);

	springDebugMeshA.set_radius(0.05);
	springDebugMeshA.set_height(0.1);
	connectDebugMeshA.set_size(Vector3(0.1, 0.1, 0.1));

	springDebugMeshB.set_radius(0.05);
	springDebugMeshB.set_height(0.1);
	connectDebugMeshB.set_size(Vector3(0.1, 0.1, 0.1));

	axisDebug0.set_mesh(cast_to<Mesh>(&genericMesh));
	axisDebugS.set_mesh(cast_to<Mesh>(&genericMesh));
	axisDebugA.set_mesh(cast_to<Mesh>(&genericMesh));

	springDebugA.set_mesh(cast_to<Mesh>(&springDebugMeshA));
	springDebugB.set_mesh(cast_to<Mesh>(&springDebugMeshB));
	connectDebugA.set_mesh(cast_to<Mesh>(&connectDebugMeshA));
	connectDebugB.set_mesh(cast_to<Mesh>(&connectDebugMeshB));

	//	side_sign = side - 1;
}

Wheel3D::~Wheel3D() { }

void Wheel3D::emit_custom_signal(const String& name, int value) {
	emit_signal("custom_signal", name, value);
}

void Wheel3D::_bind_methods() {
	BIND_METHOD(Wheel3D, get_ackermann_input);
	BIND_METHOD(Wheel3D, set_ackermann_input, "ackermann_input");

	// BIND_METHOD(Wheel3D, get_relative_transform);
	// BIND_METHOD(Wheel3D, set_relative_transform, "rel_transform");

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

	// BIND_METHOD(Wheel3D, set_vehicle_connection_point, "vehicle_connect_point");
	// BIND_METHOD(Wheel3D, get_vehicle_connection_point);
	BIND_PROPERTY_RANGED("ackermann_input", Variant::FLOAT, "0.0,1,0.001");
	BIND_PROPERTY_HINTED(
		"opposite_wheel",
		Variant::NODE_PATH,
		PROPERTY_HINT_NODE_PATH_VALID_TYPES,
		"Wheel3D"
	);
	// ADD_PROPERTY(PropertyInfo(Variant::NIL, "tester", PROPERTY_HINT_NONE, "",
	// PROPERTY_USAGE_CATEGORY), "set_vehicle_connection_point",
	// "get_vehicle_connection_point");
	BIND_PROPERTY_RANGED("tire_radius", Variant::FLOAT, "0.001,2,0.001,suffix:m");
	BIND_PROPERTY_RANGED("width", Variant::FLOAT, "0.001,2,0.001,suffix:m");
	BIND_PROPERTY_RANGED("hub_radius", Variant::FLOAT, "0.001,2,0.001,suffix:m");
	BIND_PROPERTY_ENUM("side", "Left,Center,Right");
	// BIND_PROPERTY("relative_transform", Variant::BOOL);

	BIND_PROPERTY("powered", Variant::BOOL);
	BIND_PROPERTY("steered", Variant::BOOL);
	BIND_PROPERTY_RANGED("steering", Variant::FLOAT, "-60,60,0.01,radians");
	BIND_PROPERTY_RANGED(
		"engine_torque",
		Variant::FLOAT,
		u"0,1000,0.01,or_greater,suffix:kg\u22C5m/s\u00B2 (N)"
	);
	BIND_PROPERTY_RANGED(
		"brake_torque",
		Variant::FLOAT,
		u"0,1000,0.01,or_greater,suffix:kg\u22C5m/s\u00B2 (N)"
	);
	// BIND_PROPERTY_RANGED("vehicle_connection_point", Variant::VECTOR3,
	// "-99999,99999,0.001,or_greater,or_less,hide_slider,suffix:m");
	// BIND_PROPERTY_RANGED("wheel_connection_point", Variant::VECTOR3,
	// "-99999,99999,0.001,or_greater,or_less,hide_slider,suffix:m");

	BIND_METHOD(Wheel3D, set_alignment_wheel_offset, "alignment_wheel_offset");
	BIND_METHOD(Wheel3D, get_alignment_wheel_offset);

	BIND_METHOD(
		Wheel3D,
		set_alignment_steering_axis_inclination,
		"alignment_steering_axis_inclination"
	);
	BIND_METHOD(Wheel3D, get_alignment_steering_axis_inclination);

	BIND_METHOD(Wheel3D, set_alignment_caster_angle, "alignment_caster_angle");
	BIND_METHOD(Wheel3D, get_alignment_caster_angle);

	BIND_METHOD(Wheel3D, set_alignment_camber_angle, "alignment_camber_angle");
	BIND_METHOD(Wheel3D, get_alignment_camber_angle);

	BIND_METHOD(Wheel3D, set_alignment_toe_angle, "alignment_toe_angle");
	BIND_METHOD(Wheel3D, get_alignment_toe_angle);

	ADD_GROUP("Alignment", "alignment_");
	BIND_PROPERTY_RANGED("alignment_wheel_offset", Variant::FLOAT, "0.0,2,0.001,suffix:m");
	BIND_PROPERTY_RANGED(
		"alignment_steering_axis_inclination",
		Variant::FLOAT,
		"-60,60,0.01,radians"
	);
	BIND_PROPERTY_RANGED("alignment_caster_angle", Variant::FLOAT, "-60,60,0.01,radians");
	BIND_PROPERTY_RANGED("alignment_camber_angle", Variant::FLOAT, "-60,60,0.01,radians");
	BIND_PROPERTY_RANGED("alignment_toe_angle", Variant::FLOAT, "-60,60,0.01,radians");

	BIND_METHOD(Wheel3D, set_suspension_travel, "suspension_travel");
	BIND_METHOD(Wheel3D, get_suspension_travel);

	BIND_METHOD(Wheel3D, set_spring_stiffness, "spring_stiffness");
	BIND_METHOD(Wheel3D, get_spring_stiffness);

	BIND_METHOD(Wheel3D, set_spring_anti_roll, "spring_anti_roll");
	BIND_METHOD(Wheel3D, get_spring_anti_roll);

	ADD_GROUP("Springs", "spring_");
	BIND_PROPERTY_RANGED("suspension_travel", Variant::FLOAT, "0.001,10,0.001,suffix:m");
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
	BIND_PROPERTY_RANGED(
		"damping_bump",
		Variant::FLOAT,
		u"0,1000,0.01,or_greater,suffix:N\u22C5s/m"
	);
	BIND_PROPERTY_RANGED(
		"damping_rebound",
		Variant::FLOAT,
		u"0,1000,0.01,or_greater,suffix:N\u22C5s/m"
	);

	ADD_PROPERTY(
		PropertyInfo(Variant::OBJECT, "damping_bump_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"),
		"set_damping_bump_curve",
		"get_damping_bump_curve"
	);
	ADD_PROPERTY(
		PropertyInfo(
			Variant::OBJECT,
			"damping_rebound_curve",
			PROPERTY_HINT_RESOURCE_TYPE,
			"Curve"
		),
		"set_damping_rebound_curve",
		"get_damping_rebound_curve"
	);

	// BIND_METHOD(Wheel3D, get_wheel_connection_point);
	// BIND_METHOD(Wheel3D, set_wheel_connection_point, "local_w_connect_point");

	// BIND_METHOD(Wheel3D, get_vehicle_connection_point);
	// BIND_METHOD(Wheel3D, set_vehicle_connection_point, "v_connect_point");
	BIND_METHOD(Wheel3D, get_ackermann_steering);

	BIND_METHOD(Wheel3D, get_steering_axis);

	BIND_METHOD(Wheel3D, get_axle_axis);

	BIND_METHOD(Wheel3D, get_spring_force);

	BIND_ENUM_CONSTANT(LEFT);
	BIND_ENUM_CONSTANT(CENTER);
	BIND_ENUM_CONSTANT(RIGHT);
}

void Wheel3D::_validate_property(PropertyInfo& p_property) const {
	p_property.usage = PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_INTERNAL | PROPERTY_USAGE_EDITOR;
	//	if (p_property.name == StringName("rotation")) {
	//		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	//	}
	//	if (p_property.name == StringName("wheel_connection_point")) {
	//		if (relative_transform) {
	//			p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	//		} else {
	//			p_property.usage = PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_INTERNAL |
	//				PROPERTY_USAGE_EDITOR;
	//		}
	//	}
	//	if (p_property.name == StringName("vehicle_connection_point")) {
	//		if (relative_transform) {
	//			p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	//		} else {
	//			p_property.usage = PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_INTERNAL |
	//				PROPERTY_USAGE_EDITOR;
	//		}
	//	}
}

void Wheel3D::_get_property_list(List<PropertyInfo>* p_list) const {
	p_list->push_back(PropertyInfo(
		Variant::VECTOR3,
		"wheel_connection_point",
		PROPERTY_HINT_RANGE,
		"-99999,99999,0.001,or_greater,or_less,hide_slider,suffix:m"
	));
	p_list->push_back(PropertyInfo(
		Variant::VECTOR3,
		"vehicle_connection_point",
		PROPERTY_HINT_RANGE,
		"-99999,99999,0.001,or_greater,or_less,hide_slider,suffix:m"
	));

	for (PropertyInfo& E : *p_list) {
		_validate_property(E);
	}
}

bool Wheel3D::_get(const StringName& p_name, Variant& r_ret) const {
	String name = p_name;
	if (name == "wheel_connection_point") {
		r_ret = local_w_connect_point;
		return true;
	}
	if (name == "vehicle_connection_point") {
		r_ret = local_v_connect_point_max;
		return true;
	}
	return false;
}
