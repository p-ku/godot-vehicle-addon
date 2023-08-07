#ifndef VEHICLE3D_HPP
#define VEHICLE3D_HPP

#include "macros.hpp"

#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/capsule_mesh.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/curve.hpp>
#include <godot_cpp/classes/cylinder_shape3d.hpp>
#include <godot_cpp/classes/global_constants.hpp>
#include <godot_cpp/classes/input_event_key.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/ray_cast3d.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/shape_cast3d.hpp>
#include <godot_cpp/classes/slider_joint3d.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/templates/vector.hpp>

using namespace godot;

namespace {
static inline const Vector3 ZEROS = Vector3(0, 0, 0);
static inline const Vector3 UP_AXIS_REFERENCE = Vector3(0, 1, 0);
static inline const Vector3 FORWARD_AXIS_REFERENCE = Vector3(0, 0, -1);
static inline const Vector3 RIGHT_AXIS_REFERENCE = Vector3(1, 0, 0);
static inline constexpr double HALF_PI = Math_PI / 2.0;

double yes_curve(double yMax, double xCoord, Ref<Curve> curve) {
	return yMax * xCoord * (curve->sample_baked(xCoord));
}

double no_curve(double yMax, double xCoord, Ref<Curve> curve) {
	return yMax * xCoord;
}

auto bump_function = no_curve;
auto rebound_function = no_curve;
} // namespace

class Vehicle3D;

class Wheel3D : public RigidBody3D {
	GDCLASS(Wheel3D, RigidBody3D);
	friend class Vehicle3D;

public:
	enum Sides {
		LEFT,
		CENTER,
		RIGHT
	};

private:
	Vehicle3D* body = nullptr;
	NodePath opposite_wheel;
	Wheel3D* sib;
	Transform3D relaxed_transform;
	Sides side = Sides::LEFT;
	int side_sign = side - 1;
	Transform3D wheelTransform = get_transform();
	// Transform3D centeredTransform = get_transform();
	Vector3 centered_rotation = ZEROS;
	// node3d
	Transform3D globalTransform;
	Basis globalBasis;
	Vector3 globalOrigin;
	Vector3 local_origin;
	// dimensions
	double tire_radius = 0.5;
	double tire_width = 0.3;
	double hub_radius = tire_width / 2.0;
	double corner_distance;
	double corner_angle;
	// suspension
	SliderJoint3D suspension;
	double suspension_travel = 1.0;
	double camber_angle = 0.0;
	double toe_angle = 0.0;
	double steering_axis_inclination = 0.0;
	double caster_angle = 0.0;
	double wheel_offset = 0.0;
	double signed_wheel_offset = 0.0;
	double signed_camber_angle = 0.0;
	double signed_toe_angle = 0.0;
	double signed_steering = 0.0;
	double signed_steering_axis_inclination = 0.0;
	Vector3 steering_axis = UP_AXIS_REFERENCE;
	Vector3 axle_axis = side_sign * RIGHT_AXIS_REFERENCE;
	Vector3 w_connect_point = ZEROS;
	Vector3 v_connect_point = UP_AXIS_REFERENCE;
	Vector3 local_w_connect_point = ZEROS;
	Vector3 local_v_connect_point = UP_AXIS_REFERENCE;
	Vector3 relaxed_wheel_connect_point = ZEROS;

	// spring
	double spring_stiffness = 20.0;
	double stiff_rel = spring_stiffness * suspension_travel;
	double anti_roll = 1.0;
	double anti_roll_rel = anti_roll * suspension_travel;
	// damp
	double bump = 1.0;
	double rebound = 1.0;
	Ref<Curve> bump_curve;
	Ref<Curve> rebound_curve;
	double bump_rel = bump * suspension_travel;
	double rebound_rel = rebound * suspension_travel;
	// steering
	bool steered = false;
	double steering = 0.0;
	double ackermann_input = 0.0;
	double max_ackermann = 0.57;
	double ackermann_factor = 0.0;
	double ackermann_steering = 0.0;
	// physics loop
	double compression = 0.0;
	double prev_compression = 0.0;
	double spring_force;
	double damp_force;
	// applied forces
	bool powered = true;
	double engine_torque = 0.0;
	double brake_torque = 0.0;
	// children
	ShapeCast3D tire_cast;
	SphereShape3D hub_shape;
	CollisionShape3D hub_collider;
	CylinderShape3D tire_cast_shape;
	// debug
	CapsuleMesh springDebugMeshA;
	CapsuleMesh springDebugMeshB;
	BoxMesh connectDebugMeshA;
	BoxMesh connectDebugMeshB;
	StandardMaterial3D matA;
	StandardMaterial3D matB;
	MeshInstance3D springDebugA;
	MeshInstance3D springDebugB;
	MeshInstance3D connectDebugA;
	MeshInstance3D connectDebugB;
	RayCast3D springDebugRay;

	Vector3 w_to_v;

	RayCast3D ray_center;
	RayCast3D ray_front_left;
	RayCast3D ray_front_right;
	RayCast3D ray_back_left;
	RayCast3D ray_back_right;
	void _on_wheel_rotated();
	void _on_centered_wheel_rotated();
	void _on_suspension_rotated();
	void _on_steering_changed();
	void _on_wheel_dimensions_changed();
	// void _on_curve_changed(Ref<Curve> curve);
	template<typename Function>
	void _curve_temp(Ref<Curve> curve, Function& func);
	void _update_steering_axis();
	void _update_vehicle_connect_point();
	void _update_suspension_transform();
	// void _update_suspension_length();

public:
	void _enter_tree() override;
	void _exit_tree() override;
	void _ready() override;

	//	void _notification(int p_what);

	NodePath get_opposite_wheel() const;
	void set_opposite_wheel(const NodePath& p_node);
	double get_ackermann_input() const;
	void set_ackermann_input(double p_factor);
	double get_steering() const;
	void set_steering(double p_angle);
	double get_engine_torque() const;
	void set_engine_torque(double p_torque);
	double get_brake_torque() const;
	void set_brake_torque(double p_torque);
	double get_tire_radius() const;
	void set_tire_radius(double p_radius);
	double get_width() const;
	void set_width(double p_width);
	double get_hub_radius() const;
	void set_hub_radius(double p_radius);
	double get_suspension_travel() const;
	void set_suspension_travel(double p_length);
	double get_spring_stiffness() const;
	void set_spring_stiffness(double p_value);
	double get_spring_anti_roll() const;
	void set_spring_anti_roll(double p_value);
	double get_damping_bump() const;
	void set_damping_bump(double p_value);
	Ref<Curve> get_damping_bump_curve() const;
	void set_damping_bump_curve(Ref<Curve> p_curve);
	double get_damping_rebound() const;
	void set_damping_rebound(double p_value);
	Ref<Curve> get_damping_rebound_curve() const;
	void set_damping_rebound_curve(Ref<Curve> p_value);
	Sides get_side() const;
	void set_side(Sides p_value);
	bool get_powered() const;
	void set_powered(bool p_value);
	bool get_steered() const;
	void set_steered(bool p_value);
	double get_alignment_steering_axis_inclination() const;
	void set_alignment_steering_axis_inclination(double p_angle);
	double get_alignment_camber_angle() const;
	void set_alignment_camber_angle(double p_angle);
	double get_alignment_toe_angle() const;
	void set_alignment_toe_angle(double p_angle);
	double get_alignment_caster_angle() const;
	void set_alignment_caster_angle(double p_angle);
	double get_alignment_wheel_offset() const;
	void set_alignment_wheel_offset(double p_value);
	double get_ackermann_steering() const;
	Vector3 get_steering_axis() const;
	Vector3 get_axle_axis() const;
	double get_spring_force() const;
	PackedStringArray _get_configuration_warnings() const override;
	Wheel3D();
	~Wheel3D();
	virtual void _input(const Ref<InputEvent>& event) override;
	void emit_custom_signal(const String& name, int value);

protected:
	static void _bind_methods();
	void _validate_property(PropertyInfo& p_property) const;
	void _get_property_list(List<PropertyInfo>* p_list) const;
	bool _get(const StringName& p_name, Variant& r_ret) const;
};

VARIANT_ENUM_CAST(Wheel3D::Sides);

class Vehicle3D : public RigidBody3D {
	GDCLASS(Vehicle3D, RigidBody3D);

	friend class Wheel3D;

private:
	// Transform3D v_globalTransform;
	// Transform3D v_local_xform;
	// Basis v_local_basis;
	// Vector3 v_local_origin;
	Vector<Wheel3D*> wheels;
	Transform3D globalTransform;
	Basis globalBasis;
	Vector3 globalOrigin;

protected:
	static void _bind_methods() { }

public:
	// std::vector<Wheel3D *> get_wheels() const { return wheels; }
	//	void _enter_tree() override;
	//	void _exit_tree() override;
	void _ready() override;
	void _physics_process(double delta) override;

	// void _integrate_forces(PhysicsDirectBodyState3D *state) override;

	// void _notification(int p_what);

	Vehicle3D() { }

	~Vehicle3D() { queue_free(); }
};

#endif // VEHICLE3D_HPP