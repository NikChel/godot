#include "enemy_tumbler.h"

#include "core/math/math_funcs.h"
#include "core/string/print_string.h"
#include "scene/3d/physics/collision_object_3d.h"
#include "servers/physics_3d/physics_server_3d.h"

static const Vector3 UP_AXIS = Vector3(0.0f, 1.0f, 0.0f);

void EnemyTumbler::reset() {
	state = STAND;
	tilt = Vector3();
	spin = Vector3();
	t = 0.0f;
	impact = false;
}

// Импульс j в точке r от опоры (кадр основания) — в любом состоянии. Горизонтальная часть момента — наклон;
// вертикальная (кручение) у заваленной — поворот наклона по кругу вокруг опоры: инерция вокруг вертикали ~ I · sin²θ,
// не меньше I · YAW_MIN_SIN2 (у стоящей кручения нет).
void EnemyTumbler::push(const Vector3 &p_r, const Vector3 &p_j) {
	const real_t inv = 1.0f / MAX(inertia, 1e-3f);
	const Vector3 tau = p_r.cross(p_j);
	spin += Vector3(tau.x, 0.0f, tau.z) * inv;
	const real_t a = tilt.length();
	if (a > UP_EPS) {
		const real_t s = Math::sin(a);
		const real_t wy = tau.y * inv / MAX(s * s, YAW_MIN_SIN2);
		spin += UP_AXIS.cross(tilt) * wy;
	}
}

bool EnemyTumbler::step(real_t p_dt) {
	const uint8_t was = state;
	real_t a = tilt.length();
	const Vector3 axis = a > UP_EPS ? tilt / a : Vector3();
	Vector3 acc;
	switch (state) {
		case STAND: {
			const real_t w0 = (real_t)Math::TAU * frequency;
			acc = -w0 * w0 * tilt - 2.0f * damping * w0 * spin;
		} break;
		case FALLING:
		case DOWN: {
			acc = axis * _gravity(a);
		} break;
		case RISING: {
			const real_t wr = RISE_SETTLE / MAX(rise_time, 0.05f);
			acc = axis * _gravity(a) - wr * wr * tilt - 2.0f * wr * spin;
		} break;
	}
	spin += acc * p_dt;
	tilt += spin * p_dt;
	const bool on_floor = _floor(p_dt);
	a = tilt.length();
	switch (state) {
		case STAND: {
			if (a > fall_angle) {
				state = FALLING;
			}
		} break;
		case FALLING: {
			if (on_floor) {
				_lie();
			} else if (a < fall_angle && spin.dot(tilt) < 0.0f) {
				state = STAND;
			}
		} break;
		case DOWN: {
			if (on_floor) {
				t += p_dt;
			}
			if (t >= down_time) {
				state = RISING;
			}
		} break;
		case RISING: {
			if (impact) { // Сбит до пола (лёжа у пола в начале подъёма — не удар)
				_lie();
			} else if (a < STAND_ANGLE) {
				state = STAND;
			}
		} break;
	}
	return state != was;
}

Basis EnemyTumbler::basis() const {
	const real_t a = tilt.length();
	return a > UP_EPS ? Basis(tilt / a, a) : Basis();
}

void EnemyTumbler::place(const Transform3D &p_base) const {
	CollisionObject3D *hit = Object::cast_to<CollisionObject3D>(ObjectDB::get_instance(hit_id));
	if (hit == nullptr || !hit->is_inside_tree()) {
		return;
	}
	const Transform3D pose = p_base * Transform3D(basis(), Vector3());
	hit->set_global_transform(pose);
	PhysicsServer3D::get_singleton()->body_set_state(hit->get_rid(), PhysicsServer3D::BODY_STATE_TRANSFORM, pose); // Сразу этому тику
}

void EnemyTumbler::log_state() const {
	static const char *NAMES[] = { "STAND", "FALLING", "DOWN", "RISING" };
	Node *hit = Object::cast_to<Node>(ObjectDB::get_instance(hit_id));
	Node *owner = hit != nullptr ? hit->get_parent() : nullptr;
	print_line(vformat("[tumbler] %s %s tilt=%.1f", owner != nullptr ? String(owner->get_name()) : String("?"), NAMES[state],
			Math::rad_to_deg(tilt.length())));
}

// Угловое ускорение тяжести от вертикали при наклоне a, рад/с².
real_t EnemyTumbler::_gravity(real_t p_a) const {
	return 1.5f * G / MAX(height, 0.1f) * Math::sin(p_a);
}

// Пол: наклон не больше down_angle, ход к полу гасится (удар о пол — impact), вдоль пола — трение. true — лежит на полу.
bool EnemyTumbler::_floor(real_t p_dt) {
	impact = false;
	const real_t a = tilt.length();
	if (a < down_angle - FLOOR_EPS) {
		return false;
	}
	const Vector3 axis = tilt / a;
	if (a > down_angle) {
		tilt = axis * down_angle;
	}
	const real_t radial = spin.dot(axis);
	impact = radial > 0.0f;
	const Vector3 along = spin - axis * radial;
	spin = along * Math::exp(-FLOOR_FRICTION * p_dt) + axis * MIN(radial, 0.0f);
	return true;
}

void EnemyTumbler::_lie() {
	state = DOWN;
	t = 0.0f;
}
