#include "enemy_wake.h"

#include "core/templates/list.h"
#include "scene/3d/physics/collision_object_3d.h"
#include "servers/physics_3d/physics_server_3d.h"

bool EnemyWake::update(bool p_interest, bool p_moving, real_t p_dt, const ObjectID *p_ids, int p_count) {
	if (p_interest || p_moving) {
		if (state == SLEEP) {
			_set_shapes(p_ids, p_count, true);
		}
		state = AWAKE;
		linger = LINGER_S;
		return true;
	}
	if (state == SLEEP) {
		return false;
	}
	state = LINGER;
	linger -= p_dt;
	if (linger > 0.0f) {
		return true;
	}
	state = SLEEP;
	_set_shapes(p_ids, p_count, false);
	return false;
}

bool EnemyWake::interest(const Vector3 &p_at, const PackedVector4Array &p_points) {
	const Vector4 *pts = p_points.ptr();
	for (int i = 0; i < p_points.size(); i++) {
		const Vector4 &q = pts[i];
		const real_t dx = q.x - p_at.x;
		const real_t dy = q.y - p_at.y;
		const real_t dz = q.z - p_at.z;
		if (dx * dx + dy * dy + dz * dz < q.w * q.w) {
			return true;
		}
	}
	return false;
}

// Формы хит-тел — в сервере физики (узлы не трогаем); включение возвращает выключенность владельца формы из сцены.
void EnemyWake::_set_shapes(const ObjectID *p_ids, int p_count, bool p_enabled) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	for (int k = 0; k < p_count; k++) {
		CollisionObject3D *co = Object::cast_to<CollisionObject3D>(ObjectDB::get_instance(p_ids[k]));
		if (co == nullptr || !co->is_inside_tree()) {
			continue;
		}
		const RID rid = co->get_rid();
		List<uint32_t> owners;
		co->get_shape_owners(&owners);
		for (const uint32_t owner : owners) {
			const bool off = !p_enabled || co->is_shape_owner_disabled(owner);
			for (int i = 0; i < co->shape_owner_get_shape_count(owner); i++) {
				ps->body_set_shape_disabled(rid, co->shape_owner_get_shape_index(owner, i), off);
			}
		}
	}
}
