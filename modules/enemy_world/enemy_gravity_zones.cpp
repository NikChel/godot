#include "enemy_gravity_zones.h"

#include "scene/3d/physics/area_3d.h"
#include "scene/3d/physics/collision_shape_3d.h"
#include "scene/resources/3d/box_shape_3d.h"

void EnemyGravityZones::set(const Array &p_areas) {
	zones.clear();
	for (int i = 0; i < p_areas.size(); i++) {
		Area3D *a = Object::cast_to<Area3D>(p_areas[i].get_validated_object());
		if (a == nullptr) {
			continue;
		}
		Zone z;
		z.id = a->get_instance_id();
		z.priority = a->get_priority();
		bool first = true;
		for (int c = 0; c < a->get_child_count(); c++) {
			CollisionShape3D *cs = Object::cast_to<CollisionShape3D>(a->get_child(c));
			if (cs == nullptr || cs->is_disabled()) {
				continue;
			}
			Ref<BoxShape3D> bs = cs->get_shape();
			if (bs.is_null()) {
				continue;
			}
			const Vector3 e = bs->get_size() * 0.5f;
			const AABB own(-e, e * 2.0f);
			const Transform3D t = cs->get_transform();
			const AABB bb = t.xform(own);
			z.bounds = first ? bb : z.bounds.merge(bb);
			first = false;
			Box b;
			b.rot = !t.basis.is_equal_approx(Basis());
			b.box = b.rot ? own : bb;
			b.inv = b.rot ? t.affine_inverse() : Transform3D();
			z.boxes.push_back(b);
		}
		if (z.boxes.is_empty()) {
			continue;
		}
		uint32_t at = 0; // По убыванию priority, равные — по порядку списка
		while (at < zones.size() && zones[at].priority >= z.priority) {
			at++;
		}
		zones.insert(at, z);
	}
	refresh();
}

void EnemyGravityZones::refresh() {
	for (Zone &z : zones) {
		Area3D *a = Object::cast_to<Area3D>(ObjectDB::get_instance(z.id));
		z.valid = a != nullptr && a->is_inside_tree();
		if (!z.valid) {
			continue;
		}
		z.inv = a->get_global_transform().affine_inverse();
		z.accel = a->get_gravity_direction() * a->get_gravity();
	}
}

bool EnemyGravityZones::accel_at(const Vector3 &p_world, Vector3 &r_accel) const {
	for (const Zone &z : zones) {
		if (!z.valid) {
			continue;
		}
		const Vector3 l = z.inv.xform(p_world);
		if (!z.bounds.has_point(l)) {
			continue;
		}
		for (const Box &b : z.boxes) {
			if (b.box.has_point(b.rot ? b.inv.xform(l) : l)) {
				r_accel = z.accel;
				return true;
			}
		}
	}
	return false;
}
