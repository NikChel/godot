#include "frame_spaces.h"

#include "core/string/print_string.h"
#include "scene/3d/node_3d.h"
#include "scene/3d/physics/area_3d.h"
#include "scene/3d/physics/collision_shape_3d.h"
#include "scene/resources/3d/box_shape_3d.h"
#include "servers/physics_3d/physics_server_3d.h"

// Системы этого тика (ReferenceFrame.spaces): поза запросов, шаг, зоны с коробками и тяжестью — по вызову на систему и
// на зону.
void FrameSpaces::sync(const Array &p_frames) {
	clear();
	for (int i = 0; i < p_frames.size(); i++) {
		Node3D *n = Object::cast_to<Node3D>(p_frames[i].get_validated_object());
		if (n == nullptr) {
			continue;
		}
		FrameTick f;
		f.id = n->get_instance_id();
		f.query = n->call(SNAME("query_xform"));
		f.step = n->call(SNAME("step_xform"));
		f.field = _field(n);
		const int fi = (int)frames.size();
		frames.push_back(f);
		const Array zs = n->get(SNAME("zones"));
		for (int k = 0; k < zs.size(); k++) {
			Node3D *z = Object::cast_to<Node3D>(zs[k].get_validated_object());
			if (z == nullptr) {
				continue;
			}
			ZoneTick zt;
			zt.id = z->get_instance_id();
			zt.frame = fi;
			zt.shape = _shape(z);
			const Transform3D zx = z->get_global_transform();
			zt.to_local = zx.affine_inverse();
			zt.accel = z->call(SNAME("accel"));
			const AABB &b = zt.shape->bounds;
			const real_t scale = MAX(zx.basis.get_scale_abs().x, MAX(zx.basis.get_scale_abs().y, zx.basis.get_scale_abs().z));
			const real_t r = b.size.length() * 0.5f * scale + NEAR_MARGIN;
			zt.near_c = zx.xform(b.get_center());
			zt.near_r2 = zt.shape->boxes.is_empty() ? -1.0f : r * r;
			zones.push_back(zt);
		}
	}
}

void FrameSpaces::clear() {
	frames.clear();
	zones.clear();
}

// Тяжесть зон системы — в позе этого commit (шаг системы этого тика).
void FrameSpaces::refresh_accel(int p_frame) {
	for (ZoneTick &z : zones) {
		if (z.frame == p_frame) {
			Object *o = ObjectDB::get_instance(z.id);
			if (o) {
				z.accel = o->call(SNAME("accel"));
			}
		}
	}
}

int FrameSpaces::frame(ObjectID p_id) const {
	for (uint32_t i = 0; i < frames.size(); i++) {
		if (frames[i].id == p_id) {
			return (int)i;
		}
	}
	return -1;
}

// Коробки зоны — как GravityZone._ready: BoxShape3D детей CollisionShape3D (включённых).
const FrameSpaces::ZoneShape *FrameSpaces::_shape(Node3D *p_zone) {
	const ObjectID id = p_zone->get_instance_id();
	const ZoneShape *have = shapes.getptr(id);
	if (have) {
		return have;
	}
	ZoneShape s;
	const Area3D *area = Object::cast_to<Area3D>(p_zone);
	s.priority = area ? area->get_priority() : 0;
	for (int i = 0; i < p_zone->get_child_count(); i++) {
		const CollisionShape3D *cs = Object::cast_to<CollisionShape3D>(p_zone->get_child(i));
		if (cs == nullptr || cs->is_disabled()) {
			continue;
		}
		const Ref<BoxShape3D> box = cs->get_shape();
		if (box.is_null()) {
			continue;
		}
		const Vector3 e = box->get_size() * 0.5;
		const AABB own(-e, e * 2.0);
		const Transform3D tf = cs->get_transform();
		const AABB bb = tf.xform(own);
		s.bounds = s.boxes.is_empty() ? bb : s.bounds.merge(bb);
		const bool rot = !tf.basis.is_equal_approx(Basis());
		s.boxes.push_back(rot ? own : bb);
		s.inv.push_back(rot ? tf.affine_inverse() : Transform3D());
		s.rot.push_back(rot ? 1 : 0);
	}
	return &shapes.insert(id, s)->value;
}

// Центр масс главного тела системы (первое кинематическое тело под ней — опора: корпус кабины, пол лифта) в её кадре;
// тела нет — начало системы.
Vector3 FrameSpaces::_field(Node3D *p_frame) {
	const ObjectID id = p_frame->get_instance_id();
	const Vector3 *have = fields.getptr(id);
	if (have) {
		return *have;
	}
	Vector3 c;
	String what = "origin";
	const TypedArray<Node> list = p_frame->find_children("*", "AnimatableBody3D", true, false);
	if (!list.is_empty()) {
		const CollisionObject3D *body = Object::cast_to<CollisionObject3D>(list[0]);
		if (body) {
			PhysicsDirectBodyState3D *st = PhysicsServer3D::get_singleton()->body_get_direct_state(body->get_rid());
			const Vector3 com = st ? st->get_center_of_mass_local() : Vector3();
			c = p_frame->get_global_transform().affine_inverse().xform(body->get_global_transform().xform(com));
			what = vformat("%s com_local=%s", body->get_name(), com);
		}
	}
	fields.insert(id, c);
	print_line(vformat("[frame_riders] field %s: %s -> frame %s", String(p_frame->get_path()), what, c));
	return c;
}

// Зона подвижной системы, в которой точка (мир, поза запросов этого тика): точка — шагом своей системы к позе commit,
// где уже стоят её зоны; выше priority — та. −1 — ни в одной.
int FrameSpaces::zone_at(const Vector3 &p_point) const {
	int best = -1;
	int fi = -1;
	Vector3 q;
	for (uint32_t i = 0; i < zones.size(); i++) {
		const ZoneTick &z = zones[i];
		if (z.frame != fi) {
			fi = z.frame;
			q = frames[fi].step.xform(p_point);
		}
		if ((best < 0 || z.shape->priority > zones[best].shape->priority) && _contains(z, q)) {
			best = (int)i;
		}
	}
	return best;
}

// Точка у какой-нибудь зоны систем (сфера её коробок + NEAR_MARGIN): дальше за тик в зону не попасть.
bool FrameSpaces::is_close(const Vector3 &p_point) const {
	for (const ZoneTick &z : zones) {
		if (p_point.distance_squared_to(z.near_c) < z.near_r2) {
			return true;
		}
	}
	return false;
}

bool FrameSpaces::_contains(const ZoneTick &p_z, const Vector3 &p_point) {
	const ZoneShape &s = *p_z.shape;
	const Vector3 l = p_z.to_local.xform(p_point);
	if (s.boxes.is_empty() || !s.bounds.has_point(l)) {
		return false;
	}
	for (uint32_t i = 0; i < s.boxes.size(); i++) {
		if (s.boxes[i].has_point(s.rot[i] ? s.inv[i].xform(l) : l)) {
			return true;
		}
	}
	return false;
}
