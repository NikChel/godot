#include "frame_carry.h"

#include "core/config/engine.h"
#include "core/math/math_funcs.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "core/string/print_string.h"
#include "scene/3d/physics/area_3d.h"
#include "scene/3d/physics/collision_shape_3d.h"
#include "scene/3d/physics/rigid_body_3d.h"
#include "scene/main/scene_tree.h"
#include "scene/resources/3d/box_shape_3d.h"
#include "servers/physics_3d/physics_server_3d.h"

RigidBody3D *FrameCarry::_body(ObjectID p_id) {
	return Object::cast_to<RigidBody3D>(ObjectDB::get_instance(p_id));
}

// Динамическое тело в дереве (кинематику — держимое, кобуру, зеркала сети — ведут её писатели).
bool FrameCarry::_dynamic(const RigidBody3D *p_rb) {
	if (p_rb == nullptr || !p_rb->is_inside_tree() || p_rb->is_freeze_enabled()) {
		return false;
	}
	const PhysicsServer3D::BodyMode m = PhysicsServer3D::get_singleton()->body_get_mode(p_rb->get_rid());
	return m == PhysicsServer3D::BODY_MODE_RIGID || m == PhysicsServer3D::BODY_MODE_RIGID_LINEAR;
}

int FrameCarry::_rider(ObjectID p_id) const {
	const uint32_t *i = rider_at.getptr(p_id);
	return i ? (int)*i : -1;
}

void FrameCarry::_erase(uint32_t p_i) {
	const ObjectID id = riders[p_i].body;
	const uint32_t last = riders.size() - 1;
	if (p_i != last) {
		riders[p_i] = riders[last];
		rider_at[riders[p_i].body] = p_i;
	}
	riders.resize(last);
	rider_at.erase(id);
}

int FrameCarry::_frame(ObjectID p_id) const {
	for (uint32_t i = 0; i < frames_t.size(); i++) {
		if (frames_t[i].id == p_id) {
			return (int)i;
		}
	}
	return -1;
}

// Системы этого тика (ReferenceFrame.spaces): поза запросов, шаг, зоны с коробками и тяжестью — по вызову на систему и
// на зону.
void FrameCarry::_sync(const Array &p_frames) {
	frames_t.clear();
	zones_t.clear();
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
		Callable *cb = step_cbs.getptr(f.id);
		if (cb == nullptr) {
			cb = &step_cbs.insert(f.id, callable_mp(this, &FrameCarry::_on_step).bind((int64_t)f.id))->value;
		}
		f.on_step = *cb;
		const int fi = (int)frames_t.size();
		frames_t.push_back(f);
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
			zones_t.push_back(zt);
		}
	}
}

// Коробки зоны — как GravityZone._ready: BoxShape3D детей CollisionShape3D (включённых).
const FrameCarry::ZoneShape *FrameCarry::_shape(Node3D *p_zone) {
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
Vector3 FrameCarry::_field(Node3D *p_frame) {
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
int FrameCarry::_zone_at(const Vector3 &p_point) const {
	int best = -1;
	int fi = -1;
	Vector3 q;
	for (uint32_t i = 0; i < zones_t.size(); i++) {
		const ZoneTick &z = zones_t[i];
		if (z.frame != fi) {
			fi = z.frame;
			q = frames_t[fi].step.xform(p_point);
		}
		if ((best < 0 || z.shape->priority > zones_t[best].shape->priority) && _contains(z, q)) {
			best = (int)i;
		}
	}
	return best;
}

// Точка у какой-нибудь зоны систем (сфера её коробок + NEAR_MARGIN): дальше за тик в зону не попасть.
bool FrameCarry::_near(const Vector3 &p_point) const {
	for (const ZoneTick &z : zones_t) {
		if (p_point.distance_squared_to(z.near_c) < z.near_r2) {
			return true;
		}
	}
	return false;
}

bool FrameCarry::_contains(const ZoneTick &p_z, const Vector3 &p_point) {
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

// Тело в зоне zi подвижной системы (−1 — вне пространств) на этом тике.
void FrameCarry::_place(RigidBody3D *p_rb, int p_zi, uint64_t p_t) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	const ObjectID id = p_rb->get_instance_id();
	int ri = _rider(id);
	const ObjectID fr = p_zi >= 0 ? frames_t[zones_t[p_zi].frame].id : ObjectID();
	if (ri >= 0 && riders[ri].frame != fr) {
		if (_dynamic(p_rb)) {
			const int fi = _frame(riders[ri].frame);
			if (fi >= 0) {
				_carry(riders[ri], frames_t[fi].step, frames_t[fi]); // вышло: шаг этого тика ещё с системой
			}
			riders[ri].carried = false;
		}
		_restore(riders[ri]);
		_erase(ri);
		ri = -1;
	}
	if (fr.is_null()) {
		if (_dynamic(p_rb)) {
			free_seen[id] = p_t;
		}
		return;
	}
	if (ri < 0) {
		Rider r;
		r.body = id;
		r.rid = p_rb->get_rid();
		r.frame = fr;
		PhysicsDirectBodyState3D *st = ps->body_get_direct_state(r.rid);
		r.com_l = st ? st->get_center_of_mass_local() : Vector3();
		const uint64_t *f = free_seen.getptr(id);
		r.entering = f != nullptr && *f + 1 == p_t; // вошло свободным телом корабля; иначе стало свободным внутри
		ri = (int)riders.size();
		riders.push_back(r);
		rider_at.insert(id, (uint32_t)ri);
	} else {
		riders[ri].entering = false;
	}
	free_seen.erase(id);
	Rider &r = riders[ri];
	r.tick = p_t;
	r.zone = zones_t[p_zi].id;
	r.zi = p_zi;
	r.written = false;
	r.v = ps->body_get_state(r.rid, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY);
	r.w = ps->body_get_state(r.rid, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY);
	_gravity(r, p_rb, zones_t[p_zi].accel);
	frames_t[zones_t[p_zi].frame].riders = true;
}

// Тяжесть едущего тела — к полу системы постоянной силой; тяжесть PhysX (мира и зон) снята на сервере. Запись — только
// при смене (смена силы будит тело: в стоящей системе спящее спит).
void FrameCarry::_gravity(const Rider &p_r, RigidBody3D *p_rb, const Vector3 &p_accel) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	if ((double)ps->body_get_param(p_r.rid, PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE) != 0.0) {
		ps->body_set_param(p_r.rid, PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE, 0.0);
	}
	const Vector3 f = p_accel * (p_rb->get_mass() * p_rb->get_gravity_scale()); // своих постоянных сил у предметов нет
	if (!ps->body_get_constant_force(p_r.rid).is_equal_approx(f)) {
		ps->body_set_constant_force(p_r.rid, f);
	}
}

// Тело больше не едет: тяжесть сервера — по узлу, постоянной силы нет.
void FrameCarry::_restore(const Rider &p_r) {
	RigidBody3D *rb = _body(p_r.body);
	if (rb == nullptr) {
		return;
	}
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	ps->body_set_param(p_r.rid, PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE, rb->get_gravity_scale());
	ps->body_set_constant_force(p_r.rid, Vector3());
}

// Скорость тела на шаг step системы: несение по полю главного тела системы + своя, повёрнутая шагом; поправка позы — на
// post. Вошедшее — без записи (скорость в мире та же), без поправки.
void FrameCarry::_carry(Rider &r_r, const Transform3D &p_step, const FrameTick &p_f) {
	if (p_step == Transform3D()) {
		if (r_r.written) {
			_write(r_r, r_r.v, r_r.w);
		}
		r_r.written = false;
		r_r.carried = false;
		r_r.u = Vector3();
		r_r.uw = Vector3();
		r_r.eps = Vector3();
		return;
	}
	RigidBody3D *rb = _body(r_r.body);
	if (rb == nullptr) {
		return;
	}
	const double hz = (double)Engine::get_singleton()->get_physics_ticks_per_second();
	const double dt = 1.0 / hz;
	const Vector3 c0 = p_f.query.xform(p_f.field); // центр масс главного тела в начале шага
	Quaternion q = p_step.basis.get_rotation_quaternion();
	if (q.w < 0.0) {
		q = -q; // короткий поворот — как PhysX у кинематики (ScKinematics)
	}
	// Ось — из вектора кватерниона: Quaternion::get_axis() у малого угла не нормирует ось
	const Vector3 qv(q.x, q.y, q.z);
	const double sn = qv.length();
	const double ang = 2.0 * Math::atan2(sn, (double)q.w);
	r_r.uw = sn > 1.0e-12 ? qv * (real_t)(ang * hz / sn) : Vector3();
	const Transform3D xf = rb->get_global_transform();
	const Vector3 p = xf.xform(r_r.com_l);
	r_r.u = (p_step.xform(c0) - c0) * (real_t)hz + r_r.uw.cross(p - c0);
	r_r.carried = true;
	if (r_r.entering) {
		r_r.eps = Vector3();
		return;
	}
	r_r.eps = p_step.xform(p) - p - r_r.u * (real_t)dt;
	const Basis rot(q);
	if (r_r.zi >= 0 && r_r.zi < (int)zones_t.size() && zones_t[r_r.zi].id == r_r.zone) {
		_gravity(r_r, rb, zones_t[r_r.zi].accel); // сила — к полу в позе commit (позже в тике — снова)
	}
	PhysicsDirectBodyState3D *st = PhysicsServer3D::get_singleton()->body_get_direct_state(r_r.rid);
	const double kl = st ? (double)st->get_total_linear_damp() : 0.0; // затухание, которое даст PhysX (зона + своё)
	const double ka = st ? (double)st->get_total_angular_damp() : 0.0;
	_write(r_r, r_r.u / (real_t)MAX(1.0 - kl * dt, DAMP_FLOOR) + rot.xform(r_r.v),
			r_r.uw / (real_t)MAX(1.0 - ka * dt, DAMP_FLOOR) + rot.xform(r_r.w));
	r_r.written = true;
}

void FrameCarry::_write(const Rider &p_r, const Vector3 &p_v, const Vector3 &p_w) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	ps->body_set_state(p_r.rid, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, p_v);
	ps->body_set_state(p_r.rid, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY, p_w);
}

// Шаг системы этого тика (ReferenceFrame.ride: сразу и на каждый commit этого тика) — всем её пассажирам этого тика;
// вне физ-тика (commit пробы между тиками — шаг уже прошёл) — не трогать.
void FrameCarry::_on_step(const Transform3D &p_step, int64_t p_frame) {
	const uint64_t t = Engine::get_singleton()->get_physics_frames();
	if (!Engine::get_singleton()->is_in_physics_frame() || t != pre_tick) {
		return;
	}
	const ObjectID fid = ObjectID((uint64_t)p_frame);
	const int fi = _frame(fid);
	if (fi < 0) {
		return;
	}
	for (ZoneTick &z : zones_t) { // тяжесть зон системы — в позе этого commit
		if (z.frame == fi) {
			Object *o = ObjectDB::get_instance(z.id);
			if (o) {
				z.accel = o->call(SNAME("accel"));
			}
		}
	}
	for (Rider &r : riders) {
		if (r.frame == fid && r.tick == t) {
			_carry(r, p_step, frames_t[fi]);
		}
	}
}

void FrameCarry::setup(const StringName &p_group) {
	group_name = p_group;
}

// Конец физ-тика (отложенный вызов после всех узлов): пространства тел, скорость шага системы — едущим.
void FrameCarry::pre(const Array &p_frames) {
	const uint64_t t = Engine::get_singleton()->get_physics_frames();
	pre_tick = t;
	seen = 0;
	if (p_frames.is_empty() && riders.is_empty()) {
		frames_t.clear();
		zones_t.clear();
		free_seen.clear();
		return;
	}
	_sync(p_frames);
	SceneTree *tree = SceneTree::get_singleton();
	if (tree && group_name != StringName()) {
		const Vector<Node *> items = tree->get_nodes_in_group(group_name);
		for (Node *n : items) {
			RigidBody3D *rb = Object::cast_to<RigidBody3D>(n);
			if (rb == nullptr) {
				continue;
			}
			seen++;
			const Vector3 at = rb->get_global_position();
			if (!_near(at)) { // вдали от систем: пассажир — выходит, остальные не числятся
				if (_rider(rb->get_instance_id()) >= 0) {
					_place(rb, -1, t);
				}
				continue;
			}
			_place(rb, _dynamic(rb) ? _zone_at(at) : -1, t);
		}
	}
	for (KeyValue<int64_t, Group> &kv : groups) {
		Group &g = kv.value;
		if (g.bodies.is_empty()) {
			continue;
		}
		seen++;
		const RigidBody3D *anchor = _body(g.bodies[0]);
		const bool in_reach = anchor && anchor->is_inside_tree() && _near(anchor->get_global_position());
		const int zi = in_reach ? _zone_at(anchor->get_global_position()) : -1;
		g.zone = zi >= 0 ? zones_t[zi].id : ObjectID();
		g.frame = zi >= 0 ? frames_t[zones_t[zi].frame].id : ObjectID();
		for (const ObjectID &b : g.bodies) {
			RigidBody3D *rb = _body(b);
			if (rb && (in_reach || _rider(b) >= 0)) {
				_place(rb, _dynamic(rb) ? zi : -1, t);
			}
		}
	}
	for (int i = (int)riders.size() - 1; i >= 0; i--) {
		if (riders[i].tick != t) {
			_restore(riders[i]);
			_erase((uint32_t)i);
		}
	}
	LocalVector<ObjectID> gone;
	for (const KeyValue<ObjectID, uint64_t> &kv : free_seen) {
		if (kv.value != t) {
			gone.push_back(kv.key);
		}
	}
	for (const ObjectID &id : gone) {
		free_seen.erase(id);
	}
	for (const FrameTick &f : frames_t) { // шаг системы — её пассажирам этого тика (ReferenceFrame.ride: сразу и на commit)
		if (f.riders) {
			Object *o = ObjectDB::get_instance(f.id);
			if (o) {
				o->call(SNAME("ride"), f.on_step);
			}
		}
	}
}

// Начало физ-тика (до узлов): несение прошлого шага — вычесть, поза — поправкой поля к шагу системы.
void FrameCarry::post() {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	for (Rider &r : riders) {
		if (!r.carried) {
			continue;
		}
		r.carried = false;
		RigidBody3D *rb = _body(r.body);
		if (!_dynamic(rb)) {
			continue;
		}
		if ((double)r.eps.length_squared() > EPS_MIN * EPS_MIN) {
			Transform3D xf = ps->body_get_state(r.rid, PhysicsServer3D::BODY_STATE_TRANSFORM);
			xf.origin += r.eps;
			ps->body_set_state(r.rid, PhysicsServer3D::BODY_STATE_TRANSFORM, xf);
			// Без эха: поза уже на сервере (set_ignore_transform_notification у Node3D защищён; флаг уведомления — то же
			// условие постановки в очередь, Node3D::_propagate_transform_changed)
			const bool notify = rb->is_transform_notification_enabled();
			rb->set_notify_transform(false);
			rb->set_global_transform(xf);
			rb->set_notify_transform(notify);
		}
		const Vector3 v = ps->body_get_state(r.rid, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY);
		const Vector3 w = ps->body_get_state(r.rid, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY);
		rb->set_linear_velocity(v - r.u);
		rb->set_angular_velocity(w - r.uw);
	}
}

// Группа тел (кукла): bodies — RigidBody3D, первое — якорь пространства группы.
void FrameCarry::add_group(int64_t p_id, const Array &p_bodies) {
	remove_group(p_id);
	Group g;
	for (int i = 0; i < p_bodies.size(); i++) {
		const Object *o = p_bodies[i].get_validated_object();
		if (o == nullptr) {
			continue;
		}
		g.bodies.push_back(o->get_instance_id());
		member_of.insert(o->get_instance_id(), p_id);
	}
	groups.insert(p_id, g);
}

void FrameCarry::remove_group(int64_t p_id) {
	Group *g = groups.getptr(p_id);
	if (g == nullptr) {
		return;
	}
	for (const ObjectID &b : g->bodies) {
		const int64_t *m = member_of.getptr(b);
		if (m && *m == p_id) {
			member_of.erase(b);
		}
	}
	groups.erase(p_id);
}

// Подвижная система тела (по последнему pre): пассажир — его система; тело группы — пространство группы (и
// кинематическое); иначе null (корабль).
Object *FrameCarry::frame_of(Object *p_body) const {
	if (p_body == nullptr) {
		return nullptr;
	}
	const ObjectID id = p_body->get_instance_id();
	const int ri = _rider(id);
	if (ri >= 0) {
		return ObjectDB::get_instance(riders[ri].frame);
	}
	const int64_t *m = member_of.getptr(id);
	if (m) {
		const Group *g = groups.getptr(*m);
		return g ? ObjectDB::get_instance(g->frame) : nullptr;
	}
	return nullptr;
}

// Зона подвижной системы тела (по последнему pre); null — вне пространств.
Object *FrameCarry::zone_of(Object *p_body) const {
	if (p_body == nullptr) {
		return nullptr;
	}
	const ObjectID id = p_body->get_instance_id();
	const int ri = _rider(id);
	if (ri >= 0) {
		return ObjectDB::get_instance(riders[ri].zone);
	}
	const int64_t *m = member_of.getptr(id);
	if (m) {
		const Group *g = groups.getptr(*m);
		return g ? ObjectDB::get_instance(g->zone) : nullptr;
	}
	return nullptr;
}

void FrameCarry::_bind_methods() {
	ClassDB::bind_method(D_METHOD("setup", "group"), &FrameCarry::setup);
	ClassDB::bind_method(D_METHOD("pre", "frames"), &FrameCarry::pre);
	ClassDB::bind_method(D_METHOD("post"), &FrameCarry::post);
	ClassDB::bind_method(D_METHOD("add_group", "id", "bodies"), &FrameCarry::add_group);
	ClassDB::bind_method(D_METHOD("remove_group", "id"), &FrameCarry::remove_group);
	ClassDB::bind_method(D_METHOD("frame_of", "body"), &FrameCarry::frame_of);
	ClassDB::bind_method(D_METHOD("zone_of", "body"), &FrameCarry::zone_of);
	ClassDB::bind_method(D_METHOD("count"), &FrameCarry::count);
	ClassDB::bind_method(D_METHOD("get_seen"), &FrameCarry::get_seen);
}
