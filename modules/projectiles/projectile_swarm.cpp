#include "projectile_swarm.h"

#include "core/object/class_db.h"

ProjectileSwarm::ProjectileSwarm() {
	contact_steps.push_back(0.01);
	contact_steps.push_back(0.04);
}

ProjectileSwarm::~ProjectileSwarm() {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	if (ps == nullptr) {
		return;
	}
	for (const KeyValue<real_t, RID> &kv : spheres) {
		ps->free_rid(kv.value);
	}
}

RID ProjectileSwarm::_sphere(real_t p_radius) {
	const RID *found = spheres.getptr(p_radius);
	if (found) {
		return *found;
	}
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	const RID rid = ps->sphere_shape_create();
	ps->shape_set_data(rid, p_radius); // Как SphereShape3D::_update_shape
	spheres.insert(p_radius, rid);
	return rid;
}

int ProjectileSwarm::_find(int64_t p_key) const {
	for (uint32_t i = 0; i < shots.size(); i++) {
		if (shots[i].key == p_key) {
			return (int)i;
		}
	}
	return -1;
}

void ProjectileSwarm::setup(int p_mask, const PackedFloat64Array &p_contact_steps, int p_max_live, double p_keep_s) {
	mask = (uint32_t)p_mask;
	contact_steps.clear();
	for (int i = 0; i < p_contact_steps.size(); i++) {
		contact_steps.push_back((real_t)p_contact_steps[i]);
	}
	max_live = MAX(1, p_max_live);
	keep_s = p_keep_s;
}

int64_t ProjectileSwarm::add(bool p_auth, int64_t p_born_tick, double p_t0_local, const Vector3 &p_p0,
		const Vector3 &p_v0, double p_gravity, real_t p_radius, double p_life, const TypedArray<RID> &p_exclude,
		double p_wake_extra, int p_view_type) {
	if ((int)shots.size() >= max_live) {
		removed_ev.push_back(shots[0].key);
		shots.remove_at(0);
	}
	Shot s;
	s.key = next_key++;
	s.auth = p_auth;
	s.born_tick = p_born_tick;
	s.t0_local = p_t0_local;
	s.p0 = p_p0;
	s.v0 = p_v0;
	s.gravity = p_gravity;
	s.radius = p_radius;
	s.life = p_life;
	s.wake_extra = p_wake_extra;
	s.view_type = p_view_type;
	s.shown = p_p0;
	s.q.shape_rid = _sphere(p_radius);
	s.q.collision_mask = mask;
	for (int i = 0; i < p_exclude.size(); i++) {
		const RID rid = p_exclude[i];
		s.q.exclude.insert(rid);
	}
	shots.push_back(s);
	return s.key;
}

void ProjectileSwarm::stop(int64_t p_key, double p_age, const Vector3 &p_point, const Vector3 &p_normal) {
	const int i = _find(p_key);
	if (i < 0) {
		return;
	}
	Shot &s = shots[i];
	if (s.auth || p_age > s.stop_age) {
		return; // Свой решает сам; позже своей остановки — не берётся
	}
	s.stop_age = p_age;
	s.swept_age = MIN(s.swept_age, p_age);
	s.has_impact = true;
	s.hit_point = p_point;
	s.hit_normal = p_normal;
	stopped_ev.push_back(s.key);
}

void ProjectileSwarm::step(const Ref<World3D> &p_world, int64_t p_tick, int p_tps, double p_net_now) {
	ERR_FAIL_COND(p_world.is_null());
	PhysicsDirectSpaceState3D *space = p_world->get_direct_space_state();
	ERR_FAIL_NULL(space);
	const double tps = (double)MAX(1, p_tps);
	uint32_t i = 0;
	while (i < shots.size()) {
		Shot &s = shots[i];
		const double age = s.auth ? double(p_tick - s.born_tick) / tps : p_net_now - s.t0_local;
		if (!s.stopped()) {
			_advance(space, s, MIN(age, s.life));
			if (!s.stopped() && age >= s.life) {
				s.stop_age = s.life;
				stopped_ev.push_back(s.key);
			}
		}
		if (s.stopped() && age - s.stop_age > keep_s) {
			removed_ev.push_back(s.key);
			shots.remove_at(i);
			continue;
		}
		i++;
	}
}

void ProjectileSwarm::_advance(PhysicsDirectSpaceState3D *p_space, Shot &p_s, double p_age) {
	if (p_age <= p_s.swept_age) {
		return;
	}
	const double a0 = p_s.swept_age;
	const Vector3 from = p_s.pos_at(a0);
	const Vector3 motion = p_s.pos_at(p_age) - from;
	p_s.q.transform = Transform3D(Basis(), from);
	p_s.q.motion = motion;
	real_t safe = 1.0;
	real_t unsafe = 1.0;
	if (!p_space->cast_motion(p_s.q, safe, unsafe) || unsafe >= 1.0) {
		p_s.swept_age = p_age;
		return;
	}
	const double age_hit = a0 + (p_age - a0) * (double)unsafe;
	p_s.swept_age = age_hit;
	p_s.stop_age = age_hit;
	p_s.has_impact = true;
	const Vector3 at = from + motion * unsafe;
	const Vector3 dir = motion.normalized();
	PhysicsDirectSpaceState3D::ShapeRestInfo sri;
	const bool touched = _contact(p_space, p_s, at, dir, sri);
	p_s.hit_normal = touched ? sri.normal : -dir;
	// Точка касания — на поверхности под центром сферы: point из rest_info в форке — центр формы минус глубина
	// проникновения (внутри сферы), берётся только нормаль.
	p_s.hit_point = at - p_s.hit_normal * p_s.radius;
	p_s.info = Dictionary();
	if (p_s.auth && touched) {
		Dictionary d;
		d["point"] = sri.point;
		d["normal"] = sri.normal;
		d["rid"] = sri.rid;
		d["collider_id"] = sri.collider_id;
		d["shape"] = sri.shape;
		d["linear_velocity"] = sri.linear_velocity;
		p_s.info = d;
	}
	stopped_ev.push_back(p_s.key);
}

// Коллайдер в точке касания: форма сдвигается чуть за касание по ходу (касание без проникновения overlap не видит).
bool ProjectileSwarm::_contact(PhysicsDirectSpaceState3D *p_space, Shot &p_s, const Vector3 &p_at, const Vector3 &p_dir,
		PhysicsDirectSpaceState3D::ShapeRestInfo &r_info) {
	p_s.q.motion = Vector3();
	for (const real_t step : contact_steps) {
		p_s.q.transform = Transform3D(Basis(), p_at + p_dir * step);
		if (p_space->rest_info(p_s.q, &r_info)) {
			return true;
		}
	}
	return false;
}

PackedInt64Array ProjectileSwarm::events_stopped() {
	PackedInt64Array out = stopped_ev;
	stopped_ev.clear();
	return out;
}

PackedInt64Array ProjectileSwarm::events_removed() {
	PackedInt64Array out = removed_ev;
	removed_ev.clear();
	return out;
}

Dictionary ProjectileSwarm::state(int64_t p_key) const {
	Dictionary d;
	const int i = _find(p_key);
	if (i < 0) {
		return d;
	}
	const Shot &s = shots[i];
	d["swept_age"] = s.swept_age;
	d["stop_age"] = s.stop_age;
	d["has_impact"] = s.has_impact;
	d["hit_point"] = s.hit_point;
	d["hit_normal"] = s.hit_normal;
	d["info"] = s.info;
	return d;
}

// Точки интереса сна врагов (EnemyWakePoints): свои летящие — позиция на swept_age, радиус — путь за lead плюс
// размах хит-форм shot_r плюс сплеш рецепта.
PackedVector4Array ProjectileSwarm::append_wake(const PackedVector4Array &p_points, double p_lead, double p_shot_r) const {
	PackedVector4Array out = p_points;
	for (const Shot &s : shots) {
		if (!s.auth || s.stopped()) {
			continue;
		}
		const Vector3 at = s.pos_at(s.swept_age);
		const double r = (double)s.vel_at(s.swept_age).length() * p_lead + p_shot_r + s.wake_extra;
		out.push_back(Vector4(at.x, at.y, at.z, (real_t)r));
	}
	return out;
}

void ProjectileSwarm::_bind_methods() {
	ClassDB::bind_method(D_METHOD("setup", "mask", "contact_steps", "max_live", "keep_s"), &ProjectileSwarm::setup);
	ClassDB::bind_method(D_METHOD("add", "auth", "born_tick", "t0_local", "p0", "v0", "gravity", "radius", "life",
								 "exclude", "wake_extra", "view_type"),
			&ProjectileSwarm::add);
	ClassDB::bind_method(D_METHOD("stop", "key", "age", "point", "normal"), &ProjectileSwarm::stop);
	ClassDB::bind_method(D_METHOD("step", "world", "tick", "tps", "net_now"), &ProjectileSwarm::step);
	ClassDB::bind_method(D_METHOD("events_stopped"), &ProjectileSwarm::events_stopped);
	ClassDB::bind_method(D_METHOD("events_removed"), &ProjectileSwarm::events_removed);
	ClassDB::bind_method(D_METHOD("state", "key"), &ProjectileSwarm::state);
	ClassDB::bind_method(D_METHOD("append_wake", "points", "lead", "shot_r"), &ProjectileSwarm::append_wake);
	ClassDB::bind_method(D_METHOD("count"), &ProjectileSwarm::count);
}
