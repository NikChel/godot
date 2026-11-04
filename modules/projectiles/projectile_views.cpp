#include "projectile_views.h"

#include "core/object/class_db.h"
#include "servers/rendering/rendering_server.h"

ProjectileViews::~ProjectileViews() {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}
	for (Type &t : types) {
		rs->free_rid(t.instance);
		rs->free_rid(t.multimesh);
	}
}

void ProjectileViews::setup(RID p_scenario) {
	scenario = p_scenario;
}

int ProjectileViews::type_add(RID p_mesh, real_t p_scale, const Color &p_tint) {
	RenderingServer *rs = RenderingServer::get_singleton();
	Type t;
	t.multimesh = rs->multimesh_create();
	rs->multimesh_set_mesh(t.multimesh, p_mesh);
	t.instance = rs->instance_create2(t.multimesh, scenario);
	rs->instance_geometry_set_cast_shadows_setting(t.instance, RSE::SHADOW_CASTING_SETTING_OFF);
	rs->instance_geometry_set_shader_parameter(t.instance, "tint", p_tint);
	t.scale = p_scale;
	types.push_back(t);
	return (int)types.size() - 1;
}

void ProjectileViews::set_lag_inputs(const PackedVector3Array &p_points, const PackedFloat64Array &p_lags, int p_local) {
	view_lag.set(p_points, p_lags, p_local);
}

double ProjectileViews::lag_at(const Vector3 &p_at) const {
	return view_lag.lag_at(p_at);
}

void ProjectileViews::_put(int p_type, const Vector3 &p_at, real_t p_scale) {
	if (p_type < 0 || p_type >= (int)types.size()) {
		return;
	}
	Type &t = types[p_type];
	const real_t s = p_scale * t.scale;
	if ((t.count + 1) * STRIDE > t.buf.size()) {
		// Хвост — нули: Vector::resize не заполняет float, а AABB MultiMesh считается по всему буферу.
		t.buf.resize_initialized(MAX(16, t.buf.size() / STRIDE * 2) * STRIDE);
	}
	float *w = t.buf.ptrw() + t.count * STRIDE;
	w[0] = s;
	w[1] = 0.0f;
	w[2] = 0.0f;
	w[3] = p_at.x;
	w[4] = 0.0f;
	w[5] = s;
	w[6] = 0.0f;
	w[7] = p_at.y;
	w[8] = 0.0f;
	w[9] = 0.0f;
	w[10] = s;
	w[11] = p_at.z;
	t.count++;
}

void ProjectileViews::_commit(Type &p_type) {
	RenderingServer *rs = RenderingServer::get_singleton();
	const int cap = p_type.buf.size() / STRIDE;
	if (cap != p_type.allocated) {
		rs->multimesh_allocate_data(p_type.multimesh, cap, RSE::MULTIMESH_TRANSFORM_3D);
		p_type.allocated = cap;
	}
	if (p_type.allocated == 0) {
		return;
	}
	if (p_type.count > 0) {
		rs->multimesh_set_buffer(p_type.multimesh, p_type.buf);
	}
	rs->multimesh_set_visible_instances(p_type.multimesh, p_type.count);
}

void ProjectileViews::frame(const Ref<ProjectileSwarm> &p_swarm, int64_t p_frames, double p_alpha, int p_tps,
		double p_net_now, double p_delta) {
	for (Type &t : types) {
		t.count = 0;
	}
	uint32_t f = 0;
	while (f < flashes.size()) {
		flashes[f].left -= p_delta;
		if (flashes[f].left <= 0.0) {
			flashes.remove_at(f);
			continue;
		}
		f++;
	}
	if (p_swarm.is_valid()) {
		const double tps = (double)MAX(1, p_tps);
		for (ProjectileSwarm::Shot &s : p_swarm->shots) {
			double age;
			if (s.auth) {
				age = (double(p_frames - s.born_tick) - 1.0 + p_alpha) / tps;
			} else {
				age = p_net_now - s.t0_local - lag_at(s.shown);
			}
			age = CLAMP(age, 0.0, MIN(s.swept_age, s.stop_age));
			if (s.stopped() && age >= s.stop_age) {
				if (s.has_impact && !s.flashed) {
					s.flashed = true;
					Flash fl;
					fl.type = s.view_type;
					fl.at = s.hit_point;
					fl.left = FLASH_S;
					flashes.push_back(fl);
					impacts.push_back(s.key);
				}
				continue;
			}
			const Vector3 p = s.pos_at(age);
			s.shown = p;
			_put(s.view_type, p, 1.0f);
		}
	}
	for (const Flash &fl : flashes) {
		_put(fl.type, fl.at, FLASH_SCALE * (real_t)(fl.left / FLASH_S));
	}
	for (Type &t : types) {
		_commit(t);
	}
}

PackedInt64Array ProjectileViews::take_impacts() {
	PackedInt64Array out = impacts;
	impacts.clear();
	return out;
}

int ProjectileViews::visible_count() const {
	int n = 0;
	for (const Type &t : types) {
		n += t.count;
	}
	return n;
}

void ProjectileViews::_bind_methods() {
	ClassDB::bind_method(D_METHOD("setup", "scenario"), &ProjectileViews::setup);
	ClassDB::bind_method(D_METHOD("type_add", "mesh", "scale", "tint"), &ProjectileViews::type_add, DEFVAL(1.0),
			DEFVAL(Color(1, 1, 1, 1)));
	ClassDB::bind_method(D_METHOD("take_impacts"), &ProjectileViews::take_impacts);
	ClassDB::bind_method(D_METHOD("set_lag_inputs", "points", "lags", "local"), &ProjectileViews::set_lag_inputs);
	ClassDB::bind_method(D_METHOD("lag_at", "at"), &ProjectileViews::lag_at);
	ClassDB::bind_method(D_METHOD("frame", "swarm", "frames", "alpha", "tps", "net_now", "delta"), &ProjectileViews::frame);
	ClassDB::bind_method(D_METHOD("visible_count"), &ProjectileViews::visible_count);
}
