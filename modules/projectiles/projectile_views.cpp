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

int ProjectileViews::type_add(RID p_mesh) {
	RenderingServer *rs = RenderingServer::get_singleton();
	Type t;
	t.multimesh = rs->multimesh_create();
	rs->multimesh_set_mesh(t.multimesh, p_mesh);
	t.instance = rs->instance_create2(t.multimesh, scenario);
	rs->instance_geometry_set_cast_shadows_setting(t.instance, RSE::SHADOW_CASTING_SETTING_OFF);
	types.push_back(t);
	return (int)types.size() - 1;
}

void ProjectileViews::set_lag_inputs(const PackedVector3Array &p_points, const PackedFloat64Array &p_lags, int p_local) {
	lag_points = p_points;
	lags = p_lags;
	lag_local = p_local;
}

// Расстояние от точки до зоны касания игрока, м (0 — в зоне) — ItemViewTime.zone_gap.
double ProjectileViews::_gap(int p_player, const Vector3 &p_at) const {
	const Vector3 *pts = lag_points.ptr() + p_player * 4;
	double g = MIN((double)pts[0].distance_to(p_at), (double)pts[1].distance_to(p_at)) - STEAL_BODY_R;
	g = MIN(g, (double)pts[2].distance_to(p_at) - STEAL_HAND_R);
	g = MIN(g, (double)pts[3].distance_to(p_at) - STEAL_HAND_R);
	return MAX(g, 0.0);
}

// Задержка показа точки, с — ItemViewTime.lag_at: у своих ног 0, у чужих — время его аватара, по дороге —
// пропорционально расстояниям до зон касания.
double ProjectileViews::lag_at(const Vector3 &p_at) const {
	const int n = lags.size();
	if (lag_local < 0 || lag_local >= n || lag_points.size() < n * 4) {
		return 0.0;
	}
	const double g_me = _gap(lag_local, p_at);
	double g_them = Math::INF;
	double lag_them = 0.0;
	for (int i = 0; i < n; i++) {
		if (i == lag_local) {
			continue;
		}
		const double g = _gap(i, p_at);
		if (g < g_them) {
			g_them = g;
			lag_them = lags[i];
		}
	}
	if (g_them < Math::INF && g_me + g_them > 0.0) { // Оба в зоне — у своих ног (настоящее)
		return lag_them * g_me / (g_me + g_them);
	}
	return 0.0;
}

void ProjectileViews::_put(int p_type, const Vector3 &p_at, real_t p_scale) {
	if (p_type < 0 || p_type >= (int)types.size()) {
		return;
	}
	Type &t = types[p_type];
	if ((t.count + 1) * STRIDE > t.buf.size()) {
		// Хвост — нули: Vector::resize не заполняет float, а AABB MultiMesh считается по всему буферу.
		t.buf.resize_initialized(MAX(16, t.buf.size() / STRIDE * 2) * STRIDE);
	}
	float *w = t.buf.ptrw() + t.count * STRIDE;
	w[0] = p_scale;
	w[1] = 0.0f;
	w[2] = 0.0f;
	w[3] = p_at.x;
	w[4] = 0.0f;
	w[5] = p_scale;
	w[6] = 0.0f;
	w[7] = p_at.y;
	w[8] = 0.0f;
	w[9] = 0.0f;
	w[10] = p_scale;
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

int ProjectileViews::visible_count() const {
	int n = 0;
	for (const Type &t : types) {
		n += t.count;
	}
	return n;
}

void ProjectileViews::_bind_methods() {
	ClassDB::bind_method(D_METHOD("setup", "scenario"), &ProjectileViews::setup);
	ClassDB::bind_method(D_METHOD("type_add", "mesh"), &ProjectileViews::type_add);
	ClassDB::bind_method(D_METHOD("set_lag_inputs", "points", "lags", "local"), &ProjectileViews::set_lag_inputs);
	ClassDB::bind_method(D_METHOD("lag_at", "at"), &ProjectileViews::lag_at);
	ClassDB::bind_method(D_METHOD("frame", "swarm", "frames", "alpha", "tps", "net_now", "delta"), &ProjectileViews::frame);
	ClassDB::bind_method(D_METHOD("visible_count"), &ProjectileViews::visible_count);
}
