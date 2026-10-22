#include "enemy_replicas.h"

#include "core/math/math_funcs.h"
#include "core/object/class_db.h"
#include "core/os/os.h"
#include "scene/3d/node_3d.h"

#include <cstring>

// «Нет события» — SNAME (статический StringName на уровне файла создавался бы до StringName::setup)
#define NONE_EVENT SNAME("-")

bool EnemyReplicas::_valid(int p_h) const {
	return p_h >= 0 && p_h < (int)reps.size() && reps[p_h].used;
}

void EnemyReplicas::set_views(const Ref<EnemyViews> &p_views) {
	views = p_views;
}

int EnemyReplicas::replica_add(Node3D *p_body, int p_view, int p_net_id) {
	int h;
	if (!free_handles.is_empty()) {
		h = free_handles[free_handles.size() - 1];
		free_handles.resize(free_handles.size() - 1);
	} else {
		h = (int)reps.size();
		const uint32_t n = reps.size() + 1;
		reps.resize(n);
	}
	Replica &r = reps[h];
	r = Replica();
	r.used = true;
	r.body_id = p_body ? p_body->get_instance_id() : ObjectID();
	r.view = p_view;
	r.net_id = p_net_id;
	if (p_net_id >= 0) {
		by_net[p_net_id] = h;
	}
	return h;
}

void EnemyReplicas::replica_remove(int p_h) {
	ERR_FAIL_COND(!_valid(p_h));
	Replica &r = reps[p_h];
	if (r.net_id >= 0) {
		HashMap<int, int>::Iterator e = by_net.find(r.net_id);
		if (e && e->value == p_h) {
			by_net.remove(e);
		}
	}
	r = Replica();
	free_handles.push_back(p_h);
}

// Вставка по времени хоста; дубль времени — отброшен; сверх CAP — уходит самая ранняя (как WalkerRemote.feed).
void EnemyReplicas::_feed(Replica &r_r, double p_t_local, double p_t_host, const float *p_src) {
	int i = r_r.n;
	while (i > 0 && r_r.t_host[i - 1] > p_t_host) {
		i--;
	}
	if (i > 0 && r_r.t_host[i - 1] == p_t_host) {
		return;
	}
	if (r_r.n == CAP) {
		if (i == 0) {
			return; // Новая — самая ранняя: ушла бы сразу
		}
		memmove(r_r.t_local, r_r.t_local + 1, sizeof(double) * (i - 1));
		memmove(r_r.t_host, r_r.t_host + 1, sizeof(double) * (i - 1));
		memmove(r_r.d, r_r.d + F, sizeof(float) * F * (i - 1));
		i--;
	} else {
		memmove(r_r.t_local + i + 1, r_r.t_local + i, sizeof(double) * (r_r.n - i));
		memmove(r_r.t_host + i + 1, r_r.t_host + i, sizeof(double) * (r_r.n - i));
		memmove(r_r.d + (i + 1) * F, r_r.d + i * F, sizeof(float) * F * (r_r.n - i));
		r_r.n++;
	}
	r_r.t_local[i] = p_t_local;
	r_r.t_host[i] = p_t_host;
	memcpy(r_r.d + i * F, p_src, sizeof(float) * F);
}

void EnemyReplicas::feed(int p_h, double p_t_local, double p_t_host, const PackedFloat32Array &p_data, int p_at) {
	ERR_FAIL_COND(!_valid(p_h));
	ERR_FAIL_COND(p_at < 0 || p_at + F > p_data.size());
	_feed(reps[p_h], p_t_local, p_t_host, p_data.ptr() + p_at);
}

void EnemyReplicas::feed_snapshot(double p_t_local, double p_t_host, const PackedInt32Array &p_ids, const PackedFloat32Array &p_data) {
	ERR_FAIL_COND(p_data.size() < p_ids.size() * F);
	const float *src = p_data.ptr();
	for (int i = 0; i < p_ids.size(); i++) {
		HashMap<int, int>::ConstIterator e = by_net.find(p_ids[i]);
		if (e && _valid(e->value)) {
			_feed(reps[e->value], p_t_local, p_t_host, src + i * F);
		}
	}
}

void EnemyReplicas::queue_event(int p_h, double p_t_local, const StringName &p_clip) {
	ERR_FAIL_COND(!_valid(p_h));
	Replica &r = reps[p_h];
	uint32_t i = r.ev_t.size();
	while (i > 0 && r.ev_t[i - 1] > p_t_local) {
		i--;
	}
	r.ev_t.insert(i, p_t_local);
	r.ev_clip.insert(i, p_clip);
}

StringName EnemyReplicas::_pop_due(Replica &r_r, double p_seek) {
	if (r_r.ev_t.is_empty() || r_r.ev_t[0] > p_seek) {
		return NONE_EVENT;
	}
	const StringName clip = r_r.ev_clip[0];
	r_r.ev_t.remove_at(0);
	r_r.ev_clip.remove_at(0);
	return clip;
}

StringName EnemyReplicas::pop_due(int p_h, double p_seek) {
	ERR_FAIL_COND_V(!_valid(p_h), NONE_EVENT);
	return _pop_due(reps[p_h], p_seek);
}

// Первая точка не раньше seek; показ — у конца буфера, поиск с конца (как WalkerRemote._sample).
bool EnemyReplicas::_sample(Replica &r_r, const double *p_times, double p_seek) {
	const int n = r_r.n;
	if (n == 0) {
		return false;
	}
	int b = n;
	while (b > 0 && p_times[b - 1] >= p_seek) {
		b--;
	}
	if (b == 0) {
		_take(r_r, 0, 0, 0.0f);
	} else if (b == n) {
		_take(r_r, n - 1, n - 1, 0.0f);
	} else {
		const int a = b - 1;
		const double span = p_times[b] - p_times[a];
		_take(r_r, a, b, span > 0.0 ? (real_t)CLAMP((p_seek - p_times[a]) / span, 0.0, 1.0) : 1.0f);
	}
	return true;
}

void EnemyReplicas::_take(Replica &r_r, int p_a, int p_b, real_t p_f) {
	const float *a = r_r.d + p_a * F;
	const float *b = r_r.d + p_b * F;
	r_r.pos = Vector3(Math::lerp(a[0], b[0], p_f), Math::lerp(a[1], b[1], p_f), Math::lerp(a[2], b[2], p_f));
	r_r.yaw = Math::lerp_angle(a[3], b[3], p_f);
	r_r.speed = Math::lerp(a[4], b[4], p_f);
	r_r.step_lag = Math::lerp(a[5], b[5], p_f);
}

bool EnemyReplicas::sample_local(int p_h, double p_seek) {
	ERR_FAIL_COND_V(!_valid(p_h), false);
	return _sample(reps[p_h], reps[p_h].t_local, p_seek);
}

bool EnemyReplicas::sample_host(int p_h, double p_t_host) {
	ERR_FAIL_COND_V(!_valid(p_h), false);
	return _sample(reps[p_h], reps[p_h].t_host, p_t_host);
}

int EnemyReplicas::replica_size(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0);
	return reps[p_h].n;
}

Vector3 EnemyReplicas::replica_pos(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), Vector3());
	return reps[p_h].pos;
}

real_t EnemyReplicas::replica_yaw(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0.0f);
	return reps[p_h].yaw;
}

real_t EnemyReplicas::replica_speed(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0.0f);
	return reps[p_h].speed;
}

real_t EnemyReplicas::replica_step_lag(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0.0f);
	return reps[p_h].step_lag;
}

// Физ-тик копий: поза показа в тело (система отсчёта копии — мир), поза визуалу, наступившие события.
PackedInt32Array EnemyReplicas::tick(double p_seek) {
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	due_handles.clear();
	due_clips.clear();
	for (uint32_t h = 0; h < reps.size(); h++) {
		Replica &r = reps[h];
		if (!r.used) {
			continue;
		}
		Node3D *body = r.body_id.is_valid() ? Object::cast_to<Node3D>(ObjectDB::get_instance(r.body_id)) : nullptr;
		if (body != nullptr && _sample(r, r.t_local, p_seek)) {
			body->set_global_position(r.pos);
		}
		if (body != nullptr && views.is_valid() && r.view >= 0) {
			views->view_commit(r.view, Transform3D(Basis(Vector3(0.0f, 1.0f, 0.0f), r.yaw), body->get_global_position()), r.step_lag, r.speed);
		}
		StringName e = _pop_due(r, p_seek);
		while (e != NONE_EVENT) {
			due_handles.push_back((int)h);
			due_clips.push_back(e);
			e = _pop_due(r, p_seek);
		}
	}
	last_tick_usec = OS::get_singleton()->get_ticks_usec() - t0;
	return due_handles;
}

int EnemyReplicas::get_replica_count() const {
	return (int)(reps.size() - free_handles.size());
}

void EnemyReplicas::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_views", "views"), &EnemyReplicas::set_views);
	ClassDB::bind_method(D_METHOD("replica_add", "body", "view", "net_id"), &EnemyReplicas::replica_add);
	ClassDB::bind_method(D_METHOD("replica_remove", "handle"), &EnemyReplicas::replica_remove);
	ClassDB::bind_method(D_METHOD("feed", "handle", "t_local", "t_host", "data", "at"), &EnemyReplicas::feed);
	ClassDB::bind_method(D_METHOD("feed_snapshot", "t_local", "t_host", "ids", "data"), &EnemyReplicas::feed_snapshot);
	ClassDB::bind_method(D_METHOD("queue_event", "handle", "t_local", "clip"), &EnemyReplicas::queue_event);
	ClassDB::bind_method(D_METHOD("pop_due", "handle", "seek"), &EnemyReplicas::pop_due);
	ClassDB::bind_method(D_METHOD("sample_local", "handle", "seek"), &EnemyReplicas::sample_local);
	ClassDB::bind_method(D_METHOD("sample_host", "handle", "t_host"), &EnemyReplicas::sample_host);
	ClassDB::bind_method(D_METHOD("replica_size", "handle"), &EnemyReplicas::replica_size);
	ClassDB::bind_method(D_METHOD("replica_pos", "handle"), &EnemyReplicas::replica_pos);
	ClassDB::bind_method(D_METHOD("replica_yaw", "handle"), &EnemyReplicas::replica_yaw);
	ClassDB::bind_method(D_METHOD("replica_speed", "handle"), &EnemyReplicas::replica_speed);
	ClassDB::bind_method(D_METHOD("replica_step_lag", "handle"), &EnemyReplicas::replica_step_lag);
	ClassDB::bind_method(D_METHOD("tick", "seek"), &EnemyReplicas::tick);
	ClassDB::bind_method(D_METHOD("get_due_clips"), &EnemyReplicas::get_due_clips);
	ClassDB::bind_method(D_METHOD("get_replica_count"), &EnemyReplicas::get_replica_count);
	ClassDB::bind_method(D_METHOD("get_last_tick_usec"), &EnemyReplicas::get_last_tick_usec);

	BIND_CONSTANT(CAP);
	BIND_CONSTANT(F);
}
