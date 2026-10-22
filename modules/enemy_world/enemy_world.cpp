#include "enemy_world.h"

#include "core/config/engine.h"
#include "core/object/class_db.h"
#include "core/os/os.h"
#include "servers/navigation_3d/navigation_server_3d.h"

#include <limits>

uint64_t EnemyWorld::_now() {
	return Engine::get_singleton()->get_physics_frames();
}

bool EnemyWorld::_valid(int p_h) const {
	return p_h >= 0 && p_h < (int)alive.size() && alive[p_h];
}

void EnemyWorld::_grow(uint32_t p_size) {
	alive.resize(p_size);
	prio.resize(p_size);
	in_prev.resize(p_size);
	in_curr.resize(p_size);
	in_tick.resize(p_size);
	path_new.resize(p_size);
	path_new_target.resize(p_size);
	path_tick.resize(p_size);
	path_pending.resize(p_size);
	follow.resize(p_size);
	out_vel.resize(p_size);
	out_heading.resize(p_size);
	out_rank.resize(p_size);
	out_yield.resize(p_size);
	out_state.resize(p_size);
	out_wants.resize(p_size);
	stuck_total.resize(p_size);
	blocked_total.resize(p_size);
	yield_total.resize(p_size);
	start_pos.resize(p_size);
	prev_heading.resize(p_size);
	prev_rank.resize(p_size);
	prev_yield.resize(p_size);
	start_in.resize(p_size);
	prev_state.resize(p_size);
}

const EnemyWorld::Input &EnemyWorld::_start(uint32_t p_h, uint64_t p_now) const {
	return in_tick[p_h] == p_now ? in_prev[p_h] : in_curr[p_h];
}

void EnemyWorld::set_navigation(const Ref<NavigationMesh> &p_mesh, RID p_map) {
	column_index.build(p_mesh);
	nav_map = p_map;
}

Vector3 EnemyWorld::column(const Vector3 &p_local, int p_hint) {
	Vector3 p;
	int poly = -1;
	if (column_index.find(p_local, COLUMN_H, p_hint, p, poly)) {
		column_poly = poly;
		return p;
	}
	column_poly = -1;
	return column_engine(p_local);
}

Vector3 EnemyWorld::column_engine(const Vector3 &p_local) const {
	if (!nav_map.is_valid()) {
		return p_local;
	}
	// use_collision = false: с true сервер при промахе отрезка мимо сетки отдаёт Vector3()
	return NavigationServer3D::get_singleton()->map_get_closest_point_to_segment(nav_map,
			p_local + Vector3(0.0f, COLUMN_H, 0.0f), p_local - Vector3(0.0f, COLUMN_H, 0.0f), false);
}

int EnemyWorld::agent_add(const Vector3 &p_local_pos) {
	uint32_t h;
	if (!free_handles.is_empty()) {
		h = free_handles[free_handles.size() - 1];
		free_handles.resize(free_handles.size() - 1);
	} else {
		h = alive.size();
		_grow(h + 1);
	}
	alive[h] = 1;
	prio[h] = next_prio++;
	Input in;
	in.pos = p_local_pos;
	in_prev[h] = in;
	in_curr[h] = in;
	in_tick[h] = UINT64_MAX;
	path_new[h].clear();
	path_pending[h] = 0;
	follow[h] = Follow();
	out_vel[h] = Vector3();
	out_heading[h] = Vector3();
	out_rank[h] = prio[h];
	out_yield[h] = 0;
	out_state[h] = AGENT_IDLE;
	out_wants[h] = 0;
	stuck_total[h] = 0;
	blocked_total[h] = 0;
	yield_total[h] = 0;
	prev_heading[h] = Vector3();
	prev_rank[h] = prio[h];
	prev_yield[h] = 0;
	prev_state[h] = AGENT_IDLE;
	return (int)h;
}

void EnemyWorld::agent_remove(int p_h) {
	ERR_FAIL_COND(!_valid(p_h));
	alive[p_h] = 0;
	follow[p_h] = Follow();
	free_handles.push_back((uint32_t)p_h);
}

void EnemyWorld::agent_commit(int p_h, const Vector3 &p_local_pos, const Vector3 &p_goal, bool p_has_goal, real_t p_stop_dist, real_t p_speed) {
	ERR_FAIL_COND(!_valid(p_h));
	const uint64_t now = _now();
	if (in_tick[p_h] != now) {
		in_prev[p_h] = in_curr[p_h];
		in_tick[p_h] = now;
	}
	Input &in = in_curr[p_h];
	in.pos = p_local_pos;
	in.goal = p_goal;
	in.has_goal = p_has_goal;
	in.stop_dist = p_stop_dist;
	in.speed = p_speed;
}

void EnemyWorld::agent_set_path(int p_h, const PackedVector3Array &p_points, const Vector3 &p_target) {
	ERR_FAIL_COND(!_valid(p_h));
	path_new[p_h] = p_points;
	path_new_target[p_h] = p_target;
	path_tick[p_h] = _now();
	path_pending[p_h] = 1;
}

Vector3 EnemyWorld::agent_velocity(int p_h) {
	ERR_FAIL_COND_V(!_valid(p_h), Vector3());
	_ensure_step();
	return out_vel[p_h];
}

int EnemyWorld::agent_state(int p_h) {
	ERR_FAIL_COND_V(!_valid(p_h), AGENT_IDLE);
	_ensure_step();
	return out_state[p_h];
}

bool EnemyWorld::agent_wants_path(int p_h) {
	ERR_FAIL_COND_V(!_valid(p_h), false);
	_ensure_step();
	return out_wants[p_h] != 0;
}

Vector3 EnemyWorld::agent_heading(int p_h) {
	ERR_FAIL_COND_V(!_valid(p_h), Vector3());
	_ensure_step();
	return out_heading[p_h];
}

int64_t EnemyWorld::agent_rank(int p_h) {
	ERR_FAIL_COND_V(!_valid(p_h), 0);
	_ensure_step();
	return out_rank[p_h];
}

bool EnemyWorld::agent_yielding(int p_h) {
	ERR_FAIL_COND_V(!_valid(p_h), false);
	_ensure_step();
	return out_yield[p_h] != 0;
}

int EnemyWorld::agent_stuck_total(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0);
	return stuck_total[p_h];
}

int EnemyWorld::agent_blocked_total(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0);
	return blocked_total[p_h];
}

int EnemyWorld::agent_yield_total(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0);
	return yield_total[p_h];
}

int EnemyWorld::get_agent_count() const {
	return (int)(alive.size() - free_handles.size());
}

void EnemyWorld::_ensure_step() {
	const uint64_t now = _now();
	if (stepped_tick == now) {
		return;
	}
	stepped_tick = now;
	_step();
}

void EnemyWorld::_step() {
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	const uint64_t now = _now();
	const real_t dt = 1.0f / (real_t)Engine::get_singleton()->get_physics_ticks_per_second();
	const uint32_t n = alive.size();
	for (uint32_t h = 0; h < n; h++) {
		if (!alive[h]) {
			continue;
		}
		const Input &in = _start(h, now);
		start_in[h] = in;
		start_pos[h] = in.pos;
		prev_heading[h] = out_heading[h];
		prev_rank[h] = out_rank[h];
		prev_yield[h] = out_yield[h];
		prev_state[h] = out_state[h];
	}
	for (uint32_t h = 0; h < n; h++) {
		if (alive[h]) {
			_step_agent(h, now, dt);
		}
	}
	last_step_usec = OS::get_singleton()->get_ticks_usec() - t0;
}

void EnemyWorld::_step_agent(uint32_t p_h, uint64_t p_now, real_t p_dt) {
	const Input &in = _start(p_h, p_now);
	Follow &f = follow[p_h];
	if (path_pending[p_h] && path_tick[p_h] < p_now) {
		path_pending[p_h] = 0;
		_adopt_path(p_h, f);
	}
	if (!in.has_goal) {
		out_vel[p_h] = Vector3();
		out_heading[p_h] = Vector3();
		out_state[p_h] = AGENT_IDLE;
		out_wants[p_h] = 0;
		out_rank[p_h] = prio[p_h];
		return;
	}
	_neighbors(p_h, in.pos);
	_yield_tick(p_h, f, in.pos, p_dt);
	Vector3 w = _follow_tick(p_h, f, in, p_dt);
	if (!column_index.is_empty() || nav_map.is_valid()) {
		w = _onto_mesh(f, in.pos, w, in.speed, p_dt);
	}
	out_vel[p_h] = w;
	out_state[p_h] = (uint8_t)f.state;
	out_wants[p_h] = _wants_path(f, in) ? 1 : 0;
}

void EnemyWorld::_adopt_path(uint32_t p_h, Follow &r_f) {
	const Vector<Vector3> &src = path_new[p_h];
	r_f.path.resize(src.size());
	for (int i = 0; i < src.size(); i++) {
		r_f.path[i] = src[i];
	}
	r_f.path_target = path_new_target[p_h];
	r_f.corner = 1;
	r_f.since_path = 0.0f;
	r_f.blocked_t = 0.0f;
	if (r_f.path.size() < 2) {
		r_f.path.clear();
		r_f.has_path = false;
		if (r_f.state != AGENT_BLOCKED) {
			blocked_total[p_h]++;
		}
		r_f.state = AGENT_BLOCKED;
	} else {
		r_f.has_path = true;
		if (r_f.state == AGENT_BLOCKED) {
			r_f.state = AGENT_IDLE;
		}
	}
}

void EnemyWorld::_neighbors(uint32_t p_h, const Vector3 &p_pos) {
	nb.clear();
	nb_d2.clear();
	const real_t r2 = SEPARATION_R * SEPARATION_R;
	const uint32_t n = alive.size();
	for (uint32_t j = 0; j < n; j++) {
		if (j == p_h || !alive[j]) {
			continue;
		}
		const real_t dx = start_pos[j].x - p_pos.x;
		const real_t dz = start_pos[j].z - p_pos.z;
		const real_t d2 = dx * dx + dz * dz;
		if (d2 > r2) {
			continue;
		}
		if (nb.size() >= NEIGHBORS_MAX) {
			if (d2 >= nb_d2[NEIGHBORS_MAX - 1]) {
				continue;
			}
			nb.resize(NEIGHBORS_MAX - 1);
			nb_d2.resize(NEIGHBORS_MAX - 1);
		}
		uint32_t k = nb.size();
		nb.push_back(j);
		nb_d2.push_back(d2);
		while (k > 0 && nb_d2[k - 1] > d2) {
			nb[k] = nb[k - 1];
			nb_d2[k] = nb_d2[k - 1];
			k--;
		}
		nb[k] = j;
		nb_d2[k] = d2;
	}
}

void EnemyWorld::_yield_tick(uint32_t p_h, Follow &r_f, const Vector3 &p_pos, real_t p_dt) {
	const Vector3 &dir = prev_heading[p_h]; // Направление пути прошлого расчёта
	int64_t rank = prio[p_h];
	bool leader_yields = false;
	bool oncoming = false;
	int64_t oncoming_rank = std::numeric_limits<int64_t>::max();
	if (dir.length_squared() > 0.25f) {
		for (const uint32_t j : nb) {
			Vector3 d = start_pos[j] - p_pos;
			d.y = 0.0f;
			const real_t l = d.length();
			if (l < 1e-4f || l >= SEPARATION_R || d.dot(dir) / l <= AHEAD_COS) {
				continue;
			}
			const real_t c = prev_heading[j].dot(dir);
			if (c >= FOLLOW_COS) {
				rank = MIN(rank, prev_rank[j]);
				leader_yields = leader_yields || prev_yield[j] != 0;
			} else if (c <= ONCOMING_COS) {
				oncoming = true;
				oncoming_rank = MIN(oncoming_rank, prev_rank[j]);
			}
		}
	}
	out_rank[p_h] = rank;
	// У конца пути встречные — толпа, сходящаяся к одной цели, а не конфликт в проходе: не уступаем
	if (r_f.has_path && _remaining(r_f, p_pos) <= YIELD_MIN_REMAIN) {
		r_f.yielding = false;
		r_f.conflict_t = 0.0f;
		out_yield[p_h] = 0;
		return;
	}
	if (r_f.yielding) {
		r_f.yield_t += p_dt;
		r_f.clear_t = (oncoming || leader_yields) ? 0.0f : r_f.clear_t + p_dt;
		if (r_f.clear_t >= CLEAR_S || r_f.yield_t >= MAX_S) {
			r_f.yielding = false;
			r_f.conflict_t = 0.0f;
		}
	} else {
		r_f.conflict_t = (oncoming && oncoming_rank < rank) ? r_f.conflict_t + p_dt : 0.0f;
		if (leader_yields || r_f.conflict_t >= CONFLICT_S) {
			r_f.yielding = true;
			yield_total[p_h]++;
			r_f.yield_t = 0.0f;
			r_f.clear_t = 0.0f;
		}
	}
	out_yield[p_h] = r_f.yielding ? 1 : 0;
}

Vector3 EnemyWorld::_follow_tick(uint32_t p_h, Follow &r_f, const Input &p_in, real_t p_dt) {
	const Vector3 &pos = p_in.pos;
	r_f.since_path += p_dt;
	if (r_f.state == AGENT_BLOCKED) {
		r_f.blocked_t += p_dt;
	}
	if (_flat(p_in.goal - pos).length() <= p_in.stop_dist || r_f.state == AGENT_BLOCKED || !r_f.has_path) {
		out_heading[p_h] = Vector3();
		if (r_f.state != AGENT_BLOCKED) {
			r_f.state = AGENT_IDLE;
		}
		_track_stuck(p_h, r_f, pos, 0.0f, p_dt);
		return _separation(pos);
	}
	_advance(r_f, pos);
	const uint32_t n = r_f.path.size();
	const Vector3 corner = r_f.path[r_f.corner];
	const Vector3 dir = _flat(corner - pos).normalized();
	// Конец пути (цель вне сетки) — дальше не пройти: стоим, как и в очереди за стоящим соседом
	const bool at_end = (uint32_t)r_f.corner == n - 1 && _flat(corner - pos).length() <= CORNER_REACH;
	if (_hold_tick(r_f, p_in, dir, at_end, p_dt)) {
		out_heading[p_h] = Vector3();
		r_f.state = AGENT_HOLD;
		_track_stuck(p_h, r_f, pos, 0.0f, p_dt);
		return _overlap(pos);
	}
	out_heading[p_h] = dir;
	const Vector3 sep = _separation(pos);
	if (r_f.yielding) {
		r_f.state = AGENT_YIELD;
		_track_stuck(p_h, r_f, pos, 0.0f, p_dt);
		return -dir * YIELD_SPEED + sep;
	}
	r_f.state = AGENT_MOVE;
	real_t s = p_in.speed;
	const bool reaches = !r_f.path.is_empty() && r_f.path[n - 1].distance_to(r_f.path_target) <= REACH_TOL;
	if (reaches) {
		s = p_in.speed * CLAMP((_remaining(r_f, pos) - p_in.stop_dist) / SLOW_DIST, 0.0f, 1.0f);
	}
	const Vector3 want = dir * s + sep + _keep_right(pos, dir);
	_track_stuck(p_h, r_f, pos, want.length(), p_dt);
	return want;
}

Vector3 EnemyWorld::_separation(const Vector3 &p_pos) const {
	Vector3 out;
	for (const uint32_t j : nb) {
		const Vector3 d = _flat(p_pos - start_pos[j]);
		const real_t l = d.length();
		if (l < 1e-4f || l >= SEPARATION_R) {
			continue;
		}
		out += d / l * SEPARATION_GAIN * (1.0f - l / SEPARATION_R);
	}
	return out;
}

Vector3 EnemyWorld::_keep_right(const Vector3 &p_pos, const Vector3 &p_dir) const {
	const Vector3 right(-p_dir.z, 0.0f, p_dir.x); // forward × up: при взгляде на −Z право — +X
	Vector3 out;
	for (const uint32_t j : nb) {
		const Vector3 d = _flat(start_pos[j] - p_pos);
		const real_t l = d.length();
		if (l < 1e-4f || l >= SEPARATION_R) {
			continue;
		}
		if (d.dot(p_dir) / l > 0.5f) {
			out += right * SIDE_GAIN * (1.0f - l / SEPARATION_R);
		}
	}
	return out;
}

// Стояние (HOLD): вход — конец пути или очередь (_queued); выход — конец пути (новый путь за ушедшей целью) ушёл
// дальше TARGET_MOVED от конца на входе или свободно HOLD_LEAVE_S подряд. У недостижимой цели конец пути — ближайшая
// к ней точка сетки: цель ходит по недоступному месту — кольцо стоит, пока эта точка рядом. true — стоим в этом тике.
bool EnemyWorld::_hold_tick(Follow &r_f, const Input &p_in, const Vector3 &p_dir, bool p_at_end, real_t p_dt) const {
	const bool blocked = p_at_end || _queued(r_f, p_in, p_dir);
	if (r_f.state != AGENT_HOLD) {
		if (blocked) {
			r_f.hold_end = r_f.path[r_f.path.size() - 1];
			r_f.hold_free_t = 0.0f;
			r_f.yielding = false;
		}
		return blocked;
	}
	if (r_f.path[r_f.path.size() - 1].distance_to(r_f.hold_end) > TARGET_MOVED) {
		return false;
	}
	r_f.hold_free_t = blocked ? 0.0f : r_f.hold_free_t + p_dt;
	return r_f.hold_free_t < HOLD_LEAVE_S;
}

// Очередь: у конца пути (≤ HOLD_QUEUE_REMAIN) прямо впереди (ближе HOLD_AHEAD_R, в секторе AHEAD_COS) стоит
// пришедший сосед (_arrived), который ждёт ту же цель (цели ближе HOLD_SAME_GOAL) или цели не имеет (бьёт).
bool EnemyWorld::_queued(const Follow &p_f, const Input &p_in, const Vector3 &p_dir) const {
	if (p_dir.length_squared() < 0.25f || _remaining(p_f, p_in.pos) > HOLD_QUEUE_REMAIN) {
		return false;
	}
	for (const uint32_t j : nb) {
		if (!_arrived(j)) {
			continue;
		}
		const Vector3 d = _flat(start_pos[j] - p_in.pos);
		const real_t l = d.length();
		if (l < 1e-4f || l >= HOLD_AHEAD_R || d.dot(p_dir) / l <= AHEAD_COS) {
			continue;
		}
		const Input &o = start_in[j];
		if (!o.has_goal || o.goal.distance_to(p_in.goal) <= HOLD_SAME_GOAL) {
			return true;
		}
	}
	return false;
}

// Сосед пришёл (прошлый расчёт): стоит в HOLD, или IDLE без цели (бьёт, замахивается) или у своей цели. IDLE с
// целью вдали (ждёт первый путь после спавна) — не пришёл: за ним очередь не встаёт.
bool EnemyWorld::_arrived(uint32_t p_j) const {
	if (prev_state[p_j] == AGENT_HOLD) {
		return true;
	}
	if (prev_state[p_j] != AGENT_IDLE) {
		return false;
	}
	const Input &o = start_in[p_j];
	return !o.has_goal || _flat(o.goal - o.pos).length() <= o.stop_dist;
}

// Стоящего толкает только идущий сосед ближе OVERLAP_R: пришедшие (стоят, бьют) друг друга не толкают — врозь их
// держат тела (иначе соседи вплотную толкаются вечно и тело не уходит в покой).
Vector3 EnemyWorld::_overlap(const Vector3 &p_pos) const {
	Vector3 out;
	for (const uint32_t j : nb) {
		if (_arrived(j)) {
			continue;
		}
		const Vector3 d = _flat(p_pos - start_pos[j]);
		const real_t l = d.length();
		if (l < 1e-4f || l >= OVERLAP_R) {
			continue;
		}
		out += d / l * SEPARATION_GAIN * (1.0f - l / OVERLAP_R);
	}
	return out;
}

void EnemyWorld::_advance(Follow &r_f, const Vector3 &p_pos) const {
	const int n = (int)r_f.path.size();
	while (r_f.corner < n - 1) {
		const Vector3 &a = r_f.path[r_f.corner - 1];
		const Vector3 &b = r_f.path[r_f.corner];
		const Vector3 ab = _flat(b - a);
		const real_t t = _flat(p_pos - a).dot(ab) / MAX(ab.length_squared(), 1e-6f);
		if (t >= 1.0f || _flat(b - p_pos).length() <= CORNER_REACH) {
			r_f.corner++;
		} else {
			break;
		}
	}
}

real_t EnemyWorld::_remaining(const Follow &p_f, const Vector3 &p_pos) const {
	real_t r = _flat(p_f.path[p_f.corner] - p_pos).length();
	for (uint32_t i = (uint32_t)p_f.corner; i + 1 < p_f.path.size(); i++) {
		r += _flat(p_f.path[i + 1] - p_f.path[i]).length();
	}
	return r;
}

real_t EnemyWorld::_off_path(const Follow &p_f, const Vector3 &p_pos) const {
	const Vector3 &a = p_f.path[p_f.corner - 1];
	const Vector3 &b = p_f.path[p_f.corner];
	const Vector3 ab = _flat(b - a);
	const real_t t = CLAMP(_flat(p_pos - a).dot(ab) / MAX(ab.length_squared(), 1e-6f), 0.0f, 1.0f);
	return _flat(p_pos - (a + (b - a) * t)).length();
}

bool EnemyWorld::_wants_path(const Follow &p_f, const Input &p_in) const {
	if (p_f.state == AGENT_BLOCKED) {
		return p_f.blocked_t >= BLOCKED_REPATH_S;
	}
	if (p_f.state == AGENT_HOLD) {
		return p_f.path_target.distance_to(p_in.goal) > TARGET_MOVED; // Стоим — путь только за ушедшей целью
	}
	if (_flat(p_in.goal - p_in.pos).length() <= p_in.stop_dist) {
		return false;
	}
	if (!p_f.has_path) {
		return true;
	}
	if (p_f.since_path >= REPATH_S) {
		return true;
	}
	if (p_f.path_target.distance_to(p_in.goal) > TARGET_MOVED) {
		return true;
	}
	return _off_path(p_f, p_in.pos) > OFF_PATH;
}

void EnemyWorld::_track_stuck(uint32_t p_h, Follow &r_f, const Vector3 &p_pos, real_t p_want_speed, real_t p_dt) {
	if (r_f.state != AGENT_MOVE || p_want_speed < STUCK_SPEED) {
		r_f.stuck_t = 0.0f;
		r_f.stuck_from = p_pos;
		return;
	}
	r_f.stuck_t += p_dt;
	if (r_f.stuck_t < STUCK_S) {
		return;
	}
	const real_t moved = _flat(p_pos - r_f.stuck_from).length();
	r_f.stuck_t = 0.0f;
	r_f.stuck_from = p_pos;
	if (moved >= STUCK_MOVE) {
		r_f.stuck_run = 0;
		return;
	}
	r_f.stuck_run++;
	stuck_total[p_h]++;
	r_f.path.clear(); // Новый путь — в ближайший слот
	r_f.has_path = false;
	if (r_f.stuck_run >= STUCK_LIMIT) {
		r_f.stuck_run = 0;
		r_f.state = AGENT_BLOCKED;
		r_f.blocked_t = 0.0f;
		blocked_total[p_h]++;
	}
}

// Шаг не уводит с сетки: дальше OFF_MESH_DIRECT от неё — к ближайшей её точке напрямую; шаг за край —
// срезается к ближайшей точке края (скорость не больше желаемой). Точки сетки — по вертикали (column).
Vector3 EnemyWorld::_onto_mesh(Follow &r_f, const Vector3 &p_pos, const Vector3 &p_w, real_t p_speed, real_t p_dt) {
	const Vector3 on = column(p_pos, r_f.hint);
	r_f.hint = column_poly;
	const Vector2 off(on.x - p_pos.x, on.z - p_pos.z);
	if (off.length() > OFF_MESH_DIRECT) {
		const Vector2 o = off.normalized() * p_speed;
		return Vector3(o.x, 0.0f, o.y);
	}
	if (p_w.length_squared() < 1e-6f) {
		return p_w;
	}
	const Vector3 next = p_pos + p_w * p_dt;
	const Vector3 c = column(next, r_f.hint);
	if (Vector2(c.x - next.x, c.z - next.z).length() <= MESH_EPS) {
		return p_w;
	}
	return (Vector3(c.x - p_pos.x, 0.0f, c.z - p_pos.z) / p_dt).limit_length(p_w.length());
}

void EnemyWorld::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_navigation", "mesh", "map"), &EnemyWorld::set_navigation);
	ClassDB::bind_method(D_METHOD("column", "local", "hint"), &EnemyWorld::column);
	ClassDB::bind_method(D_METHOD("get_column_poly"), &EnemyWorld::get_column_poly);
	ClassDB::bind_method(D_METHOD("column_engine", "local"), &EnemyWorld::column_engine);
	ClassDB::bind_method(D_METHOD("agent_add", "local_pos"), &EnemyWorld::agent_add);
	ClassDB::bind_method(D_METHOD("agent_remove", "handle"), &EnemyWorld::agent_remove);
	ClassDB::bind_method(D_METHOD("agent_commit", "handle", "local_pos", "goal", "has_goal", "stop_dist", "speed"), &EnemyWorld::agent_commit);
	ClassDB::bind_method(D_METHOD("agent_set_path", "handle", "points", "target"), &EnemyWorld::agent_set_path);
	ClassDB::bind_method(D_METHOD("agent_velocity", "handle"), &EnemyWorld::agent_velocity);
	ClassDB::bind_method(D_METHOD("agent_state", "handle"), &EnemyWorld::agent_state);
	ClassDB::bind_method(D_METHOD("agent_wants_path", "handle"), &EnemyWorld::agent_wants_path);
	ClassDB::bind_method(D_METHOD("agent_heading", "handle"), &EnemyWorld::agent_heading);
	ClassDB::bind_method(D_METHOD("agent_rank", "handle"), &EnemyWorld::agent_rank);
	ClassDB::bind_method(D_METHOD("agent_yielding", "handle"), &EnemyWorld::agent_yielding);
	ClassDB::bind_method(D_METHOD("agent_stuck_total", "handle"), &EnemyWorld::agent_stuck_total);
	ClassDB::bind_method(D_METHOD("agent_blocked_total", "handle"), &EnemyWorld::agent_blocked_total);
	ClassDB::bind_method(D_METHOD("agent_yield_total", "handle"), &EnemyWorld::agent_yield_total);
	ClassDB::bind_method(D_METHOD("get_agent_count"), &EnemyWorld::get_agent_count);
	ClassDB::bind_method(D_METHOD("get_last_step_usec"), &EnemyWorld::get_last_step_usec);

	BIND_ENUM_CONSTANT(AGENT_IDLE);
	BIND_ENUM_CONSTANT(AGENT_MOVE);
	BIND_ENUM_CONSTANT(AGENT_BLOCKED);
	BIND_ENUM_CONSTANT(AGENT_YIELD);
	BIND_ENUM_CONSTANT(AGENT_HOLD);
}
