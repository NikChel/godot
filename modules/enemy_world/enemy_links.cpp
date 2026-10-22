#include "enemy_links.h"

#include "core/math/math_funcs.h"
#include "core/math/vector2.h"

void EnemyLinks::set(const PackedVector3Array &p_starts, const PackedVector3Array &p_ends, const PackedInt32Array &p_kinds) {
	links.clear();
	closed_ids.clear();
	const int n = (int)MIN(p_starts.size(), MIN(p_ends.size(), p_kinds.size()));
	links.resize(n);
	for (int i = 0; i < n; i++) {
		Link &l = links[i];
		l = Link();
		l.start = p_starts[i];
		l.end = p_ends[i];
		l.kind = p_kinds[i];
	}
}

void EnemyLinks::claim(int p_l, int p_body) {
	links[p_l].busy = p_body;
	links[p_l].blocked_t = 0.0f;
}

void EnemyLinks::release(int p_l, int p_body) {
	if (valid(p_l) && links[p_l].busy == p_body) {
		links[p_l].busy = -1;
	}
}

bool EnemyLinks::blocked(int p_l, uint64_t p_tick, real_t p_dt) {
	Link &l = links[p_l];
	if (l.closed || l.blocked_tick == p_tick) {
		return false;
	}
	l.blocked_tick = p_tick;
	l.blocked_t += p_dt;
	if (l.blocked_t < CLOSE_S) {
		return false;
	}
	l.closed = true;
	l.blocked_t = 0.0f;
	l.open_t = 0.0f;
	closed_ids.push_back(p_l);
	return true;
}

bool EnemyLinks::open_tick(int p_l, bool p_free, real_t p_dt) {
	Link &l = links[p_l];
	l.open_t = p_free ? l.open_t + p_dt : 0.0f;
	if (l.open_t < OPEN_S) {
		return false;
	}
	l.closed = false;
	l.open_t = 0.0f;
	closed_ids.erase(p_l);
	return true;
}

void EnemyLinks::begin(Motion &r_m) {
	const Vector3 d = r_m.b - r_m.a;
	const real_t flat = Vector2(d.x, d.z).length();
	const real_t walk = MAX(r_m.walk, 0.1f);
	switch (r_m.kind) {
		case KIND_JUMP: {
			const real_t disc = r_m.impulse * r_m.impulse - 2.0f * r_m.gravity * d.y;
			if (r_m.impulse > 0.0f && disc >= 0.0f) {
				r_m.duration = (r_m.impulse + Math::sqrt(disc)) / r_m.gravity;
				return;
			}
			r_m.kind = d.y > 0.0f ? KIND_CLIMB : KIND_DROP;
			begin(r_m);
		} break;
		case KIND_CLIMB:
			r_m.duration = MAX(d.y, 0.0f) / CLIMB_SPEED + flat / walk;
			break;
		default:
			r_m.duration = flat / walk + Math::sqrt(2.0f * MAX(-d.y, 0.0f) / r_m.gravity);
			break;
	}
}

Vector3 EnemyLinks::at(const Motion &p_m, real_t p_t) {
	const Vector3 &a = p_m.a;
	const Vector3 &b = p_m.b;
	const real_t t = CLAMP(p_t, 0.0f, p_m.duration);
	if (t >= p_m.duration) {
		return b;
	}
	switch (p_m.kind) {
		case KIND_JUMP: {
			Vector3 p = a.lerp(b, t / p_m.duration);
			p.y = a.y + p_m.impulse * t - 0.5f * p_m.gravity * t * t;
			return p;
		}
		case KIND_CLIMB: {
			const real_t t_up = MAX(b.y - a.y, 0.0f) / CLIMB_SPEED;
			if (t < t_up) {
				return Vector3(a.x, a.y + CLIMB_SPEED * t, a.z);
			}
			const Vector3 top(a.x, b.y, a.z);
			return top.lerp(b, (t - t_up) / MAX(p_m.duration - t_up, 1e-4f));
		}
		default: {
			const real_t t_go = Vector2(b.x - a.x, b.z - a.z).length() / MAX(p_m.walk, 0.1f);
			if (t < t_go) {
				return a.lerp(Vector3(b.x, a.y, b.z), t / t_go);
			}
			const real_t s = t - t_go;
			return Vector3(b.x, MAX(a.y - 0.5f * p_m.gravity * s * s, b.y), b.z);
		}
	}
}
