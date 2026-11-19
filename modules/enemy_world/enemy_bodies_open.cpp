#include "enemy_bodies.h"

#include "core/math/math_funcs.h"
#include "scene/3d/node_3d.h"

// Пространство снаружи и поля сетки полёта по пространствам (план 2026-11-19-enemies-outside, шаг 1; заголовок
// EnemyBodies — «Пространство снаружи»).

int EnemyBodies::space_add_open(Node3D *p_root) {
	ERR_FAIL_NULL_V(p_root, -1);
	ERR_FAIL_COND_V_MSG(spaces.is_empty(), -1, "EnemyBodies: setup first (space 0 — the ship).");
	Space s;
	s.node_id = p_root->get_instance_id();
	s.open = true;
	spaces.push_back(s);
	_space_xforms();
	return (int)spaces.size() - 1;
}

void EnemyBodies::space_set_flight(int p_space, const Ref<EnemyFlightFields> &p_fields) {
	if (p_space == 0) {
		flight = p_fields;
		return;
	}
	ERR_FAIL_INDEX(p_space, (int)spaces.size());
	spaces[p_space].fields = p_fields;
}

void EnemyBodies::space_set_flow(int p_space, const Vector3 &p_velocity, real_t p_halflife) {
	ERR_FAIL_INDEX(p_space, (int)spaces.size());
	Space &s = spaces[p_space];
	s.flow = p_halflife >= 0.0f ? p_velocity : Vector3();
	s.flow_hl = p_halflife;
}

bool EnemyBodies::space_is_open(int p_space) const {
	return p_space >= 0 && p_space < (int)spaces.size() && spaces[p_space].open;
}

// Поля сетки полёта пространства: 0 — set_flight, остальные — space_set_flight; нет — nullptr.
EnemyFlightFields *EnemyBodies::_fields(int p_space) const {
	if (p_space == 0) {
		return flight.ptr();
	}
	return p_space > 0 && p_space < (int)spaces.size() ? spaces[p_space].fields.ptr() : nullptr;
}

bool EnemyBodies::_open(const Body &p_b) const {
	return space_is_open(p_b.space);
}

// Тело в пространстве снаружи с включённым потоком.
bool EnemyBodies::_flowing(const Body &p_b) const {
	return _open(p_b) && spaces[p_b.space].flow_hl >= 0.0f;
}

// Ввод полёта в потоке (заголовок): p_in — ввод мозга без потока (направление хода, длина — доля скорости полёта; срез
// сеткой её уменьшает), p_cap — ход к точке остановки не быстрее, м/с (INF — без предела). Путевая скорость по ходу d —
// полной тягой s = f·d + √(v² − |f|² + (f·d)²) (тяга v, поток f), не быстрее p_cap; ввод — (d · путевая − f) / v. Без хода —
// держится на месте (ввод −f / v); оглушён — ввода нет (сносит); поток сильнее хода — полная тяга по ходу. Потока нет —
// ввод как есть.
Vector3 EnemyBodies::_flow_input(const Body &p_b, const Vector3 &p_in, real_t p_cap) const {
	if (!_flowing(p_b) || p_b.fly_speed <= 1e-4f) {
		return p_in;
	}
	if (p_b.brain == BRAIN_STAGGER) {
		return Vector3();
	}
	const real_t v = p_b.fly_speed;
	const Vector3 f = spaces[p_b.space].flow / v; // Поток в долях скорости полёта
	const real_t m = p_in.length();
	if (m < 1e-4f) {
		return (-f).limit_length(1.0f);
	}
	const Vector3 d = p_in / m;
	const real_t fd = f.dot(d);
	const real_t disc = 1.0f - f.length_squared() + fd * fd;
	if (disc < 0.0f) {
		return d;
	}
	const real_t s = fd + Math::sqrt(disc); // Путевая скорость полной тягой, доли v
	if (s <= 0.0f) {
		return d;
	}
	const real_t cap = p_cap / v;
	const real_t share = MIN(1.0f, m / MAX(MIN(1.0f, cap), 1e-4f)); // Доля, оставленная срезом сеткой (без среза — 1)
	return (d * (MIN(s, cap) * share) - f).limit_length(1.0f);
}
