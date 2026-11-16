#include "enemy_bodies.h"

#include "core/config/engine.h"
#include "core/math/math_funcs.h"
#include "core/os/os.h"
#include "scene/3d/physics/character_body_3d.h"

// Полёт тел ходоков (бриф 2026-11-14, срез 1; заголовок EnemyBodies — «Полёт»).

static const real_t FLY_LN2 = 0.6931471805599453f;
static const real_t DRIFT_TAUS = 3.0f; // GravityZone.DRIFT_TAUS: времена закона — до 95%
static const Vector3 FLY_UP = Vector3(0.0f, 1.0f, 0.0f);

void EnemyBodies::body_set_flight(int p_h, int p_flight, real_t p_speed, real_t p_accel_time, real_t p_brake_time) {
	ERR_FAIL_COND(!_valid(p_h));
	Body &b = bodies[p_h];
	b.flight = (uint8_t)CLAMP(p_flight, (int)FLIGHT_ZERO_G, (int)FLIGHT_ALWAYS);
	b.fly_speed = MAX(p_speed, 0.0f);
	b.fly_accel_time = MAX(p_accel_time, 0.01f);
	b.fly_brake_time = p_brake_time; // < 0 — без торможения (как free_inertia игрока)
}

int EnemyBodies::body_move(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), MOVE_WALK);
	return bodies[p_h].move;
}

Vector3 EnemyBodies::body_aim(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), Vector3(0.0f, 0.0f, -1.0f));
	const Body &b = bodies[p_h];
	return _aim(b, _frame_xform(b));
}

real_t EnemyBodies::body_pitch(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0.0f);
	return bodies[p_h].pitch;
}

real_t EnemyBodies::body_fly_dist(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), -1.0f);
	return bodies[p_h].fly_dist;
}

Vector3i EnemyBodies::body_flight_stats(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), Vector3i());
	const Body &b = bodies[p_h];
	return Vector3i(b.take_offs, b.landings, b.zero_g ? 1 : 0);
}

// Прицел удара (мир): взгляд (yaw вокруг верха системы) с наклоном pitch к цели.
Vector3 EnemyBodies::_aim(const Body &p_b, const Transform3D &p_fx) {
	return (p_fx.basis * Basis(FLY_UP, p_b.yaw) * Basis(Vector3(1.0f, 0.0f, 0.0f), p_b.pitch)).xform(Vector3(0.0f, 0.0f, -1.0f));
}

bool EnemyBodies::_grid_ok(const Body &p_b) const {
	return p_b.space == 0 && flight_grid.is_valid() && flight_grid->is_ready();
}

// Тяжесть в центре тела (зоны этого тика); вне зон — тяжесть мира (не невесомость).
void EnemyBodies::_zone_tick(Body &r_b, CharacterBody3D *p_body) {
	Vector3 g;
	r_b.zero_g = gzones.accel_at(p_body->get_global_position() + r_b.up * CENTER_H, g) && g.length_squared() < ZERO_G * ZERO_G;
}

bool EnemyBodies::zero_g_at(const Vector3 &p_world) {
	gzones.refresh(); // Вызов вне тика (GDScript, физ-тик): позы и тяжесть зон сейчас
	Vector3 g;
	return gzones.accel_at(p_world, g) && g.length_squared() < ZERO_G * ZERO_G;
}

// Цели полей этого тика: цели мозга летящих и тел без тяжести (взлетать ли — путь лётом) — центры, система корабля.
void EnemyBodies::_flight_targets(uint64_t p_now) {
	const Space &sp = spaces[0];
	for (uint32_t h = 0; h < bodies.size(); h++) {
		const Body &b = bodies[h];
		if (!b.used || b.dead || b.space != 0 || (b.move != MOVE_FLY && !b.zero_g) || b.point_set) {
			continue;
		}
		const int ti = _player_index(b.target);
		if (ti < 0 || !_player_alive(ti)) {
			continue;
		}
		flight_grid->want((int64_t)b.target, sp.inv.xform(players_pos[ti] + b.up * CENTER_H), p_now);
	}
}

void EnemyBodies::flight_step(int p_budget_usec) {
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	if (flight_grid.is_valid() && flight_grid->is_ready()) {
		flight_grid->step(p_budget_usec, Engine::get_singleton()->get_physics_frames());
	}
	last_flight_usec = OS::get_singleton()->get_ticks_usec() - t0;
}

// Взлететь ли (тело идёт; заголовок): FLIGHT_ALWAYS — всегда; без тяжести — оторван от пола дольше AIRBORNE_S, пеший путь
// кончается дальше FOOT_GAP от цели или лётом в FLY_GAIN раз короче.
bool EnemyBodies::_wants_fly(const Body &p_b, CharacterBody3D *p_body, uint64_t p_now) {
	(void)p_body;
	(void)p_now;
	if (!_grid_ok(p_b) || p_b.link_state != LINK_NONE || p_b.ragdoll) {
		return false;
	}
	if (p_b.flight == FLIGHT_ALWAYS) {
		return true;
	}
	if (!p_b.zero_g) {
		return false;
	}
	if ((uint64_t)p_b.air_ticks >= _ticks(AIRBORNE_S)) {
		return true;
	}
	if (!p_b.moving || p_b.point_set || !p_b.path_known) {
		return false;
	}
	return p_b.path_gap > FOOT_GAP || (p_b.fly_dist >= 0.0f && p_b.fly_dist * FLY_GAIN < p_b.path_len);
}

// Взлёт: связь и призрак сняты, агент толпы снят (летящего толпа не видит), тело — в плавающем режиме.
void EnemyBodies::_take_off(int p_h, Body &r_b, CharacterBody3D *p_body) {
	_ghost_tick(p_h, r_b, p_body, false);
	_link_release(p_h, r_b);
	r_b.link_state = LINK_NONE;
	r_b.link = -1;
	EnemyWorld *c = _crowd(r_b);
	if (c != nullptr && r_b.crowd >= 0) {
		c->agent_remove(r_b.crowd);
	}
	r_b.crowd = -1;
	r_b.move = MOVE_FLY;
	r_b.landing = false;
	r_b.air_ticks = 0;
	r_b.on_step = false;
	r_b.backoff = Vector3();
	r_b.hint = -1;
	r_b.path_known = false;
	p_body->set_motion_mode(CharacterBody3D::MOTION_MODE_FLOATING);
	r_b.take_offs++;
}

// Посадка (тяжесть вернулась): снова идёт — агент толпы пространства (путь — заново), режим с полом; до касания пола —
// ближний ярус (падает телом с тяжестью профиля).
void EnemyBodies::_land(Body &r_b, CharacterBody3D *p_body, uint64_t p_now) {
	r_b.move = MOVE_WALK;
	r_b.pitch = 0.0f;
	r_b.fly_dist = -1.0f;
	r_b.backoff = Vector3();
	p_body->set_motion_mode(CharacterBody3D::MOTION_MODE_GROUNDED);
	const Space &sp = spaces[r_b.space];
	if (sp.crowd.is_valid() && r_b.crowd < 0) {
		r_b.crowd = sp.crowd->agent_add(sp.inv.xform(p_body->get_global_position()));
		_set_backoff_agent(r_b);
	}
	r_b.landing = true;
	r_b.fall_end = p_now + _ticks(LAND_MAX_S);
	r_b.was_floor = false;
	r_b.hint = -1;
	r_b.landings++;
}

// PlayerFreeMove.step в невесомости (покой — ноль): горизонталь и вертикаль по верху тела отдельно; ось с вводом — к
// вводу × скорость с полураспадом разгона, без ввода — к нулю с полураспадом торможения (< 0 — без него).
Vector3 EnemyBodies::_free_step(const Vector3 &p_v, const Vector3 &p_in, const Vector3 &p_up, real_t p_speed, real_t p_accel_hl, real_t p_brake_hl, real_t p_dt) {
	const real_t vy = p_v.dot(p_up);
	Vector3 h = p_v - p_up * vy;
	const real_t lift = p_in.dot(p_up);
	const Vector3 in_h = p_in - p_up * lift;
	if (in_h.length_squared() > 1e-6f) {
		const real_t k = CLAMP(1.0f - Math::exp(-FLY_LN2 * p_dt / MAX(p_accel_hl, 1e-4f)), 0.0f, 1.0f);
		h = h.lerp(in_h * p_speed, k);
	} else if (p_brake_hl >= 0.0f) {
		h = h.lerp(Vector3(), p_brake_hl > 0.0f ? CLAMP(1.0f - Math::exp(-FLY_LN2 * p_dt / p_brake_hl), 0.0f, 1.0f) : 1.0f);
	}
	real_t y = vy;
	if (Math::abs(lift) > 1e-3f) {
		y = _damp(y, lift * p_speed, p_accel_hl, p_dt);
	} else if (p_brake_hl >= 0.0f) {
		y = _damp(y, 0.0f, p_brake_hl, p_dt);
	}
	return h + p_up * y;
}

// Полураспад скорости, доходящей до 95% за p_time (PlayerFreeMove.halflife).
real_t EnemyBodies::_halflife(real_t p_time) {
	return p_time / DRIFT_TAUS * FLY_LN2;
}

// Уступка игрокам лётом (правило EnemyBackoff по капсулам вдоль верха тела: отрезки CAPSULE_LO…CAPSULE_HI над ногами
// тела и точки): ближе касания + зазора — ход к точке снят из r_v, скорость уступки — от точки, не длиннее bo_speed.
Vector3 EnemyBodies::_fly_backoff(const Body &p_b, const Vector3 &p_pos, Vector3 &r_v) const {
	Vector3 push;
	const Vector3 up = p_b.up;
	const real_t span = CAPSULE_HI - CAPSULE_LO;
	for (int i = 0; i < prio_points.size(); i++) {
		const Vector4 &p = prio_points[i];
		const Vector3 d = p_pos - Vector3(p.x, p.y, p.z);
		const real_t dv = d.dot(up);
		const Vector3 dh = d - up * dv;
		const real_t gv = MAX(Math::abs(dv) - span, 0.0f);
		const Vector3 away = dh + up * (dv >= 0.0f ? gv : -gv); // Ближайшие точки отрезков
		const real_t l = away.length();
		const real_t r = p_b.bo_radius + p.w + p_b.bo_gap;
		if (l >= r) {
			continue;
		}
		Vector3 n;
		if (l > 1e-4f) {
			n = away / l;
		} else if (Math::abs(dv) > 1e-4f) {
			n = up * (dv > 0.0f ? 1.0f : -1.0f);
		} else {
			continue; // На оси точки: направления нет
		}
		const real_t toward = -r_v.dot(n);
		if (toward > 0.0f) {
			r_v += n * toward;
		}
		push += n * MIN(p_b.bo_gain * (r - l), p_b.bo_speed);
	}
	return push.limit_length(p_b.bo_speed);
}

// Дальний ярус лётом: без физики — центр тела по свободным клеткам (шаг в занятую клетку снят по оси; из занятой — в
// свободную или в ту же).
void EnemyBodies::_fly_far(Body &r_b, CharacterBody3D *p_body, const Vector3 &p_v, real_t p_dt) {
	const Space &sp = spaces[r_b.space];
	const Vector3 c = sp.inv.xform(p_body->get_global_position() + r_b.up * CENTER_H);
	Vector3 vl = sp.inv.basis.xform(p_v);
	const real_t cs = flight_grid->get_cell();
	const Vector3 org = flight_grid->get_origin();
	Vector3 q = c;
	for (int k = 0; k < 3; k++) {
		Vector3 t = q;
		t[k] += vl[k] * p_dt;
		const bool same = Math::floor((t[k] - org[k]) / cs) == Math::floor((q[k] - org[k]) / cs);
		if (same || flight_grid->is_free(t)) {
			q = t;
		} else {
			vl[k] = 0.0f;
		}
	}
	p_body->set_global_position(sp.xf.xform(q) - r_b.up * CENTER_H);
	p_body->set_velocity(sp.xf.basis.xform(vl));
}

// Досягаемость удара лётом: центр к центру (= ноги к ногам) ближе FLY_REACH, в секторе взгляда (цель над / под телом —
// без сектора).
bool EnemyBodies::_in_reach_fly(const Body &p_b, CharacterBody3D *p_body, const Transform3D &p_fx, const Vector3 &p_target) const {
	const Vector3 d = p_target - p_body->get_global_position();
	if (d.length() > FLY_REACH) {
		return false;
	}
	const Vector3 dh = d - p_b.up * d.dot(p_b.up);
	if (dh.length() < FLY_SECTOR_H) {
		return true;
	}
	return _forward(p_b, p_fx).dot(dh.normalized()) >= Math::cos(Math::deg_to_rad(SECTOR_DEG));
}

// Тик летящего тела: мозг тот же; ввод — к точке пути сетки (центр цели), у цели слабеет и снят ближе остановки; скорость
// — закон полёта игрока плюс уступка игрокам; ближний ярус — move_and_slide в плавающем режиме, дальний — по клеткам;
// взгляд — к цели, наклон — к её центру; визуалу — без шага (скорость вдоль торса 0).
void EnemyBodies::_tick_fly(int p_h, Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, uint64_t p_now, real_t p_dt) {
	fly_count++;
	const Vector3 up = r_b.up;
	_brain(p_h, r_b, p_body, p_fx, p_now);
	const Vector3 pos = p_body->get_global_position();
	const Vector3 center = pos + up * CENTER_H;
	Vector3 input; // Мир, длина ≤ 1
	const bool fight = r_b.brain == BRAIN_CHASE || r_b.brain == BRAIN_WINDUP || r_b.brain == BRAIN_RECOVER;
	Vector3 look = r_b.moving ? r_b.world - pos : r_b.face - pos; // Ноги к ногам = центр к центру
	if (r_b.moving) {
		const Vector3 goal = r_b.world + up * CENTER_H;
		const real_t gd = (goal - center).length();
		const real_t stop = r_b.point_set ? POINT_STOP : FLY_STOP;
		Vector3 aim = goal;
		real_t dist = gd;
		if (_grid_ok(r_b)) {
			const Space &sp = spaces[r_b.space];
			Vector3 aim_l;
			flight_grid->steer(r_b.point_set ? (int64_t)0 : (int64_t)r_b.target, sp.inv.xform(center), sp.inv.xform(goal), aim_l, dist);
			aim = sp.xf.xform(aim_l);
		}
		r_b.fly_dist = dist;
		if (gd > stop) {
			const Vector3 d = aim - center;
			const real_t dl = d.length();
			if (dl > 1e-4f) {
				const real_t ramp = CLAMP((gd - stop) / MAX(r_b.fly_speed * ARRIVE_S, 0.01f), 0.0f, 1.0f);
				input = d * (ramp / dl);
			}
			if (aim.distance_squared_to(goal) > 1e-4f) {
				look = aim - center; // Огибает: смотрит по пути
			}
		}
	} else {
		r_b.fly_dist = -1.0f;
	}
	if (fight && look.length_squared() > 1e-6f) {
		const real_t lv = look.dot(up);
		r_b.pitch = Math::atan2(lv, (look - up * lv).length());
	} else {
		r_b.pitch = 0.0f;
	}
	_update_tier(r_b, pos);
	const real_t brake_hl = r_b.fly_brake_time < 0.0f ? -1.0f : _halflife(r_b.fly_brake_time);
	Vector3 v = _free_step(p_body->get_velocity() - r_b.backoff, input, up, r_b.fly_speed, _halflife(r_b.fly_accel_time), brake_hl, p_dt);
	const Vector3 push = _fly_backoff(r_b, pos, v);
	r_b.backoff = push;
	v += push;
	if (r_b.tier == TIER_FAR) {
		_fly_far(r_b, p_body, v, p_dt);
	} else {
		p_body->set_velocity(v);
		p_body->move_and_slide();
	}
	r_b.on_step = false;
	_lag(r_b, 0.0f, p_dt);
	_turn(r_b, p_body, look, up, p_dt);
	r_b.want_speed = input.length() * r_b.fly_speed;
	r_b.state = input.length_squared() > 1e-6f ? EnemyWorld::AGENT_MOVE : EnemyWorld::AGENT_IDLE;
	_publish(r_b, p_body, p_fx, Vector3(), Vector3(), false, p_now);
}
