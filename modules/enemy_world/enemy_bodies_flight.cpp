#include "enemy_bodies.h"

#include "core/config/engine.h"
#include "core/math/math_funcs.h"
#include "core/os/os.h"
#include "scene/3d/physics/character_body_3d.h"
#include "scene/resources/3d/world_3d.h"

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

void EnemyBodies::body_set_hover(int p_h, real_t p_height, real_t p_climb_speed) {
	ERR_FAIL_COND(!_valid(p_h));
	Body &b = bodies[p_h];
	b.hover_h = MAX(p_height, 0.0f);
	b.climb_speed = p_climb_speed;
	b.rays_known = false;
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

// Сетка полёта пространства тела готова (корабль — set_flight, снаружи — space_set_flight; подвижные — нет).
bool EnemyBodies::_grid_ok(const Body &p_b) const {
	const EnemyFlightFields *f = _fields(p_b.space);
	return f != nullptr && f->is_ready();
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

// Номер поля точки (летящий к точке пробы / портала): по её клетке 0.5 м, отрицательный — номера игроков (ObjectID узлов)
// положительные.
static int64_t fly_point_id(const Vector3 &p_local) {
	const int64_t x = (int64_t)Math::round(p_local.x * 2.0f) & 0xFFFFF;
	const int64_t y = (int64_t)Math::round(p_local.y * 2.0f) & 0xFFFFF;
	const int64_t z = (int64_t)Math::round(p_local.z * 2.0f) & 0xFFFFF;
	return -1 - (x | (y << 20) | (z << 40));
}

// Начало тика полёта: центры летящих всех пространств на начало тика (мир; очередь и расстояние между летящими одного
// пространства — без порядка тел) и идут ли они (мозг ведёт к цели или точке); цели полей (сетка пространства тела
// готова) — цели мозга летящих и тел без тяжести (взлетать ли — путь лётом) и точки летящих к точке (портал: путь в
// обход преград; точка в прямой видимости по клеткам — без поля: летит прямо, слот поля не занят), центры в системе
// пространства тела.
void EnemyBodies::_flight_targets(uint64_t p_now) {
	fly_pts.clear();
	fly_h.clear();
	fly_s.clear();
	fly_go.clear();
	for (uint32_t h = 0; h < bodies.size(); h++) {
		const Body &b = bodies[h];
		if (!b.used || b.dead) {
			continue;
		}
		Vector3 center; // Центр тела (мир; летящее не куклой)
		bool center_ok = false;
		if (b.move == MOVE_FLY && !b.ragdoll) {
			Node3D *n = Object::cast_to<Node3D>(ObjectDB::get_instance(b.body_id));
			if (n != nullptr) {
				center = n->get_global_position() + b.up * CENTER_H;
				center_ok = true;
				fly_pts.push_back(center);
				fly_h.push_back((int)h);
				fly_s.push_back(b.space);
				fly_go.push_back(b.moving ? 1 : 0);
			}
		}
		EnemyFlightFields *f = _fields(b.space);
		if (f == nullptr || !f->is_ready() || b.space >= (int)spaces.size() || (b.move != MOVE_FLY && !b.zero_g)) {
			continue;
		}
		const Space &sp = spaces[b.space];
		if (b.point_set) {
			if (b.move == MOVE_FLY) {
				const Vector3 c = sp.inv.xform(b.world + b.up * CENTER_H);
				if (!center_ok || !f->get_grid()->line_free(sp.inv.xform(center), c)) {
					f->want(fly_point_id(c), c, p_now);
				}
			}
			continue;
		}
		const int ti = _player_index(b.target);
		if (ti < 0 || !_player_alive(ti)) {
			continue;
		}
		f->want((int64_t)b.target, sp.inv.xform(players_pos[ti] + b.up * CENTER_H), p_now);
	}
}

// Пересчёт полей всех пространств с готовой сеткой полёта: общий бюджет, первое — по кругу (каждому по очереди первым),
// следующему — остаток.
void EnemyBodies::flight_step(int p_budget_usec) {
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	const uint64_t now = Engine::get_singleton()->get_physics_frames();
	int with[8];
	int n = 0;
	const int count = MAX((int)spaces.size(), 1);
	for (int s = 0; s < count && n < 8; s++) {
		const EnemyFlightFields *f = _fields(s);
		if (f != nullptr && f->is_ready()) {
			with[n++] = s;
		}
	}
	for (int k = 0; k < n; k++) {
		const int64_t left = (int64_t)p_budget_usec - (int64_t)(OS::get_singleton()->get_ticks_usec() - t0);
		if (left <= 0) {
			break;
		}
		_fields(with[(flight_rr + (uint32_t)k) % (uint32_t)n])->step((int)left, now);
	}
	flight_rr++;
	last_flight_usec = OS::get_singleton()->get_ticks_usec() - t0;
}

// Взлететь ли (тело идёт; заголовок; FLIGHT_ALWAYS взлетает в начале тика тела): без тяжести — оторван от пола дольше
// AIRBORNE_S, пеший путь кончается дальше FOOT_GAP от цели или лётом в FLY_GAIN раз короче.
bool EnemyBodies::_wants_fly(const Body &p_b, CharacterBody3D *p_body, uint64_t p_now) {
	(void)p_body;
	(void)p_now;
	if (!_grid_ok(p_b) || p_b.link_state != LINK_NONE || p_b.ragdoll) {
		return false;
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

// PlayerFreeMove.step (покой p_rest: в невесомости — ноль, снаружи — поток): горизонталь и вертикаль по верху тела
// отдельно; ось с вводом — к вводу × скорость + покой с полураспадом разгона, без ввода — к покою с полураспадом p_rest_hl
// (торможение или снос потоком; < 0 — без него).
Vector3 EnemyBodies::_free_step(const Vector3 &p_v, const Vector3 &p_in, const Vector3 &p_up, real_t p_speed, real_t p_accel_hl, const Vector3 &p_rest, real_t p_rest_hl, real_t p_dt) {
	const real_t vy = p_v.dot(p_up);
	Vector3 h = p_v - p_up * vy;
	const real_t lift = p_in.dot(p_up);
	const Vector3 in_h = p_in - p_up * lift;
	const real_t rest_y = p_rest.dot(p_up);
	const Vector3 rest_h = p_rest - p_up * rest_y;
	if (in_h.length_squared() > 1e-6f) {
		const real_t k = CLAMP(1.0f - Math::exp(-FLY_LN2 * p_dt / MAX(p_accel_hl, 1e-4f)), 0.0f, 1.0f);
		h = h.lerp(in_h * p_speed + rest_h, k);
	} else if (p_rest_hl >= 0.0f) {
		h = h.lerp(rest_h, p_rest_hl > 0.0f ? CLAMP(1.0f - Math::exp(-FLY_LN2 * p_dt / p_rest_hl), 0.0f, 1.0f) : 1.0f);
	}
	real_t y = vy;
	if (Math::abs(lift) > 1e-3f) {
		y = _damp(y, lift * p_speed + rest_y, p_accel_hl, p_dt);
	} else if (p_rest_hl >= 0.0f) {
		y = _damp(y, rest_y, p_rest_hl, p_dt);
	}
	return h + p_up * y;
}

// Рабочая высота (полёт с тяжестью; заголовок): лучи пола и потолка от центра тела по слою мира (раз в HOVER_RAY_TICKS,
// высоты — вдоль верха в системе тела); цель — пол + hover_h, не выше потолка − CEIL_GAP, не ниже пола + CENTER_H.
// Возвращает ввод вверх / вниз (доля скорости полёта): скорость к цели — разница за HOVER_S, вверх не быстрее набора,
// вниз — скорости полёта. Высоту не держит (hover_h 0) или пола под собой нет — 0 (тормозит на месте).
real_t EnemyBodies::_hover_lift(Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, uint64_t p_now) {
	if (r_b.hover_h <= 0.0f || r_b.fly_speed <= 1e-4f) {
		return 0.0f;
	}
	const Vector3 up = r_b.up;
	const Vector3 center = p_body->get_global_position() + up * CENTER_H;
	const Transform3D inv = p_fx.affine_inverse();
	const Vector3 lup = p_fx.basis.xform_inv(up).normalized(); // Верх в системе тела
	const real_t y = inv.xform(center).dot(lup);
	if (!r_b.rays_known || (p_now + (uint64_t)r_b.slot) % HOVER_RAY_TICKS == 0) {
		PhysicsDirectSpaceState3D *ss = p_body->get_world_3d()->get_direct_space_state();
		PhysicsDirectSpaceState3D::RayParameters rq;
		rq.from = center;
		rq.collision_mask = FLOOR_LAYER;
		rq.exclude.insert(p_body->get_rid());
		PhysicsDirectSpaceState3D::RayResult hit;
		rq.to = center - up * HOVER_PROBE;
		r_b.floor_hit = ss->intersect_ray(rq, hit);
		if (r_b.floor_hit) {
			r_b.floor_y = inv.xform(hit.position).dot(lup);
		}
		rq.to = center + up * HOVER_PROBE;
		r_b.ceil_hit = ss->intersect_ray(rq, hit);
		if (r_b.ceil_hit) {
			r_b.ceil_y = inv.xform(hit.position).dot(lup);
		}
		r_b.rays_known = true;
	}
	if (!r_b.floor_hit) {
		return 0.0f;
	}
	real_t t = r_b.floor_y + r_b.hover_h;
	if (r_b.ceil_hit) {
		t = MIN(t, r_b.ceil_y - CEIL_GAP);
	}
	t = MAX(t, r_b.floor_y + CENTER_H);
	const real_t climb = r_b.climb_speed >= 0.0f ? r_b.climb_speed : r_b.fly_speed;
	return CLAMP((t - y) / HOVER_S, -r_b.fly_speed, climb) / r_b.fly_speed;
}

// Полураспад скорости, доходящей до 95% за p_time (PlayerFreeMove.halflife).
real_t EnemyBodies::_halflife(real_t p_time) {
	return p_time / DRIFT_TAUS * FLY_LN2;
}

// Уступка игрокам лётом (правило EnemyBackoff по капсулам вдоль верха тела: отрезки CAPSULE_LO…CAPSULE_HI над ногами
// тела и точки): ближе касания + зазора — ход к точке снят из r_v, скорость уступки — от точки, не длиннее bo_speed;
// идущая на тело точка в касании (глубина не больше зазора; глубже — точка тело обогнала) — плюс перенос вдоль нормали
// (EnemyBackoff::carry_along). Точка своего пространства (тело едет в кабине-владельце) — мимо.
Vector3 EnemyBodies::_fly_backoff(const Body &p_b, const Vector3 &p_pos, Vector3 &r_v) const {
	Vector3 push;
	Vector3 carry;
	const bool moving = !prio_vels.is_empty();
	const bool owned = !prio_spaces.is_empty();
	const Vector3 up = p_b.up;
	const real_t span = CAPSULE_HI - CAPSULE_LO;
	for (int i = 0; i < prio_points.size(); i++) {
		if (owned && prio_spaces[i] == p_b.space) {
			continue;
		}
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
		if (moving && r - l <= p_b.bo_gap) {
			EnemyBackoff::carry_along(carry, n, prio_vels[i]);
		}
	}
	push = push.limit_length(p_b.bo_speed);
	if (moving) {
		push += carry;
	}
	return push;
}

// Дальний ярус лётом: без физики — центр тела по свободным клеткам (шаг в занятую клетку снят по оси; из занятой — в
// свободную или в ту же). p_free_up — ход по оси верха без клеток: к рабочей высоте (её границы — лучи пола и потолка;
// коробка клетки — под ходока, ниже рабочей высоты коптера в невысоком помещении).
void EnemyBodies::_fly_far(Body &r_b, CharacterBody3D *p_body, const Vector3 &p_v, bool p_free_up, real_t p_dt) {
	const Space &sp = spaces[r_b.space];
	const Vector3 c = sp.inv.xform(p_body->get_global_position() + r_b.up * CENTER_H);
	Vector3 vl = sp.inv.basis.xform(p_v);
	const EnemyFlightGrid *g = _fields(r_b.space)->get_grid().ptr();
	const real_t cs = g->get_cell();
	const Vector3 org = g->get_origin();
	int k_up = -1; // Ось сетки вдоль верха тела
	if (p_free_up) {
		const Vector3 lu = sp.inv.basis.xform(r_b.up).normalized();
		const int k = lu.abs().max_axis_index();
		k_up = Math::abs(lu[k]) > 0.99f ? k : -1;
	}
	Vector3 q = c;
	for (int k = 0; k < 3; k++) {
		Vector3 t = q;
		t[k] += vl[k] * p_dt;
		const bool same = Math::floor((t[k] - org[k]) / cs) == Math::floor((q[k] - org[k]) / cs);
		if (same || k == k_up || g->is_free(t)) {
			q = t;
		} else {
			vl[k] = 0.0f;
		}
	}
	p_body->set_global_position(sp.xf.xform(q) - r_b.up * CENTER_H);
	p_body->set_velocity(sp.xf.basis.xform(vl));
}

// Ход без пути по сетке полёта (отход): составляющие направления p_dir по осям сетки, ведущие из клетки центра в занятую
// соседнюю — или в занятую клетку в пределах сноса торможения своей скорости p_vel по этой оси (не меньше полуклетки), —
// сняты (вдоль преграды — скользит, в неё — нет; останавливается до занятой, а не въезжает в неё сносом).
Vector3 EnemyBodies::_grid_clip(const Body &p_b, const Vector3 &p_center, const Vector3 &p_dir, const Vector3 &p_vel) const {
	const Space &sp = spaces[p_b.space];
	const EnemyFlightGrid *g = _fields(p_b.space)->get_grid().ptr();
	const Vector3 c = sp.inv.xform(p_center);
	Vector3 dl = sp.inv.basis.xform(p_dir);
	const Vector3 vl = sp.inv.basis.xform(p_vel);
	const real_t stop_s = MAX(p_b.fly_accel_time, p_b.fly_brake_time) / DRIFT_TAUS; // Снос до остановки — скорость × столько
	for (int k = 0; k < 3; k++) {
		if (Math::abs(dl[k]) < 1e-4f) {
			continue;
		}
		const real_t sign = dl[k] > 0.0f ? 1.0f : -1.0f;
		const real_t look = MAX(g->get_cell() * 0.5f, MAX(vl[k] * sign, 0.0f) * stop_s);
		Vector3 t = c;
		t[k] += sign * g->get_cell();
		Vector3 near = c;
		near[k] += sign * look;
		if (!g->is_free(t) || !g->is_free(near)) {
			dl[k] = 0.0f;
		}
	}
	return sp.xf.basis.xform(dl);
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

// Очередь и расстояние лётом (центры летящих пространства p_space на начало тика): другой летящий ближе FLY_HOLD_R впереди
// по пути к цели и ближе к ней — ждёт (r_hold; p_behind_still — и за стоящим, иначе только за идущим: fly_go); ближе
// FLY_SEP_R — скорость расхождения от него (FLY_SEP_GAIN на метр, не быстрее FLY_SEP_SPEED).
Vector3 EnemyBodies::_fly_crowd(int p_h, int p_space, const Vector3 &p_center, const Vector3 &p_goal, bool p_behind_still, bool &r_hold) const {
	r_hold = false;
	Vector3 sep;
	const Vector3 to = p_goal - p_center;
	const real_t my = to.length();
	for (uint32_t i = 0; i < fly_pts.size(); i++) {
		if (fly_h[i] == p_h || fly_s[i] != p_space) {
			continue;
		}
		const Vector3 d = fly_pts[i] - p_center;
		const real_t l = d.length();
		if (l < FLY_SEP_R && l > 1e-4f) {
			sep -= d * (MIN(FLY_SEP_GAIN * (FLY_SEP_R - l), FLY_SEP_SPEED) / l);
		}
		if (!r_hold && (p_behind_still || fly_go[i] != 0) && my > 1e-4f && l < FLY_HOLD_R && d.dot(to) > 0.5f * l * my && fly_pts[i].distance_to(p_goal) < my) {
			r_hold = true;
		}
	}
	return sep.limit_length(FLY_SEP_SPEED);
}

// Летит вне сетки полёта по сетке навигации (заголовок «Порталы для летящих»): подвижное пространство с готовой сеткой.
bool EnemyBodies::_fly_nav(const Body &p_b) const {
	return p_b.space > 0 && p_b.space < (int)spaces.size() && spaces[p_b.space].ready && spaces[p_b.space].crowd.is_valid();
}

// Точка пола под телом (мир): луч вниз от центра по слою мира на HOVER_PROBE; пола нет — ноги.
Vector3 EnemyBodies::_fly_ground(const Body &p_b, CharacterBody3D *p_body) const {
	const Vector3 pos = p_body->get_global_position();
	PhysicsDirectSpaceState3D::RayParameters rq;
	rq.from = pos + p_b.up * CENTER_H;
	rq.to = rq.from - p_b.up * HOVER_PROBE;
	rq.collision_mask = FLOOR_LAYER;
	rq.exclude.insert(p_body->get_rid());
	PhysicsDirectSpaceState3D::RayResult hit;
	return p_body->get_world_3d()->get_direct_space_state()->intersect_ray(rq, hit) ? hit.position : pos;
}

// Запрос маршрута / пути лётом (заголовок): тело движется; в сетке полёта — только гонится (к точке летит прямо):
// EVENT_ROUTE; по сетке навигации — гонится или летит к точке: EVENT_PATH (первый в пространстве — сразу). Раз в PATH_SLOTS
// тиков со сдвигом по слоту; начало — пол под телом, цель — точка мозга (координаты пространства).
void EnemyBodies::_fly_request(int p_h, Body &r_b, CharacterBody3D *p_body, bool p_nav, uint64_t p_now) {
	if (!r_b.moving || (!p_nav && (r_b.point_set || r_b.brain != BRAIN_CHASE))) {
		return;
	}
	const bool first = p_nav && r_b.fly_req_space != r_b.space;
	if (!first && (p_now + (uint64_t)r_b.slot) % PATH_SLOTS != 0) {
		return;
	}
	const Space &sp = spaces[r_b.space];
	r_b.path_from = sp.inv.xform(_fly_ground(r_b, p_body));
	r_b.goal = sp.inv.xform(r_b.world);
	r_b.path_requests++;
	if (p_nav) {
		r_b.fly_req_space = r_b.space;
		_emit(EVENT_PATH, p_h);
	} else {
		_emit(EVENT_ROUTE, p_h);
	}
}

// Куда лететь по пути сетки навигации (мир, на высоте центра — ход по горизонтали): точка пути, к которой летит (ближе
// FLY_CORNER_R по горизонтали, позади тела по ходу следующего отрезка дальше FLY_CORNER_R / 2 или отрезок до следующей
// нулевой — следующая: тело вне сетки, начало пути — его проекция на край сетки, как у летящего из кабины лифта в
// проём); последняя — сама цель (r_last). Пути ещё нет — центр (висит); путь пустой — цель.
Vector3 EnemyBodies::_fly_corner(Body &r_b, const Vector3 &p_center, const Vector3 &p_goal, bool &r_last) {
	const Vector3 up = r_b.up;
	r_last = true;
	if (r_b.fly_path_space != r_b.space) {
		return p_center;
	}
	Vector3 a = p_goal;
	const uint32_t n = r_b.fly_path.size();
	if (n > 0) {
		const Space &sp = spaces[r_b.space];
		const Vector3 c = sp.inv.xform(p_center);
		const Vector3 lu = sp.inv.basis.xform(up).normalized();
		while (r_b.fly_path_i + 1 < n) {
			Vector3 d = r_b.fly_path[r_b.fly_path_i] - c;
			d -= lu * d.dot(lu);
			if (d.length() >= FLY_CORNER_R) {
				Vector3 seg = r_b.fly_path[r_b.fly_path_i + 1] - r_b.fly_path[r_b.fly_path_i];
				seg -= lu * seg.dot(lu);
				const real_t sl = seg.length();
				if (sl >= FLY_CORNER_R * 0.1f && -seg.dot(d) / sl < FLY_CORNER_R * 0.5f) {
					break; // Не позади тела по ходу следующего отрезка (на угол — к ней, не срезая)
				}
			}
			r_b.fly_path_i++;
		}
		r_last = r_b.fly_path_i + 1 >= n;
		if (!r_last) {
			a = sp.xf.xform(r_b.fly_path[r_b.fly_path_i]);
		}
	}
	return a + up * (p_center - a).dot(up);
}

PackedVector3Array EnemyBodies::body_fly_path(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), PackedVector3Array());
	const Body &b = bodies[p_h];
	PackedVector3Array out;
	if (b.fly_path_space != b.space || b.space >= (int)spaces.size()) {
		return out;
	}
	const Transform3D &xf = spaces[b.space].xf;
	for (uint32_t i = b.fly_path_i; i < b.fly_path.size(); i++) {
		out.push_back(xf.xform(b.fly_path[i]));
	}
	return out;
}

// Тик летящего тела: мозг тот же; ввод — к точке пути сетки полёта (центр цели; дальний бой с тяжестью — точка на рабочей
// высоте над целью), у цели слабеет и снят ближе остановки (дальний бой — дальность от центра цели); вне сетки полёта в
// пространстве с сеткой навигации — по горизонтали к точке её пути (_fly_corner); отход (дальний бой) — от цели без пути,
// с тяжестью по горизонтали, к точке отхода мозга медленнее; ни той, ни другой сетки (полёта не готова, пространство корабля) — не перемещается. С
// тяжестью: ход по пути сетки полёта — вертикаль пути, вверх не быстрее набора (весь ввод медленнее); по сетке навигации,
// висит и отходит — к рабочей высоте (_hover_lift). Скорость — закон полёта игрока плюс уступка игрокам; ближний ярус —
// move_and_slide в плавающем режиме, дальний — по клеткам; взгляд — к цели (огибает — по пути), наклон — к её центру;
// визуалу — без шага (скорость вдоль торса 0). Маршрут и путь — _fly_request. Снаружи (enemy_bodies_open.cpp): без
// рабочей высоты, ввод мозга — путевая скорость в потоке (_flow_input), покой закона — поток.
void EnemyBodies::_tick_fly(int p_h, Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, uint64_t p_now, real_t p_dt) {
	fly_count++;
	const Vector3 up = r_b.up;
	_brain(p_h, r_b, p_body, p_fx, p_now);
	const Vector3 pos = p_body->get_global_position();
	const Vector3 center = pos + up * CENTER_H;
	const bool grid = _grid_ok(r_b);
	EnemyFlightFields *fields = grid ? _fields(r_b.space) : nullptr;
	const bool nav = !grid && _fly_nav(r_b); // Вне сетки полёта — по сетке навигации пространства (кабина турели)
	const bool hold = !r_b.zero_g && !_open(r_b); // С тяжестью — рабочая высота и набор (снаружи — нет)
	const bool fire = r_b.fire_max > 0.0f && !r_b.point_set;
	Vector3 input; // Мир, длина ≤ 1 по горизонтали и по вертикали
	real_t cap = (real_t)Math::INF; // Ход к точке остановки не быстрее, м/с (путевая скорость в потоке)
	Vector3 sep; // Расхождение с летящими (мир)
	bool path = false; // Ход по пути сетки полёта (вертикаль — пути)
	const bool fight = r_b.brain == BRAIN_CHASE || r_b.brain == BRAIN_WINDUP || r_b.brain == BRAIN_RECOVER;
	Vector3 look = r_b.moving ? r_b.world - pos : r_b.face - pos; // Ноги к ногам = центр к центру
	_fly_request(p_h, r_b, p_body, nav, p_now);
	if (r_b.moving && (grid || nav)) {
		const real_t gh = (fire && hold && r_b.hover_h > 0.0f) ? r_b.hover_h : CENTER_H;
		const Vector3 goal = r_b.world + up * gh;
		const real_t gd = (goal - center).length();
		const real_t gap = fire ? (r_b.world + up * CENTER_H - center).length() - r_b.stop_dist : gd - (r_b.point_set ? POINT_STOP : FLY_STOP);
		Vector3 aim;
		bool last = true; // Точка пути — сама цель
		if (grid) {
			const Space &sp = spaces[r_b.space];
			Vector3 aim_l;
			real_t dist = gd;
			const Vector3 goal_l = sp.inv.xform(goal);
			const int64_t fid = r_b.point_set ? fly_point_id(goal_l) : (int64_t)r_b.target;
			// Поле цели готово, а от тела до неё по нему не достать (другая часть сетки: палуба за лифтом, игрок в кабине) —
			// висит, к цели не летит (маршрут — порталом); поля ещё нет — по прямой, как было
			const bool reach = fields->steer(fid, sp.inv.xform(center), goal_l, aim_l, dist) || !fields->field_ready(fid);
			aim = reach ? sp.xf.xform(aim_l) : center;
			r_b.fly_dist = reach ? dist : -1.0f;
			last = !reach || aim.distance_squared_to(goal) <= 1e-4f;
		} else {
			r_b.fly_dist = -1.0f;
			aim = _fly_corner(r_b, center, goal, last);
		}
		bool queue = false;
		sep = _fly_crowd(p_h, r_b.space, center, goal, !fire, queue); // Стрелок не ждёт за стоящим (шапка)
		if (gap > 0.0f && !queue) {
			const Vector3 d = aim - center;
			const real_t dl = d.length();
			if (dl > 1e-4f) {
				const real_t ramp = CLAMP(gap / MAX(r_b.fly_speed * ARRIVE_S, 0.01f), 0.0f, 1.0f);
				input = d * (ramp / dl);
				cap = gap / ARRIVE_S;
				path = grid;
			}
			if (!last) {
				look = aim - center; // Огибает: смотрит по пути
			}
		}
	} else {
		r_b.fly_dist = -1.0f;
		if (r_b.away && (grid || nav)) { // Отход: от центра цели (с тяжестью — по горизонтали), лицом к ней
			Vector3 a = center - (r_b.world + up * CENTER_H);
			// К точке отхода (stop_dist мозга: минимальная + FIRE_STOP_IN) — медленнее, как подлёт к точке остановки: порог
			// минимальной проходит с малым ходом и встаёт рядом, не вылетая за укрытие
			const real_t ramp = CLAMP((r_b.stop_dist - a.length()) / MAX(r_b.fly_speed * ARRIVE_S, 0.01f), 0.0f, 1.0f);
			cap = MAX(r_b.stop_dist - a.length(), 0.0f) / ARRIVE_S;
			if (hold) {
				a -= up * a.dot(up);
			}
			if (a.length_squared() < 1e-4f) { // Цель прямо под / над телом: назад от взгляда
				a = -_forward(r_b, p_fx);
				a -= up * a.dot(up);
			}
			if (a.length_squared() > 1e-6f) {
				input = a.normalized() * ramp;
				if (grid) {
					input = _grid_clip(r_b, center, input, p_body->get_velocity()); // Отход не в занятые клетки (след кабины лифта за спиной)
				}
			}
		}
		if (fire) {
			bool queue = false;
			sep = _fly_crowd(p_h, r_b.space, center, center, false, queue); // Висящие и отходящие стрелки расходятся
		}
	}
	if (hold) {
		const real_t climb = r_b.climb_speed >= 0.0f ? r_b.climb_speed : r_b.fly_speed;
		if (path) {
			const real_t lift = input.dot(up) * r_b.fly_speed;
			if (lift > climb && lift > 1e-4f) {
				input *= climb / lift; // Вверх по пути не быстрее набора: весь ход медленнее, направление пути
			}
		} else {
			input += up * _hover_lift(r_b, p_body, p_fx, p_now);
		}
	}
	const Vector3 intent = input; // Ввод мозга без потока: желаемая скорость и состояние
	input = _flow_input(r_b, input, cap); // Снаружи: путевая скорость по ходу, без хода — держится против потока
	if (fight && look.length_squared() > 1e-6f) {
		const real_t lv = look.dot(up);
		r_b.pitch = Math::atan2(lv, (look - up * lv).length());
	} else {
		r_b.pitch = 0.0f;
	}
	_update_tier(r_b, pos);
	// Покой закона: снаружи с потоком — поток (сносит без ввода), иначе ноль с торможением профиля
	const bool flowing = _flowing(r_b);
	const Vector3 rest = flowing ? spaces[r_b.space].flow : Vector3();
	const real_t rest_hl = flowing ? spaces[r_b.space].flow_hl : (r_b.fly_brake_time < 0.0f ? -1.0f : _halflife(r_b.fly_brake_time));
	Vector3 v = _free_step(p_body->get_velocity() - r_b.backoff, input, up, r_b.fly_speed, _halflife(r_b.fly_accel_time), rest, rest_hl, p_dt);
	const Vector3 push = _fly_backoff(r_b, pos, v) + sep;
	r_b.backoff = push;
	v += push;
	if (r_b.tier == TIER_FAR) {
		_fly_far(r_b, p_body, v, hold && !path, p_dt);
	} else if (input.length_squared() < 1e-6f && v.length() < REST_SPEED && (p_now + (uint64_t)r_b.slot) % REST_CHECK_TICKS != 0) {
		p_body->set_velocity(Vector3()); // Покой лётом: висит без move_and_slide (полный шаг — раз в REST_CHECK_TICKS)
		rest_count++;
	} else {
		p_body->set_velocity(v);
		p_body->move_and_slide();
	}
	r_b.on_step = false;
	_lag(r_b, 0.0f, p_dt);
	_turn(r_b, p_body, look, up, p_dt);
	r_b.want_speed = intent.length() * r_b.fly_speed;
	r_b.state = intent.length_squared() > 1e-6f ? EnemyWorld::AGENT_MOVE : EnemyWorld::AGENT_IDLE;
	_publish(r_b, p_body, p_fx, Vector3(), Vector3(), false, p_now);
}
