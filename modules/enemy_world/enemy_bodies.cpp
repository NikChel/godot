#include "enemy_bodies.h"

#include "core/config/engine.h"
#include "core/config/project_settings.h"
#include "core/math/math_funcs.h"
#include "core/object/class_db.h"
#include "core/os/os.h"
#include "scene/3d/physics/area_3d.h"
#include "scene/3d/physics/character_body_3d.h"
#include "scene/resources/3d/world_3d.h"

static const real_t LN2 = 0.6931471805599453f;
static const Vector3 UP_AXIS = Vector3(0.0f, 1.0f, 0.0f);

EnemyBodies::~EnemyBodies() {
	if (query_sphere.is_valid() && PhysicsServer3D::get_singleton()) {
		PhysicsServer3D::get_singleton()->free_rid(query_sphere);
	}
}

bool EnemyBodies::_valid(int p_h) const {
	return p_h >= 0 && p_h < (int)bodies.size() && bodies[p_h].used;
}

void EnemyBodies::setup(const Ref<EnemyWorld> &p_crowd, const Ref<EnemyViews> &p_views, Node3D *p_root) {
	crowd = p_crowd;
	views = p_views;
	root_id = p_root ? p_root->get_instance_id() : ObjectID();
	gravity = GLOBAL_GET("physics/3d/default_gravity");
	_root_xforms();
}

void EnemyBodies::_root_xforms() {
	Node3D *root = Object::cast_to<Node3D>(ObjectDB::get_instance(root_id));
	root_xf = root ? root->get_global_transform() : Transform3D();
	root_inv = root_xf.affine_inverse();
}

int EnemyBodies::body_add(CharacterBody3D *p_body, Area3D *p_item_zone, int p_view, int p_slot, real_t p_speed, real_t p_yaw) {
	ERR_FAIL_NULL_V(p_body, -1);
	int h;
	if (!free_handles.is_empty()) {
		h = free_handles[free_handles.size() - 1];
		free_handles.resize(free_handles.size() - 1);
	} else {
		h = (int)bodies.size();
		const uint32_t n = bodies.size() + 1;
		bodies.resize(n);
	}
	Body &b = bodies[h];
	b = Body();
	b.used = true;
	b.body_id = p_body->get_instance_id();
	b.zone_id = p_item_zone ? p_item_zone->get_instance_id() : ObjectID();
	b.view = p_view;
	b.slot = p_slot;
	b.speed = p_speed;
	b.yaw = p_yaw;
	b.up = p_body->get(SNAME("up_direction"));
	if (crowd.is_valid()) {
		_root_xforms();
		b.crowd = crowd->agent_add(root_inv.xform(p_body->get_global_position()));
	}
	return h;
}

void EnemyBodies::body_remove(int p_h) {
	ERR_FAIL_COND(!_valid(p_h));
	Body &b = bodies[p_h];
	_link_release(p_h, b);
	if (crowd.is_valid() && b.crowd >= 0) {
		crowd->agent_remove(b.crowd);
	}
	b = Body();
	free_handles.push_back(p_h);
}

void EnemyBodies::body_kill(int p_h) {
	ERR_FAIL_COND(!_valid(p_h));
	Body &b = bodies[p_h];
	if (b.dead) {
		return;
	}
	b.dead_stats = body_crowd_stats(p_h);
	b.dead = true;
	_link_release(p_h, b);
	b.link_state = LINK_NONE;
	b.link = -1;
	if (crowd.is_valid() && b.crowd >= 0) {
		crowd->agent_remove(b.crowd);
		b.crowd = -1;
	}
	if (views.is_valid() && b.view >= 0) {
		views->view_set_active(b.view, false);
	}
}

// Смена системы: тело остаётся на месте в мире, перенос считается от позы новой системы (FrameBody.frame).
void EnemyBodies::body_set_frame(int p_h, Node3D *p_frame) {
	ERR_FAIL_COND(!_valid(p_h));
	Body &b = bodies[p_h];
	b.frame_id = p_frame ? p_frame->get_instance_id() : ObjectID();
	b.frame_last = _frame_xform(b);
}

void EnemyBodies::body_set_point(int p_h, const Vector3 &p_world) {
	ERR_FAIL_COND(!_valid(p_h));
	Body &b = bodies[p_h];
	b.point_set = true;
	b.world = p_world;
	b.stop_dist = POINT_STOP;
	b.moving = true;
}

void EnemyBodies::body_set_path(int p_h, const PackedVector3Array &p_points, const Vector3 &p_target, const PackedInt32Array &p_links) {
	ERR_FAIL_COND(!_valid(p_h));
	const Body &b = bodies[p_h];
	if (crowd.is_valid() && b.crowd >= 0) {
		crowd->agent_set_path(b.crowd, p_points, p_target, p_links);
	}
}

void EnemyBodies::body_add_hits(int p_h, int p_hits) {
	ERR_FAIL_COND(!_valid(p_h));
	bodies[p_h].hits += p_hits;
}

void EnemyBodies::body_set_net_id(int p_h, int p_net_id) {
	ERR_FAIL_COND(!_valid(p_h));
	bodies[p_h].net_id = p_net_id;
}

real_t EnemyBodies::body_yaw(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0.0f);
	return bodies[p_h].yaw;
}

Vector3 EnemyBodies::body_forward(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), Vector3(0.0f, 0.0f, -1.0f));
	const Body &b = bodies[p_h];
	return _forward(b, _frame_xform(b));
}

Transform3D EnemyBodies::body_frame_xform(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), Transform3D());
	return _frame_xform(bodies[p_h]);
}

int EnemyBodies::body_state(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), EnemyWorld::AGENT_IDLE);
	return bodies[p_h].state;
}

int EnemyBodies::body_brain(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), BRAIN_IDLE);
	return bodies[p_h].brain;
}

bool EnemyBodies::body_moving(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), false);
	return bodies[p_h].moving;
}

int EnemyBodies::body_strikes(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0);
	return bodies[p_h].strikes;
}

int EnemyBodies::body_hits(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0);
	return bodies[p_h].hits;
}

int EnemyBodies::body_tier(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), TIER_NEAR);
	return bodies[p_h].tier;
}

real_t EnemyBodies::body_want_speed(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0.0f);
	return bodies[p_h].want_speed;
}

Vector3 EnemyBodies::body_path_from(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), Vector3());
	return bodies[p_h].path_from;
}

Vector3 EnemyBodies::body_goal(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), Vector3());
	return bodies[p_h].goal;
}

int EnemyBodies::body_path_requests(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), 0);
	return bodies[p_h].path_requests;
}

// x — застреваний, y — BLOCKED, z — уступок (после смерти — на момент смерти).
Vector3i EnemyBodies::body_crowd_stats(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), Vector3i());
	const Body &b = bodies[p_h];
	if (!crowd.is_valid() || b.crowd < 0) {
		return b.dead_stats;
	}
	return Vector3i(crowd->agent_stuck_total(b.crowd), crowd->agent_blocked_total(b.crowd), crowd->agent_yield_total(b.crowd));
}

int64_t EnemyBodies::body_net_tick(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), -1);
	return bodies[p_h].net_tick;
}

Vector3 EnemyBodies::body_net_pos(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), Vector3());
	return bodies[p_h].net_pos;
}

void EnemyBodies::set_players(const PackedVector3Array &p_pos, const PackedByteArray &p_alive, const PackedInt64Array &p_ids) {
	ERR_FAIL_COND(p_alive.size() != p_pos.size() || p_ids.size() != p_pos.size());
	players_pos = p_pos;
	players_alive = p_alive;
	players_id.resize((uint32_t)p_ids.size());
	for (int i = 0; i < p_ids.size(); i++) {
		players_id[i] = ObjectID((uint64_t)p_ids[i]);
	}
}

void EnemyBodies::set_nav_ready(bool p_ready) {
	nav_ready = p_ready;
}

void EnemyBodies::set_tier_override(int p_tier) {
	tier_override = p_tier;
}

void EnemyBodies::_emit(int p_kind, int p_h) {
	events.push_back(p_kind);
	events.push_back(p_h);
}

PackedInt32Array EnemyBodies::tick(real_t p_delta) {
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	events.clear();
	tier_counts[TIER_NEAR] = 0;
	tier_counts[TIER_FAR] = 0;
	rest_count = 0;
	_root_xforms();
	_links_tick(p_delta);
	const uint64_t now = Engine::get_singleton()->get_physics_frames();
	for (uint32_t h = 0; h < bodies.size(); h++) {
		Body &b = bodies[h];
		if (!b.used || b.dead) {
			continue;
		}
		CharacterBody3D *body = Object::cast_to<CharacterBody3D>(ObjectDB::get_instance(b.body_id));
		if (body == nullptr || !body->is_inside_tree()) {
			continue;
		}
		_tick_body((int)h, b, body, now, p_delta);
	}
	last_tick_usec = OS::get_singleton()->get_ticks_usec() - t0;
	return events;
}

// Порядок — EnemyWalker._physics_process (до этапа 2); связь — своя ветка (enemy_bodies_links.cpp).
void EnemyBodies::_tick_body(int p_h, Body &r_b, CharacterBody3D *p_body, uint64_t p_now, real_t p_dt) {
	const Transform3D fx = _carry(r_b, p_body);
	if (r_b.link_state == LINK_ON || r_b.link_state == LINK_LAND) {
		_link_move(p_h, r_b, p_body, fx, p_now, p_dt);
		return;
	}
	Vector3 want; // Мир
	uint8_t st = EnemyWorld::AGENT_IDLE;
	bool has_goal = false;
	Vector3 goal; // Корень карты
	_brain(p_h, r_b, p_body, fx, p_now);
	if (crowd.is_valid() && r_b.crowd >= 0 && nav_ready) {
		if (r_b.moving) {
			has_goal = true;
			goal = root_inv.xform(r_b.world);
		}
		const Vector3 w = crowd->agent_velocity(r_b.crowd); // По входам на начало тика
		st = (uint8_t)crowd->agent_state(r_b.crowd);
		const int at_link = st == EnemyWorld::AGENT_LINK ? crowd->agent_link(r_b.crowd) : -1;
		// Переспросить путь: толпа хочет; у входа закрытой связи; связь закрылась / открылась (стоящие)
		const bool repath = crowd->agent_wants_path(r_b.crowd) || (links.valid(at_link) && links.closed(at_link)) ||
				(r_b.path_epoch != nav_epoch && (st == EnemyWorld::AGENT_HOLD || st == EnemyWorld::AGENT_LINK));
		if (has_goal && repath && (p_now + (uint64_t)r_b.slot) % PATH_SLOTS == 0) {
			r_b.path_from = root_inv.xform(p_body->get_global_position());
			r_b.goal = goal;
			r_b.path_requests++;
			r_b.path_epoch = nav_epoch;
			_emit(EVENT_PATH, p_h);
		}
		want = root_xf.basis.xform(w);
	}
	if (st == EnemyWorld::AGENT_LINK && has_goal) {
		_link_wait(p_h, r_b, p_body, p_now, p_dt); // Ушёл на связь — двигается со следующего тика
	} else if (r_b.link_state == LINK_WAIT) {
		r_b.link_state = LINK_NONE;
		r_b.link = -1;
	}
	const Vector3 pos0 = p_body->get_global_position();
	const Vector3 look = r_b.moving ? r_b.world : r_b.face;
	const bool hold = st == EnemyWorld::AGENT_HOLD; // Стоит в толпе — лицом к цели, не по толчкам соседей
	Vector3 face = (r_b.moving && !hold && want.length() > FACE_MOVE_SPEED) ? want : look - pos0;
	if (r_b.link_state != LINK_NONE && links.valid(r_b.link)) {
		face = root_xf.xform(links.end(r_b.link)) - pos0; // У входа связи — лицом к её концу
	}
	_update_tier(r_b, pos0);
	Vector3 vel;
	if (r_b.tier == TIER_FAR) {
		vel = _move_far(r_b, p_body, want, face, p_dt);
	} else if (_rest(r_b, p_body, want, st, p_now)) {
		rest_count++;
		_lag(r_b, 0.0f, p_dt);
		_turn(r_b, p_body, face, r_b.up, p_dt);
	} else {
		_move_near(r_b, p_body, want, face, p_dt);
		vel = p_body->get_real_velocity();
		Area3D *zone = Object::cast_to<Area3D>(ObjectDB::get_instance(r_b.zone_id));
		if (zone != nullptr && zone->has_overlapping_bodies()) {
			_emit(EVENT_ITEMS, p_h);
		}
	}
	r_b.want_speed = want.length();
	r_b.state = st;
	_publish(r_b, p_body, fx, vel, goal, has_goal, p_now);
}

// Конец тика тела: вход толпе, поза и шаг визуалу, поза тика сети, счётчик яруса.
void EnemyBodies::_publish(Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, const Vector3 &p_vel, const Vector3 &p_goal, bool p_has_goal, uint64_t p_now) {
	const Vector3 pos = p_body->get_global_position();
	if (crowd.is_valid() && r_b.crowd >= 0) {
		crowd->agent_commit(r_b.crowd, root_inv.xform(pos), p_goal, p_has_goal, r_b.stop_dist, r_b.speed);
	}
	const Basis torso = p_fx.basis * Basis(UP_AXIS, r_b.yaw);
	r_b.view_speed = p_vel.dot(torso.xform(Vector3(0.0f, 0.0f, -1.0f)));
	if (views.is_valid() && r_b.view >= 0) {
		views->view_commit(r_b.view, Transform3D(torso, pos), r_b.lag_curr, r_b.view_speed);
	}
	r_b.net_tick = (int64_t)p_now;
	r_b.net_pos = p_fx.affine_inverse().xform(pos);
	tier_counts[r_b.tier]++;
}

Transform3D EnemyBodies::_frame_xform(const Body &p_b) const {
	if (p_b.frame_id.is_null()) {
		return Transform3D();
	}
	Object *f = ObjectDB::get_instance(p_b.frame_id);
	return f ? (Transform3D)f->call(SNAME("query_xform")) : Transform3D();
}

// Начало тика тела: перенос вместе с системой (FrameBody.carry). Возвращает позу системы этого тика.
Transform3D EnemyBodies::_carry(Body &r_b, CharacterBody3D *p_body) {
	const Transform3D now = _frame_xform(r_b);
	if (now != r_b.frame_last) {
		const Vector3 local = r_b.frame_last.affine_inverse().xform(p_body->get_global_position());
		const Vector3 vel = r_b.frame_last.basis.transposed().xform(p_body->get_velocity());
		p_body->set_global_transform(Transform3D(now.basis, now.xform(local)));
		p_body->set_velocity(now.basis.xform(vel));
		r_b.up = now.basis.get_column(1).normalized();
		p_body->set(SNAME("up_direction"), r_b.up);
		r_b.frame_last = now;
	}
	return now;
}

// Взгляд (мир, по верху тела): −Z торса.
Vector3 EnemyBodies::_forward(const Body &p_b, const Transform3D &p_fx) {
	return (p_fx.basis * Basis(UP_AXIS, p_b.yaw)).xform(Vector3(0.0f, 0.0f, -1.0f));
}

// --- Мозг (WalkerBrain) ---

int EnemyBodies::_player_index(ObjectID p_id) const {
	if (p_id.is_null()) {
		return -1;
	}
	for (uint32_t i = 0; i < players_id.size(); i++) {
		if (players_id[i] == p_id) {
			return (int)i;
		}
	}
	return -1;
}

bool EnemyBodies::_player_alive(int p_i) const {
	return p_i >= 0 && players_alive[p_i] != 0;
}

int EnemyBodies::_pick(const Vector3 &p_pos) const {
	int best = -1;
	real_t best_d = (real_t)Math::INF;
	for (int i = 0; i < players_pos.size(); i++) {
		if (!_player_alive(i)) {
			continue;
		}
		const real_t d = players_pos[i].distance_squared_to(p_pos);
		if (d < best_d) {
			best_d = d;
			best = i;
		}
	}
	return best;
}

// Цель ближе REACH (по горизонтали верха тела) и в секторе перед взглядом.
bool EnemyBodies::_in_reach(const Body &p_b, CharacterBody3D *p_body, const Transform3D &p_fx, const Vector3 &p_target) const {
	const Vector3 up = p_b.up;
	Vector3 d = p_target - p_body->get_global_position();
	const real_t dh = d.dot(up);
	if (Math::abs(dh) > REACH_UP) {
		return false; // Цель выше / ниже досягаемости
	}
	d -= up * dh;
	if (d.length() > REACH) {
		return false;
	}
	return d.length() < 0.05f || _forward(p_b, p_fx).dot(d.normalized()) >= Math::cos(Math::deg_to_rad(SECTOR_DEG));
}

uint64_t EnemyBodies::_ticks(real_t p_s) {
	return (uint64_t)Math::round(p_s * (real_t)Engine::get_singleton()->get_physics_ticks_per_second());
}

void EnemyBodies::_brain(int p_h, Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, uint64_t p_now) {
	if (r_b.point_set) {
		return;
	}
	int ti = _player_index(r_b.target); // Нет в снимке (игрок отключился) — цели нет
	if (ti < 0) {
		r_b.target = ObjectID();
	}
	if (p_now >= r_b.next_pick || !_player_alive(ti)) {
		r_b.next_pick = p_now + PICK_TICKS;
		ti = _pick(p_body->get_global_position());
		r_b.target = ti >= 0 ? players_id[ti] : ObjectID();
	}
	switch (r_b.brain) {
		case BRAIN_IDLE: {
			r_b.moving = false;
			if (ti >= 0) {
				r_b.brain = BRAIN_CHASE;
			}
		} break;
		case BRAIN_CHASE: {
			if (ti < 0) {
				r_b.brain = BRAIN_IDLE;
				r_b.moving = false;
				return;
			}
			r_b.moving = true;
			r_b.world = players_pos[ti];
			r_b.stop_dist = STRIKE_STOP;
			if (_in_reach(r_b, p_body, p_fx, players_pos[ti])) {
				r_b.brain = BRAIN_WINDUP;
				r_b.state_end = p_now + _ticks(WINDUP_S);
				r_b.moving = false;
				_emit(EVENT_ACTION, p_h);
			}
		} break;
		case BRAIN_WINDUP: {
			r_b.moving = false;
			if (ti >= 0) {
				r_b.face = players_pos[ti];
			}
			if (p_now >= r_b.state_end) {
				r_b.strikes++;
				_emit(EVENT_STRIKE, p_h);
				r_b.brain = BRAIN_RECOVER;
				r_b.state_end = p_now + _ticks(RECOVER_S);
			}
		} break;
		case BRAIN_RECOVER: {
			r_b.moving = false;
			if (ti >= 0) {
				r_b.face = players_pos[ti];
			}
			if (p_now >= r_b.state_end) {
				r_b.brain = ti >= 0 ? BRAIN_CHASE : BRAIN_IDLE;
			}
		} break;
	}
}

// --- Движение ---

void EnemyBodies::_update_tier(Body &r_b, const Vector3 &p_pos) {
	if (tier_override >= 0) {
		r_b.tier = (uint8_t)tier_override;
		return;
	}
	if (!nav_ready || !crowd.is_valid() || players_pos.is_empty()) {
		r_b.tier = TIER_NEAR;
		return;
	}
	real_t d = (real_t)Math::INF;
	for (int i = 0; i < players_pos.size(); i++) {
		d = MIN(d, p_pos.distance_to(players_pos[i]));
	}
	if (r_b.tier == TIER_NEAR && d > NEAR_R + TIER_HYST) {
		r_b.tier = TIER_FAR;
	} else if (r_b.tier == TIER_FAR && d < NEAR_R) {
		r_b.tier = TIER_NEAR;
	}
}

// Покой: стоящий (толпа IDLE / HOLD) на полу без своей и желаемой скорости не зовёт move_and_slide — поза та же;
// раз в REST_CHECK_TICKS (со сдвигом по слоту) — полный шаг: пол на месте, флаги пола свежие. Желаемая скорость
// (идти, расталкивание при пересечении) — сразу полный расчёт.
bool EnemyBodies::_rest(const Body &p_b, CharacterBody3D *p_body, const Vector3 &p_want, uint8_t p_state, uint64_t p_now) {
	if (p_state != EnemyWorld::AGENT_IDLE && p_state != EnemyWorld::AGENT_HOLD) {
		return false;
	}
	if (p_b.on_step || p_want.length() >= REST_SPEED || p_body->get_velocity().length() >= REST_SPEED || !p_body->is_on_floor()) {
		return false;
	}
	return (p_now + (uint64_t)p_b.slot) % REST_CHECK_TICKS != 0;
}

// WalkerMover.tick: горизонтальная скорость к желаемой с разгоном ACCEL, гравитация вдоль верха тела,
// move_and_slide; шаг на ступень с пола или в тик после принятого шага; взгляд.
void EnemyBodies::_move_near(Body &r_b, CharacterBody3D *p_body, const Vector3 &p_want, const Vector3 &p_face, real_t p_dt) {
	const Vector3 up = r_b.up;
	const Vector3 v = p_body->get_velocity();
	Vector3 v_up = up * v.dot(up);
	const Vector3 want_h = p_want - up * p_want.dot(up);
	const Vector3 v_h = (v - v_up).move_toward(want_h, ACCEL * p_dt);
	if (p_body->is_on_floor()) {
		v_up = Vector3();
	} else {
		v_up -= up * gravity * p_dt;
	}
	p_body->set_velocity(v_h + v_up);
	const bool grounded = p_body->is_on_floor();
	p_body->move_and_slide();
	real_t rise = 0.0f;
	if (grounded || r_b.on_step) {
		rise = _try_step(p_body, want_h * p_dt, up, p_body->get_collision_mask() & ~ITEM_LAYER);
	}
	r_b.on_step = rise > 0.0f;
	_lag(r_b, rise, p_dt);
	_turn(r_b, p_body, p_face, up, p_dt);
}

// FAR: без физики — шаг по сетке навигации (точка сетки по вертикали, EnemyWorld.column), тот же разгон и взгляд.
Vector3 EnemyBodies::_move_far(Body &r_b, CharacterBody3D *p_body, const Vector3 &p_want, const Vector3 &p_face, real_t p_dt) {
	const Vector3 up = r_b.up;
	const Vector3 v = p_body->get_velocity();
	const Vector3 want_h = p_want - up * p_want.dot(up);
	const Vector3 v_h = (v - up * v.dot(up)).move_toward(want_h, ACCEL * p_dt);
	const Vector3 next = p_body->get_global_position() + v_h * p_dt;
	const Vector3 on = crowd->column(root_inv.xform(next), r_b.hint);
	r_b.hint = crowd->get_column_poly();
	p_body->set_global_position(root_xf.xform(on));
	p_body->set_velocity(v_h);
	r_b.on_step = false;
	_lag(r_b, 0.0f, p_dt);
	_turn(r_b, p_body, p_face, up, p_dt);
	return v_h;
}

// Подъём шага визуал догоняет плавно: отставание гаснет за STEP_HALFLIFE (WalkerMover).
void EnemyBodies::_lag(Body &r_b, real_t p_rise, real_t p_dt) {
	r_b.lag_prev = r_b.lag_curr;
	r_b.lag_curr = _damp(r_b.lag_curr, 0.0f, STEP_HALFLIFE, p_dt) + p_rise;
}

void EnemyBodies::_turn(Body &r_b, CharacterBody3D *p_body, const Vector3 &p_face, const Vector3 &p_up, real_t p_dt) {
	const Vector3 f = p_face - p_up * p_face.dot(p_up);
	if (f.length_squared() > 1e-4f) {
		const Vector3 fl = p_body->get_global_basis().inverse().xform(f);
		r_b.yaw = Math::rotate_toward(r_b.yaw, (real_t)Math::atan2(-fl.x, -fl.z), TURN_RATE * p_dt);
	}
}

// SyncMath.damp_float.
real_t EnemyBodies::_damp(real_t p_curr, real_t p_target, real_t p_halflife, real_t p_dt) {
	if (p_dt <= 0.0f) {
		return p_curr;
	}
	if (p_halflife <= 0.0f) {
		return p_target;
	}
	const real_t k = CLAMP(1.0f - Math::exp(-LN2 * p_dt / p_halflife), 0.0f, 1.0f);
	return Math::lerp(p_curr, p_target, k);
}

// --- Шаг на ступень (StepUp; правила — _game/core/move/step_up.gd) ---

real_t EnemyBodies::_try_step(CharacterBody3D *p_body, const Vector3 &p_motion_h, const Vector3 &p_up, uint32_t p_mask) {
	const real_t want_len = p_motion_h.length();
	if (want_len < 0.0001f || !p_body->is_on_wall()) {
		return 0.0f;
	}
	const Vector3 dir = p_motion_h / want_len;
	if (!_wall_in_mask(p_body, dir, p_up, p_mask)) {
		return 0.0f; // Нет стены поперёк хода или упёрлись в предмет
	}
	const real_t rest = want_len - p_body->get_position_delta().dot(dir);
	if (rest < 0.001f) {
		return 0.0f;
	}
	PhysicsServer3D::MotionParameters params;
	params.margin = p_body->get_safe_margin();
	_bodies_outside(p_body, p_up, p_mask, params.exclude_bodies);
	return _probe(p_body, params, dir * rest, p_up, p_mask);
}

bool EnemyBodies::_wall_in_mask(CharacterBody3D *p_body, const Vector3 &p_dir, const Vector3 &p_up, uint32_t p_mask) {
	for (int i = 0; i < p_body->get_slide_collision_count(); i++) {
		const PhysicsServer3D::MotionResult r = p_body->get_slide_collision(i);
		const Vector3 n = r.collisions[0].normal;
		if (n.angle_to(p_up) <= p_body->get_floor_max_angle() || n.dot(p_dir) > -0.1f) {
			continue;
		}
		if (PhysicsServer3D::get_singleton()->body_get_collision_layer(r.collisions[0].collider) & p_mask) {
			return true;
		}
	}
	return false;
}

void EnemyBodies::_bodies_outside(CharacterBody3D *p_body, const Vector3 &p_up, uint32_t p_mask, HashSet<RID> &r_out) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	if (!query_sphere.is_valid()) {
		query_sphere = ps->sphere_shape_create();
		ps->shape_set_data(query_sphere, QUERY_R);
	}
	PhysicsDirectSpaceState3D *space = p_body->get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::ShapeParameters q;
	q.shape_rid = query_sphere;
	q.transform = Transform3D(Basis(), p_body->get_global_position() + p_up * QUERY_UP);
	q.collision_mask = ~p_mask;
	q.exclude.insert(p_body->get_rid());
	PhysicsDirectSpaceState3D::ShapeResult res[32];
	const int n = space->intersect_shape(q, res, 32);
	for (int i = 0; i < n; i++) {
		r_out.insert(res[i].rid);
	}
}

real_t EnemyBodies::_probe(CharacterBody3D *p_body, PhysicsServer3D::MotionParameters &r_params, const Vector3 &p_fwd, const Vector3 &p_up, uint32_t p_mask) {
	PhysicsServer3D::MotionResult res;
	Transform3D t = p_body->get_global_transform();
	// 1. Вверх (потолок ниже STEP_H — сколько есть)
	Vector3 up_m = p_up * STEP_H;
	if (_test(p_body, r_params, t, up_m, res)) {
		up_m = res.travel;
	}
	const real_t lift = up_m.dot(p_up);
	if (lift < 0.01f) {
		return 0.0f;
	}
	t.origin += up_m;
	// 2. Вперёд на остаток хода тика
	Vector3 fwd = p_fwd;
	if (_test(p_body, r_params, t, fwd, res)) {
		fwd = res.travel;
	}
	const Vector3 dir = p_fwd.normalized();
	if (fwd.dot(dir) < 0.001f) {
		return 0.0f;
	}
	t.origin += fwd;
	// 3. Вниз до опоры
	if (!_test(p_body, r_params, t, -p_up * lift, res)) {
		return 0.0f;
	}
	t.origin += res.travel;
	const real_t rise = (t.origin - p_body->get_global_position()).dot(p_up);
	if (rise <= 0.0f || rise > STEP_H) {
		return 0.0f;
	}
	// 4. Пол — поверхность ступени за точкой касания (луч вниз), не нормаль касания с кромкой
	const Vector3 hit_at = res.collisions[0].position + dir * EDGE_AHEAD;
	PhysicsDirectSpaceState3D::RayParameters rq;
	rq.from = hit_at + p_up * EDGE_SPAN;
	rq.to = hit_at - p_up * EDGE_SPAN;
	rq.collision_mask = p_mask;
	rq.exclude.insert(p_body->get_rid());
	PhysicsDirectSpaceState3D::RayResult hit;
	if (!p_body->get_world_3d()->get_direct_space_state()->intersect_ray(rq, hit)) {
		return 0.0f;
	}
	const real_t tread = (hit.position - p_body->get_global_position()).dot(p_up);
	if (tread < MIN_TREAD || tread > STEP_H) {
		return 0.0f;
	}
	if (hit.normal.angle_to(p_up) > p_body->get_floor_max_angle()) {
		return 0.0f;
	}
	p_body->set_global_position(t.origin);
	return rise;
}

bool EnemyBodies::_test(CharacterBody3D *p_body, PhysicsServer3D::MotionParameters &r_params, const Transform3D &p_from, const Vector3 &p_motion, PhysicsServer3D::MotionResult &r_res) {
	r_params.from = p_from;
	r_params.motion = p_motion;
	return PhysicsServer3D::get_singleton()->body_test_motion(p_body->get_rid(), r_params, &r_res);
}

// --- Сеть, счётчики ---

Array EnemyBodies::net_pack() const {
	int64_t t = -1;
	for (uint32_t h = 0; h < bodies.size(); h++) {
		const Body &b = bodies[h];
		if (b.used && !b.dead && b.net_id >= 0) {
			t = MAX(t, b.net_tick);
		}
	}
	PackedInt32Array ids;
	PackedFloat32Array data;
	if (t >= 0) {
		for (uint32_t h = 0; h < bodies.size(); h++) {
			const Body &b = bodies[h];
			if (!b.used || b.dead || b.net_id < 0 || b.net_tick != t) {
				continue;
			}
			ids.push_back(b.net_id);
			data.push_back(b.net_pos.x);
			data.push_back(b.net_pos.y);
			data.push_back(b.net_pos.z);
			data.push_back(b.yaw);
			data.push_back(b.view_speed);
			data.push_back(b.lag_curr);
		}
	}
	Array out;
	out.push_back(t);
	out.push_back(ids);
	out.push_back(data);
	return out;
}

int EnemyBodies::get_body_count() const {
	return (int)(bodies.size() - free_handles.size());
}

int EnemyBodies::get_tier_count(int p_tier) const {
	ERR_FAIL_INDEX_V(p_tier, 2, 0);
	return tier_counts[p_tier];
}

void EnemyBodies::_bind_methods() {
	ClassDB::bind_method(D_METHOD("setup", "crowd", "views", "root"), &EnemyBodies::setup);
	ClassDB::bind_method(D_METHOD("body_add", "body", "item_zone", "view", "slot", "speed", "yaw"), &EnemyBodies::body_add);
	ClassDB::bind_method(D_METHOD("body_remove", "handle"), &EnemyBodies::body_remove);
	ClassDB::bind_method(D_METHOD("body_kill", "handle"), &EnemyBodies::body_kill);
	ClassDB::bind_method(D_METHOD("body_set_frame", "handle", "frame"), &EnemyBodies::body_set_frame);
	ClassDB::bind_method(D_METHOD("body_set_point", "handle", "world"), &EnemyBodies::body_set_point);
	ClassDB::bind_method(D_METHOD("body_set_path", "handle", "points", "target", "links"), &EnemyBodies::body_set_path, DEFVAL(PackedInt32Array()));
	ClassDB::bind_method(D_METHOD("body_set_profile", "handle", "jump_impulse", "gravity"), &EnemyBodies::body_set_profile);
	ClassDB::bind_method(D_METHOD("body_link_log", "handle"), &EnemyBodies::body_link_log);
	ClassDB::bind_method(D_METHOD("body_link_state", "handle"), &EnemyBodies::body_link_state);
	ClassDB::bind_method(D_METHOD("set_links", "starts", "ends", "kinds"), &EnemyBodies::set_links);
	ClassDB::bind_method(D_METHOD("body_add_hits", "handle", "hits"), &EnemyBodies::body_add_hits);
	ClassDB::bind_method(D_METHOD("body_set_net_id", "handle", "net_id"), &EnemyBodies::body_set_net_id);
	ClassDB::bind_method(D_METHOD("body_yaw", "handle"), &EnemyBodies::body_yaw);
	ClassDB::bind_method(D_METHOD("body_forward", "handle"), &EnemyBodies::body_forward);
	ClassDB::bind_method(D_METHOD("body_frame_xform", "handle"), &EnemyBodies::body_frame_xform);
	ClassDB::bind_method(D_METHOD("body_state", "handle"), &EnemyBodies::body_state);
	ClassDB::bind_method(D_METHOD("body_brain", "handle"), &EnemyBodies::body_brain);
	ClassDB::bind_method(D_METHOD("body_moving", "handle"), &EnemyBodies::body_moving);
	ClassDB::bind_method(D_METHOD("body_strikes", "handle"), &EnemyBodies::body_strikes);
	ClassDB::bind_method(D_METHOD("body_hits", "handle"), &EnemyBodies::body_hits);
	ClassDB::bind_method(D_METHOD("body_tier", "handle"), &EnemyBodies::body_tier);
	ClassDB::bind_method(D_METHOD("body_want_speed", "handle"), &EnemyBodies::body_want_speed);
	ClassDB::bind_method(D_METHOD("body_path_from", "handle"), &EnemyBodies::body_path_from);
	ClassDB::bind_method(D_METHOD("body_goal", "handle"), &EnemyBodies::body_goal);
	ClassDB::bind_method(D_METHOD("body_path_requests", "handle"), &EnemyBodies::body_path_requests);
	ClassDB::bind_method(D_METHOD("body_crowd_stats", "handle"), &EnemyBodies::body_crowd_stats);
	ClassDB::bind_method(D_METHOD("body_net_tick", "handle"), &EnemyBodies::body_net_tick);
	ClassDB::bind_method(D_METHOD("body_net_pos", "handle"), &EnemyBodies::body_net_pos);
	ClassDB::bind_method(D_METHOD("set_players", "positions", "alive", "ids"), &EnemyBodies::set_players);
	ClassDB::bind_method(D_METHOD("set_nav_ready", "ready"), &EnemyBodies::set_nav_ready);
	ClassDB::bind_method(D_METHOD("set_tier_override", "tier"), &EnemyBodies::set_tier_override);
	ClassDB::bind_method(D_METHOD("tick", "delta"), &EnemyBodies::tick);
	ClassDB::bind_method(D_METHOD("net_pack"), &EnemyBodies::net_pack);
	ClassDB::bind_method(D_METHOD("get_body_count"), &EnemyBodies::get_body_count);
	ClassDB::bind_method(D_METHOD("get_tier_count", "tier"), &EnemyBodies::get_tier_count);
	ClassDB::bind_method(D_METHOD("get_rest_count"), &EnemyBodies::get_rest_count);
	ClassDB::bind_method(D_METHOD("get_last_tick_usec"), &EnemyBodies::get_last_tick_usec);
	ClassDB::bind_method(D_METHOD("get_traversals"), &EnemyBodies::get_traversals);
	ClassDB::bind_method(D_METHOD("get_land_close"), &EnemyBodies::get_land_close);
	ClassDB::bind_method(D_METHOD("get_closed_links"), &EnemyBodies::get_closed_links);
	ClassDB::bind_method(D_METHOD("get_on_link_count"), &EnemyBodies::get_on_link_count);

	BIND_ENUM_CONSTANT(BRAIN_IDLE);
	BIND_ENUM_CONSTANT(BRAIN_CHASE);
	BIND_ENUM_CONSTANT(BRAIN_WINDUP);
	BIND_ENUM_CONSTANT(BRAIN_RECOVER);
	BIND_ENUM_CONSTANT(TIER_NEAR);
	BIND_ENUM_CONSTANT(TIER_FAR);
	BIND_ENUM_CONSTANT(EVENT_ACTION);
	BIND_ENUM_CONSTANT(EVENT_STRIKE);
	BIND_ENUM_CONSTANT(EVENT_PATH);
	BIND_ENUM_CONSTANT(EVENT_ITEMS);
	BIND_ENUM_CONSTANT(EVENT_JUMP);
	BIND_ENUM_CONSTANT(EVENT_LINK_CLOSE);
	BIND_ENUM_CONSTANT(EVENT_LINK_OPEN);
}
