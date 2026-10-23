#include "enemy_bodies.h"

#include "scene/3d/physics/character_body_3d.h"

// Тела ходоков у связей (план docs/superpowers/plans/2026-10-22-nav-links-b.md): ждать у входа, пройти связь по
// траектории профиля, приземлиться; закрытые связи — открыть, когда место приземления свободно. Связи — только в
// сетке корабля (пространство 0; spec 2026-10-23-enemy-moving-spaces, этап 2).

void EnemyBodies::set_links(const PackedVector3Array &p_starts, const PackedVector3Array &p_ends, const PackedInt32Array &p_kinds) {
	for (uint32_t h = 0; h < bodies.size(); h++) {
		Body &b = bodies[h];
		if (!b.used) {
			continue;
		}
		_link_release((int)h, b);
		b.link_state = LINK_NONE;
		b.link = -1;
	}
	links.set(p_starts, p_ends, p_kinds);
}

void EnemyBodies::body_set_profile(int p_h, real_t p_jump_impulse, real_t p_gravity) {
	ERR_FAIL_COND(!_valid(p_h));
	Body &b = bodies[p_h];
	b.jump_impulse = p_jump_impulse;
	b.jump_gravity = p_gravity;
}

PackedInt32Array EnemyBodies::body_link_log(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), PackedInt32Array());
	const Body &b = bodies[p_h];
	PackedInt32Array out;
	for (const uint8_t k : b.link_log) {
		out.push_back(k);
	}
	return out;
}

int EnemyBodies::body_link_state(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), LINK_NONE);
	return bodies[p_h].link_state;
}

// Закрытые связи: место приземления свободно OPEN_S подряд — открыть (событие, эпоха навигации).
void EnemyBodies::_links_tick(real_t p_dt) {
	on_link = 0;
	if (links.closed_count() == 0 || spaces.is_empty() || !spaces[0].crowd.is_valid()) {
		return;
	}
	EnemyWorld *crowd = spaces[0].crowd.ptr();
	LocalVector<int> closed;
	closed = links.closed_list(); // open_tick меняет список
	for (const int l : closed) {
		if (links.open_tick(l, crowd->count_near(links.end(l), LAND_FREE_R, -1) == 0, p_dt)) {
			nav_epoch++;
			_emit(EVENT_LINK_OPEN, l);
		}
	}
}

// У входа связи (толпа: AGENT_LINK): закрыта — ждёт новый путь (_tick_body); занята другим или место приземления
// занято — ждёт (занятое место копит закрытие связи); иначе — на связь: движение профиля тела, клип, толпа его не
// видит. true — ушёл на связь.
bool EnemyBodies::_link_wait(int p_h, Body &r_b, CharacterBody3D *p_body, uint64_t p_now, real_t p_dt) {
	EnemyWorld *crowd = _crowd(r_b);
	const int l = (r_b.space == 0 && crowd != nullptr) ? crowd->agent_link(r_b.crowd) : -1;
	if (!links.valid(l)) {
		r_b.link_state = LINK_NONE;
		r_b.link = -1;
		return false;
	}
	r_b.link = l;
	r_b.link_state = LINK_WAIT;
	if (links.closed(l) || (links.busy(l) >= 0 && links.busy(l) != p_h)) {
		return false;
	}
	if (crowd->count_near(links.end(l), LAND_FREE_R, r_b.crowd) > 0) {
		if (links.blocked(l, p_now, p_dt)) {
			nav_epoch++;
			_emit(EVENT_LINK_CLOSE, l);
		}
		return false;
	}
	links.unblocked(l);
	links.claim(l, p_h);
	EnemyLinks::Motion &m = r_b.motion;
	m.kind = links.kind(l);
	m.a = spaces[0].inv.xform(p_body->get_global_position());
	m.b = links.end(l);
	m.walk = r_b.speed;
	m.impulse = r_b.jump_impulse;
	m.gravity = r_b.jump_gravity;
	EnemyLinks::begin(m);
	r_b.link_t = 0.0f;
	r_b.link_state = LINK_ON;
	crowd->agent_set_airborne(r_b.crowd, true);
	if (r_b.link_log.size() < LINK_LOG) {
		r_b.link_log.push_back((uint8_t)m.kind);
	}
	traversals++;
	_emit(EVENT_JUMP, p_h);
	return true;
}

// На связи: тело по траектории (без физики, оба яруса), лицом к концу; конец — приземление (LAND_S стоит, толпа —
// путь с конца связи).
void EnemyBodies::_link_move(int p_h, Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, uint64_t p_now, real_t p_dt) {
	const bool has_goal = r_b.moving;
	const Space &sp = spaces[0]; // На связи — всегда корабль (телепорт отпускает связь)
	const Vector3 goal = has_goal ? sp.inv.xform(r_b.world) : Vector3();
	Vector3 vel;
	if (r_b.link_state == LINK_ON) {
		on_link++;
		const Vector3 from = EnemyLinks::at(r_b.motion, r_b.link_t);
		r_b.link_t += p_dt;
		const Vector3 to = EnemyLinks::at(r_b.motion, r_b.link_t);
		p_body->set_global_position(sp.xf.xform(to));
		vel = sp.xf.basis.xform(to - from) / p_dt;
		_turn(r_b, p_body, sp.xf.basis.xform(r_b.motion.b - r_b.motion.a), r_b.up, p_dt);
		if (r_b.link_t >= r_b.motion.duration) {
			if (sp.crowd.is_valid() && sp.crowd->count_near(r_b.motion.b, LAND_CLOSE_R, r_b.crowd) > 0) {
				land_close++;
			}
			_link_release(p_h, r_b);
			r_b.link_state = LINK_LAND;
			r_b.land_end = p_now + _ticks(LAND_S);
			p_body->set_velocity(Vector3());
			vel = Vector3();
		}
	} else if (p_now >= r_b.land_end) {
		r_b.link_state = LINK_NONE;
		r_b.link = -1;
	}
	_lag(r_b, 0.0f, p_dt);
	r_b.want_speed = 0.0f;
	r_b.state = EnemyWorld::AGENT_LINK;
	_publish(r_b, p_body, p_fx, vel, goal, has_goal, p_now);
}

// Отпустить связь: занятость; на связи — толпа снова видит агента, путь — с конца связи.
void EnemyBodies::_link_release(int p_h, Body &r_b) {
	if (links.valid(r_b.link)) {
		links.release(r_b.link, p_h);
	}
	EnemyWorld *c = _crowd(r_b); // Из _set_space — тело ещё в старом пространстве
	if (r_b.link_state == LINK_ON && c != nullptr && r_b.crowd >= 0) {
		c->agent_set_airborne(r_b.crowd, false);
		c->agent_link_done(r_b.crowd);
	}
}
