#include "ragdoll_sync.h"

#include "ragdoll_pelvis_codec.h"

#include "core/object/class_db.h"
#include "scene/3d/physics/rigid_body_3d.h"

#include <cstring>

static_assert(RagdollSync::BYTES == RagdollPelvisCodec::BYTES, "RagdollSync::BYTES");

int RagdollSync::_find(int64_t p_id) const {
	for (uint32_t i = 0; i < ents.size(); i++) {
		if (ents[i].id == p_id) {
			return (int)i;
		}
	}
	return -1;
}

int RagdollSync::_owner_of(int64_t p_key) const {
	const int *o = owners.getptr(p_key);
	return o ? *o : 1;
}

double RagdollSync::_one_way(int p_peer) {
	for (uint32_t i = 0; i < ow_peer.size(); i++) {
		if (ow_peer[i] == p_peer) {
			return ow_val[i];
		}
	}
	double v = 0.0;
	if (one_way_fn.is_valid()) {
		v = (double)one_way_fn.call(p_peer);
	}
	ow_peer.push_back(p_peer);
	ow_val.push_back(v);
	return v;
}

RigidBody3D *RagdollSync::_pelvis(const Ent &p_e) const {
	return Object::cast_to<RigidBody3D>(ObjectDB::get_instance(p_e.pelvis));
}

void RagdollSync::setup(double p_buffer_s, double p_keep_s, double p_catchup_s, double p_rest_s, double p_rate_up,
		double p_rate_down, double p_move_eps, double p_turn_eps, const Callable &p_one_way, const Callable &p_space_xf) {
	params.buffer_s = p_buffer_s;
	params.keep_s = p_keep_s;
	params.catchup_s = p_catchup_s;
	params.rest_s = p_rest_s;
	params.rate_up = p_rate_up;
	params.rate_down = p_rate_down;
	move_eps = p_move_eps;
	turn_eps = p_turn_eps;
	one_way_fn = p_one_way;
	spaces.fn = p_space_xf;
}

void RagdollSync::add(int64_t p_id, int64_t p_key, RigidBody3D *p_pelvis, bool p_own, bool p_asleep, Object *p_holder,
		const Callable &p_space_of) {
	ERR_FAIL_NULL(p_pelvis);
	if (_find(p_id) >= 0) {
		return;
	}
	Ent e;
	e.id = p_id;
	e.key = p_key;
	e.pelvis = p_pelvis->get_instance_id();
	e.holder = p_holder ? p_holder->get_instance_id() : ObjectID();
	e.space_of = p_space_of;
	e.own = p_own;
	e.asleep = p_asleep;
	ents.push_back(e);
	by_key.insert(p_key, p_id);
}

void RagdollSync::remove(int64_t p_id) {
	const int i = _find(p_id);
	if (i < 0) {
		return;
	}
	const int64_t key = ents[i].key;
	ents.remove_at(i);
	const int64_t *k = by_key.getptr(key);
	if (k && *k == p_id) {
		by_key.erase(key);
	}
}

void RagdollSync::clear() {
	ents.clear();
	by_key.clear();
	owners.clear();
	rests.clear();
}

bool RagdollSync::has(int64_t p_id) const {
	return _find(p_id) >= 0;
}

void RagdollSync::set_owner(int64_t p_key, int p_peer) {
	owners.insert(p_key, p_peer);
}

void RagdollSync::clear_owners() {
	owners.clear();
}

// Своя кукла (RagdollNet._become_owner, после release): буфер заново, поток — с первого тика.
void RagdollSync::set_own(int64_t p_id) {
	const int i = _find(p_id);
	ERR_FAIL_COND(i < 0);
	Ent &e = ents[i];
	e.track.reset();
	e.own = true;
	e.rest_sent = false;
	e.sent_space = -1;
}

// Чужая кукла (RagdollNet._become_follower, после release): своя физика до потока владельца, затем буфер.
void RagdollSync::set_follower(int64_t p_id) {
	const int i = _find(p_id);
	ERR_FAIL_COND(i < 0);
	Ent &e = ents[i];
	e.track.reset();
	e.own = false;
}

void RagdollSync::set_asleep(int64_t p_id, bool p_asleep) {
	const int i = _find(p_id);
	if (i >= 0) {
		ents[i].asleep = p_asleep;
	}
}

// Финальная поза отправлена (RagdollNet._send_rest).
void RagdollSync::mark_sent(int64_t p_id, int p_space, const Transform3D &p_local) {
	const int i = _find(p_id);
	ERR_FAIL_COND(i < 0);
	Ent &e = ents[i];
	e.rest_sent = true;
	e.sent_xf = p_local;
	e.sent_space = p_space;
}

// Следующие, таз которых не дальше p_radius от точки (RagdollTouch.touching — дальше зоны не проверяются).
PackedInt64Array RagdollSync::nearby(const Vector3 &p_point, double p_radius) const {
	PackedInt64Array out;
	const double r2 = p_radius * p_radius;
	for (const Ent &e : ents) {
		if (e.own) {
			continue;
		}
		const RigidBody3D *pelvis = _pelvis(e);
		if (pelvis == nullptr || (double)pelvis->get_global_transform().origin.distance_squared_to(p_point) > r2) {
			continue;
		}
		out.push_back(e.id);
	}
	return out;
}

// Владелец (прежний RagdollNet._tick_owner на всех своих): уснула — финальная поза раз (take_rests, шлёт RagdollNet);
// в тик отправки — поза таза, если сдвинулась (пакет поз, порядок кукол — порядок добавления).
PackedByteArray RagdollSync::owner_tick(bool p_send) {
	rests.clear();
	spaces.clear();
	pack_buf.clear();
	for (Ent &e : ents) {
		if (!e.own) {
			continue;
		}
		if (e.asleep) {
			if (!e.rest_sent) {
				rests.push_back(e.id);
			}
			continue;
		}
		e.rest_sent = false;
		if (!p_send) {
			continue;
		}
		RigidBody3D *pelvis = _pelvis(e);
		if (pelvis == nullptr) {
			continue;
		}
		int sp = 0; // = walker.space() if is_instance_valid(walker) else 0
		if (ObjectDB::get_instance(e.holder) != nullptr && e.space_of.is_valid()) {
			sp = (int)e.space_of.call();
		}
		const Transform3D local = spaces.get(sp).affine_inverse() * pelvis->get_global_transform();
		if (sp == e.sent_space && (double)local.origin.distance_to(e.sent_xf.origin) < move_eps &&
				(double)local.basis.get_rotation_quaternion().angle_to(e.sent_xf.basis.get_rotation_quaternion()) < turn_eps) {
			continue; // таз стоит — не шлём (конечности — своя физика у каждого)
		}
		e.sent_xf = local;
		e.sent_space = sp;
		const uint32_t at = pack_buf.size();
		pack_buf.resize(at + BYTES);
		RagdollPelvisCodec::write(pack_buf.ptr() + at, e.key, sp, local);
	}
	PackedByteArray out;
	if (!pack_buf.is_empty()) {
		out.resize((int)pack_buf.size());
		memcpy(out.ptrw(), pack_buf.ptr(), pack_buf.size());
	}
	return out;
}

PackedInt64Array RagdollSync::take_rests() {
	PackedInt64Array out = rests;
	rests.clear();
	return out;
}

bool RagdollSync::needs_lag() const {
	for (const Ent &e : ents) {
		if (!e.own && !e.track.idle(e.asleep)) {
			return true;
		}
	}
	return false;
}

void RagdollSync::set_lag_inputs(const PackedVector3Array &p_points, const PackedFloat64Array &p_lags, int p_local) {
	view_lag.set(p_points, p_lags, p_local);
}

double RagdollSync::lag_at(const Vector3 &p_at) const {
	return view_lag.lag_at(p_at);
}

// Следующие (прежний RagdollNet._tick_follower на всех чужих): цель задержки — у видимого таза (ViewLag), задержка до
// владельца — one_way; ключи кукол, которых усыпить (финальная поза держится rest_s, а кукла не спит).
PackedInt64Array RagdollSync::tick(double p_now, int p_tps) {
	PackedInt64Array dozes;
	spaces.clear();
	ow_peer.clear();
	ow_val.clear();
	const int tps = MAX(1, p_tps);
	for (Ent &e : ents) {
		if (e.own || e.track.idle(e.asleep)) {
			continue;
		}
		RigidBody3D *pelvis = _pelvis(e);
		if (pelvis == nullptr) {
			continue;
		}
		const double target = view_lag.lag_at(pelvis->get_global_transform().origin);
		if (e.track.tick(pelvis, p_now, _one_way(_owner_of(e.key)), target, spaces, e.asleep, tps, params)) {
			dozes.push_back(e.id);
		}
	}
	return dozes;
}

// Точка от пира from (final — финальная поза уснувшей): только от владельца, только следующим.
void RagdollSync::feed(int p_from, int64_t p_key, double p_t, int p_space, const Transform3D &p_xf, bool p_final) {
	const int64_t *id = by_key.getptr(p_key);
	if (id == nullptr) {
		return;
	}
	const int i = _find(*id);
	if (i < 0 || ents[i].own || _owner_of(p_key) != p_from) {
		return;
	}
	ents[i].track.feed(p_t, p_space, p_xf, p_final, params);
}

// Пакет поз (RagdollPelvisCodec) от пира from, время точки t — на нашей шкале; битые данные — ничего.
void RagdollSync::feed_packed(int p_from, double p_t, const PackedByteArray &p_data) {
	if (p_data.size() % BYTES != 0) {
		return;
	}
	const uint8_t *r = p_data.ptr();
	for (int i = 0; i < p_data.size() / BYTES; i++) {
		int64_t key = 0;
		int space = 0;
		Transform3D xf;
		RagdollPelvisCodec::read(r + i * BYTES, key, space, xf);
		feed(p_from, key, p_t, space, xf, false);
	}
}

bool RagdollSync::following(int64_t p_id) const {
	const int i = _find(p_id);
	return i >= 0 && ents[i].track.following;
}

int RagdollSync::state(int64_t p_id) const {
	const int i = _find(p_id);
	if (i < 0) {
		return WAIT;
	}
	const RagdollPelvisTrack &t = ents[i].track;
	return t.resting ? REST : (t.following ? FOLLOW : WAIT);
}

double RagdollSync::lag(int64_t p_id) const {
	const int i = _find(p_id);
	return i >= 0 ? ents[i].track.lag : -1.0;
}

Vector3 RagdollSync::velocity(int64_t p_id, int p_tps) const {
	const int i = _find(p_id);
	return i >= 0 ? ents[i].track.velocity(p_tps) : Vector3();
}

Vector3 RagdollSync::angular_velocity(int64_t p_id, int p_tps) const {
	const int i = _find(p_id);
	return i >= 0 ? ents[i].track.angular_velocity(p_tps) : Vector3();
}

PackedByteArray RagdollSync::pack_pose(int64_t p_key, int p_space, const Transform3D &p_xf) {
	PackedByteArray out;
	out.resize(BYTES);
	RagdollPelvisCodec::write(out.ptrw(), p_key, p_space, p_xf);
	return out;
}

// [key, space, xf] позы из BYTES байт; битые данные — пусто.
Array RagdollSync::unpack_pose(const PackedByteArray &p_data) {
	Array out;
	if (p_data.size() != BYTES) {
		return out;
	}
	int64_t key = 0;
	int space = 0;
	Transform3D xf;
	RagdollPelvisCodec::read(p_data.ptr(), key, space, xf);
	out.push_back(key);
	out.push_back(space);
	out.push_back(xf);
	return out;
}

void RagdollSync::_bind_methods() {
	ClassDB::bind_method(D_METHOD("setup", "buffer_s", "keep_s", "catchup_s", "rest_s", "rate_up", "rate_down", "move_eps",
								 "turn_eps", "one_way", "space_xf"),
			&RagdollSync::setup);
	ClassDB::bind_method(D_METHOD("add", "id", "key", "pelvis", "own", "asleep", "holder", "space_of"), &RagdollSync::add);
	ClassDB::bind_method(D_METHOD("remove", "id"), &RagdollSync::remove);
	ClassDB::bind_method(D_METHOD("clear"), &RagdollSync::clear);
	ClassDB::bind_method(D_METHOD("has", "id"), &RagdollSync::has);
	ClassDB::bind_method(D_METHOD("count"), &RagdollSync::count);
	ClassDB::bind_method(D_METHOD("set_owner", "key", "peer"), &RagdollSync::set_owner);
	ClassDB::bind_method(D_METHOD("clear_owners"), &RagdollSync::clear_owners);
	ClassDB::bind_method(D_METHOD("set_own", "id"), &RagdollSync::set_own);
	ClassDB::bind_method(D_METHOD("set_follower", "id"), &RagdollSync::set_follower);
	ClassDB::bind_method(D_METHOD("set_asleep", "id", "asleep"), &RagdollSync::set_asleep);
	ClassDB::bind_method(D_METHOD("mark_sent", "id", "space", "local"), &RagdollSync::mark_sent);
	ClassDB::bind_method(D_METHOD("nearby", "point", "radius"), &RagdollSync::nearby);
	ClassDB::bind_method(D_METHOD("owner_tick", "send"), &RagdollSync::owner_tick);
	ClassDB::bind_method(D_METHOD("take_rests"), &RagdollSync::take_rests);
	ClassDB::bind_method(D_METHOD("needs_lag"), &RagdollSync::needs_lag);
	ClassDB::bind_method(D_METHOD("set_lag_inputs", "points", "lags", "local"), &RagdollSync::set_lag_inputs);
	ClassDB::bind_method(D_METHOD("lag_at", "at"), &RagdollSync::lag_at);
	ClassDB::bind_method(D_METHOD("tick", "now", "tps"), &RagdollSync::tick);
	ClassDB::bind_method(D_METHOD("feed", "from", "key", "t", "space", "xf", "final"), &RagdollSync::feed);
	ClassDB::bind_method(D_METHOD("feed_packed", "from", "t", "data"), &RagdollSync::feed_packed);
	ClassDB::bind_method(D_METHOD("following", "id"), &RagdollSync::following);
	ClassDB::bind_method(D_METHOD("state", "id"), &RagdollSync::state);
	ClassDB::bind_method(D_METHOD("lag", "id"), &RagdollSync::lag);
	ClassDB::bind_method(D_METHOD("velocity", "id", "tps"), &RagdollSync::velocity);
	ClassDB::bind_method(D_METHOD("angular_velocity", "id", "tps"), &RagdollSync::angular_velocity);
	ClassDB::bind_static_method("RagdollSync", D_METHOD("pack_pose", "key", "space", "xf"), &RagdollSync::pack_pose);
	ClassDB::bind_static_method("RagdollSync", D_METHOD("unpack_pose", "data"), &RagdollSync::unpack_pose);
	BIND_CONSTANT(BYTES);
	BIND_CONSTANT(WAIT);
	BIND_CONSTANT(FOLLOW);
	BIND_CONSTANT(REST);
}
