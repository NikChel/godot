#include "enemy_views.h"

#include "core/object/class_db.h"
#include "core/os/os.h"
#include "scene/3d/node_3d.h"
#include "scene/animation/animation_player.h"

StringName EnemyViews::_clip_name(int p_clip) {
	return p_clip == CLIP_WALK ? SNAME("walker/beg") : SNAME("walker/stay");
}

real_t EnemyViews::_walk_rate(real_t p_speed) {
	const real_t s = Math::abs(p_speed);
	const real_t k = MAX(s - BLEND_SPEED, (real_t)0.0) / (RUN_SPEED - BLEND_SPEED);
	const real_t rate = MIN(Math::lerp((real_t)1.0, RUN_RATE, k), MAX_RATE);
	return p_speed < 0.0f ? -rate : rate;
}

// Уровень с гистерезисом: граница сдвинута от текущего уровня на TIER_HYST.
int EnemyViews::_tier_for(int p_tier, real_t p_dist) {
	const real_t near_r = NEAR_R + (p_tier == TIER_NEAR ? TIER_HYST : -TIER_HYST);
	const real_t far_r = FAR_R + (p_tier == TIER_FAR ? -TIER_HYST : TIER_HYST);
	if (p_dist < near_r) {
		return TIER_NEAR;
	}
	if (p_dist > far_r) {
		return TIER_FAR;
	}
	return TIER_MID;
}

bool EnemyViews::_valid(int p_h) const {
	return p_h >= 0 && p_h < (int)used.size() && used[p_h];
}

int EnemyViews::view_add(Node3D *p_root, AnimationPlayer *p_player) {
	ERR_FAIL_NULL_V(p_root, -1);
	ERR_FAIL_NULL_V(p_player, -1);
	ERR_FAIL_COND_V_MSG(!p_player->has_animation(_clip_name(CLIP_STAND)) || !p_player->has_animation(_clip_name(CLIP_WALK)), -1,
			"EnemyViews: AnimationPlayer has no walker/stay or walker/beg.");
	int h;
	if (free_handles.size() > 0) {
		h = free_handles[free_handles.size() - 1];
		free_handles.remove_at(free_handles.size() - 1);
	} else {
		h = (int)used.size();
		const uint32_t n = used.size() + 1;
		root_id.resize(n);
		player_id.resize(n);
		pose_prev.resize(n);
		pose_curr.resize(n);
		space_prev.resize(n);
		space_curr.resize(n);
		lag_prev.resize(n);
		lag_curr.resize(n);
		speed.resize(n);
		anim_dt.resize(n);
		tier.resize(n);
		clip.resize(n);
		used.resize(n);
		has_pose.resize(n);
		acting.resize(n);
		action.resize(n);
	}
	root_id[h] = p_root->get_instance_id();
	player_id[h] = p_player->get_instance_id();
	pose_prev[h] = Transform3D();
	pose_curr[h] = Transform3D();
	space_prev[h] = -1;
	space_curr[h] = -1;
	lag_prev[h] = 0.0f;
	lag_curr[h] = 0.0f;
	speed[h] = 0.0f;
	anim_dt[h] = 0.0f;
	tier[h] = TIER_NEAR;
	clip[h] = CLIP_STAND;
	used[h] = 1;
	has_pose[h] = 0;
	acting[h] = 0;
	action[h] = StringName();
	p_player->set_callback_mode_process(AnimationMixer::ANIMATION_CALLBACK_MODE_PROCESS_MANUAL);
	p_player->play(_clip_name(CLIP_STAND));
	p_player->advance(0.0); // первый шаг после play идёт с нулевым временем (AnimationPlayer::_process_playback_data, p_started)
	return h;
}

void EnemyViews::view_remove(int p_h) {
	ERR_FAIL_COND(!_valid(p_h));
	used[p_h] = 0;
	root_id[p_h] = ObjectID();
	player_id[p_h] = ObjectID();
	free_handles.push_back(p_h);
}

void EnemyViews::view_commit(int p_h, const Transform3D &p_pose, real_t p_step_lag, real_t p_speed, int p_space) {
	ERR_FAIL_COND(!_valid(p_h));
	if (has_pose[p_h]) {
		pose_prev[p_h] = pose_curr[p_h];
		space_prev[p_h] = space_curr[p_h];
		lag_prev[p_h] = lag_curr[p_h];
	} else {
		pose_prev[p_h] = p_pose;
		space_prev[p_h] = p_space;
		lag_prev[p_h] = p_step_lag;
		has_pose[p_h] = 1;
	}
	pose_curr[p_h] = p_pose;
	space_curr[p_h] = p_space;
	lag_curr[p_h] = p_step_lag;
	speed[p_h] = p_speed;
}

void EnemyViews::set_root(Node3D *p_root) {
	ERR_FAIL_NULL(p_root);
	spaces.clear();
	Space s;
	s.node_id = p_root->get_instance_id();
	spaces.push_back(s);
}

int EnemyViews::space_add(Node3D *p_frame) {
	ERR_FAIL_NULL_V(p_frame, -1);
	ERR_FAIL_COND_V_MSG(spaces.is_empty(), -1, "EnemyViews: set_root first (space 0 — the map root).");
	Space s;
	s.node_id = p_frame->get_instance_id();
	s.frame = true;
	spaces.push_back(s);
	return (int)spaces.size() - 1;
}

// Поза кадра пространства; −1 или реестра нет — мир, номер вне реестра — корень (0).
Transform3D EnemyViews::_space_xf(int p_space) const {
	if (p_space < 0 || spaces.is_empty()) {
		return Transform3D();
	}
	return spaces[p_space < (int)spaces.size() ? p_space : 0].xf;
}

void EnemyViews::view_set_active(int p_h, bool p_active) {
	ERR_FAIL_COND(!_valid(p_h));
	if (p_active == (tier[p_h] != TIER_OFF)) {
		return;
	}
	tier[p_h] = p_active ? TIER_NEAR : TIER_OFF;
	anim_dt[p_h] = 0.0f;
}

void EnemyViews::view_reset(int p_h) {
	ERR_FAIL_COND(!_valid(p_h));
	has_pose[p_h] = 0;
}

void EnemyViews::view_play_action(int p_h, const StringName &p_clip) {
	ERR_FAIL_COND(!_valid(p_h));
	AnimationPlayer *player = ObjectDB::get_instance<AnimationPlayer>(player_id[p_h]);
	ERR_FAIL_NULL(player);
	ERR_FAIL_COND_MSG(!player->has_animation(p_clip), vformat("EnemyViews: no clip %s.", p_clip));
	player->play(p_clip, ACTION_BLEND);
	player->set_speed_scale(1.0f);
	acting[p_h] = 1;
	action[p_h] = p_clip;
}

bool EnemyViews::view_is_acting(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), false);
	return acting[p_h] != 0;
}

void EnemyViews::frame(real_t p_alpha, const Vector3 &p_camera, real_t p_delta) {
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	frame_no++;
	for (int &c : tier_counts) {
		c = 0;
	}
	for (Space &s : spaces) {
		Object *o = ObjectDB::get_instance(s.node_id);
		if (s.frame) {
			s.xf = o ? (Transform3D)o->call(SNAME("render_xform"), p_alpha) : Transform3D();
		} else {
			Node3D *n = Object::cast_to<Node3D>(o);
			s.xf = n ? n->get_global_transform() : Transform3D();
		}
	}
	for (uint32_t i = 0; i < used.size(); i++) {
		if (!used[i]) {
			continue;
		}
		if (tier[i] == TIER_OFF || !has_pose[i]) {
			tier_counts[tier[i]]++;
			continue;
		}
		Node3D *root = ObjectDB::get_instance<Node3D>(root_id[i]);
		AnimationPlayer *player = ObjectDB::get_instance<AnimationPlayer>(player_id[i]);
		if (root == nullptr || player == nullptr) {
			continue;
		}
		Transform3D xf;
		if (space_prev[i] == space_curr[i]) {
			xf = _space_xf(space_curr[i]) * pose_prev[i].interpolate_with(pose_curr[i], p_alpha);
		} else {
			xf = (_space_xf(space_prev[i]) * pose_prev[i]).interpolate_with(_space_xf(space_curr[i]) * pose_curr[i], p_alpha);
		}
		xf.origin -= xf.basis.get_column(1).normalized() * Math::lerp(lag_prev[i], lag_curr[i], p_alpha);
		root->set_global_transform(xf);
		const int t = tier_override >= 0 ? tier_override : _tier_for(tier[i], xf.origin.distance_to(p_camera));
		tier[i] = (uint8_t)t;
		tier_counts[t]++;
		anim_dt[i] += p_delta;
		const uint64_t every = (uint64_t)(t == TIER_NEAR ? 1 : (t == TIER_MID ? MID_EVERY : FAR_EVERY));
		if ((frame_no + i) % every != 0) {
			continue;
		}
		if (acting[i]) {
			if (player->is_playing() && player->get_current_animation() == action[i]) {
				player->advance(anim_dt[i]);
				anim_dt[i] = 0.0f;
				continue;
			}
			acting[i] = 0;
			clip[i] = CLIP_NONE;
		}
		const real_t s = Math::abs(speed[i]);
		const int c = s > (clip[i] == CLIP_WALK ? WALK_OFF : WALK_ON) ? CLIP_WALK : CLIP_STAND;
		if (c != clip[i]) {
			clip[i] = (uint8_t)c;
			player->play(_clip_name(c), CLIP_BLEND);
		}
		player->set_speed_scale(c == CLIP_WALK ? _walk_rate(speed[i]) : 1.0f);
		player->advance(anim_dt[i]);
		anim_dt[i] = 0.0f;
	}
	last_frame_usec = (int)(OS::get_singleton()->get_ticks_usec() - t0);
}

int EnemyViews::view_tier(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), TIER_OFF);
	return tier[p_h];
}

int EnemyViews::view_clip(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), CLIP_STAND);
	return clip[p_h];
}

int EnemyViews::get_tier_count(int p_tier) const {
	ERR_FAIL_INDEX_V(p_tier, 4, 0);
	return tier_counts[p_tier];
}

int EnemyViews::get_view_count() const {
	int n = 0;
	for (uint32_t i = 0; i < used.size(); i++) {
		n += used[i];
	}
	return n;
}

void EnemyViews::set_tier_override(int p_tier) {
	ERR_FAIL_COND(p_tier < -1 || p_tier > TIER_FAR);
	tier_override = p_tier;
}

void EnemyViews::_bind_methods() {
	ClassDB::bind_method(D_METHOD("view_add", "root", "player"), &EnemyViews::view_add);
	ClassDB::bind_method(D_METHOD("view_remove", "handle"), &EnemyViews::view_remove);
	ClassDB::bind_method(D_METHOD("view_commit", "handle", "pose", "step_lag", "speed", "space"), &EnemyViews::view_commit, DEFVAL(-1));
	ClassDB::bind_method(D_METHOD("set_root", "root"), &EnemyViews::set_root);
	ClassDB::bind_method(D_METHOD("space_add", "frame"), &EnemyViews::space_add);
	ClassDB::bind_method(D_METHOD("get_space_count"), &EnemyViews::get_space_count);
	ClassDB::bind_method(D_METHOD("view_set_active", "handle", "active"), &EnemyViews::view_set_active);
	ClassDB::bind_method(D_METHOD("view_reset", "handle"), &EnemyViews::view_reset);
	ClassDB::bind_method(D_METHOD("view_play_action", "handle", "clip"), &EnemyViews::view_play_action);
	ClassDB::bind_method(D_METHOD("view_is_acting", "handle"), &EnemyViews::view_is_acting);
	ClassDB::bind_method(D_METHOD("frame", "alpha", "camera", "delta"), &EnemyViews::frame);
	ClassDB::bind_method(D_METHOD("view_tier", "handle"), &EnemyViews::view_tier);
	ClassDB::bind_method(D_METHOD("view_clip", "handle"), &EnemyViews::view_clip);
	ClassDB::bind_method(D_METHOD("get_tier_count", "tier"), &EnemyViews::get_tier_count);
	ClassDB::bind_method(D_METHOD("get_view_count"), &EnemyViews::get_view_count);
	ClassDB::bind_method(D_METHOD("get_last_frame_usec"), &EnemyViews::get_last_frame_usec);
	ClassDB::bind_method(D_METHOD("set_tier_override", "tier"), &EnemyViews::set_tier_override);

	BIND_ENUM_CONSTANT(TIER_NEAR);
	BIND_ENUM_CONSTANT(TIER_MID);
	BIND_ENUM_CONSTANT(TIER_FAR);
	BIND_ENUM_CONSTANT(TIER_OFF);
	BIND_ENUM_CONSTANT(CLIP_STAND);
	BIND_ENUM_CONSTANT(CLIP_WALK);
	BIND_ENUM_CONSTANT(CLIP_NONE);
}
