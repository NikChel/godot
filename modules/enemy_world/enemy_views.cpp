#include "enemy_views.h"

#include "core/config/engine.h"
#include "core/object/class_db.h"
#include "core/os/os.h"
#include "scene/3d/node_3d.h"
#include "scene/3d/skeleton_3d.h"

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

int EnemyViews::view_add(Node3D *p_root, Skeleton3D *p_skeleton, const Ref<EnemyRig> &p_rig) {
	ERR_FAIL_NULL_V(p_root, -1);
	int h;
	if (free_handles.size() > 0) {
		h = free_handles[free_handles.size() - 1];
		free_handles.remove_at(free_handles.size() - 1);
	} else {
		h = (int)used.size();
		const uint32_t n = used.size() + 1;
		root_id.resize(n);
		skel_id.resize(n);
		rig.resize(n);
		pose_prev.resize(n);
		pose_curr.resize(n);
		space_prev.resize(n);
		space_curr.resize(n);
		lag_prev.resize(n);
		lag_curr.resize(n);
		anim_prev.resize(n);
		anim_curr.resize(n);
		tilt_prev.resize(n * EnemyRig::ARMS);
		tilt_curr.resize(n * EnemyRig::ARMS);
		tier.resize(n);
		used.resize(n);
		has_pose.resize(n);
		has_anim.resize(n);
	}
	root_id[h] = p_root->get_instance_id();
	skel_id[h] = p_skeleton ? p_skeleton->get_instance_id() : ObjectID();
	rig[h] = p_rig;
	pose_prev[h] = Transform3D();
	pose_curr[h] = Transform3D();
	space_prev[h] = -1;
	space_curr[h] = -1;
	lag_prev[h] = 0.0f;
	lag_curr[h] = 0.0f;
	anim_prev[h] = EnemyAnim();
	anim_curr[h] = EnemyAnim();
	for (int s = 0; s < EnemyRig::ARMS; s++) {
		tilt_prev[h * EnemyRig::ARMS + s] = Quaternion();
		tilt_curr[h * EnemyRig::ARMS + s] = Quaternion();
	}
	tier[h] = TIER_NEAR;
	used[h] = 1;
	has_pose[h] = 0;
	has_anim[h] = 0;
	return h;
}

void EnemyViews::view_remove(int p_h) {
	ERR_FAIL_COND(!_valid(p_h));
	used[p_h] = 0;
	root_id[p_h] = ObjectID();
	skel_id[p_h] = ObjectID();
	rig[p_h].unref();
	free_handles.push_back(p_h);
}

void EnemyViews::view_commit(int p_h, const Transform3D &p_pose, real_t p_step_lag, int p_space) {
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
}

void EnemyViews::view_anim(int p_h, const EnemyAnim &p_anim, const Quaternion p_tilt[EnemyRig::ARMS]) {
	ERR_FAIL_COND(!_valid(p_h));
	const uint32_t k = (uint32_t)p_h * EnemyRig::ARMS;
	if (has_anim[p_h]) {
		anim_prev[p_h] = anim_curr[p_h];
		for (int s = 0; s < EnemyRig::ARMS; s++) {
			tilt_prev[k + s] = tilt_curr[k + s];
		}
	} else {
		anim_prev[p_h] = p_anim;
		for (int s = 0; s < EnemyRig::ARMS; s++) {
			tilt_prev[k + s] = p_tilt[s];
		}
		has_anim[p_h] = 1;
	}
	anim_curr[p_h] = p_anim;
	for (int s = 0; s < EnemyRig::ARMS; s++) {
		tilt_curr[k + s] = p_tilt[s];
	}
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
}

void EnemyViews::view_reset(int p_h) {
	ERR_FAIL_COND(!_valid(p_h));
	has_pose[p_h] = 0;
}

void EnemyViews::frame(real_t p_alpha, const Vector3 &p_camera, real_t p_delta) {
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	frame_no++;
	const real_t tick_dt = 1.0f / (real_t)MAX(1, Engine::get_singleton()->get_physics_ticks_per_second());
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
		if (root == nullptr) {
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
		const uint64_t every = (uint64_t)(t == TIER_NEAR ? 1 : (t == TIER_MID ? MID_EVERY : FAR_EVERY));
		if ((frame_no + i) % every != 0 || !has_anim[i] || rig[i].is_null() || !rig[i]->is_built()) {
			continue;
		}
		Skeleton3D *skel = ObjectDB::get_instance<Skeleton3D>(skel_id[i]);
		if (skel == nullptr) {
			continue;
		}
		const uint32_t k = i * EnemyRig::ARMS;
		Quaternion tilt[EnemyRig::ARMS];
		for (int s = 0; s < EnemyRig::ARMS; s++) {
			tilt[s] = tilt_prev[k + s].slerp(tilt_curr[k + s], p_alpha);
		}
		rig[i]->apply(anim_prev[i].advanced(p_alpha * tick_dt), tilt, skel);
	}
	last_frame_usec = (int)(OS::get_singleton()->get_ticks_usec() - t0);
}

int EnemyViews::view_tier(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), TIER_OFF);
	return tier[p_h];
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
	ClassDB::bind_method(D_METHOD("view_add", "root", "skeleton", "rig"), &EnemyViews::view_add);
	ClassDB::bind_method(D_METHOD("view_remove", "handle"), &EnemyViews::view_remove);
	ClassDB::bind_method(D_METHOD("view_commit", "handle", "pose", "step_lag", "space"), &EnemyViews::view_commit, DEFVAL(-1));
	ClassDB::bind_method(D_METHOD("set_root", "root"), &EnemyViews::set_root);
	ClassDB::bind_method(D_METHOD("space_add", "frame"), &EnemyViews::space_add);
	ClassDB::bind_method(D_METHOD("get_space_count"), &EnemyViews::get_space_count);
	ClassDB::bind_method(D_METHOD("view_set_active", "handle", "active"), &EnemyViews::view_set_active);
	ClassDB::bind_method(D_METHOD("view_reset", "handle"), &EnemyViews::view_reset);
	ClassDB::bind_method(D_METHOD("frame", "alpha", "camera", "delta"), &EnemyViews::frame);
	ClassDB::bind_method(D_METHOD("view_tier", "handle"), &EnemyViews::view_tier);
	ClassDB::bind_method(D_METHOD("get_tier_count", "tier"), &EnemyViews::get_tier_count);
	ClassDB::bind_method(D_METHOD("get_view_count"), &EnemyViews::get_view_count);
	ClassDB::bind_method(D_METHOD("get_last_frame_usec"), &EnemyViews::get_last_frame_usec);
	ClassDB::bind_method(D_METHOD("set_tier_override", "tier"), &EnemyViews::set_tier_override);

	BIND_ENUM_CONSTANT(TIER_NEAR);
	BIND_ENUM_CONSTANT(TIER_MID);
	BIND_ENUM_CONSTANT(TIER_FAR);
	BIND_ENUM_CONSTANT(TIER_OFF);
}
