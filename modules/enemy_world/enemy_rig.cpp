#include "enemy_rig.h"

#include "core/math/math_funcs.h"
#include "core/object/class_db.h"
#include "scene/3d/node_3d.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/animation/animation_player.h"

bool EnemyRig::build(Node3D *p_root, Skeleton3D *p_skeleton, AnimationPlayer *p_player, const PackedStringArray &p_clips, const PackedStringArray &p_arm_bones) {
	ERR_FAIL_NULL_V(p_root, false);
	ERR_FAIL_NULL_V(p_skeleton, false);
	ERR_FAIL_NULL_V(p_player, false);
	ERR_FAIL_COND_V_MSG(p_clips.size() < 2, false, "EnemyRig: clips — stand, walk, then actions.");
	ERR_FAIL_COND_V_MSG(p_arm_bones.size() != 2 * ARMS, false, "EnemyRig: arm bones — upper and fore, left then right.");
	ERR_FAIL_COND_V_MSG(!p_root->is_inside_tree() || !p_skeleton->is_inside_tree(), false, "EnemyRig: root and skeleton must be in the tree.");
	Node *anim_root = p_player->get_node_or_null(p_player->get_root_node());
	ERR_FAIL_NULL_V_MSG(anim_root, false, "EnemyRig: AnimationPlayer root node not found.");

	const int n = p_skeleton->get_bone_count();
	parent.resize(n);
	rest.resize(n);
	rest_rot.resize(n);
	has_pos.resize(n);
	anim_bone.resize(n);
	for (int b = 0; b < n; b++) {
		parent[b] = p_skeleton->get_bone_parent(b);
		rest[b] = p_skeleton->get_bone_rest(b);
		rest_rot[b] = rest[b].basis.get_rotation_quaternion();
		has_pos[b] = 0;
		anim_bone[b] = 0;
	}
	lean_rot.clear();
	lean_pos.clear();
	lean_has.clear();
	lean_w = 0.0f;
	spin_bone.clear();
	spin_rate.clear();

	clips.clear();
	clip_index.clear();
	for (int i = 0; i < p_clips.size(); i++) {
		const StringName name = p_clips[i];
		Clip c;
		if (!_load_clip(anim_root, p_skeleton, p_player, name, c)) {
			return false;
		}
		clip_index[name] = (int)clips.size();
		clips.push_back(c);
	}

	for (int s = 0; s < ARMS; s++) {
		upper[s] = p_skeleton->find_bone(p_arm_bones[2 * s]);
		fore[s] = p_skeleton->find_bone(p_arm_bones[2 * s + 1]);
		ERR_FAIL_COND_V_MSG(upper[s] < 0 || fore[s] < 0, false, vformat("EnemyRig: no arm bone %s / %s.", p_arm_bones[2 * s], p_arm_bones[2 * s + 1]));
		chain[s].clear();
		bool through_upper = false;
		for (int b = fore[s]; b >= 0; b = parent[b]) {
			chain[s].insert(0, b);
			through_upper = through_upper || b == upper[s];
		}
		ERR_FAIL_COND_V_MSG(!through_upper, false, "EnemyRig: fore bone is not below the upper bone.");
		anim_bone[upper[s]] = 1;
	}
	_rebuild_written();
	skel_xf = p_root->get_global_transform().affine_inverse() * p_skeleton->get_global_transform();
	built = true;
	return true;
}

// Клип p_name из p_player: дорожки поворота и позиции костей p_skeleton (их кости — в anim_bone / has_pos).
bool EnemyRig::_load_clip(Node *p_anim_root, Skeleton3D *p_skeleton, AnimationPlayer *p_player, const StringName &p_name, Clip &r_clip) {
	ERR_FAIL_COND_V_MSG(!p_player->has_animation(p_name), false, vformat("EnemyRig: no clip %s.", p_name));
	const int n = (int)parent.size();
	r_clip.name = p_name;
	r_clip.anim = p_player->get_animation(p_name);
	r_clip.length = (real_t)r_clip.anim->get_length();
	r_clip.loop = r_clip.anim->get_loop_mode() != Animation::LOOP_NONE;
	r_clip.rot.resize(n);
	r_clip.pos.resize(n);
	for (int b = 0; b < n; b++) {
		r_clip.rot[b] = -1;
		r_clip.pos[b] = -1;
	}
	for (int t = 0; t < r_clip.anim->get_track_count(); t++) {
		const Animation::TrackType type = r_clip.anim->track_get_type(t);
		if ((type != Animation::TYPE_ROTATION_3D && type != Animation::TYPE_POSITION_3D) || !r_clip.anim->track_is_enabled(t)) {
			continue;
		}
		const NodePath path = r_clip.anim->track_get_path(t);
		if (path.get_subname_count() != 1) {
			continue;
		}
		Node *target = p_anim_root->get_node_or_null(NodePath(path.get_names(), path.is_absolute()));
		if (target != p_skeleton) {
			continue;
		}
		const int b = p_skeleton->find_bone(path.get_subname(0));
		if (b < 0) {
			continue;
		}
		anim_bone[b] = 1;
		if (type == Animation::TYPE_ROTATION_3D) {
			r_clip.rot[b] = t;
		} else {
			r_clip.pos[b] = t;
			has_pos[b] = 1;
		}
	}
	return true;
}

void EnemyRig::_rebuild_written() {
	written.clear();
	for (uint32_t b = 0; b < anim_bone.size(); b++) {
		if (anim_bone[b]) {
			written.push_back((int)b);
		}
	}
}

bool EnemyRig::set_lean(Skeleton3D *p_skeleton, AnimationPlayer *p_player, const PackedStringArray &p_clips, real_t p_time) {
	ERR_FAIL_COND_V_MSG(!built, false, "EnemyRig: set_lean after build.");
	ERR_FAIL_NULL_V(p_skeleton, false);
	ERR_FAIL_NULL_V(p_player, false);
	ERR_FAIL_COND_V_MSG(p_clips.size() != LEANS, false, "EnemyRig: lean clips — forward, back, left, right.");
	ERR_FAIL_COND_V_MSG(p_skeleton->get_bone_count() != (int)parent.size(), false, "EnemyRig: lean — not the rig's skeleton.");
	Node *anim_root = p_player->get_node_or_null(p_player->get_root_node());
	ERR_FAIL_NULL_V_MSG(anim_root, false, "EnemyRig: AnimationPlayer root node not found.");
	const int n = (int)parent.size();
	lean_rot.resize(LEANS * n);
	lean_pos.resize(LEANS * n);
	lean_has.resize(LEANS * n);
	for (int i = 0; i < LEANS; i++) {
		Clip c;
		if (!_load_clip(anim_root, p_skeleton, p_player, p_clips[i], c)) {
			lean_rot.clear();
			lean_pos.clear();
			lean_has.clear();
			lean_w = 0.0f;
			return false;
		}
		for (int b = 0; b < n; b++) {
			const int k = i * n + b;
			lean_has[k] = (c.rot[b] >= 0 || c.pos[b] >= 0) ? 1 : 0;
			lean_rot[k] = Quaternion();
			lean_pos[k] = Vector3();
			if (!lean_has[k]) {
				continue;
			}
			Quaternion q0; // Клип стойки, начало
			Vector3 p0;
			_clip_pose(clips[0], 0.0f, b, q0, p0);
			Quaternion q;
			Vector3 p;
			_clip_pose(c, 0.0f, b, q, p);
			lean_rot[k] = (q0.inverse() * q).normalized();
			lean_pos[k] = p - p0;
		}
	}
	lean_w = LEAN_95 / MAX(p_time, (real_t)0.01f);
	_rebuild_written();
	return true;
}

bool EnemyRig::set_spin(Skeleton3D *p_skeleton, const PackedStringArray &p_bones, const Vector3 &p_axis, const PackedFloat32Array &p_rates) {
	ERR_FAIL_COND_V_MSG(!built, false, "EnemyRig: set_spin after build.");
	ERR_FAIL_NULL_V(p_skeleton, false);
	ERR_FAIL_COND_V_MSG(p_bones.size() != p_rates.size(), false, "EnemyRig: spin — one rate per bone.");
	ERR_FAIL_COND_V_MSG(p_axis.length_squared() < 1e-8f, false, "EnemyRig: spin axis is zero.");
	spin_bone.clear();
	spin_rate.clear();
	for (int i = 0; i < p_bones.size(); i++) {
		const int b = p_skeleton->find_bone(p_bones[i]);
		ERR_FAIL_COND_V_MSG(b < 0 || b >= (int)parent.size(), false, vformat("EnemyRig: no spin bone %s.", p_bones[i]));
		spin_bone.push_back(b);
		spin_rate.push_back(p_rates[i]);
		anim_bone[b] = 1;
	}
	spin_axis = p_axis.normalized();
	_rebuild_written();
	return true;
}

int EnemyRig::clip_find(const StringName &p_name) const {
	const int *i = clip_index.getptr(p_name);
	return i != nullptr ? *i : -1;
}

real_t EnemyRig::clip_length(int p_clip) const {
	ERR_FAIL_INDEX_V(p_clip, (int)clips.size(), 0.0f);
	return clips[p_clip].length;
}

real_t EnemyRig::clip_wrap(int p_clip, real_t p_time) const {
	if (p_clip < 0 || p_clip >= (int)clips.size()) {
		return p_time;
	}
	const Clip &c = clips[p_clip];
	return (c.loop && c.length > 0.0f) ? Math::fposmod(p_time, c.length) : p_time;
}

real_t EnemyRig::_clip_time(const Clip &p_c, real_t p_t) const {
	if (p_c.length <= 0.0f) {
		return 0.0f;
	}
	return p_c.loop ? Math::fposmod(p_t, p_c.length) : CLAMP(p_t, (real_t)0.0, p_c.length);
}

void EnemyRig::_clip_pose(const Clip &p_c, real_t p_t, int p_bone, Quaternion &r_rot, Vector3 &r_pos) const {
	r_rot = rest_rot[p_bone];
	r_pos = rest[p_bone].origin;
	const double t = _clip_time(p_c, p_t);
	if (p_c.rot[p_bone] >= 0) {
		Quaternion q;
		if (p_c.anim->try_rotation_track_interpolate(p_c.rot[p_bone], t, &q) == OK) {
			r_rot = q;
		}
	}
	if (p_c.pos[p_bone] >= 0) {
		Vector3 p;
		if (p_c.anim->try_position_track_interpolate(p_c.pos[p_bone], t, &p) == OK) {
			r_pos = p;
		}
	}
}

void EnemyRig::_pose(const EnemyAnim &p_a, int p_bone, Quaternion &r_rot, Vector3 &r_pos) const {
	if (p_a.clip < 0 || p_a.clip >= (int)clips.size()) {
		r_rot = rest_rot[p_bone];
		r_pos = rest[p_bone].origin;
	} else {
		_clip_pose(clips[p_a.clip], p_a.time, p_bone, r_rot, r_pos);
		const real_t w = p_a.weight();
		if (p_a.from >= 0 && p_a.from < (int)clips.size() && w < 1.0f) {
			Quaternion q;
			Vector3 p;
			_clip_pose(clips[p_a.from], p_a.from_time, p_bone, q, p);
			r_rot = q.slerp(r_rot, w);
			r_pos = p.lerp(r_pos, w);
		}
	}
	if (lean_w > 0.0f) { // Наклон по ходу поверх клипов: вперёд / назад, затем вбок
		_lean(p_a.lean.y >= 0.0f ? LEAN_FWD : LEAN_BACK, Math::abs(p_a.lean.y), p_bone, r_rot, r_pos);
		_lean(p_a.lean.x >= 0.0f ? LEAN_RIGHT : LEAN_LEFT, Math::abs(p_a.lean.x), p_bone, r_rot, r_pos);
	}
}

// Доля p_w разницы клипа направления p_lean у кости (кадр кости) — поверх позы.
void EnemyRig::_lean(int p_lean, real_t p_w, int p_bone, Quaternion &r_rot, Vector3 &r_pos) const {
	const uint32_t k = (uint32_t)p_lean * parent.size() + (uint32_t)p_bone;
	if (p_w <= 1e-5f || !lean_has[k]) {
		return;
	}
	const real_t w = MIN(p_w, (real_t)1.0f);
	r_rot = (r_rot * Quaternion().slerp(lean_rot[k], w)).normalized();
	r_pos += lean_pos[k] * w;
}

Quaternion EnemyRig::_spin(int p_bone, double p_time) const {
	for (uint32_t i = 0; i < spin_bone.size(); i++) {
		if (spin_bone[i] == p_bone) {
			const double turns = Math::fmod((double)spin_rate[i] * p_time, 1.0);
			return Quaternion(spin_axis, (real_t)(Math::TAU * turns));
		}
	}
	return Quaternion();
}

void EnemyRig::arm_frames(const EnemyAnim &p_a, int p_side, Transform3D &r_upper, Transform3D &r_fore) const {
	ERR_FAIL_COND(!built);
	ERR_FAIL_INDEX(p_side, ARMS);
	Transform3D g;
	Transform3D up;
	for (const int b : chain[p_side]) {
		Quaternion q;
		Vector3 p;
		_pose(p_a, b, q, p);
		g = g * Transform3D(Basis(q), p);
		if (b == upper[p_side]) {
			up = g;
		}
	}
	r_upper = skel_xf * up;
	r_upper.basis.orthonormalize();
	r_fore = skel_xf * g;
	r_fore.basis.orthonormalize();
}

Transform3D EnemyRig::arm_rest(int p_side) const {
	ERR_FAIL_COND_V(!built, Transform3D());
	ERR_FAIL_INDEX_V(p_side, ARMS, Transform3D());
	Transform3D g;
	bool below = false;
	for (const int b : chain[p_side]) {
		if (below) {
			g = g * rest[b];
		}
		below = below || b == upper[p_side];
	}
	g.basis.orthonormalize();
	return g;
}

void EnemyRig::apply(const EnemyAnim &p_a, const Quaternion p_tilt[ARMS], Skeleton3D *p_skeleton, double p_spin_time) const {
	ERR_FAIL_COND(!built);
	ERR_FAIL_NULL(p_skeleton);
	for (const int b : written) {
		Quaternion q;
		Vector3 p;
		_pose(p_a, b, q, p);
		for (int s = 0; s < ARMS; s++) {
			if (b == upper[s]) {
				q = q * p_tilt[s];
			}
		}
		if (!spin_bone.is_empty()) {
			q = q * _spin(b, p_spin_time);
		}
		p_skeleton->set_bone_pose_rotation(b, q);
		if (has_pos[b]) {
			p_skeleton->set_bone_pose_position(b, p);
		}
	}
}

void EnemyRig::apply_spin(const EnemyAnim &p_a, Skeleton3D *p_skeleton, double p_spin_time) const {
	ERR_FAIL_COND(!built);
	ERR_FAIL_NULL(p_skeleton);
	for (uint32_t i = 0; i < spin_bone.size(); i++) {
		const int b = spin_bone[i];
		Quaternion q;
		Vector3 p;
		_pose(p_a, b, q, p);
		p_skeleton->set_bone_pose_rotation(b, q * _spin(b, p_spin_time));
	}
}

void EnemyRig::_bind_methods() {
	ClassDB::bind_method(D_METHOD("build", "root", "skeleton", "player", "clips", "arm_bones"), &EnemyRig::build);
	ClassDB::bind_method(D_METHOD("is_built"), &EnemyRig::is_built);
	ClassDB::bind_method(D_METHOD("clip_find", "name"), &EnemyRig::clip_find);
	ClassDB::bind_method(D_METHOD("get_clip_count"), &EnemyRig::get_clip_count);
	ClassDB::bind_method(D_METHOD("clip_length", "clip"), &EnemyRig::clip_length);
	ClassDB::bind_method(D_METHOD("set_lean", "skeleton", "player", "clips", "time"), &EnemyRig::set_lean);
	ClassDB::bind_method(D_METHOD("has_lean"), &EnemyRig::has_lean);
	ClassDB::bind_method(D_METHOD("lean_omega"), &EnemyRig::lean_omega);
	ClassDB::bind_method(D_METHOD("set_spin", "skeleton", "bones", "axis", "rates"), &EnemyRig::set_spin);
	ClassDB::bind_method(D_METHOD("has_spin"), &EnemyRig::has_spin);
}
