#pragma once

#include "enemy_anim.h"

#include "core/math/quaternion.h"
#include "core/math/transform_3d.h"
#include "core/object/ref_counted.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"
#include "scene/resources/animation.h"

class AnimationPlayer;
class Node3D;
class Skeleton3D;

// Риг модели врага (spec docs/superpowers/specs/2026-10-27-walker-arms-design.md §2): клипы библиотеки аватара и
// скелет — один на модель, общий для всех врагов её типа (WalkerAvatar строит при первом аватаре). Выборка локальных
// поз костей по состоянию EnemyAnim (смешивание: поворот — slerp, позиция — lerp; кости без дорожки — покой):
// физ-тик — только цепочка рук (кадры костей плеча и предплечья в системе корня аватара — EnemyArms), кадр —
// все анимированные кости в Skeleton3D (EnemyViews) и наклон руки поверх клипа (поворот в кадре кости плеча).
// Клипы по порядку build: 0 — стойка, 1 — шаг (повторяются), дальше разовые.
class EnemyRig : public RefCounted {
	GDCLASS(EnemyRig, RefCounted);

public:
	static constexpr int ARMS = 2; // 0 — левая, 1 — правая

private:
	struct Clip {
		StringName name;
		Ref<Animation> anim;
		real_t length = 0.0f;
		bool loop = false;
		LocalVector<int> rot; // Дорожка поворота кости (−1 — нет)
		LocalVector<int> pos; // Дорожка позиции кости (−1 — нет)
	};

	LocalVector<int> parent;
	LocalVector<Transform3D> rest;
	LocalVector<Quaternion> rest_rot;
	LocalVector<int> written; // Кости, которые пишет кадр (есть дорожка хоть в одном клипе; плечи рук)
	LocalVector<uint8_t> has_pos; // Позиция кости анимирована хоть в одном клипе
	LocalVector<Clip> clips;
	HashMap<StringName, int> clip_index;
	Transform3D skel_xf; // Скелет в системе корня аватара
	int upper[ARMS] = { -1, -1 }; // Кость плеча
	int fore[ARMS] = { -1, -1 }; // Кость предплечья
	LocalVector<int> chain[ARMS]; // От корня скелета до предплечья
	bool built = false;

	real_t _clip_time(const Clip &p_c, real_t p_t) const;
	void _clip_pose(const Clip &p_c, real_t p_t, int p_bone, Quaternion &r_rot, Vector3 &r_pos) const;
	void _pose(const EnemyAnim &p_a, int p_bone, Quaternion &r_rot, Vector3 &r_pos) const;

protected:
	static void _bind_methods();

public:
	// p_clips — имена в AnimationPlayer («библиотека/клип»): стойка, шаг, разовые; p_arm_bones — плечо и предплечье
	// левой, плечо и предплечье правой. Корень аватара и скелет — в дереве.
	bool build(Node3D *p_root, Skeleton3D *p_skeleton, AnimationPlayer *p_player, const PackedStringArray &p_clips, const PackedStringArray &p_arm_bones);
	bool is_built() const { return built; }
	int clip_find(const StringName &p_name) const; // −1 — нет
	int get_clip_count() const { return (int)clips.size(); }
	real_t clip_length(int p_clip) const;
	real_t clip_wrap(int p_clip, real_t p_time) const; // Повторяемый — по кругу; разовый — как есть (конец виден)
	// Кадры костей плеча и предплечья руки по анимации (система корня аватара, базисы ортонормированы)
	void arm_frames(const EnemyAnim &p_a, int p_side, Transform3D &r_upper, Transform3D &r_fore) const;
	Transform3D arm_rest(int p_side) const; // Покой кости предплечья в кадре кости плеча
	// Кадр: позы анимированных костей — в скелет; плечу руки — наклон p_tilt[сторона] поверх клипа (кадр кости)
	void apply(const EnemyAnim &p_a, const Quaternion p_tilt[ARMS], Skeleton3D *p_skeleton) const;
};
