#pragma once

#include "core/math/transform_3d.h"
#include "core/object/object_id.h"
#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"
#include "core/variant/type_info.h"
#include "enemy_anim.h"
#include "enemy_rig.h"

class Node3D;
class Skeleton3D;

// Визуал врагов карты (этап 3 EnemyWorld; план docs/superpowers/plans/2026-10-21-enemy-world-stage3-anim.md):
// один проход кадра рендера на всех (AGENTS §2.9). Тик врага сдаёт позу (view_commit: тело с базисом торса в
// системе пространства — корень карты set_root, подвижные системы space_add; −1 — в мире; отставание подъёма на
// ступень, скорость вдоль торса); кадр (frame) ставит аватар между двумя последними тиками в позе пространства
// кадра (render_xform системы — как кабина и игрок в ней; spec 2026-10-23-enemy-moving-spaces §6; тики разных
// пространств — смешение в мире) минус отставание, выбирает уровень по расстоянию до камеры (гистерезис TIER_HYST) и
// ставит кости скелета: NEAR — каждый кадр, MID — раз в MID_EVERY, FAR — раз в FAR_EVERY кадров (кадры раскиданы по
// ручкам). Анимация — состояние тика (view_anim: EnemyAnim и наклоны рук; spec 2026-10-27-walker-arms-design §2):
// кадр показывает прошлый тик, продвинутый на долю кадра (alpha / ticks), позы костей — EnemyRig модели. Поза —
// каждый кадр у всех. Пишет только аватары, состояние тика не читает и не пишет (AGENTS §2.2).
class EnemyViews : public RefCounted {
	GDCLASS(EnemyViews, RefCounted);

public:
	enum Tier {
		TIER_NEAR,
		TIER_MID,
		TIER_FAR,
		TIER_OFF,
	};

	static constexpr real_t NEAR_R = 8.0f;
	static constexpr real_t FAR_R = 20.0f;
	static constexpr real_t TIER_HYST = 1.0f;
	static constexpr int MID_EVERY = 3;
	static constexpr int FAR_EVERY = 8;

private:
	LocalVector<ObjectID> root_id;
	LocalVector<ObjectID> skel_id;
	LocalVector<Ref<EnemyRig>> rig;
	LocalVector<Transform3D> pose_prev;
	LocalVector<Transform3D> pose_curr;
	LocalVector<int> space_prev; // Пространство позы (−1 — мир)
	LocalVector<int> space_curr;

	struct Space {
		ObjectID node_id;
		bool frame = false; // Подвижная система (render_xform); нет — корень карты (global_transform)
		Transform3D xf; // Поза кадра
	};
	LocalVector<Space> spaces;
	LocalVector<real_t> lag_prev;
	LocalVector<real_t> lag_curr;
	LocalVector<EnemyAnim> anim_prev;
	LocalVector<EnemyAnim> anim_curr;
	LocalVector<Quaternion> tilt_prev; // Наклоны рук: по EnemyRig::ARMS на вид
	LocalVector<Quaternion> tilt_curr;
	LocalVector<uint8_t> tier;
	LocalVector<uint8_t> used;
	LocalVector<uint8_t> has_pose;
	LocalVector<uint8_t> has_anim;
	LocalVector<int> free_handles;
	uint64_t frame_no = 0;
	int tier_override = -1;
	int last_frame_usec = 0;
	int tier_counts[4] = {};

	static int _tier_for(int p_tier, real_t p_dist);
	bool _valid(int p_h) const;
	Transform3D _space_xf(int p_space) const;

protected:
	static void _bind_methods();

public:
	int view_add(Node3D *p_root, Skeleton3D *p_skeleton, const Ref<EnemyRig> &p_rig);
	void view_remove(int p_h);
	void view_commit(int p_h, const Transform3D &p_pose, real_t p_step_lag, int p_space = -1);
	void view_anim(int p_h, const EnemyAnim &p_anim, const Quaternion p_tilt[EnemyRig::ARMS]); // Анимация тика (после view_commit)
	void set_root(Node3D *p_root); // Пространство 0 — корень карты (сбрасывает реестр)
	int space_add(Node3D *p_frame); // Подвижная система (ReferenceFrame: render_xform) — следующий номер
	int get_space_count() const { return (int)spaces.size(); }
	void view_set_active(int p_h, bool p_active);
	void view_reset(int p_h); // Следующая поза — без интерполяции (телепорт)
	void frame(real_t p_alpha, const Vector3 &p_camera, real_t p_delta);

	int view_tier(int p_h) const;
	int get_tier_count(int p_tier) const;
	int get_view_count() const;
	int get_last_frame_usec() const { return last_frame_usec; }
	void set_tier_override(int p_tier);
};

VARIANT_ENUM_CAST(EnemyViews::Tier);
