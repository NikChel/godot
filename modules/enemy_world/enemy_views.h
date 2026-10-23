#pragma once

#include "core/math/transform_3d.h"
#include "core/object/object_id.h"
#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"
#include "core/variant/type_info.h"

class AnimationPlayer;
class Node3D;

// Визуал врагов карты (этап 3 EnemyWorld; план docs/superpowers/plans/2026-10-21-enemy-world-stage3-anim.md):
// один проход кадра рендера на всех (AGENTS §2.9). Тик врага сдаёт позу (view_commit: тело в мире с базисом
// торса, отставание подъёма на ступень, скорость вдоль торса); кадр (frame) ставит аватар между двумя
// последними тиками минус отставание, выбирает уровень по расстоянию до камеры (гистерезис TIER_HYST) и
// клип модели (стойка / шаг, гистерезис WALK_ON / WALK_OFF, темп — как AvatarAnimator игрока) и шагает
// AnimationPlayer (режим MANUAL) накопленным временем: NEAR — каждый кадр, MID — раз в MID_EVERY, FAR — раз
// в FAR_EVERY кадров (кадры шага раскиданы по ручкам). Поза — каждый кадр у всех. Пишет только аватары,
// состояние тика не читает и не пишет (AGENTS §2.2). Клипы — библиотека «walker» аватара (stay, beg; разовые —
// view_play_action: играет до конца со скоростью 1, затем снова стойка / шаг).
class EnemyViews : public RefCounted {
	GDCLASS(EnemyViews, RefCounted);

public:
	enum Tier {
		TIER_NEAR,
		TIER_MID,
		TIER_FAR,
		TIER_OFF,
	};

	enum Clip {
		CLIP_STAND,
		CLIP_WALK,
		CLIP_NONE = 255, // после разового клипа — выбрать заново
	};

	static constexpr real_t NEAR_R = 8.0f;
	static constexpr real_t FAR_R = 20.0f;
	static constexpr real_t TIER_HYST = 1.0f;
	static constexpr int MID_EVERY = 3;
	static constexpr int FAR_EVERY = 8;
	static constexpr real_t WALK_ON = 0.4f; // м/с
	static constexpr real_t WALK_OFF = 0.2f;
	static constexpr real_t CLIP_BLEND = 0.2f; // с
	static constexpr real_t ACTION_BLEND = 0.1f; // с
	// Темп шага — как AvatarAnimator игрока (blend_speed, run_speed, run_rate, max_rate)
	static constexpr real_t BLEND_SPEED = 0.5f;
	static constexpr real_t RUN_SPEED = 4.5f;
	static constexpr real_t RUN_RATE = 2.0f;
	static constexpr real_t MAX_RATE = 2.5f;

private:
	LocalVector<ObjectID> root_id;
	LocalVector<ObjectID> player_id;
	LocalVector<Transform3D> pose_prev;
	LocalVector<Transform3D> pose_curr;
	LocalVector<real_t> lag_prev;
	LocalVector<real_t> lag_curr;
	LocalVector<real_t> speed;
	LocalVector<real_t> anim_dt;
	LocalVector<uint8_t> tier;
	LocalVector<uint8_t> clip;
	LocalVector<uint8_t> used;
	LocalVector<uint8_t> has_pose;
	LocalVector<uint8_t> acting;
	LocalVector<StringName> action;
	LocalVector<int> free_handles;
	uint64_t frame_no = 0;
	int tier_override = -1;
	int last_frame_usec = 0;
	int tier_counts[4] = {};

	static StringName _clip_name(int p_clip);
	static real_t _walk_rate(real_t p_speed);
	static int _tier_for(int p_tier, real_t p_dist);
	bool _valid(int p_h) const;

protected:
	static void _bind_methods();

public:
	int view_add(Node3D *p_root, AnimationPlayer *p_player);
	void view_remove(int p_h);
	void view_commit(int p_h, const Transform3D &p_pose, real_t p_step_lag, real_t p_speed);
	void view_set_active(int p_h, bool p_active);
	void view_reset(int p_h); // Следующая поза — без интерполяции (телепорт)
	void view_play_action(int p_h, const StringName &p_clip);
	bool view_is_acting(int p_h) const;
	void frame(real_t p_alpha, const Vector3 &p_camera, real_t p_delta);

	int view_tier(int p_h) const;
	int view_clip(int p_h) const;
	int get_tier_count(int p_tier) const;
	int get_view_count() const;
	int get_last_frame_usec() const { return last_frame_usec; }
	void set_tier_override(int p_tier);
};

VARIANT_ENUM_CAST(EnemyViews::Tier);
VARIANT_ENUM_CAST(EnemyViews::Clip);
