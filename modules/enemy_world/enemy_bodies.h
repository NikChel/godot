#pragma once

#include "enemy_views.h"
#include "enemy_world.h"

#include "core/math/transform_3d.h"
#include "core/object/object_id.h"
#include "core/object/ref_counted.h"
#include "core/templates/hash_set.h"
#include "core/templates/local_vector.h"
#include "core/variant/type_info.h"
#include "servers/physics_3d/physics_server_3d.h"

class Area3D;
class CharacterBody3D;
class Node3D;

// Тела ходоков карты (этап 2 EnemyWorld; план docs/superpowers/plans/2026-10-22-enemy-world-stage2.md): один проход
// физ-тика на всех ходоков хоста / Solo (AGENTS §2.9) — порт EnemyWalker._physics_process, WalkerBrain, WalkerMover,
// StepUp (для ходока) и FrameBody один в один. Тик (tick), по каждому живому телу: перенос с системой отсчёта →
// мозг (цель — ближайший живой игрок снимка set_players, пересмотр раз в PICK_TICKS; погоня до STRIKE_STOP; в зоне
// удара — замах WINDUP_S, удар, восстановление RECOVER_S; точка пробы — без боя) → желаемая скорость у толпы
// (EnemyWorld; входы на начало тика) → ярус по расстоянию до ближайшего игрока (NEAR ближе NEAR_R — физика тела:
// move_and_slide и шаг на ступень; FAR дальше NEAR_R + TIER_HYST — без физики, по сетке навигации; VR и Flat
// одинаково) → движение и взгляд (yaw вокруг верха системы, не быстрее TURN_RATE; стоящий в толпе (HOLD) смотрит на цель; стоящий на полу без
// скорости — покой: без move_and_slide, полный шаг раз в REST_CHECK_TICKS) → вход толпе, поза визуалу
// (EnemyViews), поза тика для сети (net_pack). Бой и решения — у GDScript: тик возвращает события (пары «вид,
// ручка»: клип замаха, удар, запрос пути в слот ходока, предметы у ног), их решает проход карты (EnemyPass).
// Тела — CharacterBody3D (ObjectID; пропавшее тело пропускается). Координаты толпы — корня карты.
class EnemyBodies : public RefCounted {
	GDCLASS(EnemyBodies, RefCounted);

public:
	enum Brain {
		BRAIN_IDLE,
		BRAIN_CHASE,
		BRAIN_WINDUP,
		BRAIN_RECOVER,
	};

	enum Tier {
		TIER_NEAR,
		TIER_FAR,
	};

	enum Event {
		EVENT_ACTION, // Клип замаха (GDScript: EnemyWalker.play_action)
		EVENT_STRIKE, // Удар (GDScript: EnemyWalker.strike → body_add_hits)
		EVENT_PATH, // Запрос пути (GDScript: NavWorld.query → body_set_path)
		EVENT_ITEMS, // Предметы у ног (GDScript: WalkerItemPush.tick)
	};

	// Мозг (WalkerBrain)
	static constexpr int PICK_TICKS = 10;
	static constexpr real_t STRIKE_STOP = 0.9f;
	static constexpr real_t POINT_STOP = 0.3f;
	static constexpr real_t REACH = 1.3f;
	static constexpr real_t REACH_UP = 1.0f; // Цель выше / ниже по верху тела — не достать (стоит на конусе, на ящике)
	static constexpr real_t SECTOR_DEG = 45.0f;
	static constexpr real_t WINDUP_S = 0.5f;
	static constexpr real_t RECOVER_S = 0.7f;
	// Движение (WalkerMover, EnemyWalker)
	static constexpr real_t ACCEL = 8.0f;
	static constexpr real_t TURN_RATE = 8.0f; // рад/с
	static constexpr real_t STEP_HALFLIFE = 0.05f; // с
	static constexpr real_t FACE_MOVE_SPEED = 0.2f; // Быстрее — смотрит по ходу
	// Покой тела (план 2026-10-22-crowd-hold)
	static constexpr real_t REST_SPEED = 0.05f; // Своя и желаемая скорость ниже — тело в покое
	static constexpr uint64_t REST_CHECK_TICKS = 9; // Полный шаг покоя раз в столько тиков (~0.1 с, сдвиг по слоту)
	// Шаг на ступень (StepUp)
	static constexpr real_t STEP_H = 0.35f;
	static constexpr real_t MIN_TREAD = 0.005f;
	static constexpr real_t EDGE_AHEAD = 0.02f;
	static constexpr real_t EDGE_SPAN = 0.05f;
	static constexpr real_t QUERY_R = 1.5f;
	static constexpr real_t QUERY_UP = 1.0f;
	static constexpr uint32_t ITEM_LAYER = 8;
	// Ярусы (решение пользователя 2026-10-22)
	static constexpr real_t NEAR_R = 12.0f;
	static constexpr real_t TIER_HYST = 1.0f;
	static constexpr int PATH_SLOTS = 16; // = NavWorld.SLOTS

private:
	struct Body {
		bool used = false;
		bool dead = false;
		ObjectID body_id;
		ObjectID zone_id;
		ObjectID frame_id;
		Transform3D frame_last;
		Vector3 up = Vector3(0.0f, 1.0f, 0.0f); // Верх тела (up_direction): у CharacterBody3D в C++ он приватный — держим здесь
		int crowd = -1; // Ручка EnemyWorld
		int view = -1; // Ручка EnemyViews
		int slot = 0;
		real_t speed = 3.0f;
		// Движение
		real_t yaw = 0.0f;
		bool on_step = false;
		real_t lag_prev = 0.0f;
		real_t lag_curr = 0.0f;
		uint8_t tier = TIER_NEAR;
		uint8_t state = EnemyWorld::AGENT_IDLE;
		real_t view_speed = 0.0f;
		real_t want_speed = 0.0f;
		int hint = -1; // Полигон сетки прошлого шага FAR
		Vector3i dead_stats;
		// Мозг
		uint8_t brain = BRAIN_IDLE;
		bool moving = false;
		bool point_set = false;
		Vector3 world;
		real_t stop_dist = STRIKE_STOP;
		Vector3 face;
		ObjectID target;
		uint64_t next_pick = 0;
		uint64_t state_end = 0;
		int strikes = 0;
		int hits = 0;
		// Запрос пути (EVENT_PATH)
		Vector3 path_from;
		Vector3 goal;
		int path_requests = 0;
		// Сеть
		int net_id = -1;
		int64_t net_tick = -1;
		Vector3 net_pos;
	};

	LocalVector<Body> bodies;
	LocalVector<int> free_handles;
	Ref<EnemyWorld> crowd;
	Ref<EnemyViews> views;
	ObjectID root_id;
	Transform3D root_xf;
	Transform3D root_inv;
	bool nav_ready = false;
	int tier_override = -1;
	real_t gravity = 9.8f;
	RID query_sphere;
	PackedVector3Array players_pos;
	PackedByteArray players_alive;
	LocalVector<ObjectID> players_id;
	PackedInt32Array events;
	uint64_t last_tick_usec = 0;
	int tier_counts[2] = {};
	int rest_count = 0;

	bool _valid(int p_h) const;
	void _root_xforms();
	void _emit(int p_kind, int p_h);
	void _tick_body(int p_h, Body &r_b, CharacterBody3D *p_body, uint64_t p_now, real_t p_dt);
	Transform3D _frame_xform(const Body &p_b) const;
	Transform3D _carry(Body &r_b, CharacterBody3D *p_body);
	static Vector3 _forward(const Body &p_b, const Transform3D &p_fx);
	// Мозг
	int _player_index(ObjectID p_id) const;
	bool _player_alive(int p_i) const;
	int _pick(const Vector3 &p_pos) const;
	bool _in_reach(const Body &p_b, CharacterBody3D *p_body, const Transform3D &p_fx, const Vector3 &p_target) const;
	void _brain(int p_h, Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, uint64_t p_now);
	static uint64_t _ticks(real_t p_s);
	// Движение
	void _update_tier(Body &r_b, const Vector3 &p_pos);
	static bool _rest(const Body &p_b, CharacterBody3D *p_body, const Vector3 &p_want, uint8_t p_state, uint64_t p_now);
	void _move_near(Body &r_b, CharacterBody3D *p_body, const Vector3 &p_want, const Vector3 &p_face, real_t p_dt);
	Vector3 _move_far(Body &r_b, CharacterBody3D *p_body, const Vector3 &p_want, const Vector3 &p_face, real_t p_dt);
	static void _lag(Body &r_b, real_t p_rise, real_t p_dt);
	static void _turn(Body &r_b, CharacterBody3D *p_body, const Vector3 &p_face, const Vector3 &p_up, real_t p_dt);
	static real_t _damp(real_t p_curr, real_t p_target, real_t p_halflife, real_t p_dt);
	// Шаг на ступень (StepUp)
	real_t _try_step(CharacterBody3D *p_body, const Vector3 &p_motion_h, const Vector3 &p_up, uint32_t p_mask);
	static bool _wall_in_mask(CharacterBody3D *p_body, const Vector3 &p_dir, const Vector3 &p_up, uint32_t p_mask);
	void _bodies_outside(CharacterBody3D *p_body, const Vector3 &p_up, uint32_t p_mask, HashSet<RID> &r_out);
	static real_t _probe(CharacterBody3D *p_body, PhysicsServer3D::MotionParameters &r_params, const Vector3 &p_fwd, const Vector3 &p_up, uint32_t p_mask);
	static bool _test(CharacterBody3D *p_body, PhysicsServer3D::MotionParameters &r_params, const Transform3D &p_from, const Vector3 &p_motion, PhysicsServer3D::MotionResult &r_res);

protected:
	static void _bind_methods();

public:
	~EnemyBodies();

	// Толпа и сетка карты, визуал, корень карты (координаты толпы)
	void setup(const Ref<EnemyWorld> &p_crowd, const Ref<EnemyViews> &p_views, Node3D *p_root);
	int body_add(CharacterBody3D *p_body, Area3D *p_item_zone, int p_view, int p_slot, real_t p_speed, real_t p_yaw);
	void body_remove(int p_h);
	void body_kill(int p_h); // Смерть: толпа отпускает агента, вид выключен, тик пропускает тело
	void body_set_frame(int p_h, Node3D *p_frame); // Система отсчёта (ReferenceFrame: query_xform), null — мир
	void body_set_point(int p_h, const Vector3 &p_world); // Точка пробы: идти туда без боя
	void body_set_path(int p_h, const PackedVector3Array &p_points, const Vector3 &p_target);
	void body_add_hits(int p_h, int p_hits);
	void body_set_net_id(int p_h, int p_net_id);
	real_t body_yaw(int p_h) const;
	Vector3 body_forward(int p_h) const;
	Transform3D body_frame_xform(int p_h) const;
	int body_state(int p_h) const;
	int body_brain(int p_h) const;
	bool body_moving(int p_h) const;
	int body_strikes(int p_h) const;
	int body_hits(int p_h) const;
	int body_tier(int p_h) const;
	real_t body_want_speed(int p_h) const;
	Vector3 body_path_from(int p_h) const;
	Vector3 body_goal(int p_h) const;
	int body_path_requests(int p_h) const;
	Vector3i body_crowd_stats(int p_h) const;
	int64_t body_net_tick(int p_h) const;
	Vector3 body_net_pos(int p_h) const;

	// Снимок игроков этого тика: позы (мир), живы ли, instance id
	void set_players(const PackedVector3Array &p_pos, const PackedByteArray &p_alive, const PackedInt64Array &p_ids);
	void set_nav_ready(bool p_ready);
	void set_tier_override(int p_tier); // -1 — по расстоянию (пробы: TIER_NEAR / TIER_FAR)
	PackedInt32Array tick(real_t p_delta); // События: пары (Event, ручка)
	Array net_pack() const; // [тик, PackedInt32Array net id, PackedFloat32Array по 6 чисел: x, y, z, yaw, speed, step_lag]
	int get_body_count() const;
	int get_tier_count(int p_tier) const;
	int get_rest_count() const { return rest_count; } // Тел в покое за последний тик
	int64_t get_last_tick_usec() const { return (int64_t)last_tick_usec; }
};

VARIANT_ENUM_CAST(EnemyBodies::Brain);
VARIANT_ENUM_CAST(EnemyBodies::Tier);
VARIANT_ENUM_CAST(EnemyBodies::Event);
