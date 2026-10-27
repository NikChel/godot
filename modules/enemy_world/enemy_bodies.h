#pragma once

#include "enemy_anim.h"
#include "enemy_arms.h"
#include "enemy_links.h"
#include "enemy_rig.h"
#include "enemy_tumbler.h"
#include "enemy_views.h"
#include "enemy_world.h"

#include "core/math/transform_3d.h"
#include "core/object/object_id.h"
#include "core/object/ref_counted.h"
#include "core/templates/hash_map.h"
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
// Тела — CharacterBody3D (ObjectID; пропавшее тело пропускается). Координаты толпы — пространства тела (Space:
// корень карты или подвижная ReferenceFrame; spec 2026-10-23-enemy-moving-spaces §3): пространство меняют правило
// пола (луч вниз: тело опоры — владелец по set_floor_spaces) и телепорт (body_teleport). Связи (план
// 2026-10-22-nav-links-b; enemy_bodies_links.cpp): у входа связи (толпа AGENT_LINK) — ждёт (закрыта, занята, место
// приземления занято — копит закрытие), на связи — поза по траектории профиля без физики (оба яруса), приземлился —
// LAND_S стоит; события — клип прыжка, закрыть / открыть связь (ручка — номер связи).
// Неваляшка (spec 2026-10-26-walker-tumbler-design): толчок (body_push) — наклон (EnemyTumbler) и отброс скорости тела
// (только здесь, у хоста); шаг наклона — в начале тика тела; оглушён — мозг STAGGER (до точки пробы); визуал — поза с
// наклоном, хит-тело — основание (базис системы в точке тела) × наклон в конце тика тела.
// Анимация и руки (spec 2026-10-27-walker-arms-design): часы анимации (EnemyAnim) — шаг в начале тика тела по скорости
// вдоль торса прошлого тика, разовый клип — body_play (замах, прыжок; GDScript по событию); визуалу — с позой; руки
// (EnemyArms: неваляшки на плечах, хит-тела по кадрам костей EnemyRig) — шаг в начале тика, поза в конце.
class EnemyBodies : public RefCounted {
	GDCLASS(EnemyBodies, RefCounted);

public:
	enum Brain {
		BRAIN_IDLE,
		BRAIN_CHASE,
		BRAIN_WINDUP,
		BRAIN_RECOVER,
		BRAIN_STAGGER, // Оглушён (неваляшка): не идёт, не бьёт
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
		EVENT_JUMP, // Клип связи (GDScript: EnemyWalker.play_action(JUMP_CLIP))
		EVENT_LINK_CLOSE, // Закрыть связь в карте (ручка — номер связи; NavLinkSet.set_enabled)
		EVENT_LINK_OPEN, // Открыть связь в карте (ручка — номер связи)
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
	// Связи (план 2026-10-22-nav-links-b)
	static constexpr real_t LAND_FREE_R = 0.8f; // Место приземления занято: другой ходок ближе
	static constexpr real_t LAND_CLOSE_R = 0.5f; // Приземлился ближе к другому — счётчик land_close (проба)
	static constexpr real_t LAND_S = 0.25f; // После приземления стоит
	static constexpr uint32_t LINK_LOG = 16; // Видов пройденных связей в журнале тела (пробы)
	// Пространства (spec 2026-10-23-enemy-moving-spaces §3)
	static constexpr uint64_t FLOOR_CHECK_TICKS = 9; // Правило пола: луч вниз раз в столько тиков (сдвиг по слоту) и при касании
	static constexpr real_t FLOOR_RAY_UP = 0.3f; // Начало луча над ногами
	static constexpr real_t FLOOR_RAY_DOWN = 0.5f; // Конец луча под ногами (длиннее прилипания 0.35)
	static constexpr uint32_t FLOOR_LAYER = 1;

private:
	enum LinkState {
		LINK_NONE,
		LINK_WAIT, // У входа
		LINK_ON, // На связи
		LINK_LAND, // Приземлился (LAND_S)
	};

	struct Space {
		Ref<EnemyWorld> crowd; // Толпа пространства (NavSpace.crowd)
		ObjectID node_id; // Корень карты (пространство 0) или ReferenceFrame
		bool frame = false; // Подвижное: поза — query_xform(), тело переносится с ним; корабль — поза узла, система тела — мир
		bool ready = false; // Сетка собрана (NavSpace.is_ready)
		Transform3D xf; // Поза этого тика (координаты толпы)
		Transform3D inv;
	};

	struct Body {
		bool used = false;
		bool dead = false;
		ObjectID body_id;
		ObjectID zone_id;
		int space = 0; // Пространство тела (индекс в spaces)
		uint32_t platform_layers = UINT32_MAX; // Слои платформ тела вне подвижного пространства (в нём — 0: переносим сами)
		Transform3D frame_last;
		Vector3 up = Vector3(0.0f, 1.0f, 0.0f); // Верх тела (up_direction): у CharacterBody3D в C++ он приватный — держим здесь
		int crowd = -1; // Ручка EnemyWorld
		int view = -1; // Ручка EnemyViews
		int slot = 0;
		real_t speed = 3.0f;
		// Движение
		real_t yaw = 0.0f;
		bool on_step = false;
		bool was_floor = true; // На полу в прошлом тике (касание после полёта — проверка пола)
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
		// Связи
		uint8_t link_state = LINK_NONE;
		int link = -1; // Номер связи (ждёт у входа / на ней)
		EnemyLinks::Motion motion;
		real_t link_t = 0.0f;
		int portal_space = -1; // Касание после полёта портала — в это пространство
		uint64_t land_end = 0;
		real_t jump_impulse = 4.2f; // Профиль тела (NavMoveProfile)
		real_t jump_gravity = 9.8f;
		uint32_t path_epoch = 0; // Эпоха навигации последнего запроса пути
		LocalVector<uint8_t> link_log; // Виды пройденных связей (пробы)
		// Неваляшка
		EnemyTumbler tumb;
		// Анимация и руки
		Ref<EnemyRig> rig;
		EnemyAnim anim;
		EnemyArms arms;
	};

	LocalVector<Body> bodies;
	LocalVector<int> free_handles;
	Ref<EnemyViews> views;
	LocalVector<Space> spaces; // 0 — корабль (корень карты)
	HashMap<ObjectID, int> floor_space; // Тело пола -> пространство
	int space_changes = 0;
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
	EnemyLinks links;
	uint32_t nav_epoch = 0; // Связь закрыта / открыта — стоящие (HOLD, у входа) переспрашивают путь
	int traversals = 0;
	int land_close = 0;
	int on_link = 0;
	int portal_transits = 0;

	bool _valid(int p_h) const;
	void _space_xforms();
	EnemyWorld *_crowd(const Body &p_b) const;
	bool _nav_ok(const Body &p_b) const;
	void _set_space(int p_h, Body &r_b, int p_space, CharacterBody3D *p_body);
	bool _follow_floor(int p_h, Body &r_b, CharacterBody3D *p_body, uint64_t p_now);
	void _emit(int p_kind, int p_h);
	void _tick_body(int p_h, Body &r_b, CharacterBody3D *p_body, uint64_t p_now, real_t p_dt);
	void _publish(Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, const Vector3 &p_vel, const Vector3 &p_goal, bool p_has_goal, uint64_t p_now);
	// Связи (enemy_bodies_links.cpp)
	void _links_tick(real_t p_dt);
	bool _link_wait(int p_h, Body &r_b, CharacterBody3D *p_body, uint64_t p_now, real_t p_dt);
	void _link_move(int p_h, Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, uint64_t p_now, real_t p_dt);
	void _link_release(int p_h, Body &r_b);
	Transform3D _frame_xform(const Body &p_b) const;
	Transform3D _carry(Body &r_b, CharacterBody3D *p_body);
	static Vector3 _forward(const Body &p_b, const Transform3D &p_fx);
	Transform3D _tumble_base(const Body &p_b, CharacterBody3D *p_body) const;
	Transform3D _avatar_xf(const Body &p_b, CharacterBody3D *p_body) const;
	void _view_anim(const Body &p_b);
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

	// Толпа корабля, визуал, корень карты (пространство 0)
	void setup(const Ref<EnemyWorld> &p_crowd, const Ref<EnemyViews> &p_views, Node3D *p_root);
	int body_add(CharacterBody3D *p_body, Area3D *p_item_zone, int p_view, int p_slot, real_t p_speed, real_t p_yaw);
	void body_remove(int p_h);
	void body_kill(int p_h); // Смерть: толпа отпускает агента, вид выключен, тик пропускает тело
	int space_add(const Ref<EnemyWorld> &p_crowd, Node3D *p_frame); // Подвижное пространство (ReferenceFrame); индекс
	void space_set_ready(int p_space, bool p_ready);
	int get_space_count() const { return (int)spaces.size(); }
	void set_floor_spaces(const PackedInt64Array &p_ids, const PackedInt32Array &p_spaces); // Тело пола (instance id) → пространство
	int body_space(int p_h) const;
	void body_teleport(int p_h, int p_space, const Transform3D &p_world); // Мгновенно в позу (мир) и в пространство
	int get_space_changes() const { return space_changes; } // Смен пространства всего (пробы)
	void body_set_point(int p_h, const Vector3 &p_world); // Точка пробы: идти туда без боя
	void body_set_path(int p_h, const PackedVector3Array &p_points, const Vector3 &p_target, const PackedInt32Array &p_links = PackedInt32Array());
	void body_set_profile(int p_h, real_t p_jump_impulse, real_t p_gravity); // Профиль навигации тела (связи)
	PackedInt32Array body_link_log(int p_h) const; // Виды пройденных связей по порядку
	int body_link_state(int p_h) const;
	// Переход через портал (spec 2026-10-23-enemy-moving-spaces §3): полёт в пространстве корабля по траектории
	// профиля в p_end (мир, конец в момент касания), касание — пространство p_space
	void body_portal(int p_h, const Vector3 &p_end, int p_kind, int p_space);
	void body_clear_point(int p_h); // Снова мозг (точка пробы / портала снята)
	int get_portal_transits() const { return portal_transits; }
	static real_t motion_time(int p_kind, const Vector3 &p_a, const Vector3 &p_b, real_t p_walk, real_t p_impulse, real_t p_gravity); // EnemyLinks::begin
	static int motion_kind(int p_kind, const Vector3 &p_a, const Vector3 &p_b, real_t p_impulse, real_t p_gravity); // Вид после begin (прыжок не по силам — залезть / спрыгнуть)
	// Таблица связей карты (координаты пространства корабля; номер — NavLinkSet.links)
	void set_links(const PackedVector3Array &p_starts, const PackedVector3Array &p_ends, const PackedInt32Array &p_kinds);
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
	// Неваляшка: хит-тело и настройки (углы — рад), толчок (плечо и импульс — кадр основания), основание (мир), наклон
	void body_set_tumbler(int p_h, Node3D *p_hit_body, real_t p_inertia, real_t p_frequency, real_t p_damping, real_t p_fall_angle, real_t p_down_angle, real_t p_height, real_t p_down_time, real_t p_rise_time, real_t p_stagger_angle, real_t p_knockback);
	void body_push(int p_h, const Vector3 &p_r, const Vector3 &p_j);
	Transform3D body_tumble_base(int p_h) const;
	Vector3 body_tumble_spin(int p_h) const;
	Vector3 body_tilt(int p_h) const;
	int body_tumble_state(int p_h) const;
	// Анимация и руки: риг модели и хит-тела рук (левая, правая); разовый клип («библиотека/клип» рига); идёт ли он, клип
	// и темп (пробы); неваляшка руки (сторона 0 — левая): настройки (предел — рад), толчок (кадр основания), основание
	// (мир: кадр кости плеча), угловая скорость, наклон
	void body_set_rig(int p_h, const Ref<EnemyRig> &p_rig, Node3D *p_arm_l, Node3D *p_arm_r);
	void body_play(int p_h, const StringName &p_clip);
	bool body_acting(int p_h) const;
	int body_anim_clip(int p_h) const;
	real_t body_anim_rate(int p_h) const;
	void body_set_arm_tumbler(int p_h, int p_side, real_t p_inertia, real_t p_frequency, real_t p_damping, real_t p_limit);
	void body_arm_push(int p_h, int p_side, const Vector3 &p_r, const Vector3 &p_j);
	Transform3D body_arm_base(int p_h, int p_side) const;
	Vector3 body_arm_spin(int p_h, int p_side) const;
	Vector3 body_arm_tilt(int p_h, int p_side) const;

	// Снимок игроков этого тика: позы (мир), живы ли, instance id
	void set_players(const PackedVector3Array &p_pos, const PackedByteArray &p_alive, const PackedInt64Array &p_ids);
	void set_nav_ready(bool p_ready); // Сетка корабля (пространство 0)
	void set_tier_override(int p_tier); // -1 — по расстоянию (пробы: TIER_NEAR / TIER_FAR)
	PackedInt32Array tick(real_t p_delta); // События: пары (Event, ручка)
	Array net_pack() const; // [тик, PackedInt32Array net id, PackedFloat32Array по 7 чисел: x, y, z, yaw, speed, step_lag, пространство] (поза и взгляд — в системе пространства)
	int get_body_count() const;
	int get_tier_count(int p_tier) const;
	int get_rest_count() const { return rest_count; } // Тел в покое за последний тик
	int64_t get_last_tick_usec() const { return (int64_t)last_tick_usec; }
	int get_traversals() const { return traversals; } // Входов на связь всего
	int get_land_close() const { return land_close; } // Приземлений ближе LAND_CLOSE_R к другому
	int get_closed_links() const { return links.closed_count(); }
	int get_on_link_count() const { return on_link; } // На связи в последнем тике
};

VARIANT_ENUM_CAST(EnemyBodies::Brain);
VARIANT_ENUM_CAST(EnemyBodies::Tier);
VARIANT_ENUM_CAST(EnemyBodies::Event);
