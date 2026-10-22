#pragma once

#include "nav_column_index.h"

#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"

// Мир врагов карты (spec docs/superpowers/specs/2026-10-21-enemy-world-design.md; поведение —
// docs/superpowers/specs/2026-10-20-enemy-walker-c1-design.md §3, §5): агенты — ручки, состояние — плоские
// массивы, координаты — корня карты (NavWorld), верх +Y. Этап 1 — толпа: соседи (NEIGHBORS_MAX ближайших в
// SEPARATION_R), расталкивание, правая рука, уступка (приоритет колонны, YIELD), следование по пути,
// застревание / BLOCKED, стояние у цели (HOLD: конец пути к недостижимой цели, очередь за стоящим соседом), удержание на сетке (NavColumnIndex), связи пути (стоп у начала, очередь, агент на связи невидим — план 2026-10-22-nav-links-b). Расчёт — один проход на всех агентов за
// физ-тик, запускает первый запрос выходов в тике; читает входы всех агентов на начало тика (сданные
// раньше этого тика) и намерения соседей прошлого расчёта, каждый агент пишет только своё — порядок нод
// и кто спросил первым не влияют (AGENTS §2.5). Новый путь (agent_set_path) — со следующего тика.
// Порт PathFollower / CrowdGrid / CrowdAgent / CrowdYield / NavColumnIndex (GDScript, 2026-10-21).
class EnemyWorld : public RefCounted {
	GDCLASS(EnemyWorld, RefCounted);

public:
	enum AgentState {
		AGENT_IDLE,
		AGENT_MOVE,
		AGENT_BLOCKED,
		AGENT_YIELD,
		AGENT_HOLD, // Стоит у цели: конец пути к недостижимой цели или очередь за стоящим соседом с той же целью
		AGENT_LINK, // У начала связи (ждёт; связь ведёт EnemyBodies) или на ней (agent_set_airborne)
	};

	// Следование (PathFollower)
	static constexpr real_t CORNER_REACH = 0.3f;
	static constexpr real_t SLOW_DIST = 1.0f;
	static constexpr real_t REACH_TOL = 1.0f;
	static constexpr real_t REPATH_S = 0.75f;
	static constexpr real_t TARGET_MOVED = 1.0f;
	static constexpr real_t OFF_PATH = 1.0f;
	static constexpr real_t STUCK_SPEED = 0.5f;
	static constexpr real_t STUCK_S = 1.0f;
	static constexpr real_t STUCK_MOVE = 0.1f;
	static constexpr int STUCK_LIMIT = 3;
	static constexpr real_t BLOCKED_REPATH_S = 2.0f;
	static constexpr real_t SEPARATION_R = 1.0f;
	static constexpr real_t SEPARATION_GAIN = 1.5f; // м/с при касании
	static constexpr real_t SIDE_GAIN = 1.0f; // м/с шага вправо при касании
	static constexpr uint32_t NEIGHBORS_MAX = 6;
	static constexpr real_t YIELD_SPEED = 1.5f; // м/с назад по пути
	// Уступка (CrowdYield); радиус — SEPARATION_R
	static constexpr real_t AHEAD_COS = 0.5f;
	static constexpr real_t FOLLOW_COS = 0.7f;
	static constexpr real_t ONCOMING_COS = -0.5f;
	static constexpr real_t CONFLICT_S = 0.5f;
	static constexpr real_t CLEAR_S = 1.0f;
	static constexpr real_t MAX_S = 4.0f;
	// Стояние (HOLD; план docs/superpowers/plans/2026-10-22-crowd-hold.md)
	static constexpr real_t HOLD_AHEAD_R = 0.8f; // Стоящий сосед впереди ближе — встаём за ним
	static constexpr real_t HOLD_QUEUE_REMAIN = 6.0f; // Очередь — только у конца пути, не в проходах
	static constexpr real_t HOLD_SAME_GOAL = 0.75f; // Сосед ждёт ту же цель — цели ближе
	static constexpr real_t HOLD_LEAVE_S = 0.3f; // Свободно столько подряд — идём
	static constexpr real_t OVERLAP_R = 0.7f; // Стоящего толкает только идущий сосед ближе этого (пришедших врозь держат тела)
	static constexpr real_t YIELD_MIN_REMAIN = 3.0f; // Ближе к концу пути не уступаем: встречные там — толпа к одной цели
	// Связи (план docs/superpowers/plans/2026-10-22-nav-links-b.md)
	static constexpr real_t LINK_REACH = 0.4f; // Ближе к началу связи — стоит у входа (AGENT_LINK)
	static constexpr real_t LINK_SLOW_MIN = 0.3f; // Доля скорости у самого входа (подход замедляется на SLOW_DIST)
	static constexpr real_t NEAR_H = 0.9f; // count_near: по высоте не дальше (этаж выше / ниже — не рядом)
	// Удержание на сетке (EnemyWalker._onto_mesh)
	static constexpr real_t OFF_MESH_DIRECT = 1.5f;
	static constexpr real_t MESH_EPS = 0.05f;
	static constexpr real_t COLUMN_H = 0.5f;

private:
	struct Input { // Вход агента, сданный в тике in_tick
		Vector3 pos;
		Vector3 goal;
		real_t stop_dist = 2.0f;
		real_t speed = 3.0f;
		bool has_goal = false;
	};
	struct Follow { // Своё состояние агента — пишет только его расчёт
		LocalVector<Vector3> path;
		LocalVector<int> links; // Номер связи карты на точку пути (точка — начало связи), -1 — нет
		Vector3 path_target;
		bool has_path = false;
		int corner = 1;
		AgentState state = AGENT_IDLE;
		real_t since_path = 0.0f;
		real_t blocked_t = 0.0f;
		real_t stuck_t = 0.0f;
		Vector3 stuck_from;
		int stuck_run = 0;
		bool yielding = false;
		real_t conflict_t = 0.0f;
		real_t clear_t = 0.0f;
		real_t yield_t = 0.0f;
		Vector3 hold_end; // Конец пути, у которого стоим (hold_by_end)
		bool hold_by_end = false; // Стоим у конца пути (иначе — в очереди)
		real_t hold_free_t = 0.0f; // Сколько подряд свободно (выход из HOLD)
		int hint = -1; // Полигон прошлого ответа индекса
	};

	// Агенты (индекс — ручка)
	LocalVector<uint8_t> alive;
	LocalVector<int64_t> prio; // Порядок добавления — приоритет уступки (меньше — сильнее)
	LocalVector<Input> in_prev;
	LocalVector<Input> in_curr;
	LocalVector<uint64_t> in_tick;
	LocalVector<Vector<Vector3>> path_new;
	LocalVector<Vector3> path_new_target;
	LocalVector<uint64_t> path_tick;
	LocalVector<uint8_t> path_pending;
	LocalVector<Vector<int>> path_new_links;
	LocalVector<uint8_t> airborne; // На связи (тело ведёт EnemyBodies): соседи его не видят, расчёта нет
	LocalVector<uint8_t> link_done; // Связь пройдена — со следующего расчёта путь с конца связи
	LocalVector<Follow> follow;
	// Выходы расчёта
	LocalVector<Vector3> out_vel;
	LocalVector<Vector3> out_heading;
	LocalVector<int64_t> out_rank;
	LocalVector<uint8_t> out_yield;
	LocalVector<uint8_t> out_state;
	LocalVector<uint8_t> out_wants;
	LocalVector<int> out_link; // Связь текущего угла пути (начало связи впереди), -1 — нет
	LocalVector<int> stuck_total;
	LocalVector<int> blocked_total;
	LocalVector<int> yield_total;
	// Снимок расчёта: позы на начало тика и намерения прошлого расчёта
	LocalVector<Vector3> start_pos;
	LocalVector<Vector3> prev_heading;
	LocalVector<int64_t> prev_rank;
	LocalVector<uint8_t> prev_yield;
	LocalVector<Input> start_in; // Входы на начало тика (цель, остановка — для очереди)
	LocalVector<uint8_t> prev_state;
	LocalVector<int> prev_link;
	LocalVector<uint32_t> free_handles;
	int64_t next_prio = 0;
	uint64_t stepped_tick = UINT64_MAX;
	uint64_t last_step_usec = 0;
	// Соседи текущего агента (по возрастанию расстояния)
	LocalVector<uint32_t> nb;
	LocalVector<real_t> nb_d2;
	// Сетка
	NavColumnIndex column_index;
	RID nav_map;
	int column_poly = -1;

	static Vector3 _flat(const Vector3 &p_v) { return Vector3(p_v.x, 0.0f, p_v.z); }
	static uint64_t _now();
	bool _valid(int p_h) const;
	void _grow(uint32_t p_size);
	const Input &_start(uint32_t p_h, uint64_t p_now) const;
	void _ensure_step();
	void _step();
	void _step_agent(uint32_t p_h, uint64_t p_now, real_t p_dt);
	void _adopt_path(uint32_t p_h, Follow &r_f);
	void _neighbors(uint32_t p_h, const Vector3 &p_pos);
	void _yield_tick(uint32_t p_h, Follow &r_f, const Vector3 &p_pos, real_t p_dt);
	Vector3 _follow_tick(uint32_t p_h, Follow &r_f, const Input &p_in, real_t p_dt);
	Vector3 _separation(const Vector3 &p_pos) const;
	Vector3 _keep_right(const Vector3 &p_pos, const Vector3 &p_dir) const;
	bool _hold_tick(Follow &r_f, const Input &p_in, const Vector3 &p_dir, bool p_at_end, real_t p_dt) const;
	bool _queued(const Follow &p_f, const Input &p_in, const Vector3 &p_dir) const;
	bool _arrived(uint32_t p_j) const;
	Vector3 _overlap(const Vector3 &p_pos) const;
	void _advance(Follow &r_f, const Vector3 &p_pos) const;
	real_t _remaining(const Follow &p_f, const Vector3 &p_pos) const;
	int _corner_link(const Follow &p_f) const;
	real_t _off_path(const Follow &p_f, const Vector3 &p_pos) const;
	bool _wants_path(const Follow &p_f, const Input &p_in) const;
	void _track_stuck(uint32_t p_h, Follow &r_f, const Vector3 &p_pos, real_t p_want_speed, real_t p_dt);
	Vector3 _onto_mesh(Follow &r_f, const Vector3 &p_pos, const Vector3 &p_w, real_t p_speed, real_t p_dt);

protected:
	static void _bind_methods();

public:
	// Сетка карты: индекс полигонов строится сразу; p_map — запасной путь (сервер навигации)
	void set_navigation(const Ref<NavigationMesh> &p_mesh, RID p_map);
	// Точка сетки по вертикали (±COLUMN_H) через локальную точку; p_hint — полигон прошлого ответа или -1
	Vector3 column(const Vector3 &p_local, int p_hint);
	int get_column_poly() const { return column_poly; } // Полигон последнего column (-1 — ответил сервер навигации)
	Vector3 column_engine(const Vector3 &p_local) const; // То же сервером навигации (эталон пробы)

	int agent_add(const Vector3 &p_local_pos);
	void agent_remove(int p_h);
	// Вход после своего тика: поза, цель (has_goal = false — цели нет), stop_dist, скорость
	void agent_commit(int p_h, const Vector3 &p_local_pos, const Vector3 &p_goal, bool p_has_goal, real_t p_stop_dist, real_t p_speed);
	// Путь (точки, цель запроса, номера связей по точкам — пусто: без связей); пустой — пути нет (BLOCKED).
	// Применяется со следующего тика.
	void agent_set_path(int p_h, const PackedVector3Array &p_points, const Vector3 &p_target, const PackedInt32Array &p_links = PackedInt32Array());
	Vector3 agent_velocity(int p_h); // Желаемая скорость (локальная, XZ)
	int agent_state(int p_h);
	bool agent_wants_path(int p_h);
	Vector3 agent_heading(int p_h);
	int64_t agent_rank(int p_h);
	bool agent_yielding(int p_h);
	int agent_stuck_total(int p_h) const;
	int agent_blocked_total(int p_h) const;
	int agent_yield_total(int p_h) const;
	int agent_link(int p_h); // Связь текущего угла пути, -1 — нет
	void agent_set_airborne(int p_h, bool p_on); // На связи: соседи не видят, расчёта нет (со следующего)
	void agent_link_done(int p_h); // Связь пройдена: путь — с её конца (со следующего расчёта)
	// Агентов не на связи ближе p_r по горизонтали (по высоте ±NEAR_H) к локальной точке; позы на начало тика
	int count_near(const Vector3 &p_local, real_t p_r, int p_exclude);
	int get_agent_count() const;
	int64_t get_last_step_usec() const { return (int64_t)last_step_usec; }
};

VARIANT_ENUM_CAST(EnemyWorld::AgentState);
