#pragma once

#include "enemy_flight_grid.h"

#include "core/math/vector3.h"
#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"

// Поля расстояний по сетке полёта (бриф полёта 2026-11-14, срез 1; EnemyFlightGrid — занятость): поле цели — обход в
// ширину по 6 соседям от свободных клеток у цели (расстояние в клетках), кусками по бюджету (step: заливка рабочего
// буфера, затем обход). Цели — want (тела в тике), до MAX_TARGETS; поле цели считается заново по кругу, когда она сошла
// с клетки посева; готовое поле держится до замены (рабочий буфер — один на все цели). Цель без want RELEASE_TICKS тиков
// — слот свободен. Сетка сменилась (эпоха) — поля заново.
// Путь тела (steer): прямая видимость по клеткам — к цели; иначе спуск по полю (26 соседей) до DESCENT клеток от лучшей
// клетки у тела и самая дальняя видимая клетка спуска; не видна ни одна — свободный сосед клетки тела по грани с меньшим
// полем (ход дальнего яруса — по осям), клетка тела вне поля — первая клетка спуска.
// Число слотов целей — свойство экземпляра (set_max_targets; по умолчанию MAX_TARGETS): у наружной сетки целей больше
// (игроки и наружные шлюзы). Открытая сетка (EnemyFlightGrid::set_open): тело вне внутренних клеток, прямая закрыта — к
// точке входа отрезка во внутренние клетки (там — поле); цель вне коробки засевается у края (клетка посева зажата внутрь).
class EnemyFlightFields : public RefCounted {
	GDCLASS(EnemyFlightFields, RefCounted);

public:
	static constexpr int MAX_TARGETS = 4; // Слотов по умолчанию
	static constexpr int TARGETS_LIMIT = 16;
	static constexpr uint16_t UNREACHED = 0xFFFF;
	static constexpr int DESCENT = 10; // Клеток спуска по полю (≈5 м при 0.5)
	static constexpr uint64_t RELEASE_TICKS = 270; // Цель без want столько тиков — слот свободен
	static constexpr uint32_t FILL_CHUNK = 262144; // Заливка рабочего буфера — кусками (проверка бюджета)

private:
	struct Target {
		bool used = false;
		int64_t id = 0;
		Vector3 pos; // Центр цели (локально) последнего want
		uint64_t wanted = 0; // Тик последнего want
		bool ready = false;
		uint32_t seed = 0; // Клетка посева готового поля
		int buf = 0; // Буфер готового поля (bufs)
	};

	Ref<EnemyFlightGrid> grid;
	uint32_t epoch = 0; // Эпоха сетки, для которой поля
	int max_targets = MAX_TARGETS;
	LocalVector<Target> targets; // max_targets
	LocalVector<LocalVector<uint16_t>> bufs; // Поля целей и рабочий, max_targets + 1 (по клетке; UNREACHED — не дошёл)
	int work_buf = MAX_TARGETS;
	LocalVector<uint32_t> queue;
	uint32_t q_head = 0;
	int work_slot = -1; // Слот, чьё поле считается; -1 — нет
	int64_t work_id = 0;
	uint32_t work_seed = 0;
	uint32_t work_fill = 0; // Залито ячеек рабочего буфера (до cells — заливка, затем обход)
	bool work_seeded = false;
	uint32_t work_visited = 0;
	uint64_t work_usec = 0;
	int work_steps = 0;
	int rr = 0; // Следующий слот по кругу
	uint64_t now = 0;
	uint64_t last_step_usec = 0;
	uint64_t last_field_usec = 0;
	int last_field_steps = 0;
	int last_field_cells = 0;
	int fields_done = 0;

	bool _sync();
	void _reset();
	int _slot(int64_t p_id) const;
	uint32_t _seed_cell(const Vector3 &p_local) const;
	bool _seed(uint32_t p_cell);
	bool _start_field();
	const uint16_t *_field(int64_t p_id) const;
	bool _best_start(const uint16_t *p_f, const Vector3 &p_from, uint32_t &r_cell) const;

protected:
	static void _bind_methods();

public:
	EnemyFlightFields();
	void set_grid(const Ref<EnemyFlightGrid> &p_grid);
	void set_max_targets(int p_count); // Слотов целей (1…TARGETS_LIMIT); поля заново
	int get_max_targets() const { return max_targets; }
	Ref<EnemyFlightGrid> get_grid() const { return grid; }
	bool is_ready() const { return grid.is_valid() && grid->is_ready(); }
	void want(int64_t p_id, const Vector3 &p_local, uint64_t p_now); // Цель нужна в этом тике (центр, локально)
	bool step(int p_budget_usec, uint64_t p_now); // Пересчёт полей по бюджету; true — есть работа
	bool field_ready(int64_t p_id) const { return _field(p_id) != nullptr; }
	real_t field_dist(int64_t p_id, const Vector3 &p_local) const; // Путь лётом по полю (клетки × cell), м; < 0 — нет
	// Куда лететь от p_from к p_goal (центры тел, локально): r_aim — точка (локально), r_dist — путь лётом (м, по полю;
	// прямая видимость — по прямой). false — поля нет или от p_from не достать (r_aim — сама цель)
	bool steer(int64_t p_id, const Vector3 &p_from, const Vector3 &p_goal, Vector3 &r_aim, real_t &r_dist) const;
	Vector3 steer_point(int64_t p_id, const Vector3 &p_from, const Vector3 &p_goal) const; // То же для GDScript (пробы)
	// Счётчики (пробы)
	int64_t get_memory_bytes() const;
	int64_t get_last_step_usec() const { return (int64_t)last_step_usec; }
	int64_t get_last_field_usec() const { return (int64_t)last_field_usec; } // Последнее готовое поле: время всех его шагов
	int get_last_field_steps() const { return last_field_steps; }
	int get_last_field_cells() const { return last_field_cells; } // Клеток, до которых дошёл обход
	int get_fields_done() const { return fields_done; }
	int get_target_count() const;
};
