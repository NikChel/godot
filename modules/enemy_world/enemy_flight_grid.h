#pragma once

#include "core/math/aabb.h"
#include "core/math/vector3.h"
#include "core/math/vector3i.h"
#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"
#include "core/variant/variant.h"

// Объёмная сетка полёта врагов (бриф полёта 2026-11-14, срез 1; AGENTS §2.9): клетки cell пространства (корабль —
// координаты корня карты) «занято / свободно» и поля расстояний от целей (игроков) по свободным клеткам.
// Занято: коробка тела (центр клетки ± half) пересекает треугольник геометрии навигации (статические тела слоя 1 —
// разбор NavSpace) или центр клетки вне объёма (коробки зоны тяжести корабля); крайние клетки сетки заняты всегда (обход
// без проверки границ). Построение — кусками по бюджету (build_step); готовое — в кэш (get_bits / restore).
// Поле цели — обход в ширину по 6 соседям от свободных клеток у цели (расстояние в клетках), кусками по бюджету
// (step: заливка рабочего буфера, затем обход). Цели — want (тело в тике), до MAX_TARGETS; поле цели считается заново по
// кругу, когда она сошла с клетки посева; готовое поле держится до замены (рабочий буфер — один на все цели). Цель без
// want RELEASE_TICKS тиков — слот свободен.
// Путь тела (steer): прямая видимость по клеткам (клетки между концами свободны) — к цели; иначе спуск по полю (26
// соседей) до DESCENT клеток от лучшей клетки у тела и самая дальняя видимая клетка спуска.
class EnemyFlightGrid : public RefCounted {
	GDCLASS(EnemyFlightGrid, RefCounted);

public:
	static constexpr int MAX_TARGETS = 4;
	static constexpr uint16_t UNREACHED = 0xFFFF;
	static constexpr int DESCENT = 10; // Клеток спуска по полю (≈5 м при 0.5)
	static constexpr uint64_t RELEASE_TICKS = 270; // Цель без want столько тиков — слот свободен
	static constexpr uint32_t FILL_CHUNK = 262144; // Заливка рабочего буфера — кусками (проверка бюджета)

private:
	enum Build {
		BUILD_NONE,
		BUILD_RUN,
		BUILD_READY,
	};

	struct Target {
		bool used = false;
		int64_t id = 0;
		Vector3 pos; // Центр цели (локально) последнего want
		uint64_t wanted = 0; // Тик последнего want
		bool ready = false;
		uint32_t seed = 0; // Клетка посева готового поля
		int buf = 0; // Буфер готового поля (bufs)
	};

	// Сетка
	uint8_t build = BUILD_NONE;
	Vector3 origin; // Угол сетки (локально)
	real_t cell = 0.5f;
	Vector3 half = Vector3(0.25f, 0.75f, 0.25f);
	int nx = 0;
	int ny = 0;
	int nz = 0;
	LocalVector<uint8_t> occ; // 1 — занято
	LocalVector<Vector3> tris; // Треугольники построения (по 3 вершины)
	uint32_t build_next = 0;
	uint64_t build_usec = 0;
	int build_steps = 0;
	int free_count = 0;
	// Поля
	Target targets[MAX_TARGETS];
	LocalVector<uint16_t> bufs[MAX_TARGETS + 1]; // Поля целей и рабочий (по клетке; UNREACHED — не дошёл)
	int work_buf = MAX_TARGETS;
	uint32_t work_visited = 0;
	LocalVector<uint32_t> queue;
	uint32_t q_head = 0;
	int work_slot = -1; // Слот, чьё поле считается; -1 — нет
	int64_t work_id = 0;
	uint32_t work_seed = 0;
	uint32_t work_fill = 0; // Залито ячеек рабочего буфера (до cells — заливка, затем обход)
	bool work_seeded = false;
	uint64_t work_usec = 0;
	int work_steps = 0;
	int rr = 0; // Следующий слот по кругу
	uint64_t now = 0;
	uint64_t last_step_usec = 0;
	uint64_t last_field_usec = 0;
	int last_field_steps = 0;
	int last_field_cells = 0;
	int fields_done = 0;

	uint32_t _cells() const { return (uint32_t)nx * (uint32_t)ny * (uint32_t)nz; }
	uint32_t _idx(int p_x, int p_y, int p_z) const { return (uint32_t)p_x + (uint32_t)nx * ((uint32_t)p_y + (uint32_t)ny * (uint32_t)p_z); }
	Vector3i _cell_of(const Vector3 &p_local) const;
	bool _in(const Vector3i &p_c) const { return p_c.x >= 0 && p_c.y >= 0 && p_c.z >= 0 && p_c.x < nx && p_c.y < ny && p_c.z < nz; }
	Vector3 _center(uint32_t p_i) const;
	void _mark_border();
	void _voxelize(const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_c);
	static bool _tri_box(const Vector3 &p_center, const Vector3 &p_half, const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_c);
	void _finish_build();
	void _reset_fields();
	int _slot(int64_t p_id) const;
	bool _seed(uint32_t p_cell);
	bool _start_field();
	const uint16_t *_field(int64_t p_id) const;
	bool _best_start(const uint16_t *p_f, const Vector3 &p_from, uint32_t &r_cell) const;

protected:
	static void _bind_methods();

public:
	// Построение: вершины и индексы треугольников (локально — как разбор NavigationMeshSourceGeometryData3D), объём
	// (пары min, max коробок; клетка с центром вне них занята), клетка, полуразмеры коробки тела
	void build_begin(const PackedFloat32Array &p_vertices, const PackedInt32Array &p_indices, const PackedVector3Array &p_volume, real_t p_cell, const Vector3 &p_half);
	bool build_step(int p_budget_usec); // true — готово
	bool is_ready() const { return build == BUILD_READY; }
	bool is_building() const { return build == BUILD_RUN; }
	// Кэш: биты занятости (по клетке), восстановление тем же разбором
	PackedByteArray get_bits() const;
	bool restore(const Vector3 &p_origin, const Vector3i &p_dims, real_t p_cell, const Vector3 &p_half, const PackedByteArray &p_bits);
	Vector3 get_origin() const { return origin; }
	Vector3i get_dims() const { return Vector3i(nx, ny, nz); }
	real_t get_cell() const { return cell; }
	Vector3 get_half() const { return half; }
	int get_cell_count() const { return (int)_cells(); }
	int get_free_count() const { return free_count; }
	int64_t get_memory_bytes() const;
	int64_t get_build_usec() const { return (int64_t)build_usec; }
	int get_build_steps() const { return build_steps; }
	// Клетки
	bool contains(const Vector3 &p_local) const { return build == BUILD_READY && _in(_cell_of(p_local)); }
	bool is_free(const Vector3 &p_local) const;
	bool line_free(const Vector3 &p_a, const Vector3 &p_b) const; // Клетки между концами (без самих концов) свободны
	// Цели и поля
	void want(int64_t p_id, const Vector3 &p_local, uint64_t p_now); // Цель нужна в этом тике (центр, локально)
	bool step(int p_budget_usec, uint64_t p_now); // Пересчёт полей по бюджету; true — есть работа
	bool field_ready(int64_t p_id) const { return _field(p_id) != nullptr; }
	real_t field_dist(int64_t p_id, const Vector3 &p_local) const; // Путь лётом, м; < 0 — нет поля / не достать
	// Куда лететь от p_from к p_goal (центры тел, локально): r_aim — точка (локально), r_dist — путь лётом (м, по полю;
	// прямая видимость — по прямой). false — поля нет или от p_from не достать (r_aim — сама цель)
	bool steer(int64_t p_id, const Vector3 &p_from, const Vector3 &p_goal, Vector3 &r_aim, real_t &r_dist) const;
	Vector3 steer_point(int64_t p_id, const Vector3 &p_from, const Vector3 &p_goal) const; // То же для GDScript (пробы)
	// Счётчики (пробы)
	int64_t get_last_step_usec() const { return (int64_t)last_step_usec; }
	int64_t get_last_field_usec() const { return (int64_t)last_field_usec; } // Последнее готовое поле: время всех его шагов
	int get_last_field_steps() const { return last_field_steps; }
	int get_last_field_cells() const { return last_field_cells; } // Клеток, до которых дошёл обход
	int get_fields_done() const { return fields_done; }
	int get_target_count() const;
};
