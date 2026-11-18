#pragma once

#include "core/math/aabb.h"
#include "core/math/vector3.h"
#include "core/math/vector3i.h"
#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"
#include "core/variant/variant.h"

// Объёмная сетка полёта врагов (бриф полёта 2026-11-14, срез 1; AGENTS §2.9): клетки cell пространства (корабль —
// координаты корня карты) «занято / свободно». Занято: коробка тела (центр клетки ± half) пересекает треугольник
// геометрии навигации (статические тела слоя 1 — разбор NavSpace) или центр клетки вне объёма (коробки зоны тяжести
// корабля); крайние клетки сетки заняты всегда (обход соседей без проверки границ). Построение — кусками по бюджету
// (build_step); готовое — в кэш (get_bits / restore). Прямая видимость (line_free) — клетки отрезка между концами
// свободны. Поля расстояний и путь тела — EnemyFlightFields (по этой сетке).
// Части (срез Б «лифт для летящих», бриф 2026-11-18): готовая сетка — номера связных частей свободных клеток (по 6
// соседям, один обход в ширину по готовности; номера с 1, больше 65535 частей — последний общий); часть точки
// (component_at) — её клетки или ближайшей свободной в кубе радиуса 1, затем 2. Маршрут летящего (GDScript NavPortals)
// различает по ним части корабля, связанные только подвижными системами (палубы лифта).
class EnemyFlightGrid : public RefCounted {
	GDCLASS(EnemyFlightGrid, RefCounted);

	enum Build {
		BUILD_NONE,
		BUILD_RUN,
		BUILD_READY,
	};

	uint8_t build = BUILD_NONE;
	uint32_t epoch = 0; // Сетка сменилась (build_begin / restore) — поля заново
	Vector3 origin; // Угол сетки (локально)
	real_t cell = 0.5f;
	Vector3 half = Vector3(0.25f, 0.75f, 0.25f);
	int nx = 0;
	int ny = 0;
	int nz = 0;
	LocalVector<uint8_t> occ; // 1 — занято
	LocalVector<uint16_t> part; // Часть клетки (0 — занята)
	LocalVector<Vector3> tris; // Треугольники построения (по 3 вершины)
	uint32_t build_next = 0;
	uint64_t build_usec = 0;
	int build_steps = 0;
	int free_count = 0;
	int part_count = 0;
	uint64_t part_usec = 0;

	void _mark_border();
	void _label_parts();
	void _voxelize(const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_c);
	static bool _tri_box(const Vector3 &p_center, const Vector3 &p_half, const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_c);
	void _finish_build();

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
	int get_cell_count() const { return (int)cells(); }
	int get_free_count() const { return free_count; }
	int64_t get_memory_bytes() const { return (int64_t)occ.size() + (int64_t)part.size() * 2; }
	int64_t get_build_usec() const { return (int64_t)build_usec; }
	int get_build_steps() const { return build_steps; }
	// Клетки
	bool contains(const Vector3 &p_local) const { return build == BUILD_READY && in(cell_of(p_local)); }
	bool is_free(const Vector3 &p_local) const;
	bool line_free(const Vector3 &p_a, const Vector3 &p_b) const; // Клетки между концами (без самих концов) свободны
	int component_at(const Vector3 &p_local) const; // Часть точки (шапка); −1 — нет свободной клетки рядом / не готова
	int get_component_count() const { return part_count; }
	int64_t get_component_usec() const { return (int64_t)part_usec; } // Разметка частей, мкс
	// Разбор клеток (C++: EnemyFlightFields, EnemyBodies)
	uint32_t get_epoch() const { return epoch; }
	uint32_t cells() const { return (uint32_t)nx * (uint32_t)ny * (uint32_t)nz; }
	int dim_x() const { return nx; }
	int dim_y() const { return ny; }
	int dim_z() const { return nz; }
	const uint8_t *occupancy() const { return occ.ptr(); }
	uint32_t index(int p_x, int p_y, int p_z) const { return (uint32_t)p_x + (uint32_t)nx * ((uint32_t)p_y + (uint32_t)ny * (uint32_t)p_z); }
	Vector3i cell_of(const Vector3 &p_local) const;
	bool in(const Vector3i &p_c) const { return p_c.x >= 0 && p_c.y >= 0 && p_c.z >= 0 && p_c.x < nx && p_c.y < ny && p_c.z < nz; }
	Vector3 center(uint32_t p_i) const;
};
