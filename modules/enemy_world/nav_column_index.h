#pragma once

#include "core/math/plane.h"
#include "core/math/vector2i.h"
#include "core/math/vector3.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"
#include "scene/resources/navigation_mesh.h"

// Точка сетки навигации по вертикали (spec docs/superpowers/specs/2026-10-21-enemy-world-design.md §2):
// полигоны сетки (координаты корня карты, верх +Y) разложены по клеткам CELL плана XZ по своим габаритам.
// Запрос: полигон, внутри которого точка по XZ и чья высота над/под ней не дальше half_h, — его точка (из
// нескольких — ближайшая по высоте); первым — полигон-подсказка (прошлый ответ агента — обычно он же);
// полигон, чьи габариты не накрывают точку, отсеивается до проверки «внутри». Не внутри ни одного —
// ближайшая по XZ точка края полигонов клеток 3 × 3 вокруг (той же высоты). Ничего — false (вызывающий
// спрашивает сервер навигации). Порт NavColumnIndex.gd (ветка проекта wip/walker-gdscript-crowd).
class NavColumnIndex {
public:
	static constexpr real_t CELL = 2.0f;
	static constexpr real_t EDGE_EPS = 1e-6f;
	static constexpr real_t MIN_NY = 0.01f; // Круче (почти вертикальный полигон) — не в индексе

	void build(const Ref<NavigationMesh> &p_mesh);
	void clear();
	bool is_empty() const { return polys.is_empty(); }
	// Точка сетки по вертикали через p_point; p_hint — полигон прошлого ответа или -1.
	bool find(const Vector3 &p_point, real_t p_half_h, int p_hint, Vector3 &r_point, int &r_poly) const;
	// Точка сетки по вертикали через p_point (полигон накрывает точку по XZ, высота ±p_half_h; из нескольких — ближайший
	// по высоте); не накрыта — ближайшая по XZ точка края не дальше p_max_xz (высота края ±p_half_h); p_max_xz 0 —
	// только накрытие. Без подсказки и без запаса клеток 3 × 3 (find): для генератора связей и отбора узких мест.
	bool snap(const Vector3 &p_point, real_t p_half_h, real_t p_max_xz, Vector3 &r_point, int &r_poly) const;

private:
	struct Poly {
		uint32_t first = 0; // Первый индекс в corners
		uint32_t count = 0;
		Plane plane;
		real_t min_x = 0.0f;
		real_t min_z = 0.0f;
		real_t max_x = 0.0f;
		real_t max_z = 0.0f;
		bool indexed = false; // Не вертикальный — в клетках
	};

	LocalVector<Vector3> verts;
	LocalVector<uint32_t> corners;
	LocalVector<Poly> polys;
	HashMap<Vector2i, LocalVector<uint32_t>> cells;

	static Vector2i cell_of(real_t p_x, real_t p_z);
	bool covers(const Poly &p_poly, real_t p_x, real_t p_z) const;
	static real_t height(const Poly &p_poly, real_t p_x, real_t p_z);
	bool nearest_edge(const Vector3 &p_point, real_t p_half_h, const Vector2i &p_cell, Vector3 &r_point, int &r_poly) const;
};
