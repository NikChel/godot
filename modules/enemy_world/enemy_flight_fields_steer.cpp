#include "enemy_flight_fields.h"

#include "core/math/math_funcs.h"

// Путь тела по полям (EnemyFlightFields, шапка): лучшая клетка у тела, путь лётом по полю, куда лететь (steer).
// Слоты целей и пересчёт полей — enemy_flight_fields.cpp.

// Лучшая клетка у тела: свободная, с полем, наименьшее расстояние (равные — ближе к телу); куб радиуса 1, нет — 2.
bool EnemyFlightFields::_best_start(const uint16_t *p_f, const Vector3 &p_from, uint32_t &r_cell) const {
	const Vector3i c0 = grid->cell_of(p_from);
	for (int r = 1; r <= 2; r++) {
		uint16_t best = UNREACHED;
		real_t best_d = (real_t)Math::INF;
		for (int dz = -r; dz <= r; dz++) {
			for (int dy = -r; dy <= r; dy++) {
				for (int dx = -r; dx <= r; dx++) {
					const Vector3i c(c0.x + dx, c0.y + dy, c0.z + dz);
					if (!grid->in(c)) {
						continue;
					}
					const uint32_t j = grid->index(c.x, c.y, c.z);
					const uint16_t v = p_f[j];
					if (v == UNREACHED || v > best) {
						continue;
					}
					const real_t d = grid->center(j).distance_squared_to(p_from);
					if (v < best || d < best_d) {
						best = v;
						best_d = d;
						r_cell = j;
					}
				}
			}
		}
		if (best != UNREACHED) {
			return true;
		}
	}
	return false;
}

real_t EnemyFlightFields::field_dist(int64_t p_id, const Vector3 &p_local) const {
	const uint16_t *f = _field(p_id);
	uint32_t s = 0;
	if (f == nullptr || !_best_start(f, p_local, s)) {
		return -1.0f;
	}
	return (real_t)f[s] * grid->get_cell();
}

bool EnemyFlightFields::steer(int64_t p_id, const Vector3 &p_from, const Vector3 &p_goal, Vector3 &r_aim, real_t &r_dist) const {
	r_aim = p_goal;
	r_dist = p_from.distance_to(p_goal);
	if (!is_ready()) {
		return false;
	}
	if (grid->line_free(p_from, p_goal)) {
		return true;
	}
	if (grid->is_open() && !grid->interior(p_from)) {
		r_aim = grid->entry_point(p_from, p_goal); // Открытая сетка, тело вне внутренних клеток: к входу отрезка в них
		return true;
	}
	const uint16_t *f = _field(p_id);
	uint32_t s = 0;
	if (f == nullptr || !_best_start(f, p_from, s)) {
		return false;
	}
	r_dist = (real_t)f[s] * grid->get_cell();
	uint32_t chain[DESCENT];
	int n = 0;
	uint32_t c = s;
	const int64_t sy = grid->dim_x();
	const int64_t sz = sy * grid->dim_y();
	for (int k = 0; k < DESCENT; k++) {
		uint32_t best = c;
		uint16_t bv = f[c];
		for (int dz = -1; dz <= 1; dz++) {
			for (int dy = -1; dy <= 1; dy++) {
				for (int dx = -1; dx <= 1; dx++) {
					// Клетка с полем свободна — не у края сетки: соседи в ней
					const uint32_t j = (uint32_t)((int64_t)c + dx + dy * sy + dz * sz);
					const uint16_t v = f[j];
					if (v < bv) {
						bv = v;
						best = j;
					}
				}
			}
		}
		if (best == c) {
			break;
		}
		chain[n++] = best;
		c = best;
	}
	if (n == 0) {
		return true; // У цели (клетка посева): прямо к ней
	}
	for (int i = n - 1; i >= 0; i--) {
		const Vector3 p = grid->center(chain[i]);
		if (grid->line_free(p_from, p)) {
			r_aim = p;
			return true;
		}
	}
	// Ни одна клетка спуска не видна (клетка старта и спуск — через ребро или угол преграды): шаг по грани — свободный
	// сосед клетки тела с меньшим полем. Ход дальнего яруса — по осям и в занятую клетку не входит: курс на клетку за
	// преградой его останавливает. Клетка тела вне поля (занята, другая часть сетки) — первая клетка спуска.
	const Vector3i b = grid->cell_of(p_from);
	if (grid->in(b)) {
		const uint32_t bi = grid->index(b.x, b.y, b.z);
		uint16_t bv = f[bi];
		if (bv != UNREACHED) { // Клетка с полем свободна — не у края сетки: соседи в ней
			uint32_t best = bi;
			const uint32_t nb[6] = { bi + 1, bi - 1, (uint32_t)((int64_t)bi + sy), (uint32_t)((int64_t)bi - sy),
				(uint32_t)((int64_t)bi + sz), (uint32_t)((int64_t)bi - sz) };
			for (int k = 0; k < 6; k++) {
				if (f[nb[k]] < bv) {
					bv = f[nb[k]];
					best = nb[k];
				}
			}
			if (best != bi) {
				r_aim = grid->center(best);
				return true;
			}
		}
	}
	r_aim = grid->center(chain[0]);
	return true;
}

Vector3 EnemyFlightFields::steer_point(int64_t p_id, const Vector3 &p_from, const Vector3 &p_goal) const {
	Vector3 aim;
	real_t dist = 0.0f;
	steer(p_id, p_from, p_goal, aim, dist);
	return aim;
}
