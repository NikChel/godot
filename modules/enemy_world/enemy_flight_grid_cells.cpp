#include "enemy_flight_grid.h"

#include "core/math/math_funcs.h"

// Запросы по клеткам готовой сетки полёта (EnemyFlightGrid, шапка): часть точки, проходимость клетки, прямая
// видимость; открытая сетка — вне коробки и кайма свободны, вход отрезка во внутренние клетки. Построение и кэш —
// enemy_flight_grid.cpp.

int EnemyFlightGrid::component_at(const Vector3 &p_local) const {
	if (build != BUILD_READY || part.size() != occ.size()) {
		return -1;
	}
	const Vector3i c0 = cell_of(p_local);
	if (in(c0) && part[index(c0.x, c0.y, c0.z)] != 0) {
		return part[index(c0.x, c0.y, c0.z)];
	}
	for (int r = 1; r <= 2; r++) {
		int best = -1;
		real_t best_d = (real_t)Math::INF;
		for (int dz = -r; dz <= r; dz++) {
			for (int dy = -r; dy <= r; dy++) {
				for (int dx = -r; dx <= r; dx++) {
					const Vector3i c(c0.x + dx, c0.y + dy, c0.z + dz);
					if (!in(c)) {
						continue;
					}
					const uint32_t j = index(c.x, c.y, c.z);
					if (part[j] == 0) {
						continue;
					}
					const real_t d = center(j).distance_squared_to(p_local);
					if (d < best_d) {
						best_d = d;
						best = part[j];
					}
				}
			}
		}
		if (best >= 0) {
			return best;
		}
	}
	return -1;
}

bool EnemyFlightGrid::_blocked(const Vector3i &p_c) const {
	if (!in(p_c)) {
		return !open;
	}
	if (open && (p_c.x == 0 || p_c.y == 0 || p_c.z == 0 || p_c.x == nx - 1 || p_c.y == ny - 1 || p_c.z == nz - 1)) {
		return false;
	}
	return occ[index(p_c.x, p_c.y, p_c.z)] != 0;
}

bool EnemyFlightGrid::is_free(const Vector3 &p_local) const {
	if (build != BUILD_READY) {
		return false;
	}
	return !_blocked(cell_of(p_local));
}

// Отрезок r_a → r_b обрезан коробкой [p_lo, p_hi] (Лианг — Барски); false — коробку не пересекает.
static bool clip_box(Vector3 &r_a, Vector3 &r_b, const Vector3 &p_lo, const Vector3 &p_hi) {
	real_t t0 = 0.0f;
	real_t t1 = 1.0f;
	const Vector3 a = r_a;
	const Vector3 d = r_b - r_a;
	for (int k = 0; k < 3; k++) {
		if (Math::abs(d[k]) < 1e-9f) {
			if (a[k] < p_lo[k] || a[k] > p_hi[k]) {
				return false;
			}
			continue;
		}
		real_t ta = (p_lo[k] - a[k]) / d[k];
		real_t tb = (p_hi[k] - a[k]) / d[k];
		if (ta > tb) {
			SWAP(ta, tb);
		}
		t0 = MAX(t0, ta);
		t1 = MIN(t1, tb);
		if (t0 > t1) {
			return false;
		}
	}
	r_a = a + d * t0;
	r_b = a + d * t1;
	return true;
}

bool EnemyFlightGrid::interior(const Vector3 &p_local) const {
	const Vector3i c = cell_of(p_local);
	return build == BUILD_READY && c.x >= 1 && c.y >= 1 && c.z >= 1 && c.x <= nx - 2 && c.y <= ny - 2 && c.z <= nz - 2;
}

Vector3 EnemyFlightGrid::entry_point(const Vector3 &p_a, const Vector3 &p_b) const {
	if (build != BUILD_READY || interior(p_a)) {
		return p_a;
	}
	constexpr real_t EPS = 1e-3f;
	Vector3 pa = (p_a - origin) / cell;
	Vector3 pb = (p_b - origin) / cell;
	if (!clip_box(pa, pb, Vector3(1.0f + EPS, 1.0f + EPS, 1.0f + EPS), Vector3((real_t)(nx - 1) - EPS, (real_t)(ny - 1) - EPS, (real_t)(nz - 1) - EPS))) {
		return p_b;
	}
	return origin + pa * cell;
}

// Обход клеток отрезка (Amanatides–Woo): клетки между концами свободны; концы не проверяются (тело у пола — в занятой
// клетке своей коробки, цель — тоже). Открытая сетка: отрезок обрезан коробкой сетки (вне неё свободно).
bool EnemyFlightGrid::line_free(const Vector3 &p_a, const Vector3 &p_b) const {
	if (build != BUILD_READY) {
		return false;
	}
	Vector3 pa = (p_a - origin) / cell;
	Vector3 pb = (p_b - origin) / cell;
	if (open) {
		constexpr real_t EPS = 1e-3f;
		if (!clip_box(pa, pb, Vector3(EPS, EPS, EPS), Vector3((real_t)nx - EPS, (real_t)ny - EPS, (real_t)nz - EPS))) {
			return true; // Отрезок вне коробки сетки
		}
	}
	Vector3i c((int)Math::floor(pa.x), (int)Math::floor(pa.y), (int)Math::floor(pa.z));
	const Vector3i e((int)Math::floor(pb.x), (int)Math::floor(pb.y), (int)Math::floor(pb.z));
	if (!in(c) || !in(e)) {
		return false;
	}
	const Vector3 d = pb - pa;
	int step[3];
	real_t t_max[3];
	real_t t_delta[3];
	for (int k = 0; k < 3; k++) {
		if (d[k] > 1e-9f) {
			step[k] = 1;
			t_max[k] = ((real_t)c[k] + 1.0f - pa[k]) / d[k];
			t_delta[k] = 1.0f / d[k];
		} else if (d[k] < -1e-9f) {
			step[k] = -1;
			t_max[k] = (pa[k] - (real_t)c[k]) / -d[k];
			t_delta[k] = -1.0f / d[k];
		} else {
			step[k] = 0;
			t_max[k] = (real_t)Math::INF;
			t_delta[k] = (real_t)Math::INF;
		}
	}
	const int limit = Math::abs(e.x - c.x) + Math::abs(e.y - c.y) + Math::abs(e.z - c.z) + 3;
	for (int s = 0; s < limit && c != e; s++) {
		const int k = t_max[0] < t_max[1] ? (t_max[0] < t_max[2] ? 0 : 2) : (t_max[1] < t_max[2] ? 1 : 2);
		if (t_max[k] > 1.0f) {
			return true; // Конец отрезка в этой клетке (округление)
		}
		c[k] += step[k];
		t_max[k] += t_delta[k];
		if (c == e) {
			return true;
		}
		if (_blocked(c)) {
			return false;
		}
	}
	return true;
}
