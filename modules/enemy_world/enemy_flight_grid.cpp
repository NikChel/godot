#include "enemy_flight_grid.h"

#include "core/math/math_funcs.h"
#include "core/object/class_db.h"
#include "core/os/os.h"

#include <string.h>

static constexpr uint64_t MAX_CELLS = 64ull * 1024ull * 1024ull; // Предел сетки (память занятости — байт на клетку)

Vector3i EnemyFlightGrid::cell_of(const Vector3 &p_local) const {
	const Vector3 p = (p_local - origin) / cell;
	return Vector3i((int)Math::floor(p.x), (int)Math::floor(p.y), (int)Math::floor(p.z));
}

Vector3 EnemyFlightGrid::center(uint32_t p_i) const {
	const uint32_t x = p_i % (uint32_t)nx;
	const uint32_t t = p_i / (uint32_t)nx;
	const uint32_t y = t % (uint32_t)ny;
	const uint32_t z = t / (uint32_t)ny;
	return origin + Vector3((real_t)x + 0.5f, (real_t)y + 0.5f, (real_t)z + 0.5f) * cell;
}

void EnemyFlightGrid::build_begin(const PackedFloat32Array &p_vertices, const PackedInt32Array &p_indices, const PackedVector3Array &p_volume, real_t p_cell, const Vector3 &p_half) {
	build = BUILD_NONE;
	occ.clear();
	part.clear();
	part_count = 0;
	tris.clear();
	free_count = 0;
	build_next = 0;
	build_usec = 0;
	build_steps = 0;
	nx = ny = nz = 0;
	epoch++;
	ERR_FAIL_COND_MSG(p_cell < 0.05f, "EnemyFlightGrid: cell too small.");
	ERR_FAIL_COND_MSG(p_volume.size() < 2 || (p_volume.size() & 1) != 0, "EnemyFlightGrid: volume — pairs of box min, max.");
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	cell = p_cell;
	half = p_half;
	AABB vol(p_volume[0], p_volume[1] - p_volume[0]);
	for (int i = 2; i + 1 < p_volume.size(); i += 2) {
		vol = vol.merge(AABB(p_volume[i], p_volume[i + 1] - p_volume[i]));
	}
	origin = vol.position - Vector3(cell, cell, cell); // Кайма в клетку — занята
	nx = (int)Math::ceil(vol.size.x / cell) + 2;
	ny = (int)Math::ceil(vol.size.y / cell) + 2;
	nz = (int)Math::ceil(vol.size.z / cell) + 2;
	const uint64_t n = (uint64_t)nx * (uint64_t)ny * (uint64_t)nz;
	if (n > MAX_CELLS || nx < 3 || ny < 3 || nz < 3) {
		nx = ny = nz = 0;
		ERR_FAIL_MSG(vformat("EnemyFlightGrid: grid %d cells out of range.", (int64_t)n));
	}
	occ.resize((uint32_t)n);
	memset(occ.ptr(), 1, (size_t)n);
	// Свободны клетки с центром в объёме
	for (int i = 0; i + 1 < p_volume.size(); i += 2) {
		const Vector3 lo = (p_volume[i] - origin) / cell;
		const Vector3 hi = (p_volume[i + 1] - origin) / cell;
		const int x0 = MAX(1, (int)Math::ceil(lo.x - 0.5f));
		const int y0 = MAX(1, (int)Math::ceil(lo.y - 0.5f));
		const int z0 = MAX(1, (int)Math::ceil(lo.z - 0.5f));
		const int x1 = MIN(nx - 2, (int)Math::floor(hi.x - 0.5f));
		const int y1 = MIN(ny - 2, (int)Math::floor(hi.y - 0.5f));
		const int z1 = MIN(nz - 2, (int)Math::floor(hi.z - 0.5f));
		for (int z = z0; z <= z1; z++) {
			for (int y = y0; y <= y1; y++) {
				if (x1 >= x0) {
					memset(occ.ptr() + index(x0, y, z), 0, (size_t)(x1 - x0 + 1));
				}
			}
		}
	}
	const int vn = p_vertices.size() / 3;
	const float *v = p_vertices.ptr();
	tris.reserve((uint32_t)p_indices.size());
	for (int i = 0; i + 2 < p_indices.size(); i += 3) {
		const int a = p_indices[i];
		const int b = p_indices[i + 1];
		const int c = p_indices[i + 2];
		if (a < 0 || b < 0 || c < 0 || a >= vn || b >= vn || c >= vn) {
			continue;
		}
		tris.push_back(Vector3(v[a * 3], v[a * 3 + 1], v[a * 3 + 2]));
		tris.push_back(Vector3(v[b * 3], v[b * 3 + 1], v[b * 3 + 2]));
		tris.push_back(Vector3(v[c * 3], v[c * 3 + 1], v[c * 3 + 2]));
	}
	build = BUILD_RUN;
	build_usec = OS::get_singleton()->get_ticks_usec() - t0;
}

bool EnemyFlightGrid::build_step(int p_budget_usec) {
	if (build != BUILD_RUN) {
		return build == BUILD_READY;
	}
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	build_steps++;
	const uint32_t n = tris.size() / 3;
	while (build_next < n) {
		const uint32_t k = build_next * 3;
		_voxelize(tris[k], tris[k + 1], tris[k + 2]);
		build_next++;
		if (OS::get_singleton()->get_ticks_usec() - t0 > (uint64_t)MAX(p_budget_usec, 1)) {
			break;
		}
	}
	if (build_next >= n) {
		_finish_build();
	}
	build_usec += OS::get_singleton()->get_ticks_usec() - t0;
	return build == BUILD_READY;
}

// Клетки, чья коробка тела (центр ± half) пересекает треугольник, — заняты.
void EnemyFlightGrid::_voxelize(const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_c) {
	const Vector3 lo = (p_a.min(p_b).min(p_c) - half - origin) / cell;
	const Vector3 hi = (p_a.max(p_b).max(p_c) + half - origin) / cell;
	if (hi.x < 0.0f || hi.y < 0.0f || hi.z < 0.0f || lo.x > (real_t)nx || lo.y > (real_t)ny || lo.z > (real_t)nz) {
		return;
	}
	const int x0 = MAX(1, (int)Math::ceil(lo.x - 0.5f));
	const int y0 = MAX(1, (int)Math::ceil(lo.y - 0.5f));
	const int z0 = MAX(1, (int)Math::ceil(lo.z - 0.5f));
	const int x1 = MIN(nx - 2, (int)Math::floor(hi.x - 0.5f));
	const int y1 = MIN(ny - 2, (int)Math::floor(hi.y - 0.5f));
	const int z1 = MIN(nz - 2, (int)Math::floor(hi.z - 0.5f));
	for (int z = z0; z <= z1; z++) {
		for (int y = y0; y <= y1; y++) {
			uint32_t i = index(x0, y, z);
			for (int x = x0; x <= x1; x++, i++) {
				if (occ[i]) {
					continue;
				}
				const Vector3 c = origin + Vector3((real_t)x + 0.5f, (real_t)y + 0.5f, (real_t)z + 0.5f) * cell;
				if (_tri_box(c, half, p_a, p_b, p_c)) {
					occ[i] = 1;
				}
			}
		}
	}
}

// Пересечение треугольника и коробки (центр, полуразмеры): разделяющие оси — 9 рёбер × осей коробки, 3 оси коробки,
// нормаль треугольника (Akenine-Möller).
bool EnemyFlightGrid::_tri_box(const Vector3 &p_center, const Vector3 &p_half, const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_c) {
	const Vector3 v0 = p_a - p_center;
	const Vector3 v1 = p_b - p_center;
	const Vector3 v2 = p_c - p_center;
	// Оси коробки
	if (MIN(v0.x, MIN(v1.x, v2.x)) > p_half.x || MAX(v0.x, MAX(v1.x, v2.x)) < -p_half.x) {
		return false;
	}
	if (MIN(v0.y, MIN(v1.y, v2.y)) > p_half.y || MAX(v0.y, MAX(v1.y, v2.y)) < -p_half.y) {
		return false;
	}
	if (MIN(v0.z, MIN(v1.z, v2.z)) > p_half.z || MAX(v0.z, MAX(v1.z, v2.z)) < -p_half.z) {
		return false;
	}
	const Vector3 e[3] = { v1 - v0, v2 - v1, v0 - v2 };
	for (int k = 0; k < 3; k++) {
		const Vector3 &ed = e[k];
		const Vector3 axes[3] = { Vector3(0.0f, -ed.z, ed.y), Vector3(ed.z, 0.0f, -ed.x), Vector3(-ed.y, ed.x, 0.0f) };
		for (int j = 0; j < 3; j++) {
			const Vector3 &ax = axes[j];
			const real_t p0 = v0.dot(ax);
			const real_t p1 = v1.dot(ax);
			const real_t p2 = v2.dot(ax);
			const real_t r = p_half.x * Math::abs(ax.x) + p_half.y * Math::abs(ax.y) + p_half.z * Math::abs(ax.z);
			if (MIN(p0, MIN(p1, p2)) > r || MAX(p0, MAX(p1, p2)) < -r) {
				return false;
			}
		}
	}
	// Плоскость треугольника
	const Vector3 nrm = e[0].cross(e[1]);
	const real_t d = nrm.dot(v0);
	const real_t r = p_half.x * Math::abs(nrm.x) + p_half.y * Math::abs(nrm.y) + p_half.z * Math::abs(nrm.z);
	return Math::abs(d) <= r;
}

void EnemyFlightGrid::_mark_border() {
	for (int z = 0; z < nz; z++) {
		for (int y = 0; y < ny; y++) {
			if (z == 0 || y == 0 || z == nz - 1 || y == ny - 1) {
				memset(occ.ptr() + index(0, y, z), 1, (size_t)nx);
			} else {
				occ[index(0, y, z)] = 1;
				occ[index(nx - 1, y, z)] = 1;
			}
		}
	}
}

void EnemyFlightGrid::_finish_build() {
	_mark_border();
	tris = LocalVector<Vector3>();
	free_count = 0;
	for (uint32_t i = 0; i < occ.size(); i++) {
		free_count += occ[i] ? 0 : 1;
	}
	_label_parts();
	build = BUILD_READY;
}

// Части свободных клеток (шапка): обход в ширину по 6 соседям от каждой ещё не помеченной свободной клетки.
void EnemyFlightGrid::_label_parts() {
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	const uint32_t n = occ.size();
	part.resize(n);
	memset(part.ptr(), 0, sizeof(uint16_t) * n);
	part_count = 0;
	const uint32_t sy = (uint32_t)nx;
	const uint32_t sz = sy * (uint32_t)ny;
	LocalVector<uint32_t> q;
	for (uint32_t i = 0; i < n; i++) {
		if (occ[i] || part[i]) {
			continue;
		}
		part_count++;
		const uint16_t id = (uint16_t)MIN(part_count, (int)UINT16_MAX);
		q.clear();
		q.push_back(i);
		part[i] = id;
		for (uint32_t k = 0; k < q.size(); k++) {
			const uint32_t c = q[k];
			const uint32_t nb[6] = { c + 1, c - 1, c + sy, c - sy, c + sz, c - sz }; // Свободная клетка — не у края
			for (int j = 0; j < 6; j++) {
				const uint32_t m = nb[j];
				if (!occ[m] && !part[m]) {
					part[m] = id;
					q.push_back(m);
				}
			}
		}
	}
	part_usec = OS::get_singleton()->get_ticks_usec() - t0;
}

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

PackedByteArray EnemyFlightGrid::get_bits() const {
	PackedByteArray out;
	if (build != BUILD_READY) {
		return out;
	}
	const uint32_t n = occ.size();
	out.resize((int)((n + 7) / 8));
	uint8_t *w = out.ptrw();
	memset(w, 0, (size_t)out.size());
	for (uint32_t i = 0; i < n; i++) {
		if (occ[i]) {
			w[i >> 3] |= (uint8_t)(1u << (i & 7));
		}
	}
	return out;
}

bool EnemyFlightGrid::restore(const Vector3 &p_origin, const Vector3i &p_dims, real_t p_cell, const Vector3 &p_half, const PackedByteArray &p_bits) {
	build = BUILD_NONE;
	tris.clear();
	epoch++;
	const uint64_t n = (uint64_t)MAX(p_dims.x, 0) * (uint64_t)MAX(p_dims.y, 0) * (uint64_t)MAX(p_dims.z, 0);
	ERR_FAIL_COND_V(n == 0 || n > MAX_CELLS || p_cell < 0.05f, false);
	ERR_FAIL_COND_V((uint64_t)p_bits.size() != (n + 7) / 8, false);
	origin = p_origin;
	nx = p_dims.x;
	ny = p_dims.y;
	nz = p_dims.z;
	cell = p_cell;
	half = p_half;
	occ.resize((uint32_t)n);
	const uint8_t *r = p_bits.ptr();
	for (uint32_t i = 0; i < (uint32_t)n; i++) {
		occ[i] = (uint8_t)((r[i >> 3] >> (i & 7)) & 1u);
	}
	build_steps = 0;
	build_usec = 0;
	_finish_build();
	return true;
}

bool EnemyFlightGrid::is_free(const Vector3 &p_local) const {
	if (build != BUILD_READY) {
		return false;
	}
	const Vector3i c = cell_of(p_local);
	return in(c) && occ[index(c.x, c.y, c.z)] == 0;
}

// Обход клеток отрезка (Amanatides–Woo): клетки между концами свободны; концы не проверяются (тело у пола — в занятой
// клетке своей коробки, цель — тоже).
bool EnemyFlightGrid::line_free(const Vector3 &p_a, const Vector3 &p_b) const {
	if (build != BUILD_READY) {
		return false;
	}
	const Vector3 pa = (p_a - origin) / cell;
	const Vector3 pb = (p_b - origin) / cell;
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
		if (!in(c) || occ[index(c.x, c.y, c.z)]) {
			return false;
		}
	}
	return true;
}

void EnemyFlightGrid::_bind_methods() {
	ClassDB::bind_method(D_METHOD("build_begin", "vertices", "indices", "volume", "cell", "half"), &EnemyFlightGrid::build_begin);
	ClassDB::bind_method(D_METHOD("build_step", "budget_usec"), &EnemyFlightGrid::build_step);
	ClassDB::bind_method(D_METHOD("is_ready"), &EnemyFlightGrid::is_ready);
	ClassDB::bind_method(D_METHOD("is_building"), &EnemyFlightGrid::is_building);
	ClassDB::bind_method(D_METHOD("get_bits"), &EnemyFlightGrid::get_bits);
	ClassDB::bind_method(D_METHOD("restore", "origin", "dims", "cell", "half", "bits"), &EnemyFlightGrid::restore);
	ClassDB::bind_method(D_METHOD("get_origin"), &EnemyFlightGrid::get_origin);
	ClassDB::bind_method(D_METHOD("get_dims"), &EnemyFlightGrid::get_dims);
	ClassDB::bind_method(D_METHOD("get_cell"), &EnemyFlightGrid::get_cell);
	ClassDB::bind_method(D_METHOD("get_half"), &EnemyFlightGrid::get_half);
	ClassDB::bind_method(D_METHOD("get_cell_count"), &EnemyFlightGrid::get_cell_count);
	ClassDB::bind_method(D_METHOD("get_free_count"), &EnemyFlightGrid::get_free_count);
	ClassDB::bind_method(D_METHOD("get_memory_bytes"), &EnemyFlightGrid::get_memory_bytes);
	ClassDB::bind_method(D_METHOD("get_build_usec"), &EnemyFlightGrid::get_build_usec);
	ClassDB::bind_method(D_METHOD("get_build_steps"), &EnemyFlightGrid::get_build_steps);
	ClassDB::bind_method(D_METHOD("contains", "local"), &EnemyFlightGrid::contains);
	ClassDB::bind_method(D_METHOD("is_free", "local"), &EnemyFlightGrid::is_free);
	ClassDB::bind_method(D_METHOD("line_free", "a", "b"), &EnemyFlightGrid::line_free);
	ClassDB::bind_method(D_METHOD("component_at", "local"), &EnemyFlightGrid::component_at);
	ClassDB::bind_method(D_METHOD("get_component_count"), &EnemyFlightGrid::get_component_count);
	ClassDB::bind_method(D_METHOD("get_component_usec"), &EnemyFlightGrid::get_component_usec);
}
