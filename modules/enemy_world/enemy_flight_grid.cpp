#include "enemy_flight_grid.h"

#include "core/math/math_funcs.h"
#include "core/object/class_db.h"
#include "core/os/os.h"

#include <string.h>

static constexpr uint64_t MAX_CELLS = 64ull * 1024ull * 1024ull; // Предел сетки (память занятости — байт на клетку)
static constexpr uint32_t QUEUE_COMPACT = 1u << 20; // Очередь обхода сжимается, когда пройдено больше

Vector3i EnemyFlightGrid::_cell_of(const Vector3 &p_local) const {
	const Vector3 p = (p_local - origin) / cell;
	return Vector3i((int)Math::floor(p.x), (int)Math::floor(p.y), (int)Math::floor(p.z));
}

Vector3 EnemyFlightGrid::_center(uint32_t p_i) const {
	const uint32_t x = p_i % (uint32_t)nx;
	const uint32_t t = p_i / (uint32_t)nx;
	const uint32_t y = t % (uint32_t)ny;
	const uint32_t z = t / (uint32_t)ny;
	return origin + Vector3((real_t)x + 0.5f, (real_t)y + 0.5f, (real_t)z + 0.5f) * cell;
}

void EnemyFlightGrid::_reset_fields() {
	for (int i = 0; i < MAX_TARGETS; i++) {
		targets[i] = Target();
		targets[i].buf = i;
	}
	work_buf = MAX_TARGETS;
	work_slot = -1;
	queue.clear();
	q_head = 0;
}

void EnemyFlightGrid::build_begin(const PackedFloat32Array &p_vertices, const PackedInt32Array &p_indices, const PackedVector3Array &p_volume, real_t p_cell, const Vector3 &p_half) {
	build = BUILD_NONE;
	occ.clear();
	tris.clear();
	free_count = 0;
	build_next = 0;
	build_usec = 0;
	build_steps = 0;
	nx = ny = nz = 0;
	_reset_fields();
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
					memset(occ.ptr() + _idx(x0, y, z), 0, (size_t)(x1 - x0 + 1));
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
			uint32_t i = _idx(x0, y, z);
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
				memset(occ.ptr() + _idx(0, y, z), 1, (size_t)nx);
			} else {
				occ[_idx(0, y, z)] = 1;
				occ[_idx(nx - 1, y, z)] = 1;
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
	build = BUILD_READY;
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
	_reset_fields();
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

int64_t EnemyFlightGrid::get_memory_bytes() const {
	int64_t m = (int64_t)occ.size() + (int64_t)queue.size() * 4;
	for (int i = 0; i <= MAX_TARGETS; i++) {
		m += (int64_t)bufs[i].size() * 2;
	}
	return m;
}

bool EnemyFlightGrid::is_free(const Vector3 &p_local) const {
	if (build != BUILD_READY) {
		return false;
	}
	const Vector3i c = _cell_of(p_local);
	return _in(c) && occ[_idx(c.x, c.y, c.z)] == 0;
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
	if (!_in(c) || !_in(e)) {
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
		if (!_in(c) || occ[_idx(c.x, c.y, c.z)]) {
			return false;
		}
	}
	return true;
}

int EnemyFlightGrid::_slot(int64_t p_id) const {
	for (int i = 0; i < MAX_TARGETS; i++) {
		if (targets[i].used && targets[i].id == p_id) {
			return i;
		}
	}
	return -1;
}

void EnemyFlightGrid::want(int64_t p_id, const Vector3 &p_local, uint64_t p_now) {
	now = MAX(now, p_now);
	int s = _slot(p_id);
	if (s < 0) {
		for (int i = 0; i < MAX_TARGETS; i++) {
			if (!targets[i].used || now > targets[i].wanted + RELEASE_TICKS) {
				s = i;
				break;
			}
		}
		if (s < 0) {
			return; // Все слоты заняты нужными целями
		}
		if (work_slot == s) {
			work_slot = -1; // Считали поле ушедшей цели
		}
		Target &t = targets[s];
		t.used = true;
		t.id = p_id;
		t.ready = false;
	}
	targets[s].pos = p_local;
	targets[s].wanted = now;
}

// Клетка посева цели: клетка центра, зажатая внутрь сетки.
static uint32_t seed_index(const Vector3i &p_c, int p_nx, int p_ny, int p_nz) {
	const int x = CLAMP(p_c.x, 1, p_nx - 2);
	const int y = CLAMP(p_c.y, 1, p_ny - 2);
	const int z = CLAMP(p_c.z, 1, p_nz - 2);
	return (uint32_t)x + (uint32_t)p_nx * ((uint32_t)y + (uint32_t)p_ny * (uint32_t)z);
}

// Следующая цель по кругу, чьё поле нужно: нет готового или цель сошла с клетки посева.
bool EnemyFlightGrid::_start_field() {
	for (int k = 0; k < MAX_TARGETS; k++) {
		const int s = (rr + k) % MAX_TARGETS;
		const Target &t = targets[s];
		if (!t.used || now > t.wanted + RELEASE_TICKS) {
			continue;
		}
		const uint32_t seed = seed_index(_cell_of(t.pos), nx, ny, nz);
		if (t.ready && seed == t.seed) {
			continue;
		}
		work_slot = s;
		work_id = t.id;
		work_seed = seed;
		work_fill = 0;
		work_seeded = false;
		work_usec = 0;
		work_steps = 0;
		work_visited = 0;
		rr = s + 1;
		return true;
	}
	return false;
}

// Посев: свободные клетки куба радиуса 1 у клетки цели (нет — радиуса 2), расстояние 0.
bool EnemyFlightGrid::_seed(uint32_t p_cell) {
	uint16_t *w = bufs[work_buf].ptr();
	const int x = (int)(p_cell % (uint32_t)nx);
	const int y = (int)((p_cell / (uint32_t)nx) % (uint32_t)ny);
	const int z = (int)(p_cell / ((uint32_t)nx * (uint32_t)ny));
	for (int r = 1; r <= 2; r++) {
		for (int dz = -r; dz <= r; dz++) {
			for (int dy = -r; dy <= r; dy++) {
				for (int dx = -r; dx <= r; dx++) {
					const Vector3i c(x + dx, y + dy, z + dz);
					if (!_in(c)) {
						continue;
					}
					const uint32_t j = _idx(c.x, c.y, c.z);
					if (occ[j] == 0 && w[j] == UNREACHED) {
						w[j] = 0;
						queue.push_back(j);
					}
				}
			}
		}
		if (!queue.is_empty()) {
			return true;
		}
	}
	return false;
}

bool EnemyFlightGrid::step(int p_budget_usec, uint64_t p_now) {
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	const uint64_t budget = (uint64_t)MAX(p_budget_usec, 1);
	now = MAX(now, p_now);
	last_step_usec = 0;
	if (build != BUILD_READY || (work_slot < 0 && !_start_field())) {
		return false;
	}
	const uint32_t n = _cells();
	LocalVector<uint16_t> &wb = bufs[work_buf];
	if (wb.size() != n) {
		wb.resize(n);
	}
	while (work_fill < n) {
		const uint32_t m = MIN(FILL_CHUNK, n - work_fill);
		memset(wb.ptr() + work_fill, 0xFF, sizeof(uint16_t) * m);
		work_fill += m;
		if (OS::get_singleton()->get_ticks_usec() - t0 > budget) {
			break;
		}
	}
	if (work_fill >= n && !work_seeded) {
		queue.clear();
		q_head = 0;
		work_seeded = true;
		if (!_seed(work_seed)) {
			work_slot = -1; // Цель в стене / вне объёма — поля нет (старое держится)
		}
	}
	if (work_slot >= 0 && work_seeded) {
		uint16_t *w = wb.ptr();
		const uint8_t *o = occ.ptr();
		const uint32_t sy = (uint32_t)nx;
		const uint32_t sz = (uint32_t)nx * (uint32_t)ny;
		uint32_t count = 0;
		while (q_head < queue.size()) {
			const uint32_t i = queue[q_head++];
			const uint16_t d = (uint16_t)(w[i] + 1);
			if (d >= UNREACHED) {
				continue;
			}
			const uint32_t nb[6] = { i + 1, i - 1, i + sy, i - sy, i + sz, i - sz }; // Свободная клетка — не у края
			for (int k = 0; k < 6; k++) {
				const uint32_t j = nb[k];
				if (o[j] == 0 && w[j] == UNREACHED) {
					w[j] = d;
					queue.push_back(j);
				}
			}
			if ((++count & 4095u) == 0) {
				if (q_head > QUEUE_COMPACT && q_head * 2 > queue.size()) {
					const uint32_t rest = queue.size() - q_head;
					memmove(queue.ptr(), queue.ptr() + q_head, sizeof(uint32_t) * rest);
					work_visited += q_head;
					queue.resize(rest);
					q_head = 0;
				}
				if (OS::get_singleton()->get_ticks_usec() - t0 > budget) {
					break;
				}
			}
		}
		if (q_head >= queue.size()) { // Обход закончен: поле цели — это
			work_visited += q_head;
			Target &t = targets[work_slot];
			if (t.used && t.id == work_id) {
				const int b = t.buf;
				t.buf = work_buf;
				work_buf = b;
				t.ready = true;
				t.seed = work_seed;
			}
			work_steps++;
			work_usec += OS::get_singleton()->get_ticks_usec() - t0;
			last_field_usec = work_usec;
			last_field_steps = work_steps;
			last_field_cells = (int)work_visited;
			fields_done++;
			work_slot = -1;
			queue.clear();
			q_head = 0;
			last_step_usec = OS::get_singleton()->get_ticks_usec() - t0;
			return true;
		}
	}
	work_steps++;
	last_step_usec = OS::get_singleton()->get_ticks_usec() - t0;
	work_usec += last_step_usec;
	return true;
}

const uint16_t *EnemyFlightGrid::_field(int64_t p_id) const {
	const int s = _slot(p_id);
	if (s < 0 || !targets[s].ready) {
		return nullptr;
	}
	return bufs[targets[s].buf].ptr();
}

int EnemyFlightGrid::get_target_count() const {
	int n = 0;
	for (int i = 0; i < MAX_TARGETS; i++) {
		n += (targets[i].used && now <= targets[i].wanted + RELEASE_TICKS) ? 1 : 0;
	}
	return n;
}

// Лучшая клетка у тела: свободная, с полем, наименьшее расстояние (равные — ближе к телу); куб радиуса 1, нет — 2.
bool EnemyFlightGrid::_best_start(const uint16_t *p_f, const Vector3 &p_from, uint32_t &r_cell) const {
	const Vector3i c0 = _cell_of(p_from);
	for (int r = 1; r <= 2; r++) {
		uint16_t best = UNREACHED;
		real_t best_d = (real_t)Math::INF;
		for (int dz = -r; dz <= r; dz++) {
			for (int dy = -r; dy <= r; dy++) {
				for (int dx = -r; dx <= r; dx++) {
					const Vector3i c(c0.x + dx, c0.y + dy, c0.z + dz);
					if (!_in(c)) {
						continue;
					}
					const uint32_t j = _idx(c.x, c.y, c.z);
					const uint16_t v = p_f[j];
					if (v == UNREACHED || v > best) {
						continue;
					}
					const real_t d = _center(j).distance_squared_to(p_from);
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

real_t EnemyFlightGrid::field_dist(int64_t p_id, const Vector3 &p_local) const {
	const uint16_t *f = _field(p_id);
	uint32_t s = 0;
	if (f == nullptr || !_best_start(f, p_local, s)) {
		return -1.0f;
	}
	return (real_t)f[s] * cell;
}

bool EnemyFlightGrid::steer(int64_t p_id, const Vector3 &p_from, const Vector3 &p_goal, Vector3 &r_aim, real_t &r_dist) const {
	r_aim = p_goal;
	r_dist = p_from.distance_to(p_goal);
	if (build != BUILD_READY) {
		return false;
	}
	if (line_free(p_from, p_goal)) {
		return true;
	}
	const uint16_t *f = _field(p_id);
	uint32_t s = 0;
	if (f == nullptr || !_best_start(f, p_from, s)) {
		return false;
	}
	r_dist = (real_t)f[s] * cell;
	uint32_t chain[DESCENT];
	int n = 0;
	uint32_t c = s;
	const uint32_t sz = (uint32_t)nx * (uint32_t)ny;
	for (int k = 0; k < DESCENT; k++) {
		uint32_t best = c;
		uint16_t bv = f[c];
		for (int dz = -1; dz <= 1; dz++) {
			for (int dy = -1; dy <= 1; dy++) {
				for (int dx = -1; dx <= 1; dx++) {
					// Клетка с полем свободна — не у края сетки: соседи в ней
					const uint32_t j = (uint32_t)((int64_t)c + dx + (int64_t)dy * nx + (int64_t)dz * (int64_t)sz);
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
		const Vector3 p = _center(chain[i]);
		if (line_free(p_from, p)) {
			r_aim = p;
			return true;
		}
	}
	r_aim = _center(chain[0]);
	return true;
}

Vector3 EnemyFlightGrid::steer_point(int64_t p_id, const Vector3 &p_from, const Vector3 &p_goal) const {
	Vector3 aim;
	real_t dist = 0.0f;
	steer(p_id, p_from, p_goal, aim, dist);
	return aim;
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
	ClassDB::bind_method(D_METHOD("want", "id", "local", "now"), &EnemyFlightGrid::want);
	ClassDB::bind_method(D_METHOD("step", "budget_usec", "now"), &EnemyFlightGrid::step);
	ClassDB::bind_method(D_METHOD("field_ready", "id"), &EnemyFlightGrid::field_ready);
	ClassDB::bind_method(D_METHOD("field_dist", "id", "local"), &EnemyFlightGrid::field_dist);
	ClassDB::bind_method(D_METHOD("steer_point", "id", "from", "goal"), &EnemyFlightGrid::steer_point);
	ClassDB::bind_method(D_METHOD("get_last_step_usec"), &EnemyFlightGrid::get_last_step_usec);
	ClassDB::bind_method(D_METHOD("get_last_field_usec"), &EnemyFlightGrid::get_last_field_usec);
	ClassDB::bind_method(D_METHOD("get_last_field_steps"), &EnemyFlightGrid::get_last_field_steps);
	ClassDB::bind_method(D_METHOD("get_last_field_cells"), &EnemyFlightGrid::get_last_field_cells);
	ClassDB::bind_method(D_METHOD("get_fields_done"), &EnemyFlightGrid::get_fields_done);
	ClassDB::bind_method(D_METHOD("get_target_count"), &EnemyFlightGrid::get_target_count);
	BIND_CONSTANT(MAX_TARGETS);
}
