#include "enemy_flight_fields.h"

#include "core/math/math_funcs.h"
#include "core/object/class_db.h"
#include "core/os/os.h"

#include <string.h>

static constexpr uint32_t QUEUE_COMPACT = 1u << 20; // Очередь обхода сжимается, когда пройдено больше

EnemyFlightFields::EnemyFlightFields() {
	targets.resize((uint32_t)max_targets);
	bufs.resize((uint32_t)max_targets + 1);
	_reset();
}

void EnemyFlightFields::set_grid(const Ref<EnemyFlightGrid> &p_grid) {
	grid = p_grid;
	_reset();
}

void EnemyFlightFields::set_max_targets(int p_count) {
	max_targets = CLAMP(p_count, 1, TARGETS_LIMIT);
	targets.resize((uint32_t)max_targets);
	bufs.resize((uint32_t)max_targets + 1);
	rr = 0;
	_reset();
}

void EnemyFlightFields::_reset() {
	for (int i = 0; i < max_targets; i++) {
		targets[i] = Target();
		targets[i].buf = i;
	}
	work_buf = max_targets;
	work_slot = -1;
	queue.clear();
	q_head = 0;
	epoch = grid.is_valid() ? grid->get_epoch() : 0;
}

// Сетка готова и та же, что у полей (сменилась — поля заново).
bool EnemyFlightFields::_sync() {
	if (!is_ready()) {
		return false;
	}
	if (grid->get_epoch() != epoch) {
		_reset();
	}
	return true;
}

int EnemyFlightFields::_slot(int64_t p_id) const {
	for (int i = 0; i < max_targets; i++) {
		if (targets[i].used && targets[i].id == p_id) {
			return i;
		}
	}
	return -1;
}

void EnemyFlightFields::want(int64_t p_id, const Vector3 &p_local, uint64_t p_now) {
	if (!_sync()) {
		return;
	}
	now = MAX(now, p_now);
	int s = _slot(p_id);
	if (s < 0) {
		for (int i = 0; i < max_targets; i++) {
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
uint32_t EnemyFlightFields::_seed_cell(const Vector3 &p_local) const {
	const Vector3i c = grid->cell_of(p_local);
	return grid->index(CLAMP(c.x, 1, grid->dim_x() - 2), CLAMP(c.y, 1, grid->dim_y() - 2), CLAMP(c.z, 1, grid->dim_z() - 2));
}

// Следующая цель по кругу, чьё поле нужно: нет готового или цель сошла с клетки посева.
bool EnemyFlightFields::_start_field() {
	for (int k = 0; k < max_targets; k++) {
		const int s = (rr + k) % max_targets;
		const Target &t = targets[s];
		if (!t.used || now > t.wanted + RELEASE_TICKS) {
			continue;
		}
		const uint32_t seed = _seed_cell(t.pos);
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
bool EnemyFlightFields::_seed(uint32_t p_cell) {
	uint16_t *w = bufs[work_buf].ptr();
	const uint8_t *o = grid->occupancy();
	const uint32_t nx = (uint32_t)grid->dim_x();
	const uint32_t ny = (uint32_t)grid->dim_y();
	const int x = (int)(p_cell % nx);
	const int y = (int)((p_cell / nx) % ny);
	const int z = (int)(p_cell / (nx * ny));
	for (int r = 1; r <= 2; r++) {
		for (int dz = -r; dz <= r; dz++) {
			for (int dy = -r; dy <= r; dy++) {
				for (int dx = -r; dx <= r; dx++) {
					const Vector3i c(x + dx, y + dy, z + dz);
					if (!grid->in(c)) {
						continue;
					}
					const uint32_t j = grid->index(c.x, c.y, c.z);
					if (o[j] == 0 && w[j] == UNREACHED) {
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

bool EnemyFlightFields::step(int p_budget_usec, uint64_t p_now) {
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	const uint64_t budget = (uint64_t)MAX(p_budget_usec, 1);
	now = MAX(now, p_now);
	last_step_usec = 0;
	if (!_sync() || (work_slot < 0 && !_start_field())) {
		return false;
	}
	const uint32_t n = grid->cells();
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
		const uint8_t *o = grid->occupancy();
		const uint32_t sy = (uint32_t)grid->dim_x();
		const uint32_t sz = sy * (uint32_t)grid->dim_y();
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

const uint16_t *EnemyFlightFields::_field(int64_t p_id) const {
	if (!is_ready() || grid->get_epoch() != epoch) {
		return nullptr;
	}
	const int s = _slot(p_id);
	if (s < 0 || !targets[s].ready) {
		return nullptr;
	}
	return bufs[targets[s].buf].ptr();
}

int EnemyFlightFields::get_target_count() const {
	int n = 0;
	for (int i = 0; i < max_targets; i++) {
		n += (targets[i].used && now <= targets[i].wanted + RELEASE_TICKS) ? 1 : 0;
	}
	return n;
}

int64_t EnemyFlightFields::get_memory_bytes() const {
	int64_t m = (int64_t)queue.size() * 4;
	for (int i = 0; i <= max_targets; i++) {
		m += (int64_t)bufs[i].size() * 2;
	}
	return m;
}

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

void EnemyFlightFields::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_grid", "grid"), &EnemyFlightFields::set_grid);
	ClassDB::bind_method(D_METHOD("get_grid"), &EnemyFlightFields::get_grid);
	ClassDB::bind_method(D_METHOD("is_ready"), &EnemyFlightFields::is_ready);
	ClassDB::bind_method(D_METHOD("want", "id", "local", "now"), &EnemyFlightFields::want);
	ClassDB::bind_method(D_METHOD("step", "budget_usec", "now"), &EnemyFlightFields::step);
	ClassDB::bind_method(D_METHOD("field_ready", "id"), &EnemyFlightFields::field_ready);
	ClassDB::bind_method(D_METHOD("field_dist", "id", "local"), &EnemyFlightFields::field_dist);
	ClassDB::bind_method(D_METHOD("steer_point", "id", "from", "goal"), &EnemyFlightFields::steer_point);
	ClassDB::bind_method(D_METHOD("get_memory_bytes"), &EnemyFlightFields::get_memory_bytes);
	ClassDB::bind_method(D_METHOD("get_last_step_usec"), &EnemyFlightFields::get_last_step_usec);
	ClassDB::bind_method(D_METHOD("get_last_field_usec"), &EnemyFlightFields::get_last_field_usec);
	ClassDB::bind_method(D_METHOD("get_last_field_steps"), &EnemyFlightFields::get_last_field_steps);
	ClassDB::bind_method(D_METHOD("get_last_field_cells"), &EnemyFlightFields::get_last_field_cells);
	ClassDB::bind_method(D_METHOD("get_fields_done"), &EnemyFlightFields::get_fields_done);
	ClassDB::bind_method(D_METHOD("get_target_count"), &EnemyFlightFields::get_target_count);
	ClassDB::bind_method(D_METHOD("set_max_targets", "count"), &EnemyFlightFields::set_max_targets);
	ClassDB::bind_method(D_METHOD("get_max_targets"), &EnemyFlightFields::get_max_targets);
	BIND_CONSTANT(MAX_TARGETS);
}
