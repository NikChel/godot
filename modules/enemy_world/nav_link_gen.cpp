#include "nav_link_gen.h"

#include "core/math/vector2.h"
#include "core/object/class_db.h"
#include "core/variant/dictionary.h"
#include "core/os/os.h"
#include "core/templates/hash_map.h"
#include "servers/navigation_3d/navigation_server_3d.h"

EnemyLinkGen::~EnemyLinkGen() {
	if (capsule.is_valid() && PhysicsServer3D::get_singleton()) {
		PhysicsServer3D::get_singleton()->free_rid(capsule);
	}
}

// ---------- Подготовка (поток) ----------

void EnemyLinkGen::prepare(const Ref<NavigationMesh> &p_full, const Ref<NavigationMesh> &p_main) {
	ERR_FAIL_COND(p_full.is_null() || p_main.is_null());
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	NavColumnIndex main_index;
	main_index.build(p_main);
	const Vector<Vector3> fv = p_full->get_vertices();
	Vector<Vector<int>> keep;
	const int fc = p_full->get_polygon_count();
	for (int i = 0; i < fc; i++) {
		const Vector<int> poly = p_full->get_polygon(i);
		if (poly.size() >= 3 && !_near_main(main_index, fv, poly)) {
			keep.push_back(poly);
		}
	}
	narrow = _rebuild(p_full, fv, keep);
	merged = _merge(p_main, narrow);
	index.build(merged);
	main_polys = p_main->get_polygon_count();
	probe_point = narrow->get_polygon_count() > 0 ? _centroid(narrow->get_vertices(), narrow->get_polygon(0)) : Vector3();
	_components();
	links.clear();
	queue.clear();
	cursor = 0;
	scan_k = 0;
	done = false;
	samples = 0;
	candidates = 0;
	shortcuts = 0;
	blocked = 0;
	narrow_scanned = 0;
	steps = 0;
	gen_usec = 0;
	max_step_usec = 0;
	_add_samples(p_main, false, nullptr);
	prepare_usec = OS::get_singleton()->get_ticks_usec() - t0;
}

Vector3 EnemyLinkGen::_centroid(const Vector<Vector3> &p_verts, const Vector<int> &p_poly) {
	Vector3 c;
	for (int k = 0; k < p_poly.size(); k++) {
		c += p_verts[p_poly[k]];
	}
	return c / (real_t)p_poly.size();
}

// Рядом основная: её накрывает центр, точка кольца или вершина (сдвинутая к центру).
bool EnemyLinkGen::_near_main(const NavColumnIndex &p_main, const Vector<Vector3> &p_verts, const Vector<int> &p_poly) {
	const Vector3 c = _centroid(p_verts, p_poly);
	Vector3 at;
	int poly = -1;
	if (p_main.snap(c, COLUMN_H, 0.0f, at, poly)) {
		return true;
	}
	for (int k = 0; k < RING_N; k++) {
		const real_t a = (real_t)(Math::TAU * k / RING_N);
		if (p_main.snap(c + Vector3(Math::cos(a), 0.0f, Math::sin(a)) * RING_R, COLUMN_H, 0.0f, at, poly)) {
			return true;
		}
	}
	for (int k = 0; k < p_poly.size(); k++) {
		const Vector3 &v = p_verts[p_poly[k]];
		if (p_main.snap(v + (c - v).limit_length(VERT_IN), COLUMN_H, 0.0f, at, poly)) {
			return true;
		}
	}
	return false;
}

Ref<NavigationMesh> EnemyLinkGen::_rebuild(const Ref<NavigationMesh> &p_src, const Vector<Vector3> &p_verts, const Vector<Vector<int>> &p_polys) {
	Ref<NavigationMesh> out;
	out.instantiate();
	out->set_cell_size(p_src->get_cell_size());
	out->set_cell_height(p_src->get_cell_height());
	HashMap<int, int> remap;
	Vector<Vector3> nv;
	Vector<Vector<int>> np;
	for (int i = 0; i < p_polys.size(); i++) {
		const Vector<int> &poly = p_polys[i];
		Vector<int> q;
		q.resize(poly.size());
		for (int k = 0; k < poly.size(); k++) {
			const int *at = remap.getptr(poly[k]);
			if (at) {
				q.write[k] = *at;
			} else {
				remap.insert(poly[k], nv.size());
				q.write[k] = nv.size();
				nv.push_back(p_verts[poly[k]]);
			}
		}
		np.push_back(q);
	}
	out->set_data(nv, np);
	return out;
}

Ref<NavigationMesh> EnemyLinkGen::_merge(const Ref<NavigationMesh> &p_main, const Ref<NavigationMesh> &p_narrow) {
	Ref<NavigationMesh> out;
	out.instantiate();
	out->set_cell_size(p_main->get_cell_size());
	out->set_cell_height(p_main->get_cell_height());
	Vector<Vector3> v = p_main->get_vertices();
	const int shift = v.size();
	v.append_array(p_narrow->get_vertices());
	Vector<Vector<int>> polys;
	const int mc = p_main->get_polygon_count();
	for (int i = 0; i < mc; i++) {
		polys.push_back(p_main->get_polygon(i));
	}
	const int nc = p_narrow->get_polygon_count();
	for (int i = 0; i < nc; i++) {
		Vector<int> q = p_narrow->get_polygon(i);
		for (int k = 0; k < q.size(); k++) {
			q.write[k] += shift;
		}
		polys.push_back(q);
	}
	out->set_data(v, polys);
	return out;
}

int EnemyLinkGen::_root_of(int p_i) {
	while (comp[p_i] != p_i) {
		comp[p_i] = comp[comp[p_i]];
		p_i = comp[p_i];
	}
	return p_i;
}

// Связные куски узкой сетки (многоугольники с общими вершинами).
void EnemyLinkGen::_components() {
	const int n = narrow->get_polygon_count();
	comp.resize(n);
	comp_reached.resize(n);
	comp_scanned.resize(n);
	for (int i = 0; i < n; i++) {
		comp[i] = i;
		comp_reached[i] = 0;
		comp_scanned[i] = 0;
	}
	HashMap<int, int> first; // Вершина → первый многоугольник с ней
	for (int i = 0; i < n; i++) {
		const Vector<int> poly = narrow->get_polygon(i);
		for (int k = 0; k < poly.size(); k++) {
			const int *f = first.getptr(poly[k]);
			if (f) {
				const int ra = _root_of(i);
				const int rb = _root_of(*f);
				comp[ra] = rb;
			} else {
				first.insert(poly[k], i);
			}
		}
	}
	for (int i = 0; i < n; i++) {
		comp[i] = _root_of(i);
	}
}

uint64_t EnemyLinkGen::_edge_key(int p_a, int p_b) {
	return ((uint64_t)(uint32_t)MIN(p_a, p_b) << 32) | (uint64_t)(uint32_t)MAX(p_a, p_b);
}

// Точки краёв (рёбра, не общие с соседним многоугольником) mesh; p_only — только отмеченные многоугольники.
void EnemyLinkGen::_add_samples(const Ref<NavigationMesh> &p_mesh, bool p_narrow, const LocalVector<uint8_t> *p_only) {
	const Vector<Vector3> v = p_mesh->get_vertices();
	const int n = p_mesh->get_polygon_count();
	LocalVector<Vector<int>> polys;
	polys.resize(n);
	HashMap<uint64_t, int> uses;
	for (int i = 0; i < n; i++) {
		polys[i] = p_mesh->get_polygon(i);
		const Vector<int> &poly = polys[i];
		for (int k = 0; k < poly.size(); k++) {
			uses[_edge_key(poly[k], poly[(k + 1) % poly.size()])] += 1;
		}
	}
	for (int i = 0; i < n; i++) {
		if (p_only && (*p_only)[i] == 0) {
			continue;
		}
		const Vector<int> &poly = polys[i];
		if (poly.size() < 3) {
			continue;
		}
		const Vector3 c = _centroid(v, poly);
		for (int k = 0; k < poly.size(); k++) {
			const int ia = poly[k];
			const int ib = poly[(k + 1) % poly.size()];
			if (uses[_edge_key(ia, ib)] == 1) {
				_add_edge(v[ia], v[ib], c, p_narrow);
			}
		}
	}
}

void EnemyLinkGen::_add_edge(const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_center, bool p_narrow) {
	const Vector3 ab(p_b.x - p_a.x, 0.0f, p_b.z - p_a.z);
	const real_t len = ab.length();
	if (len < 0.05f) {
		return;
	}
	const Vector3 dir = ab / len;
	Vector3 n(-dir.z, 0.0f, dir.x);
	const Vector3 mid = (p_a + p_b) * 0.5f;
	if (n.dot(Vector3(mid.x - p_center.x, 0.0f, mid.z - p_center.z)) < 0.0f) {
		n = -n;
	}
	const int count = MAX(1, (int)Math::floor(len / SAMPLE_STEP));
	for (int i = 0; i < count; i++) {
		Sample s;
		s.p = p_a.lerp(p_b, (i + 0.5f) / count);
		s.n = n;
		s.narrow = p_narrow;
		queue.push_back(s);
	}
	samples += count;
}

// ---------- Генерация (главный поток) ----------

void EnemyLinkGen::clear_profiles() {
	profiles.clear();
}

void EnemyLinkGen::add_profile(int p_layer_bit, real_t p_max_drop, real_t p_jump_impulse, real_t p_jump_speed, real_t p_gravity, real_t p_climb, bool p_narrow) {
	Profile p;
	p.bit = (uint32_t)p_layer_bit;
	p.max_drop = p_max_drop;
	p.impulse = p_jump_impulse;
	p.speed = p_jump_speed;
	p.gravity = p_gravity;
	p.climb = p_climb;
	p.walks_narrow = p_narrow;
	profiles.push_back(p);
}

void EnemyLinkGen::start(RID p_map, int p_walk_layers) {
	map = p_map;
	can_jump = false;
	for (const Profile &p : profiles) {
		can_jump = can_jump || p.reach(0.0f) > 0.0f;
	}
	scan_n = (int)((can_jump ? SCAN_MAX : NEAR_D + SNAP_H) / SCAN_STEP);
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	if (!capsule.is_valid()) {
		capsule = ps->capsule_shape_create();
		Dictionary d;
		d["radius"] = BODY_R;
		d["height"] = BODY_H;
		ps->shape_set_data(capsule, d);
	}
	query.instantiate();
	query->set_map(map);
	query->set_navigation_layers((uint32_t)p_walk_layers);
	query_result.instantiate();
}

bool EnemyLinkGen::step(Object *p_space, const Transform3D &p_root, int p_budget_usec) {
	if (done) {
		return true;
	}
	space = Object::cast_to<PhysicsDirectSpaceState3D>(p_space);
	ERR_FAIL_NULL_V(space, false);
	root = p_root;
	root_inv = p_root.affine_inverse();
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	while (true) {
		if (scan_k == 0) {
			if (cursor >= queue.size() && !_expand_narrow()) {
				done = true;
				break;
			}
			levels.clear();
			scan_k = 1;
		}
		_scan_step(queue[cursor], scan_k);
		scan_k++;
		if (scan_k > scan_n) {
			scan_k = 0;
			cursor++;
		}
		if (OS::get_singleton()->get_ticks_usec() - t0 >= (uint64_t)p_budget_usec) {
			break;
		}
	}
	const uint64_t spent = OS::get_singleton()->get_ticks_usec() - t0;
	gen_usec += spent;
	max_step_usec = MAX(max_step_usec, spent);
	steps++;
	space = nullptr;
	return done;
}

// Очередь пройдена: точки краёв узких кусков, на которые ведёт принятая связь и которые ещё не обходились.
bool EnemyLinkGen::_expand_narrow() {
	const int n = (int)comp.size();
	LocalVector<uint8_t> only;
	only.resize(n);
	bool any = false;
	for (int i = 0; i < n; i++) {
		const int c = comp[i];
		const bool take = comp_reached[c] && !comp_scanned[c];
		only[i] = take ? 1 : 0;
		if (take) {
			any = true;
			narrow_scanned++;
		}
	}
	if (!any) {
		return false;
	}
	for (int c = 0; c < n; c++) {
		if (comp_reached[c]) {
			comp_scanned[c] = 1;
		}
	}
	_add_samples(narrow, true, &only);
	return cursor < queue.size();
}

// Шаг k вперёд от точки края: на каждый уровень приземления — ближний кандидат, прошедший все проверки (уровень
// занимает и «рядом пешком»: дальше на нём — тоже пешком; задетый телом — нет: дальше может пройти). Уровни точки —
// levels (сброс в step в начале точки).
void EnemyLinkGen::_scan_step(const Sample &p_s, int p_k) {
	Vector3 hit;
	if (!_ground(p_s.p + p_s.n * (p_k * SCAN_STEP), p_s.p.y, hit)) {
		return;
	}
	Vector3 land;
	int poly = -1;
	if (!index.snap(hit, SNAP_V, SNAP_H, land, poly) || _level_taken(land.y)) {
		return;
	}
	const real_t dh = land.y - p_s.p.y;
	const real_t dist = Vector2(land.x - p_s.p.x, land.z - p_s.p.z).length();
	const int kind = _kind(dh, dist);
	if (kind < 0) {
		return;
	}
	const bool end_narrow = poly >= main_polys;
	const uint32_t lay = _layers(kind, dh, dist, p_s.narrow || end_narrow);
	if (lay == 0) {
		return;
	}
	candidates++;
	_trajectory(kind, p_s.p, land, _strongest(lay), traj);
	if (!_clear(traj)) {
		blocked++;
		return;
	}
	levels.push_back(land.y);
	if (_walkable(p_s.p, land)) {
		shortcuts++;
		return;
	}
	if (_repeat(kind, p_s.p, land)) {
		return;
	}
	Link l;
	l.kind = (Kind)kind;
	l.start = p_s.p;
	l.end = land;
	l.layers = lay;
	l.start_narrow = p_s.narrow;
	l.traj = traj;
	links.push_back(l);
	if (end_narrow) {
		comp_reached[comp[poly - main_polys]] = 1;
	}
}

bool EnemyLinkGen::_level_taken(real_t p_y) const {
	for (const real_t l : levels) {
		if (Math::abs(p_y - l) <= STEP_H) {
			return true;
		}
	}
	return false;
}

// Попадание луча вниз через q (от base_y + RAY_UP), координаты корня карты.
bool EnemyLinkGen::_ground(const Vector3 &p_q, real_t p_base_y, Vector3 &r_hit) const {
	PhysicsDirectSpaceState3D::RayParameters rq;
	rq.from = root.xform(Vector3(p_q.x, p_base_y + RAY_UP, p_q.z));
	rq.to = root.xform(Vector3(p_q.x, p_base_y - RAY_DOWN, p_q.z));
	rq.collision_mask = WORLD_MASK;
	PhysicsDirectSpaceState3D::RayResult r;
	if (!space->intersect_ray(rq, r)) {
		return false;
	}
	r_hit = root_inv.xform(r.position);
	return true;
}

int EnemyLinkGen::_kind(real_t p_dh, real_t p_dist) {
	if (Math::abs(p_dh) <= STEP_H) {
		return p_dist > NEAR_D ? KIND_JUMP : -1;
	}
	if (p_dist <= NEAR_D) {
		return p_dh > 0.0f ? KIND_CLIMB : KIND_DROP;
	}
	return KIND_JUMP;
}

uint32_t EnemyLinkGen::_layers(int p_kind, real_t p_dh, real_t p_dist, bool p_touches_narrow) const {
	uint32_t out = 0;
	for (const Profile &p : profiles) {
		if (p.can(p_kind, p_dh, p_dist, p_touches_narrow)) {
			out |= p.bit;
		}
	}
	return out;
}

// Профиль из слоёв с наибольшей дальностью прыжка (его дуга проверяется и рисуется); слои не пусты.
const EnemyLinkGen::Profile &EnemyLinkGen::_strongest(uint32_t p_layers) const {
	const Profile *best = nullptr;
	for (const Profile &p : profiles) {
		if ((p_layers & p.bit) && (best == nullptr || p.reach(0.0f) > best->reach(0.0f))) {
			best = &p;
		}
	}
	return *best;
}

// Траектория ног (корень карты): прыжок — дуга, залезть — вверх и вперёд, спрыгнуть — вперёд над b и вниз.
void EnemyLinkGen::_trajectory(int p_kind, const Vector3 &p_a, const Vector3 &p_b, const Profile &p_prof, LocalVector<Vector3> &r_out) {
	r_out.clear();
	r_out.push_back(p_a);
	switch (p_kind) {
		case KIND_JUMP: {
			const real_t t = MAX(p_prof.land_time(p_b.y - p_a.y), 0.0f);
			for (int i = 1; i <= ARC_STEPS; i++) {
				const real_t f = (real_t)i / ARC_STEPS;
				const real_t s = t * f;
				Vector3 p = p_a.lerp(p_b, f);
				p.y = p_a.y + p_prof.impulse * s - 0.5f * p_prof.gravity * s * s;
				r_out.push_back(p);
			}
			r_out[r_out.size() - 1] = p_b;
		} break;
		case KIND_CLIMB:
			r_out.push_back(Vector3(p_a.x, p_b.y, p_a.z));
			r_out.push_back(p_b);
			break;
		default:
			r_out.push_back(Vector3(p_b.x, p_a.y, p_b.z));
			r_out.push_back(p_b);
			break;
	}
}

// Тело ходока (капсула над ногами на LIFT) проходит траекторию без касаний мира.
bool EnemyLinkGen::_clear(const LocalVector<Vector3> &p_traj) const {
	const Vector3 lift(0.0f, LIFT + BODY_H * 0.5f, 0.0f);
	PhysicsDirectSpaceState3D::ShapeParameters q;
	q.shape_rid = capsule;
	q.collision_mask = WORLD_MASK;
	for (uint32_t i = 1; i < p_traj.size(); i++) {
		q.transform = Transform3D(root.basis, root.xform(p_traj[i - 1] + lift));
		q.motion = root.basis.xform(p_traj[i] - p_traj[i - 1]);
		real_t safe = 1.0f;
		real_t unsafe = 1.0f;
		space->cast_motion(q, safe, unsafe);
		if (safe < 1.0f) {
			return false;
		}
	}
	return true;
}

// Пешком от from до to рядом (связь не нужна): поиск не дальше предела от начала — короче предела путь всё равно
// не уходит дальше него, а через весь корабль не ищется.
bool EnemyLinkGen::_walkable(const Vector3 &p_from, const Vector3 &p_to) {
	const real_t direct = p_from.distance_to(p_to);
	const real_t limit = MAX(SHORTCUT_MUL * direct, direct + SHORTCUT_EXTRA);
	query->set_start_position(p_from);
	query->set_target_position(p_to);
	query->set_path_search_max_distance(limit);
	NavigationServer3D::get_singleton()->query_path(query, query_result);
	const Vector<Vector3> &path = query_result->get_path();
	if (path.is_empty() || path[path.size() - 1].distance_to(p_to) > REACH_TOL) {
		return false;
	}
	real_t walk = 0.0f;
	for (int i = 1; i < path.size(); i++) {
		walk += path[i - 1].distance_to(path[i]);
	}
	return walk <= limit;
}

bool EnemyLinkGen::_repeat(int p_kind, const Vector3 &p_from, const Vector3 &p_to) const {
	const uint32_t n = links.size();
	for (uint32_t i = n > (uint32_t)DEDUP_WINDOW ? n - DEDUP_WINDOW : 0; i < n; i++) {
		const Link &l = links[i];
		if (l.kind == p_kind && l.start.distance_to(p_from) < DEDUP_R && l.end.distance_to(p_to) < DEDUP_R) {
			return true;
		}
	}
	return false;
}

// ---------- Результат ----------

PackedVector3Array EnemyLinkGen::get_starts() const {
	PackedVector3Array out;
	out.resize(links.size());
	for (uint32_t i = 0; i < links.size(); i++) {
		out.set(i, links[i].start);
	}
	return out;
}

PackedVector3Array EnemyLinkGen::get_ends() const {
	PackedVector3Array out;
	out.resize(links.size());
	for (uint32_t i = 0; i < links.size(); i++) {
		out.set(i, links[i].end);
	}
	return out;
}

PackedInt32Array EnemyLinkGen::get_kinds() const {
	PackedInt32Array out;
	out.resize(links.size());
	for (uint32_t i = 0; i < links.size(); i++) {
		out.set(i, links[i].kind);
	}
	return out;
}

PackedInt32Array EnemyLinkGen::get_layers() const {
	PackedInt32Array out;
	out.resize(links.size());
	for (uint32_t i = 0; i < links.size(); i++) {
		out.set(i, (int32_t)links[i].layers);
	}
	return out;
}

PackedInt32Array EnemyLinkGen::get_start_narrow() const {
	PackedInt32Array out;
	out.resize(links.size());
	for (uint32_t i = 0; i < links.size(); i++) {
		out.set(i, links[i].start_narrow ? 1 : 0);
	}
	return out;
}

PackedVector3Array EnemyLinkGen::get_trajectory(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int)links.size(), PackedVector3Array());
	const LocalVector<Vector3> &t = links[p_index].traj;
	PackedVector3Array out;
	out.resize(t.size());
	for (uint32_t i = 0; i < t.size(); i++) {
		out.set(i, t[i]);
	}
	return out;
}

void EnemyLinkGen::_bind_methods() {
	ClassDB::bind_method(D_METHOD("prepare", "full", "main"), &EnemyLinkGen::prepare);
	ClassDB::bind_method(D_METHOD("get_narrow"), &EnemyLinkGen::get_narrow);
	ClassDB::bind_method(D_METHOD("get_merged"), &EnemyLinkGen::get_merged);
	ClassDB::bind_method(D_METHOD("get_probe_point"), &EnemyLinkGen::get_probe_point);
	ClassDB::bind_method(D_METHOD("clear_profiles"), &EnemyLinkGen::clear_profiles);
	ClassDB::bind_method(D_METHOD("add_profile", "layer_bit", "max_drop", "jump_impulse", "jump_speed", "gravity", "climb", "narrow"), &EnemyLinkGen::add_profile);
	ClassDB::bind_method(D_METHOD("start", "map", "walk_layers"), &EnemyLinkGen::start);
	ClassDB::bind_method(D_METHOD("step", "space", "root", "budget_usec"), &EnemyLinkGen::step);
	ClassDB::bind_method(D_METHOD("get_link_count"), &EnemyLinkGen::get_link_count);
	ClassDB::bind_method(D_METHOD("get_starts"), &EnemyLinkGen::get_starts);
	ClassDB::bind_method(D_METHOD("get_ends"), &EnemyLinkGen::get_ends);
	ClassDB::bind_method(D_METHOD("get_kinds"), &EnemyLinkGen::get_kinds);
	ClassDB::bind_method(D_METHOD("get_layers"), &EnemyLinkGen::get_layers);
	ClassDB::bind_method(D_METHOD("get_start_narrow"), &EnemyLinkGen::get_start_narrow);
	ClassDB::bind_method(D_METHOD("get_trajectory", "index"), &EnemyLinkGen::get_trajectory);
	ClassDB::bind_method(D_METHOD("get_samples"), &EnemyLinkGen::get_samples);
	ClassDB::bind_method(D_METHOD("get_candidates"), &EnemyLinkGen::get_candidates);
	ClassDB::bind_method(D_METHOD("get_shortcuts"), &EnemyLinkGen::get_shortcuts);
	ClassDB::bind_method(D_METHOD("get_blocked"), &EnemyLinkGen::get_blocked);
	ClassDB::bind_method(D_METHOD("get_narrow_scanned"), &EnemyLinkGen::get_narrow_scanned);
	ClassDB::bind_method(D_METHOD("get_steps"), &EnemyLinkGen::get_steps);
	ClassDB::bind_method(D_METHOD("get_prepare_usec"), &EnemyLinkGen::get_prepare_usec);
	ClassDB::bind_method(D_METHOD("get_gen_usec"), &EnemyLinkGen::get_gen_usec);
	ClassDB::bind_method(D_METHOD("get_max_step_usec"), &EnemyLinkGen::get_max_step_usec);
}
