#include "nav_column_index.h"

#include "core/math/geometry_2d.h"

#include <limits>

Vector2i NavColumnIndex::cell_of(real_t p_x, real_t p_z) {
	return Vector2i((int)Math::floor(p_x / CELL), (int)Math::floor(p_z / CELL));
}

void NavColumnIndex::clear() {
	verts.clear();
	corners.clear();
	polys.clear();
	cells.clear();
}

void NavColumnIndex::build(const Ref<NavigationMesh> &p_mesh) {
	clear();
	ERR_FAIL_COND(p_mesh.is_null());
	const Vector<Vector3> src = p_mesh->get_vertices();
	verts.resize(src.size());
	for (int i = 0; i < src.size(); i++) {
		verts[i] = src[i];
	}
	const int count = p_mesh->get_polygon_count();
	polys.resize(count);
	for (int i = 0; i < count; i++) {
		const Vector<int> ids = p_mesh->get_polygon(i);
		Poly &pg = polys[i];
		pg.first = corners.size();
		pg.count = ids.size();
		for (int k = 0; k < ids.size(); k++) {
			corners.push_back((uint32_t)ids[k]);
		}
		if (ids.size() < 3) {
			continue;
		}
		pg.plane = Plane(verts[ids[0]], verts[ids[1]], verts[ids[2]]);
		pg.min_x = std::numeric_limits<real_t>::max();
		pg.min_z = std::numeric_limits<real_t>::max();
		pg.max_x = -std::numeric_limits<real_t>::max();
		pg.max_z = -std::numeric_limits<real_t>::max();
		for (int k = 0; k < ids.size(); k++) {
			const Vector3 &v = verts[ids[k]];
			pg.min_x = MIN(pg.min_x, v.x);
			pg.min_z = MIN(pg.min_z, v.z);
			pg.max_x = MAX(pg.max_x, v.x);
			pg.max_z = MAX(pg.max_z, v.z);
		}
		if (Math::abs(pg.plane.normal.y) < MIN_NY) {
			continue;
		}
		pg.indexed = true;
		const Vector2i lo = cell_of(pg.min_x, pg.min_z);
		const Vector2i hi = cell_of(pg.max_x, pg.max_z);
		for (int cx = lo.x; cx <= hi.x; cx++) {
			for (int cz = lo.y; cz <= hi.y; cz++) {
				cells[Vector2i(cx, cz)].push_back((uint32_t)i);
			}
		}
	}
}

bool NavColumnIndex::covers(const Poly &p_poly, real_t p_x, real_t p_z) const {
	if (p_x < p_poly.min_x || p_x > p_poly.max_x || p_z < p_poly.min_z || p_z > p_poly.max_z) {
		return false;
	}
	const Vector3 *a = &verts[corners[p_poly.first + p_poly.count - 1]];
	real_t side = 0.0f;
	for (uint32_t k = 0; k < p_poly.count; k++) {
		const Vector3 &b = verts[corners[p_poly.first + k]];
		const real_t cr = (b.x - a->x) * (p_z - a->z) - (b.z - a->z) * (p_x - a->x);
		if (cr > EDGE_EPS) {
			if (side < 0.0f) {
				return false;
			}
			side = 1.0f;
		} else if (cr < -EDGE_EPS) {
			if (side > 0.0f) {
				return false;
			}
			side = -1.0f;
		}
		a = &b;
	}
	return true;
}

real_t NavColumnIndex::height(const Poly &p_poly, real_t p_x, real_t p_z) {
	const Plane &pl = p_poly.plane;
	return (pl.d - pl.normal.x * p_x - pl.normal.z * p_z) / pl.normal.y;
}

bool NavColumnIndex::find(const Vector3 &p_point, real_t p_half_h, int p_hint, Vector3 &r_point, int &r_poly) const {
	r_poly = -1;
	if (p_hint >= 0 && p_hint < (int)polys.size()) {
		const Poly &hp = polys[p_hint];
		if (hp.indexed && covers(hp, p_point.x, p_point.z)) {
			const real_t hy = height(hp, p_point.x, p_point.z);
			if (Math::abs(hy - p_point.y) <= p_half_h) {
				r_poly = p_hint;
				r_point = Vector3(p_point.x, hy, p_point.z);
				return true;
			}
		}
	}
	const Vector2i c = cell_of(p_point.x, p_point.z);
	const LocalVector<uint32_t> *list = cells.getptr(c);
	if (list) {
		real_t best = p_half_h;
		for (const uint32_t i : *list) {
			const Poly &pg = polys[i];
			if (!covers(pg, p_point.x, p_point.z)) {
				continue;
			}
			const real_t y = height(pg, p_point.x, p_point.z);
			const real_t dy = Math::abs(y - p_point.y);
			if (dy <= best) {
				best = dy;
				r_poly = (int)i;
				r_point = Vector3(p_point.x, y, p_point.z);
			}
		}
		if (r_poly >= 0) {
			return true;
		}
	}
	return nearest_edge(p_point, p_half_h, c, r_point, r_poly);
}

bool NavColumnIndex::nearest_edge(const Vector3 &p_point, real_t p_half_h, const Vector2i &p_cell, Vector3 &r_point, int &r_poly) const {
	const Vector2 q(p_point.x, p_point.z);
	real_t best = std::numeric_limits<real_t>::max();
	for (int dx = -1; dx <= 1; dx++) {
		for (int dz = -1; dz <= 1; dz++) {
			const LocalVector<uint32_t> *list = cells.getptr(Vector2i(p_cell.x + dx, p_cell.y + dz));
			if (!list) {
				continue;
			}
			for (const uint32_t i : *list) {
				const Poly &pg = polys[i];
				const Vector3 *a = &verts[corners[pg.first + pg.count - 1]];
				for (uint32_t k = 0; k < pg.count; k++) {
					const Vector3 &b = verts[corners[pg.first + k]];
					const Vector2 e = Geometry2D::get_closest_point_to_segment(q, Vector2(a->x, a->z), Vector2(b.x, b.z));
					const real_t d = e.distance_squared_to(q);
					if (d < best) {
						const real_t y = height(pg, e.x, e.y);
						if (Math::abs(y - p_point.y) <= p_half_h) {
							best = d;
							r_poly = (int)i;
							r_point = Vector3(e.x, y, e.y);
						}
					}
					a = &b;
				}
			}
		}
	}
	return r_poly >= 0;
}

bool NavColumnIndex::snap(const Vector3 &p_point, real_t p_half_h, real_t p_max_xz, Vector3 &r_point, int &r_poly) const {
	r_poly = -1;
	const LocalVector<uint32_t> *list = cells.getptr(cell_of(p_point.x, p_point.z));
	if (list) {
		real_t best_dy = p_half_h;
		for (const uint32_t i : *list) {
			const Poly &pg = polys[i];
			if (!covers(pg, p_point.x, p_point.z)) {
				continue;
			}
			const real_t y = height(pg, p_point.x, p_point.z);
			const real_t dy = Math::abs(y - p_point.y);
			if (dy <= best_dy) {
				best_dy = dy;
				r_poly = (int)i;
				r_point = Vector3(p_point.x, y, p_point.z);
			}
		}
		if (r_poly >= 0) {
			return true;
		}
	}
	if (p_max_xz <= 0.0f) {
		return false;
	}
	const Vector2 q(p_point.x, p_point.z);
	real_t best_d2 = p_max_xz * p_max_xz;
	const Vector2i lo = cell_of(p_point.x - p_max_xz, p_point.z - p_max_xz);
	const Vector2i hi = cell_of(p_point.x + p_max_xz, p_point.z + p_max_xz);
	for (int cx = lo.x; cx <= hi.x; cx++) {
		for (int cz = lo.y; cz <= hi.y; cz++) {
			const LocalVector<uint32_t> *bucket = cells.getptr(Vector2i(cx, cz));
			if (!bucket) {
				continue;
			}
			for (const uint32_t i : *bucket) {
				const Poly &pg = polys[i];
				if (q.x < pg.min_x - p_max_xz || q.x > pg.max_x + p_max_xz || q.y < pg.min_z - p_max_xz || q.y > pg.max_z + p_max_xz) {
					continue;
				}
				const Vector3 *a = &verts[corners[pg.first + pg.count - 1]];
				for (uint32_t k = 0; k < pg.count; k++) {
					const Vector3 &b = verts[corners[pg.first + k]];
					const Vector2 e = Geometry2D::get_closest_point_to_segment(q, Vector2(a->x, a->z), Vector2(b.x, b.z));
					const real_t d2 = e.distance_squared_to(q);
					if (d2 <= best_d2) {
						const real_t y = height(pg, e.x, e.y);
						if (Math::abs(y - p_point.y) <= p_half_h) {
							best_d2 = d2;
							r_poly = (int)i;
							r_point = Vector3(e.x, y, e.y);
						}
					}
					a = &b;
				}
			}
		}
	}
	return r_poly >= 0;
}
