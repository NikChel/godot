#include "enemy_bodies.h"

#include "core/math/math_funcs.h"

// Облёт частей подвижных систем снаружи (план 2026-11-20-cabin-character, срез 4; заголовок EnemyBodies — «Облёт
// подвижных преград снаружи»): курс летящего в открытом пространстве мимо сфер точек приоритета с пространством-владельцем.

namespace {
// Сфера преграды, видимая из тела: направление на центр, порог задевания и проекции направления на ось курса и базис.
struct DetourSphere {
	Vector3 v; // На центр сферы от тела, единичный
	real_t kappa = 0.0f; // Направление r задевает сферу, если r·v > kappa
	real_t a = 0.0f; // v · ось «тело → цель»
	real_t x = 0.0f; // v · e1 (к верху тела)
	real_t y = 0.0f; // v · e2
};

// Отрезок от p_from по единичному p_dir длиной p_len подходит к точке p_s ближе p_q.
bool seg_near(const Vector3 &p_from, const Vector3 &p_dir, real_t p_len, const Vector3 &p_s, real_t p_q) {
	const Vector3 v = p_s - p_from;
	const real_t t = CLAMP(v.dot(p_dir), 0.0f, p_len);
	return (v - p_dir * t).length_squared() < p_q * p_q;
}
} // namespace

int EnemyBodies::body_detour(int p_h) const {
	ERR_FAIL_COND_V(!_valid(p_h), -1);
	return bodies[p_h].detour;
}

// Курс тела r_b с центром p_center к точке r_aim (мир) мимо сфер преград (шапка): r_aim — новая точка курса; true —
// точка сменилась (облёт или цель вынесена из сферы).
bool EnemyBodies::_detour(Body &r_b, const Vector3 &p_center, Vector3 &r_aim) const {
	if (prio_spaces.is_empty() || prio_spaces.size() != prio_points.size()) {
		r_b.detour = -1;
		return false;
	}
	const Vector3 up = r_b.up;
	const real_t touch = r_b.bo_radius + r_b.bo_gap + DETOUR_MARGIN;
	Vector3 sc[DETOUR_MAX];
	real_t sr[DETOUR_MAX];
	int n = 0;
	for (int i = 0; i < prio_points.size() && n < DETOUR_MAX; i++) {
		const int s = prio_spaces[i];
		if (s < 0 || s == r_b.space) {
			continue; // Не часть подвижной системы (ноги игрока) или тело в её пространстве
		}
		const Vector4 &p = prio_points[i];
		const Vector3 c = Vector3(p.x, p.y, p.z) + up * CENTER_H;
		const real_t r = p.w + touch;
		if (c.distance_to(p_center) - r > DETOUR_LOOK) {
			continue;
		}
		sc[n] = c;
		sr[n] = r;
		n++;
	}
	if (n == 0) {
		r_b.detour = -1;
		return false;
	}
	// Цель внутри сферы — на её поверхность (два прохода: сферы рядом)
	Vector3 g = r_aim;
	bool moved = false;
	for (int pass = 0; pass < 2; pass++) {
		for (int j = 0; j < n; j++) {
			const Vector3 d = g - sc[j];
			const real_t l = d.length();
			if (l >= sr[j]) {
				continue;
			}
			Vector3 dir = l > 1e-3f ? d / l : p_center - sc[j];
			if (l <= 1e-3f) {
				dir = dir.length_squared() > 1e-8f ? dir.normalized() : up;
			}
			g = sc[j] + dir * sr[j];
			moved = true;
		}
	}
	const Vector3 to = g - p_center;
	const real_t len = to.length();
	if (len < 1e-3f) {
		r_b.detour = -1;
		r_aim = g;
		return moved;
	}
	const Vector3 a = to / len;
	// Сферы на пути: ход не ближе q = min(радиус, расстояние − DETOUR_TOL) к центру (в сфере — не глубже, чем стоит)
	DetourSphere ds[DETOUR_MAX];
	int m = 0;
	bool blocked = false;
	for (int j = 0; j < n; j++) {
		const Vector3 v = sc[j] - p_center;
		const real_t d = v.length();
		const real_t q = MIN(sr[j], d - DETOUR_TOL);
		if (q <= 0.0f || d - q >= len) {
			continue; // Тело почти в центре (уступка) или сфера за целью
		}
		blocked = blocked || seg_near(p_center, a, len, sc[j], q);
		DetourSphere &e = ds[m++];
		e.v = v / d;
		e.kappa = Math::sqrt(MAX(1.0f - (q * q) / (d * d), 0.0f));
	}
	if (!blocked) {
		r_b.detour = -1;
		r_aim = g;
		return moved;
	}
	// Полуплоскости вокруг a: u_k = cos φ_k e1 + sin φ_k e2, e1 — к верху тела поперёк a
	Vector3 e1 = up - a * up.dot(a);
	if (e1.length_squared() < 1e-4f) {
		e1 = a.cross(Vector3(1.0f, 0.0f, 0.0f));
		if (e1.length_squared() < 1e-4f) {
			e1 = a.cross(Vector3(0.0f, 0.0f, 1.0f));
		}
	}
	e1.normalize();
	const Vector3 e2 = a.cross(e1);
	for (int j = 0; j < m; j++) {
		ds[j].a = ds[j].v.dot(a);
		ds[j].x = ds[j].v.dot(e1);
		ds[j].y = ds[j].v.dot(e2);
	}
	real_t theta[DETOUR_PLANES];
	real_t lo[DETOUR_MAX * 3];
	real_t hi[DETOUR_MAX * 3];
	real_t best = (real_t)Math::INF;
	int best_k = -1;
	const int k0 = r_b.slot % DETOUR_PLANES; // Равные — с полуплоскости по слоту тела: толпа расходится в обе стороны
	for (int kk = 0; kk < DETOUR_PLANES; kk++) {
		const int k = (k0 + kk) % DETOUR_PLANES;
		const real_t ph = (real_t)Math::TAU * (real_t)k / (real_t)DETOUR_PLANES;
		const real_t cp = Math::cos(ph);
		const real_t sp = Math::sin(ph);
		// Луч cos θ a + sin θ u задевает сферу j при M cos(θ − φ0) > kappa: θ ∈ (φ0 − δ, φ0 + δ) (и со сдвигом 2π)
		int c = 0;
		for (int j = 0; j < m; j++) {
			const DetourSphere &e = ds[j];
			const real_t b = cp * e.x + sp * e.y;
			const real_t mm = Math::sqrt(e.a * e.a + b * b);
			if (mm <= e.kappa) {
				continue;
			}
			const real_t f0 = Math::atan2(b, e.a);
			const real_t dl = Math::acos(CLAMP(e.kappa / mm, -1.0f, 1.0f));
			for (int w = -1; w <= 1; w++) {
				const real_t l = f0 - dl + (real_t)Math::TAU * (real_t)w;
				const real_t h = f0 + dl + (real_t)Math::TAU * (real_t)w;
				if (h > 0.0f && l < (real_t)Math::PI) {
					int i = c++;
					while (i > 0 && lo[i - 1] > l) { // По началу
						lo[i] = lo[i - 1];
						hi[i] = hi[i - 1];
						i--;
					}
					lo[i] = l;
					hi[i] = h;
				}
			}
		}
		real_t t = 0.0f; // Наименьший θ вне интервалов
		for (int i = 0; i < c; i++) {
			if (lo[i] > t) {
				break;
			}
			if (hi[i] > t) {
				t = hi[i] + 1e-3f;
			}
		}
		theta[k] = t <= (real_t)Math::PI ? t : (real_t)Math::INF;
		if (theta[k] < best) {
			best = theta[k];
			best_k = k;
		}
	}
	if (best_k < 0) {
		r_b.detour = -1; // Свободного направления нет — курс прежний (уступка не пускает внутрь)
		r_aim = g;
		return moved;
	}
	if (r_b.detour >= 0 && r_b.detour < DETOUR_PLANES && theta[r_b.detour] <= best + DETOUR_KEEP) {
		best_k = r_b.detour;
	}
	const real_t ph = (real_t)Math::TAU * (real_t)best_k / (real_t)DETOUR_PLANES;
	const Vector3 u = e1 * Math::cos(ph) + e2 * Math::sin(ph);
	const real_t th = theta[best_k];
	r_b.detour = (int8_t)best_k;
	r_aim = p_center + (a * Math::cos(th) + u * Math::sin(th)) * len;
	return true;
}
