#pragma once

#include "core/math/math_funcs.h"
#include "core/math/vector3.h"
#include "core/templates/local_vector.h"
#include "core/variant/variant.h"

// Задержка показа точки по правилу предметов (ItemViewTime.lag_at / zone_gap; AGENTS §3.4) — одна формула на показ
// снарядов (ProjectileViews) и таз кукол у следующего (RagdollSync). Входы — ViewLagInputs (GDScript,
// ItemSyncWorldV2.view_lag_inputs): на игрока 4 точки — тело, тело −0.5 м по Y, цель левой и правой руки (нет — INF), его
// задержка показа, с; local — номер своего (−1 — нет: задержка 0). Своя копия входов: вызывающий переписывает свои
// массивы каждый кадр (тик), общий буфер копировался бы при каждой его записи. Правка формулы — вместе с
// ItemViewTime.lag_at (проба net_shot, фаза lag).
struct ViewLag {
	static constexpr double STEAL_BODY_R = 0.70; // = ItemViewTime.STEAL_BODY_R
	static constexpr double STEAL_HAND_R = 0.30; // = ItemViewTime.STEAL_HAND_R

	LocalVector<Vector3> points; // На игрока 4: тело, тело −0.5 м по Y, цель левой руки, цель правой (нет — INF)
	LocalVector<double> lags; // Задержка показа игрока, с (свой — 0)
	int local = -1; // Номер своего, −1 — нет (задержка 0)

	void set(const PackedVector3Array &p_points, const PackedFloat64Array &p_lags, int p_local) {
		points.resize((uint32_t)p_points.size());
		for (int i = 0; i < p_points.size(); i++) {
			points[i] = p_points[i];
		}
		lags.resize((uint32_t)p_lags.size());
		for (int i = 0; i < p_lags.size(); i++) {
			lags[i] = p_lags[i];
		}
		local = p_local;
	}

	// Расстояние от точки до зоны касания игрока, м (0 — в зоне) — ItemViewTime.zone_gap.
	double gap(int p_player, const Vector3 &p_at) const {
		const Vector3 *pts = points.ptr() + p_player * 4;
		double g = MIN((double)pts[0].distance_to(p_at), (double)pts[1].distance_to(p_at)) - STEAL_BODY_R;
		g = MIN(g, (double)pts[2].distance_to(p_at) - STEAL_HAND_R);
		g = MIN(g, (double)pts[3].distance_to(p_at) - STEAL_HAND_R);
		return MAX(g, 0.0);
	}

	// Задержка показа точки, с — ItemViewTime.lag_at: у своих ног 0, у чужих — время его аватара, по дороге —
	// пропорционально расстояниям до зон касания.
	double lag_at(const Vector3 &p_at) const {
		const int n = (int)lags.size();
		if (local < 0 || local >= n || (int)points.size() < n * 4) {
			return 0.0;
		}
		const double g_me = gap(local, p_at);
		double g_them = Math::INF;
		double lag_them = 0.0;
		for (int i = 0; i < n; i++) {
			if (i == local) {
				continue;
			}
			const double g = gap(i, p_at);
			if (g < g_them) {
				g_them = g;
				lag_them = lags[i];
			}
		}
		if (g_them < Math::INF && g_me + g_them > 0.0) { // Оба в зоне — у своих ног (настоящее)
			return lag_them * g_me / (g_me + g_them);
		}
		return 0.0;
	}
};
