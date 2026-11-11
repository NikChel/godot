#pragma once

#include "core/math/math_funcs.h"
#include "core/math/vector3.h"
#include "core/math/vector4.h"
#include "core/variant/variant.h"

// Правило уступки игрокам (AGENTS §3.8, решение пользователя 2026-11-11) — одно для тел хоста (EnemyWorld: агенты толпы)
// и копий клиента (EnemyReplicas: местная уступка своему игроку). Точки — xyz ноги в системе расчёта, w — радиус касания
// точки; числа агента — радиус касания, зазор, скорость на метр глубины, предел скорости. Ближе касания + зазора (радиус
// агента + радиус точки + gap; по высоте ±H) — скорость уступки от точки min(gain · глубина, speed), сумма — не длиннее
// speed; свой ход агента к такой точке снимается.
struct EnemyBackoff {
	// По высоте дальше — точка не касается (этаж выше / ниже)
	static constexpr real_t H = 0.9f;
	// Глубже — теснит (призрак у тел хоста); мельче скорость уступки ниже покоя тела (EnemyBodies::REST_SPEED)
	static constexpr real_t PRESS_EPS = 0.005f;

	real_t radius = 0.32f; // Радиус касания агента (сфера ног), м
	real_t gap = 0.1f; // Зазор: ближе касания + зазора — уступает, м
	real_t gain = 30.0f; // Скорость уступки на метр глубины, 1/с
	real_t speed = 3.0f; // Предел скорости уступки (не быстрее своего шага), м/с

	// Скорость уступки агента в p_pos (XZ системы). r_pressed — точка глубже PRESS_EPS (и на её оси: направления нет —
	// скорости нет).
	Vector3 velocity(const Vector3 &p_pos, const PackedVector4Array &p_points, bool &r_pressed) const {
		r_pressed = false;
		Vector3 out;
		for (int i = 0; i < p_points.size(); i++) {
			const Vector4 &p = p_points[i];
			if (Math::abs(p.y - p_pos.y) > H) {
				continue;
			}
			const real_t r = radius + p.w + gap;
			const real_t dx = p_pos.x - p.x;
			const real_t dz = p_pos.z - p.z;
			const real_t l2 = dx * dx + dz * dz;
			if (l2 >= r * r) {
				continue;
			}
			const real_t l = Math::sqrt(l2);
			r_pressed = r_pressed || r - l > PRESS_EPS;
			if (l < 1e-4f) {
				continue;
			}
			out += Vector3(dx, 0.0f, dz) * (MIN(gain * (r - l), speed) / l);
		}
		return out.limit_length(speed);
	}

	// Свой ход p_w агента в p_pos к точкам, которые его теснят (ближе касания + зазора), снят: игрок главнее. Ход —
	// скорость или шаг (снимается проекция).
	Vector3 clip_toward(const Vector3 &p_pos, const Vector3 &p_w, const PackedVector4Array &p_points) const {
		Vector3 w = p_w;
		for (int i = 0; i < p_points.size(); i++) {
			const Vector4 &p = p_points[i];
			if (Math::abs(p.y - p_pos.y) > H) {
				continue;
			}
			const real_t r = radius + p.w + gap;
			const Vector3 d(p.x - p_pos.x, 0.0f, p.z - p_pos.z);
			const real_t l2 = d.length_squared();
			if (l2 >= r * r || l2 < 1e-8f) {
				continue;
			}
			const real_t toward = w.dot(d);
			if (toward > 0.0f) {
				w -= d * (toward / l2);
			}
		}
		return w;
	}
};
