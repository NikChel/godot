#pragma once

#include "core/math/basis.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3.h"
#include "core/object/object_id.h"

// Неваляшка врага (spec docs/superpowers/specs/2026-10-26-walker-tumbler-design.md): порт TumblerModel (GDScript,
// spec 2026-10-24-melee-holster-design §5) один в один — перевёрнутый маятник с опорой у ног. Наклон — вектор поворота в
// горизонтальной плоскости кадра основания (ось × угол, Y — вверх), угловая скорость — такой же вектор. Физика идёт
// всегда: импульс j в точке r от опоры — Δω = (r × j) / I (кручение у заваленной — поворот наклона вокруг опоры); пол —
// наклон не больше down_angle. Состояние выбирает силу:
//   STAND   — пружина и демпфер к вертикали; наклон больше fall_angle — FALLING;
//   FALLING — тяжесть (стержень высоты height); лёг на пол — DOWN; отброшен назад ниже fall_angle — STAND;
//   DOWN    — тяжесть, лежит; пролежал down_time — RISING;
//   RISING  — подъём к вертикали без перелёта против тяжести; сбит до пола — DOWN; почти вертикально — STAND.
// Без падения (can_fall = false, рука ходока — spec 2026-10-27-walker-arms-design): всегда STAND, fall_angle — предел
// наклона (ход наружу на пределе гасится).
// Плюс оглушение врага (staggered) и хит-тело: поза основание × наклон — в узел и сразу в PhysicsServer (видна
// запросам того же тика). Общая для тел хоста (EnemyBodies) и копий клиента (EnemyReplicas).
struct EnemyTumbler {
	enum State {
		STAND,
		FALLING,
		DOWN,
		RISING,
	};

	static constexpr real_t G = 9.8f;
	static constexpr real_t UP_EPS = 1e-6f; // Наклон меньше — стоит ровно, рад
	static constexpr real_t STAND_ANGLE = 0.05235988f; // 3°: RISING ближе к вертикали — STAND
	static constexpr real_t FLOOR_EPS = 0.00872665f; // 0.5°: на полу — наклон не меньше down_angle − этого
	static constexpr real_t FLOOR_FRICTION = 4.0f; // Затухание хода вдоль пола, 1/с
	static constexpr real_t RISE_SETTLE = 4.7f; // Критическое затухание: до 5% за rise_time
	static constexpr real_t YAW_MIN_SIN2 = 0.25f; // sin²(30°): ниже — кручение почти не поворачивает наклон

	// Настройки (WalkerTumble)
	real_t inertia = 2.0f; // Момент инерции относительно опоры, кг·м²
	real_t frequency = 1.0f; // Частота качания, Гц
	real_t damping = 0.25f; // Затухание (доля критического)
	real_t fall_angle = 0.5235988f; // 30°
	real_t down_angle = 1.3089969f; // 75°
	real_t height = 1.8f; // Высота стержня падения, м
	real_t down_time = 2.0f; // Лежит на полу, с
	real_t rise_time = 1.0f; // Встаёт, с
	real_t stagger_angle = 0.17453293f; // 10°: оглушён, пока наклон больше (или не STAND)
	real_t knockback = 0.15f; // Отброс: Δv основания = горизонталь импульса × это, м/с на Н·с
	bool can_fall = true; // Нет — не падает, fall_angle — предел наклона

	// Состояние
	uint8_t state = STAND;
	Vector3 tilt; // Наклон: ось × угол (кадр основания), рад
	Vector3 spin; // Угловая скорость (кадр основания), рад/с
	ObjectID hit_id; // Хит-тело (CollisionObject3D), нет — без хит-формы

	void reset();
	void push(const Vector3 &p_r, const Vector3 &p_j);
	bool step(real_t p_dt); // true — сменилось состояние
	Basis basis() const;
	Vector3 point_velocity(const Vector3 &p_r) const { return spin.cross(p_r); }
	bool staggered() const { return state != STAND || tilt.length() > stagger_angle; }
	void place(const Transform3D &p_base) const; // Хит-тело в позу p_base × наклон (мир)
	void log_state() const; // Строка лога смены состояния (имя врага — родитель хит-тела)

private:
	real_t t = 0.0f; // Время на полу в DOWN, с
	bool impact = false; // Этот шаг — удар о пол

	real_t _gravity(real_t p_a) const;
	bool _floor(real_t p_dt);
	void _lie();
	void _limit();
};
