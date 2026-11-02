#pragma once

#include "core/math/vector3.h"
#include "core/object/object_id.h"
#include "core/variant/variant.h"

// Сон хит-форм врага (spec docs/superpowers/specs/2026-10-27-walker-arms-design.md §5): хит-тела (торс и руки) нужны,
// только когда в них может что-то попасть. Точки интереса тика (set_wake_points: игроки, свои снаряды в полёте — центр
// и радиус, мир) ближе своего радиуса к точке врага (над ногами на CENTER_H по верху) или неваляшка тела / рук не в
// покое — AWAKE: формы включены, хит-тела ставятся в позу тика. Ни того ни другого — LINGER (LINGER_S), затем SLEEP:
// формы выключены (PhysicsServer: выключенная форма уходит из запросов и из шага физики), позы не ставятся. Проснулся —
// поза ставится в том же тике: она — чистая функция состояния тика (анимация, наклон), истории не нужно.
// Общий у тел хоста (EnemyBodies) и копий клиента (EnemyReplicas).
struct EnemyWake {
	enum State {
		AWAKE,
		LINGER,
		SLEEP,
	};

	static constexpr real_t LINGER_S = 0.3f;
	static constexpr real_t CENTER_H = 1.0f; // м

	uint8_t state = AWAKE;
	real_t linger = LINGER_S; // Осталось до сна в LINGER, с

	// Тик: p_interest — точка интереса рядом, p_moving — неваляшка не в покое; смена сна включает / выключает формы хит-тел
	// p_ids (выключенные в сцене — остаются выключенными). true — хит-тела ставятся в позу.
	bool update(bool p_interest, bool p_moving, real_t p_dt, const ObjectID *p_ids, int p_count);
	static bool interest(const Vector3 &p_at, const PackedVector4Array &p_points);

private:
	static void _set_shapes(const ObjectID *p_ids, int p_count, bool p_enabled);
};
