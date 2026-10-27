#pragma once

#include "core/math/math_defs.h"

class EnemyRig;

// Часы анимации врага (spec docs/superpowers/specs/2026-10-27-walker-arms-design.md §2): клип, время и смешивание —
// состояние физ-тика (его читает физика — хит-формы рук, EnemyArms; кадр показывает его — EnemyViews; AGENTS §2.2).
// Правила — бывшие EnemyViews (этап 3): стойка / шаг по скорости вдоль торса (гистерезис WALK_ON / WALK_OFF, темп
// шага — как AvatarAnimator игрока), разовый клип (request — заказ вне тика, включает шаг) — до конца со скоростью 1,
// затем снова по скорости; смена — смешивание с прошлым клипом (он идёт дальше, его вес линейно падает до нуля). Смена
// — только в шаге, после хода времени, с весом 0: кадр (прошлый тик + доля) непрерывен и при доле 1 равен тику. Клипы — номера EnemyRig: 0 —
// стойка, 1 — шаг, дальше разовые. Общая для тел хоста (EnemyBodies) и копий клиента (EnemyReplicas).
struct EnemyAnim {
	static constexpr int CLIP_STAND = 0;
	static constexpr int CLIP_WALK = 1;
	static constexpr real_t WALK_ON = 0.4f; // м/с
	static constexpr real_t WALK_OFF = 0.2f;
	static constexpr real_t CLIP_BLEND = 0.2f; // с
	static constexpr real_t ACTION_BLEND = 0.1f; // с
	// Темп шага — как AvatarAnimator игрока (blend_speed, run_speed, run_rate, max_rate)
	static constexpr real_t BLEND_SPEED = 0.5f;
	static constexpr real_t RUN_SPEED = 4.5f;
	static constexpr real_t RUN_RATE = 2.0f;
	static constexpr real_t MAX_RATE = 2.5f;

	int clip = CLIP_STAND;
	real_t time = 0.0f; // Время клипа, с
	real_t rate = 1.0f; // Темп (шаг назад — меньше нуля)
	int from = -1; // Уходящий клип (−1 — смешивания нет)
	real_t from_time = 0.0f;
	real_t from_rate = 1.0f;
	real_t blend = 0.0f; // Прошло смешивания, с
	real_t blend_len = 0.0f;
	bool acting = false; // Идёт разовый клип
	int pending = -1; // Заказанный разовый клип (включит следующий шаг)

	void request(int p_clip) { pending = p_clip; }
	bool busy() const { return acting || pending >= 0; } // Разовый клип идёт или заказан
	void play(int p_clip, real_t p_blend, bool p_action);
	void step(const EnemyRig &p_rig, real_t p_dt, real_t p_speed); // Тик: время и правила
	EnemyAnim advanced(real_t p_dt) const; // Время вперёд без правил (кадр — доля тика; чистая функция)
	real_t weight() const; // Вес текущего клипа (уходящего — 1 − это)
	static real_t walk_rate(real_t p_speed);
};
