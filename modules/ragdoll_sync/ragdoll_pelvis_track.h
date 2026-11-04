#pragma once

#include "core/math/transform_3d.h"
#include "core/templates/local_vector.h"
#include "core/variant/callable.h"

class RigidBody3D;

// Позы пространств одного вызова (NavWorld.space_xform через Callable карты): каждое пространство спрашивается раз за
// вызов — в пределах тика поза системы не меняется.
struct RagdollSpaceXforms {
	Callable fn; // space_xf(space: int) -> Transform3D
	LocalVector<Transform3D> xf;
	LocalVector<uint8_t> have;

	void clear();
	Transform3D get(int p_space);
};

// Таз куклы у следующего (spec docs/superpowers/specs/2026-11-02-walker-ragdoll-design.md §2.7, §5 этап 5) — перенос
// RagdollPelvisFollow (GDScript) один в один: кинематическое тело по буферу поз таза владельца, физ-тик, без пружины.
// Момент показа — сетевое время тика плюс задержка до владельца минус задержка показа lag. Цель lag — правило предметов
// (ViewLag у мира), но не меньше задержки буфера (one_way + buffer_s): за последней точкой таз не досчитывается — тело с
// касаниями (AGENTS §2.2 п.3), стоит в последней точке. lag идёт к цели не быстрее rate_up / rate_down
// (ItemFlightPredictor). Точки — в системе пространства; другое пространство — буфер заново. Пока момент показа не дошёл
// до первой точки — ЖДУ (таз — своя физика); дошёл — СЛЕДУЮ (заморозка — кинематика) с текущей позы: разница с буфером
// тает за catchup_s (smoothstep), рывка нет. Неподвижный таз не трогается (is_equal_approx). Финальная поза владельца
// показана — ФИНАЛ (resting): таз стоит в ней; кукла не уснула за rest_s — tick просит усыпить её; новая точка — снова
// СЛЕДУЮ. Арифметика — как у GDScript: время и доли — double, позы — real_t.
struct RagdollPelvisTrack {
	struct Params {
		double buffer_s = 0.06; // ObjectStreamBufferV2.LATENCY_MS / 1000
		double keep_s = 0.6; // ObjectStreamBufferV2.BUF_MAX_MS / 1000
		double catchup_s = 0.3;
		double rest_s = 0.5;
		double rate_up = 1.0; // ItemFlightPredictor.RATE_UP
		double rate_down = 0.5; // ItemFlightPredictor.RATE_DOWN
	};

	bool following = false; // Таз кинематический по буферу
	bool resting = false; // Финальная поза владельца показана (до новой точки)
	double lag = -1.0; // Задержка показа, с (−1 — ещё не задана)

	LocalVector<double> t; // Время точек на нашей шкале (NetClockOffset.now), по возрастанию
	LocalVector<Transform3D> x; // Позы таза в системе пространства space
	int space = -1;
	Vector3 catch_p; // Разница своей позы и буфера в начале следования (мир)
	bool catch_pending = false; // Разница ещё не снята (у GDScript — catch_p = Vector3.INF)
	Quaternion catch_q;
	double catch_t = -1.0; // Время таяния разницы (−1 — не тает)
	Transform3D last; // Последняя поза, данная тазу
	Transform3D prev;
	bool driven = false;
	double final_t = -1.0; // Время финальной позы (последняя точка буфера), −1 — поток идёт
	int rest_ticks = 0;
	bool fed = false; // Пришла точка после прошлого тика

	// Точка потока владельца: время на нашей шкале, поза в системе пространства; final — уснула.
	void feed(double p_t, int p_space, const Transform3D &p_xf, bool p_final, const Params &p_params);
	// Буфер заново, таз — своя физика (тело не трогается).
	void reset();
	// Тику нечего делать: точек нет или кукла спит в финальной позе на корабле (пространство 0), новых точек нет.
	bool idle(bool p_asleep) const;
	// Физ-тик: true — финальная поза держится rest_s, а кукла не спит: усыпить.
	bool tick(RigidBody3D *p_pelvis, double p_now, double p_one_way, double p_target, RagdollSpaceXforms &p_spaces,
			bool p_asleep, int p_tps, const Params &p_params);
	// Скорость таза по двум последним шагам кинематики (м/с, рад/с, мир).
	Vector3 velocity(int p_tps) const;
	Vector3 angular_velocity(int p_tps) const;

	// Поза буфера на момент seek: между точками — интерполяция, за краями — крайняя точка.
	Transform3D sample(double p_seek) const;
	// SyncMath.quat_slerp_safe.
	static Basis slerp_safe(const Basis &p_from, const Basis &p_to, double p_factor);
};
