#pragma once

#include "core/math/color.h"
#include "core/math/math_defs.h"
#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"
#include "projectile_swarm.h"

// Показ снарядов карты (spec docs/superpowers/specs/2026-11-02-projectile-swarm-cpp-design.md §4): один проход кадра
// рендера на все снаряды роя (AGENTS §2.9), перенос прежнего ProjectileView (GDScript) один в один. Возраст показа:
// свой — между двумя последними тиками ((frames − born_tick − 1 + alpha) / tps), чужой — «настоящее стрелка» минус
// задержка показа точки (lag_at — формула ItemViewTime.lag_at / zone_gap; входы — set_lag_inputs раз в кадр, своя
// копия); зажат в [0; min(swept_age, stop_age)]. Остановленный, долетевший до точки остановки, не показан; при касании —
// одна вспышка FLASH_S с масштабом FLASH_SCALE → 0 и событие касания (ключ снаряда — take_impacts; облик попадания
// пушек, spec 2026-11-02-modular-guns §6.3). Тип снаряда — MultiMesh в сценарии (тени выключены) с масштабом меша
// (меш облика — для радиуса 1 м) и цветом (параметр экземпляра tint), буфер — одним вызовом на тип. Пишет только
// MultiMesh и поля кадра роя (shown, flashed); состояние тика не меняет (AGENTS §2.2).
class ProjectileViews : public RefCounted {
	GDCLASS(ProjectileViews, RefCounted);

public:
	static constexpr double FLASH_S = 0.15;
	static constexpr real_t FLASH_SCALE = 3.0;
	static constexpr double STEAL_BODY_R = 0.70; // = ItemViewTime.STEAL_BODY_R
	static constexpr double STEAL_HAND_R = 0.30; // = ItemViewTime.STEAL_HAND_R
	static constexpr int STRIDE = 12; // Transform3D в буфере MultiMesh

private:
	struct Type {
		RID multimesh;
		RID instance;
		real_t scale = 1.0; // Масштаб меша показа
		int allocated = 0;
		int count = 0;
		Vector<float> buf;
	};
	struct Flash {
		int type = 0;
		Vector3 at;
		double left = 0.0;
	};

	RID scenario;
	LocalVector<Type> types;
	LocalVector<Flash> flashes;
	LocalVector<Vector3> lag_points; // На игрока 4: тело, тело −0.5 м по Y, цель левой руки, цель правой (нет — INF)
	LocalVector<double> lags; // Задержка показа игрока, с (свой — 0)
	int lag_local = -1; // Номер своего, −1 — нет (задержка 0)
	PackedInt64Array impacts; // Ключи снарядов, показ которых дошёл до касания с прошлого take_impacts

	double _gap(int p_player, const Vector3 &p_at) const;
	void _put(int p_type, const Vector3 &p_at, real_t p_scale);
	void _commit(Type &p_type);

protected:
	static void _bind_methods();

public:
	void setup(RID p_scenario);
	int type_add(RID p_mesh, real_t p_scale = 1.0, const Color &p_tint = Color(1, 1, 1, 1));
	void set_lag_inputs(const PackedVector3Array &p_points, const PackedFloat64Array &p_lags, int p_local);
	double lag_at(const Vector3 &p_at) const;
	void frame(const Ref<ProjectileSwarm> &p_swarm, int64_t p_frames, double p_alpha, int p_tps, double p_net_now,
			double p_delta);
	PackedInt64Array take_impacts();
	int visible_count() const;

	~ProjectileViews();
};
