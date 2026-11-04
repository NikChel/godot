#pragma once

#include "core/math/color.h"
#include "core/math/math_defs.h"
#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"
#include "projectile_swarm.h"
#include "view_lag.h"

// Показ снарядов карты (spec docs/superpowers/specs/2026-11-02-projectile-swarm-cpp-design.md §4): один проход кадра
// рендера на все снаряды роя (AGENTS §2.9), перенос прежнего ProjectileView (GDScript) один в один. Возраст показа:
// свой — между двумя последними тиками ((frames − born_tick − 1 + alpha) / tps), чужой — «настоящее стрелка» минус
// задержка показа точки (lag_at — ViewLag: формула ItemViewTime.lag_at / zone_gap; входы — set_lag_inputs раз в кадр,
// своя копия); зажат в [0; min(swept_age, stop_age)]. Остановленный, долетевший до точки остановки, не показан; при касании —
// одна вспышка FLASH_S с масштабом FLASH_SCALE → 0 и событие касания (ключ снаряда — take_impacts; облик попадания
// пушек, spec 2026-11-02-modular-guns §6.3). Тип снаряда — MultiMesh в сценарии (тени выключены) с масштабом меша
// (меш облика — для радиуса 1 м), цветом (параметр экземпляра tint) и трассой: trail > 0 — меш вытянут вдоль пути
// от позы снаряда на trail с раньше (не раньше вылета) до показанной, концы — на радиус за ними; вспышка — без
// трассы. Буфер — одним вызовом на тип. Пишет только
// MultiMesh и поля кадра роя (shown, flashed); состояние тика не меняет (AGENTS §2.2).
class ProjectileViews : public RefCounted {
	GDCLASS(ProjectileViews, RefCounted);

public:
	static constexpr double FLASH_S = 0.15;
	static constexpr real_t FLASH_SCALE = 3.0;
	static constexpr int STRIDE = 12; // Transform3D в буфере MultiMesh

private:
	struct Type {
		RID multimesh;
		RID instance;
		real_t scale = 1.0; // Масштаб меша показа
		double trail = 0.0; // Трасса, с полёта за показанной точкой; 0 — без трассы
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
	ViewLag view_lag; // Входы задержки показа чужих (set_lag_inputs)
	PackedInt64Array impacts; // Ключи снарядов, показ которых дошёл до касания с прошлого take_impacts

	void _put(int p_type, const Vector3 &p_head, const Vector3 &p_tail, real_t p_scale);
	void _commit(Type &p_type);

protected:
	static void _bind_methods();

public:
	void setup(RID p_scenario);
	int type_add(RID p_mesh, real_t p_scale = 1.0, const Color &p_tint = Color(1, 1, 1, 1), double p_trail = 0.0);
	void set_lag_inputs(const PackedVector3Array &p_points, const PackedFloat64Array &p_lags, int p_local);
	double lag_at(const Vector3 &p_at) const;
	void frame(const Ref<ProjectileSwarm> &p_swarm, int64_t p_frames, double p_alpha, int p_tps, double p_net_now,
			double p_delta);
	PackedInt64Array take_impacts();
	int visible_count() const;

	~ProjectileViews();
};
