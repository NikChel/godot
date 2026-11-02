#pragma once

#include "core/math/math_defs.h"
#include "core/object/ref_counted.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"
#include "core/variant/typed_array.h"
#include "scene/resources/3d/world_3d.h"
#include "servers/physics_3d/physics_server_3d.h"

// Рой снарядов карты (spec docs/superpowers/specs/2026-11-02-projectile-swarm-cpp-design.md §3): состояние всех
// снарядов и свип всех за физ-тик одним проходом (AGENTS §2.9). Перенос прежнего ProjectileWorld (GDScript) один в
// один: закон полёта p(a) = p0 + v0·a + ½·g·a² (g — вниз); возраст «настоящего» своего — по тикам от born_tick,
// чужого — по часам (net_now − t0_local); свип сферы radius от swept_age до min(age, life) (cast_motion), первое
// касание — остановка; нормаль — rest_info сферы, сдвинутой за касание по ходу на contact_steps (пусто — против хода),
// точка — центр минус нормаль·radius; не попал и age ≥ life — остановка в life. Остановленный хранится keep_s; сверх
// max_live вытесняется старейший. Снаряд — по ключу роя (выдаёт add; номер выстрела рою не нужен). События
// (остановки, удаления) забирает ProjectileWorld; разбор касания в Hit — у него. Поля кадра (shown, flashed) пишет и
// читает только ProjectileViews (AGENTS §2.2).
class ProjectileSwarm : public RefCounted {
	GDCLASS(ProjectileSwarm, RefCounted);
	friend class ProjectileViews;

	struct Shot {
		int64_t key = 0;
		bool auth = true;
		int64_t born_tick = 0; // Свой: физ-тик рождения
		double t0_local = 0.0; // Чужой: момент вылета на нашей шкале «настоящего» стрелка, с
		Vector3 p0;
		Vector3 v0;
		double gravity = 0.0;
		real_t radius = 0.1;
		double life = 1.0;
		double wake_extra = 0.0; // Радиус сплеша рецепта для точки интереса, м
		int view_type = 0; // Номер типа показа (ProjectileViews)
		PhysicsDirectSpaceState3D::ShapeParameters q; // Запрос свипа: форма и exclude — при добавлении
		double swept_age = 0.0;
		double stop_age = Math::INF;
		bool has_impact = false;
		Vector3 hit_point;
		Vector3 hit_normal;
		Dictionary info; // Касание своего: словарь как get_rest_info; иначе пусто
		// Поля кадра (ProjectileViews)
		Vector3 shown; // Последняя показанная точка (задержка показа чужого)
		bool flashed = false;

		bool stopped() const { return stop_age < Math::INF; }
		Vector3 pos_at(double p_age) const {
			return p0 + v0 * (real_t)p_age + Vector3(0, -1, 0) * (real_t)(0.5 * gravity * p_age * p_age);
		}
		Vector3 vel_at(double p_age) const {
			return v0 + Vector3(0, -1, 0) * (real_t)(gravity * p_age);
		}
	};

	LocalVector<Shot> shots; // Порядок добавления
	HashMap<real_t, RID> spheres; // Форма сферы по радиусу
	uint32_t mask = 1 | 4 | 8 | 16 | 128;
	LocalVector<real_t> contact_steps;
	int max_live = 512;
	double keep_s = 1.0;
	int64_t next_key = 1;
	PackedInt64Array stopped_ev;
	PackedInt64Array removed_ev;

	RID _sphere(real_t p_radius);
	int _find(int64_t p_key) const;
	void _advance(PhysicsDirectSpaceState3D *p_space, Shot &p_s, double p_age);
	bool _contact(PhysicsDirectSpaceState3D *p_space, Shot &p_s, const Vector3 &p_at, const Vector3 &p_dir,
			PhysicsDirectSpaceState3D::ShapeRestInfo &r_info);

protected:
	static void _bind_methods();

public:
	void setup(int p_mask, const PackedFloat64Array &p_contact_steps, int p_max_live, double p_keep_s);
	int64_t add(bool p_auth, int64_t p_born_tick, double p_t0_local, const Vector3 &p_p0, const Vector3 &p_v0,
			double p_gravity, real_t p_radius, double p_life, const TypedArray<RID> &p_exclude, double p_wake_extra,
			int p_view_type);
	void stop(int64_t p_key, double p_age, const Vector3 &p_point, const Vector3 &p_normal);
	void step(const Ref<World3D> &p_world, int64_t p_tick, int p_tps, double p_net_now);
	PackedInt64Array events_stopped();
	PackedInt64Array events_removed();
	Dictionary state(int64_t p_key) const;
	PackedVector4Array append_wake(const PackedVector4Array &p_points, double p_lead, double p_shot_r) const;
	int count() const { return (int)shots.size(); }

	ProjectileSwarm();
	~ProjectileSwarm();
};
