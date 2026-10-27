#pragma once

#include "enemy_anim.h"
#include "enemy_arms.h"
#include "enemy_rig.h"
#include "enemy_tumbler.h"
#include "enemy_views.h"

#include "core/math/transform_3d.h"
#include "core/math/vector3.h"
#include "core/object/object_id.h"
#include "core/object/ref_counted.h"
#include "core/string/string_name.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"

class Node3D;

// Копии ходоков клиента (этап 2 EnemyWorld; план docs/superpowers/plans/2026-10-22-enemy-world-stage2.md; порт
// WalkerRemote один в один): буфер снимков хоста — поза в системе отсчёта ходока (пространство: корабль или
// подвижная система; spec 2026-10-23-enemy-moving-spaces §6), взгляд относительно неё, скорость вдоль взгляда, подъём
// шага, номер пространства — и события (клип, удар — пустое имя) по времени хоста на нашей шкале. Тик (tick)
// показывает момент seek (now − AVATAR_LAG_S, считает GDScript): поза — интерполяция соседних точек (за краями —
// крайняя), в мир — позой пространства у этого пира в этом тике (корень карты — set_root, системы — space_add,
// query_xform): в тело; соседние точки разных пространств (переход) — каждая в мир позой своего пространства и
// смешение в мире; взгляд и поза — визуалу (EnemyViews: в системе пространства, переход — в мире), события, чьё время прошло, — по порядку наружу (ручки и
// клипы; удар по своему игроку решает GDScript). Точки — по возрастанию времени хоста, не больше CAP; пакет
// снимка — по net id (feed_snapshot), один вызов на пакет. Тела — Node3D (ObjectID; нет тела — только буфер).
// Неваляшка (spec 2026-10-26-walker-tumbler-design): наклон каждый пир считает сам по толчкам (replica_push; отброс —
// снимками хоста); шаг — в тике копий, визуал — с наклоном, хит-тело — основание × наклон.
// Анимация и руки (spec 2026-10-27-walker-arms-design): как у тел хоста (EnemyBodies) — часы анимации по скорости
// снимка, разовый клип — replica_play (GDScript по событию хоста), качание рук каждый пир считает сам.
class EnemyReplicas : public RefCounted {
	GDCLASS(EnemyReplicas, RefCounted);

public:
	static constexpr int CAP = 16; // Точек (≈0.5 с при 30 Гц)
	static constexpr int F = 7; // Чисел на точку: x, y, z, yaw, speed, step_lag, пространство

private:
	struct Replica {
		ObjectID body_id;
		int view = -1;
		int net_id = -1;
		bool used = false;
		int n = 0;
		double t_local[CAP] = {};
		double t_host[CAP] = {};
		float d[CAP * F] = {};
		LocalVector<double> ev_t;
		LocalVector<StringName> ev_clip;
		Vector3 pos; // В системе пространства space
		int space = 0; // Пространство выборки (переход — ближайшей по времени точки)
		bool cross = false; // Соседние точки разных пространств: a — pos_a / space_a, b — pos_b / space_b, доля cross_f
		Vector3 pos_a;
		Vector3 pos_b;
		int space_a = 0;
		int space_b = 0;
		real_t cross_f = 0.0f;
		real_t yaw = 0.0f;
		real_t speed = 0.0f;
		real_t step_lag = 0.0f;
		EnemyTumbler tumb;
		Ref<EnemyRig> rig;
		EnemyAnim anim;
		EnemyArms arms;
	};

	struct Space {
		ObjectID node_id;
		bool frame = false; // Подвижная система (query_xform); нет — корень карты (global_transform)
		Transform3D xf;
	};

	LocalVector<Replica> reps;
	LocalVector<Space> spaces; // 0 — корень карты; пусто — мир
	LocalVector<int> free_handles;
	HashMap<int, int> by_net;
	Ref<EnemyViews> views;
	PackedInt32Array due_handles;
	Array due_clips;
	uint64_t last_tick_usec = 0;

	bool _valid(int p_h) const;
	static void _feed(Replica &r_r, double p_t_local, double p_t_host, const float *p_src);
	static bool _sample(Replica &r_r, const double *p_times, double p_seek);
	static void _take(Replica &r_r, int p_a, int p_b, real_t p_f);
	static StringName _pop_due(Replica &r_r, double p_seek);
	void _space_xforms();
	Transform3D _space_xf(int p_space) const;
	Transform3D _tumble_base(const Replica &p_r, Node3D *p_body) const;

protected:
	static void _bind_methods();

public:
	void set_views(const Ref<EnemyViews> &p_views);
	void set_root(Node3D *p_root); // Пространство 0 — корень карты (сбрасывает реестр)
	int space_add(Node3D *p_frame); // Подвижная система (ReferenceFrame: query_xform) — следующий номер
	int get_space_count() const { return (int)spaces.size(); }
	int replica_add(Node3D *p_body, int p_view, int p_net_id);
	void replica_remove(int p_h);
	void feed(int p_h, double p_t_local, double p_t_host, const PackedFloat32Array &p_data, int p_at);
	void feed_snapshot(double p_t_local, double p_t_host, const PackedInt32Array &p_ids, const PackedFloat32Array &p_data);
	void queue_event(int p_h, double p_t_local, const StringName &p_clip);
	StringName pop_due(int p_h, double p_seek); // «-» — нет события
	bool sample_local(int p_h, double p_seek);
	bool sample_host(int p_h, double p_t_host);
	int replica_size(int p_h) const;
	Vector3 replica_pos(int p_h) const;
	real_t replica_yaw(int p_h) const;
	int replica_space(int p_h) const;
	real_t replica_speed(int p_h) const;
	real_t replica_step_lag(int p_h) const;
	// Неваляшка (как EnemyBodies.body_set_tumbler / body_push; отброса у копии нет)
	void replica_set_tumbler(int p_h, Node3D *p_hit_body, real_t p_inertia, real_t p_frequency, real_t p_damping, real_t p_fall_angle, real_t p_down_angle, real_t p_height, real_t p_down_time, real_t p_rise_time, real_t p_stagger_angle, real_t p_knockback);
	void replica_push(int p_h, const Vector3 &p_r, const Vector3 &p_j);
	Transform3D replica_tumble_base(int p_h) const;
	Vector3 replica_tumble_spin(int p_h) const;
	Vector3 replica_tilt(int p_h) const;
	int replica_tumble_state(int p_h) const;
	// Анимация и руки (как EnemyBodies.body_set_rig / body_play / body_arm_*)
	void replica_set_rig(int p_h, const Ref<EnemyRig> &p_rig, Node3D *p_arm_l, Node3D *p_arm_r);
	void replica_play(int p_h, const StringName &p_clip);
	bool replica_acting(int p_h) const;
	void replica_set_arm_tumbler(int p_h, int p_side, real_t p_inertia, real_t p_frequency, real_t p_damping, real_t p_limit);
	void replica_arm_push(int p_h, int p_side, const Vector3 &p_r, const Vector3 &p_j);
	Transform3D replica_arm_base(int p_h, int p_side) const;
	Vector3 replica_arm_spin(int p_h, int p_side) const;
	Vector3 replica_arm_tilt(int p_h, int p_side) const;
	// Тик копий: ручки копий с наступившими событиями (клипы — get_due_clips, по порядку)
	PackedInt32Array tick(double p_seek);
	Array get_due_clips() const { return due_clips; }
	int get_replica_count() const;
	int64_t get_last_tick_usec() const { return (int64_t)last_tick_usec; }
};
