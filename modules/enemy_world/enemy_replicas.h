#pragma once

#include "enemy_views.h"

#include "core/math/vector3.h"
#include "core/object/object_id.h"
#include "core/object/ref_counted.h"
#include "core/string/string_name.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"

class Node3D;

// Копии ходоков клиента (этап 2 EnemyWorld; план docs/superpowers/plans/2026-10-22-enemy-world-stage2.md; порт
// WalkerRemote один в один): буфер снимков хоста — поза в системе отсчёта ходока (сейчас мир), взгляд, скорость
// вдоль взгляда, подъём шага — и события (клип, удар — пустое имя) по времени хоста на нашей шкале. Тик (tick)
// показывает момент seek (now − AVATAR_LAG_S, считает GDScript): поза — интерполяция соседних точек (за краями —
// крайняя) в тело, взгляд и поза — визуалу (EnemyViews), события, чьё время прошло, — по порядку наружу (ручки и
// клипы; удар по своему игроку решает GDScript). Точки — по возрастанию времени хоста, не больше CAP; пакет
// снимка — по net id (feed_snapshot), один вызов на пакет. Тела — Node3D (ObjectID; нет тела — только буфер).
class EnemyReplicas : public RefCounted {
	GDCLASS(EnemyReplicas, RefCounted);

public:
	static constexpr int CAP = 16; // Точек (≈0.5 с при 30 Гц)
	static constexpr int F = 6; // Чисел на точку: x, y, z, yaw, speed, step_lag

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
		Vector3 pos;
		real_t yaw = 0.0f;
		real_t speed = 0.0f;
		real_t step_lag = 0.0f;
	};

	LocalVector<Replica> reps;
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

protected:
	static void _bind_methods();

public:
	void set_views(const Ref<EnemyViews> &p_views);
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
	real_t replica_speed(int p_h) const;
	real_t replica_step_lag(int p_h) const;
	// Тик копий: ручки копий с наступившими событиями (клипы — get_due_clips, по порядку)
	PackedInt32Array tick(double p_seek);
	Array get_due_clips() const { return due_clips; }
	int get_replica_count() const;
	int64_t get_last_tick_usec() const { return (int64_t)last_tick_usec; }
};
