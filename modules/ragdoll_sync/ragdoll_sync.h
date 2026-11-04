#pragma once

#include "core/object/object_id.h"
#include "core/object/ref_counted.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"
#include "core/variant/callable.h"
#include "modules/projectiles/view_lag.h"
#include "ragdoll_pelvis_track.h"

class RigidBody3D;

// Сеть таза кукол карты — горячий путь (spec docs/superpowers/specs/2026-11-02-walker-ragdoll-design.md §2.7, §5 этап
// 5; AGENTS §2.9): один проход за физ-тик на всех кукол сети, перенос из GDScript один в один. Решения остаются у
// RagdollNet (GDScript): владение, эпохи, RPC, касания, роли, разморозка таза при смене роли (release), финальная поза
// владельца (_send_rest), часы тика. Здесь — плоские записи на куклу (id — instance id куклы, key — сетевой номер её
// ходока) в порядке добавления:
//   владелец (own) — owner_tick: уснула — просьба финальной позы (take_rests), иначе раз в отправку поза таза в системе
//     пространства ходока (space_of куклы, space_xf карты), если сдвинулась больше move_eps или повернулась больше
//     turn_eps — пакетом RagdollPelvisCodec (формат RagdollPelvisPose);
//   следующий — tick: RagdollPelvisTrack каждой (ЖДУ / СЛЕДУЮ / ФИНАЛ), цель задержки — ViewLag (входы — set_lag_inputs
//     тика), задержка до владельца — one_way карты; возвращает кукол «усыпить». Точки потока — feed_packed / feed:
//     только от владельца (зеркало владельцев RagdollNet — set_owner) и только следующим.
// Сон куклы (asleep) — push из Ragdoll (set_asleep), тик его не спрашивает. Поза таза пишется узлу (RigidBody3D, как
// прежде — кинематике цель шага физики уходит при сбросе преобразований дерева), заморозка при переходе ЖДУ → СЛЕДУЮ —
// в тике в том же порядке.
class RagdollSync : public RefCounted {
	GDCLASS(RagdollSync, RefCounted);

public:
	static constexpr int BYTES = 21; // = RagdollPelvisCodec::BYTES
	// Состояние следующего (state): ЖДУ — поток не дошёл (своя физика), СЛЕДУЮ — кинематика по буферу, ФИНАЛ — стоит в
	// финальной позе владельца.
	static constexpr int WAIT = 0;
	static constexpr int FOLLOW = 1;
	static constexpr int REST = 2;

private:
	struct Ent {
		int64_t id = 0;
		int64_t key = -1;
		ObjectID pelvis;
		ObjectID holder; // Ходок куклы: нет — пространство 0
		Callable space_of; // Пространство ходока (EnemyWalker.space_fn: метод мира тел / копий с ручкой) -> int
		bool own = false;
		bool asleep = false;
		RagdollPelvisTrack track;
		Transform3D sent_xf; // Владелец: последняя отправленная поза таза (система пространства)
		int sent_space = -1;
		bool rest_sent = false;
	};

	LocalVector<Ent> ents; // Порядок добавления (как Dictionary RagdollNet._entries)
	HashMap<int64_t, int64_t> by_key; // Номер куклы → id (последняя добавленная)
	HashMap<int64_t, int> owners; // Номер куклы → владелец (нет — хост 1) — зеркало RagdollNet._owner
	RagdollPelvisTrack::Params params;
	double move_eps = 0.002;
	double turn_eps = 0.005;
	Callable one_way_fn; // one_way(peer: int) -> float
	RagdollSpaceXforms spaces;
	ViewLag view_lag;
	LocalVector<int> ow_peer; // Кэш one_way тика
	LocalVector<double> ow_val;
	PackedInt64Array rests;
	LocalVector<uint8_t> pack_buf; // Пакет поз отправки (без перевыделения на позу)

	int _find(int64_t p_id) const;
	int _owner_of(int64_t p_key) const;
	double _one_way(int p_peer);
	RigidBody3D *_pelvis(const Ent &p_e) const;

protected:
	static void _bind_methods();

public:
	void setup(double p_buffer_s, double p_keep_s, double p_catchup_s, double p_rest_s, double p_rate_up,
			double p_rate_down, double p_move_eps, double p_turn_eps, const Callable &p_one_way, const Callable &p_space_xf);
	void add(int64_t p_id, int64_t p_key, RigidBody3D *p_pelvis, bool p_own, bool p_asleep, Object *p_holder,
			const Callable &p_space_of);
	void remove(int64_t p_id);
	void clear();
	bool has(int64_t p_id) const;
	int count() const { return (int)ents.size(); }

	void set_owner(int64_t p_key, int p_peer);
	void clear_owners();
	void set_own(int64_t p_id);
	void set_follower(int64_t p_id);
	void set_asleep(int64_t p_id, bool p_asleep);
	void mark_sent(int64_t p_id, int p_space, const Transform3D &p_local);

	PackedInt64Array nearby(const Vector3 &p_point, double p_radius) const;
	PackedByteArray owner_tick(bool p_send);
	PackedInt64Array take_rests();
	bool needs_lag() const;
	void set_lag_inputs(const PackedVector3Array &p_points, const PackedFloat64Array &p_lags, int p_local);
	double lag_at(const Vector3 &p_at) const;
	PackedInt64Array tick(double p_now, int p_tps);

	void feed(int p_from, int64_t p_key, double p_t, int p_space, const Transform3D &p_xf, bool p_final);
	void feed_packed(int p_from, double p_t, const PackedByteArray &p_data);

	bool following(int64_t p_id) const;
	int state(int64_t p_id) const;
	double lag(int64_t p_id) const;
	Vector3 velocity(int64_t p_id, int p_tps) const;
	Vector3 angular_velocity(int64_t p_id, int p_tps) const;

	// Формат позы (пробы: байты = RagdollPelvisPose.pack).
	static PackedByteArray pack_pose(int64_t p_key, int p_space, const Transform3D &p_xf);
	static Array unpack_pose(const PackedByteArray &p_data);
};
