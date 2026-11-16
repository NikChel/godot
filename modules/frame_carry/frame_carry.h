#pragma once

#include "core/math/aabb.h"
#include "core/math/transform_3d.h"
#include "core/object/ref_counted.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"
#include "core/variant/array.h"
#include "core/variant/callable.h"

class Node3D;
class RigidBody3D;

// Пассажиры подвижных систем отсчёта карты — горячий путь мира FrameRiders (GDScript проекта; закон езды, ловушки и
// решения — шапка _game/core/frame/frame_riders.gd; AGENTS §2.9, §3.5): перенос один в один, плоские записи, один
// проход в конце физ-тика (pre: пространство тела по зонам систем — геометрией, тяжесть к полу системы, несение шагом
// системы — ReferenceFrame.ride раз на систему, не на тело) и один в начале следующего (post: поправка позы, вычет
// несения). Тела — предметы карты (группа дерева, setup) и группы (add_group: тела куклы, первое — якорь: пространство
// группы по нему, в каком бы режиме он ни был; кинематические тела группы не едут — их ведут писатели, — но их
// пространство — пространство группы). Работа с телом — PhysicsServer3D и узел (C++), без вызовов GDScript на тело; на
// систему и зону за тик — query_xform / step_xform / zones / ride / accel (GDScript). Поза тела в несении — узла: тело,
// перенесённое записью узла в этом тике (кукла из запаса), сервер получит только на сбросе преобразований дерева после
// отложенных вызовов.
class FrameCarry : public RefCounted {
	GDCLASS(FrameCarry, RefCounted);

public:
	static constexpr double EPS_MIN = 1.0e-6; // Поправка позы меньше — не писать, м
	static constexpr double DAMP_FLOOR = 1.0e-3; // Нижний предел 1 − k·dt (затухание больше тика)
	// Опрос: тело дальше этого от сферы коробок зон систем (поза commit) за тик в зону не попадёт (шаг системы до ~2 м +
	// своя скорость до ~500 м/с) — принадлежность не проверяется и «свободным у системы» оно не числится; пассажир —
	// выходит. Тела на корабле вдали от систем стоят опросу поиска в списке, а не проверки зон.
	static constexpr real_t NEAR_MARGIN = 8.0;

private:
	// Коробки зоны (как GravityZone._ready): без поворота — в кадре зоны, с поворотом — в своём кадре (inv).
	struct ZoneShape {
		int priority = 0;
		LocalVector<AABB> boxes;
		LocalVector<Transform3D> inv;
		LocalVector<uint8_t> rot;
		AABB bounds;
	};
	// Система этого тика (pre).
	struct FrameTick {
		ObjectID id;
		Transform3D query; // Поза запросов этого тика
		Transform3D step; // Шаг на pre (commit этого тика уже был — его шаг; не было — единичный)
		Vector3 field; // Центр масс главного тела в кадре системы
		Callable on_step;
		bool riders = false; // Есть пассажиры этого тика (ride)
	};
	// Зона системы этого тика (порядок — системы по порядку, зоны — по ReferenceFrame.zones).
	struct ZoneTick {
		ObjectID id;
		int frame = -1; // Индекс в frames_t
		const ZoneShape *shape = nullptr;
		Transform3D to_local; // Мир → кадр зоны (поза на pre)
		Vector3 accel; // GravityZone.accel (на pre; на шаге системы — снова)
		Vector3 near_c; // Сфера коробок зоны (мир, поза на pre) — опрос
		real_t near_r2 = 0.0; // (радиус + NEAR_MARGIN)²
	};
	struct Rider {
		ObjectID body;
		RID rid;
		ObjectID frame;
		ObjectID zone;
		int zi = -1; // Индекс зоны в zones_t этого тика
		Vector3 com_l; // Центр масс в кадре тела
		uint64_t tick = 0; // Физ-тик последнего pre
		bool entering = true; // Первый тик в пространстве: скорость в мире не трогать
		Vector3 v; // Своя скорость на pre (относительно системы, мировые оси позы запросов)
		Vector3 w;
		Vector3 u; // Скорость несения центра масс, отданная шагу
		Vector3 uw;
		Vector3 eps; // Поправка позы после шага
		bool carried = false; // Шаг этого тика с несением (post вычтет)
		bool written = false; // Скорость тела переписана на этом тике
	};
	struct Group {
		LocalVector<ObjectID> bodies; // Первое — якорь пространства
		ObjectID frame; // Пространство группы по последнему pre
		ObjectID zone;
	};

	StringName group_name; // Группа дерева предметов карты (setup)
	LocalVector<Rider> riders;
	HashMap<ObjectID, uint32_t> rider_at;
	HashMap<ObjectID, uint64_t> free_seen; // Свободные динамические тела корабля → физ-тик pre, где их видели
	HashMap<int64_t, Group> groups;
	HashMap<ObjectID, int64_t> member_of; // Тело группы → группа
	HashMap<ObjectID, ZoneShape> shapes; // Коробки зон (раз на зону)
	HashMap<ObjectID, Vector3> fields; // Центр масс главного тела системы в её кадре (раз на систему)
	HashMap<ObjectID, Callable> step_cbs; // Шаг системы → _on_step (раз на систему)
	LocalVector<FrameTick> frames_t;
	LocalVector<ZoneTick> zones_t;
	uint64_t pre_tick = UINT64_MAX;
	int seen = 0; // Тел и групп опрошено на последнем pre (цена)

	static RigidBody3D *_body(ObjectID p_id);
	static bool _dynamic(const RigidBody3D *p_rb);
	int _rider(ObjectID p_id) const;
	void _erase(uint32_t p_i);
	int _frame(ObjectID p_id) const;
	void _sync(const Array &p_frames);
	const ZoneShape *_shape(Node3D *p_zone);
	Vector3 _field(Node3D *p_frame);
	int _zone_at(const Vector3 &p_point) const;
	bool _near(const Vector3 &p_point) const;
	static bool _contains(const ZoneTick &p_z, const Vector3 &p_point);
	void _place(RigidBody3D *p_rb, int p_zi, uint64_t p_t);
	void _gravity(const Rider &p_r, RigidBody3D *p_rb, const Vector3 &p_accel);
	void _restore(const Rider &p_r);
	void _carry(Rider &r_r, const Transform3D &p_step, const FrameTick &p_f);
	void _write(const Rider &p_r, const Vector3 &p_v, const Vector3 &p_w);
	void _on_step(const Transform3D &p_step, int64_t p_frame);

protected:
	static void _bind_methods();

public:
	void setup(const StringName &p_group);
	void pre(const Array &p_frames);
	void post();
	void add_group(int64_t p_id, const Array &p_bodies);
	void remove_group(int64_t p_id);
	Object *frame_of(Object *p_body) const;
	Object *zone_of(Object *p_body) const;
	int count() const { return (int)riders.size(); }
	int get_seen() const { return seen; }
};
