#pragma once

#include "core/math/aabb.h"
#include "core/math/transform_3d.h"
#include "core/object/object_id.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"
#include "core/variant/array.h"

class Node3D;

// Геометрия подвижных систем и их зон этого тика для FrameCarry: системы (ReferenceFrame.spaces) — поза запросов, шаг,
// центр масс главного тела; зоны (ReferenceFrame.zones, GravityZone) — коробки, поза, тяжесть, сфера опроса. sync —
// снимок систем и зон на pre (вызов GDScript на систему и зону); refresh_accel — тяжесть зон системы в позе её commit;
// zone_at — зона точки (мир, поза запросов); is_close — точка у какой-нибудь зоны (дальше за тик в зону не попасть).
// Коробки зон и центры масс главных тел — раз на зону и систему (кэш до конца мира).
struct FrameSpaces {
	// Опрос: тело дальше этого от сферы коробок зон систем (поза commit) за тик в зону не попадёт (шаг системы до ~2 м +
	// своя скорость до ~500 м/с) — принадлежность не проверяется и «свободным у системы» оно не числится; пассажир —
	// выходит. Тела на корабле вдали от систем стоят опросу поиска в списке, а не проверки зон.
	static constexpr real_t NEAR_MARGIN = 8.0;

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
	};
	// Зона системы этого тика (порядок — системы по порядку, зоны — по ReferenceFrame.zones).
	struct ZoneTick {
		ObjectID id;
		int frame = -1; // Индекс в frames
		const ZoneShape *shape = nullptr;
		Transform3D to_local; // Мир → кадр зоны (поза на pre)
		Vector3 accel; // GravityZone.accel (на pre; на шаге системы — снова)
		Vector3 near_c; // Сфера коробок зоны (мир, поза на pre) — опрос
		real_t near_r2 = 0.0; // (радиус + NEAR_MARGIN)²
	};

	LocalVector<FrameTick> frames;
	LocalVector<ZoneTick> zones;

	void sync(const Array &p_frames);
	void clear();
	void refresh_accel(int p_frame);
	int frame(ObjectID p_id) const;
	int zone_at(const Vector3 &p_point) const;
	bool is_close(const Vector3 &p_point) const;

private:
	HashMap<ObjectID, ZoneShape> shapes; // Коробки зон (раз на зону)
	HashMap<ObjectID, Vector3> fields; // Центр масс главного тела системы в её кадре (раз на систему)

	const ZoneShape *_shape(Node3D *p_zone);
	Vector3 _field(Node3D *p_frame);
	static bool _contains(const ZoneTick &p_z, const Vector3 &p_point);
};
