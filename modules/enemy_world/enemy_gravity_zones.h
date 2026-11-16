#pragma once

#include "core/math/aabb.h"
#include "core/math/transform_3d.h"
#include "core/object/object_id.h"
#include "core/templates/local_vector.h"
#include "core/variant/array.h"

// Зоны тяжести карты для тел врагов (полёт, бриф 2026-11-14): Area3D зон (GravityZone, GDScript) — коробки детей
// (CollisionShape3D с BoxShape3D, как GravityZone._ready), по убыванию priority; тяжесть зоны — её gravity_direction ×
// gravity (ставит GravityZone.set_on / follow_frame — то же, что берёт PhysX). refresh — позы и тяжесть этого тика;
// accel_at — тяжесть точки (мир): зона выше priority из содержащих; вне зон — false (тяжесть мира).
struct EnemyGravityZones {
	struct Box {
		AABB box; // Без поворота — в кадре зоны; с поворотом — в своём кадре (inv)
		Transform3D inv; // Кадр зоны -> кадр повёрнутой коробки
		bool rot = false;
	};
	struct Zone {
		ObjectID id;
		int priority = 0;
		LocalVector<Box> boxes;
		AABB bounds; // Все коробки в кадре зоны
		Transform3D inv; // Мир -> кадр зоны (этого тика)
		Vector3 accel; // Тяжесть зоны этого тика, мир
		bool valid = false;
	};

	LocalVector<Zone> zones;

	void set(const Array &p_areas);
	void refresh();
	bool accel_at(const Vector3 &p_world, Vector3 &r_accel) const;
	bool is_empty() const { return zones.is_empty(); }
};
