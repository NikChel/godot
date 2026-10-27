#pragma once

#include "enemy_anim.h"
#include "enemy_rig.h"
#include "enemy_tumbler.h"

#include "core/math/transform_3d.h"
#include "core/object/object_id.h"

class Node3D;

// Руки врага (spec docs/superpowers/specs/2026-10-27-walker-arms-design.md): на руку — неваляшка без падения
// (EnemyTumbler, can_fall = false) на плече и хит-тело (StaticBody3D руки: капсулы Upper — в кадре кости плеча, Fore —
// в кадре кости предплечья вместе с кистью; капсулы в сцене — в позе покоя руки). Основание неваляшки — кадр кости
// плеча по анимации этого тика (мир; Y — вдоль кости), наклон поворачивает всю руку вокруг плеча. Конец тика тела
// (place): хит-тело — основание × наклон, Fore — кадр предплечья относительно плеча; сразу в PhysicsServer.
// Общая для тел хоста (EnemyBodies) и копий клиента (EnemyReplicas).
struct EnemyArms {
	EnemyTumbler tumb[EnemyRig::ARMS];
	ObjectID fore_id[EnemyRig::ARMS]; // Капсула предплечья (CollisionShape3D «Fore» хит-тела)
	Transform3D fore_offset[EnemyRig::ARMS]; // Fore в кадре кости предплечья (из сцены)
	Transform3D base[EnemyRig::ARMS]; // Основание последнего place (мир)

	void setup(int p_side, Node3D *p_hit_body, const EnemyRig &p_rig);
	void step(real_t p_dt);
	// Основания по анимации и поза тел; p_avatar — поза корня аватара этого тика (мир)
	void place(const EnemyRig &p_rig, const EnemyAnim &p_anim, const Transform3D &p_avatar);
	Quaternion tilt(int p_side) const; // Наклон (кадр кости плеча) — визуалу
};
