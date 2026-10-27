#include "enemy_arms.h"

#include "core/math/math_funcs.h"
#include "scene/3d/node_3d.h"
#include "scene/3d/physics/collision_shape_3d.h"

static const StringName &_fore_name() {
	static const StringName n = StringName("Fore");
	return n;
}

// Хит-тело руки (в сцене — кадр кости плеча) и его капсула Fore: в сцене задана в позе покоя руки, смещение в кадре
// кости предплечья — через покой предплечья относительно плеча (EnemyRig.arm_rest).
void EnemyArms::setup(int p_side, Node3D *p_hit_body, const EnemyRig &p_rig) {
	ERR_FAIL_INDEX(p_side, EnemyRig::ARMS);
	EnemyTumbler &t = tumb[p_side];
	t.reset();
	t.can_fall = false;
	t.down_angle = (real_t)Math::PI; // Пола у руки нет
	t.hit_id = p_hit_body ? p_hit_body->get_instance_id() : ObjectID();
	fore_id[p_side] = ObjectID();
	fore_offset[p_side] = Transform3D();
	Node3D *fore = p_hit_body ? Object::cast_to<Node3D>(p_hit_body->get_node_or_null(NodePath(_fore_name()))) : nullptr;
	if (fore != nullptr) {
		fore_id[p_side] = fore->get_instance_id();
		fore_offset[p_side] = p_rig.arm_rest(p_side).affine_inverse() * fore->get_transform();
	}
}

void EnemyArms::step(real_t p_dt) {
	for (EnemyTumbler &t : tumb) {
		if (t.hit_id.is_valid()) {
			t.step(p_dt);
		}
	}
}

void EnemyArms::place(const EnemyRig &p_rig, const EnemyAnim &p_anim, const Transform3D &p_avatar) {
	if (!p_rig.is_built()) {
		return;
	}
	Transform3D av = p_avatar;
	av.basis.orthonormalize();
	for (int s = 0; s < EnemyRig::ARMS; s++) {
		if (!tumb[s].hit_id.is_valid()) {
			continue;
		}
		Transform3D up;
		Transform3D fo;
		p_rig.arm_frames(p_anim, s, up, fo);
		base[s] = av * up;
		CollisionShape3D *fore = Object::cast_to<CollisionShape3D>(ObjectDB::get_instance(fore_id[s]));
		if (fore != nullptr) {
			fore->set_transform(up.affine_inverse() * fo * fore_offset[s]); // До позы тела: тело шлёт формы сразу
		}
		tumb[s].place(base[s]);
	}
}

Quaternion EnemyArms::tilt(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, EnemyRig::ARMS, Quaternion());
	return tumb[p_side].basis().get_rotation_quaternion();
}
