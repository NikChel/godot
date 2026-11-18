#pragma once

#include "enemy_anim.h"

#include "core/math/quaternion.h"
#include "core/math/transform_3d.h"
#include "core/object/ref_counted.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"
#include "scene/resources/animation.h"

class AnimationPlayer;
class Node3D;
class Skeleton3D;

// Риг модели врага (spec docs/superpowers/specs/2026-10-27-walker-arms-design.md §2): клипы библиотеки аватара и
// скелет — один на модель, общий для всех врагов её типа (WalkerAvatar строит при первом аватаре). Выборка локальных
// поз костей по состоянию EnemyAnim (смешивание: поворот — slerp, позиция — lerp; кости без дорожки — покой):
// физ-тик — только цепочка рук (кадры костей плеча и предплечья в системе корня аватара — EnemyArms), кадр —
// все анимированные кости в Skeleton3D (EnemyViews) и наклон руки поверх клипа (поворот в кадре кости плеча).
// Клипы по порядку build: 0 — стойка, 1 — шаг (повторяются), дальше разовые.
// Наклон по ходу (бриф коптера 2026-11-17, шаг 4; set_lean — тип отдаёт отдельно от клипов build): клипы направлений —
// вперёд, назад, влево, вправо (статичные позы, начало клипа); у кости — разница клипа с клипом стойки (начало) в её
// кадре, поверх позы клипов: поворот — доля разницы (slerp от единицы), позиция — доля сдвига; доли — EnemyAnim::lean
// (вперёд / назад и вбок — обе разом: по диагонали смесь). Время входа в наклон — ω пружины EnemyAnim (lean_omega). И в
// тике (руки), и в кадре. Рига без set_lean (ходок) — поза прежняя.
// Вращение (set_spin; читает только глаз): кости вокруг своей оси с темпом на кость (об/с, знак — сторона) — в кадре
// (apply / apply_spin по времени вращения вида EnemyViews), в тике — нет.
class EnemyRig : public RefCounted {
	GDCLASS(EnemyRig, RefCounted);

public:
	static constexpr int ARMS = 2; // 0 — левая, 1 — правая
	enum Lean {
		LEAN_FWD,
		LEAN_BACK,
		LEAN_LEFT,
		LEAN_RIGHT,
		LEANS,
	};
	static constexpr real_t LEAN_95 = 4.744f; // ω·t, при котором пружина с критическим затуханием прошла 95% шага

private:
	struct Clip {
		StringName name;
		Ref<Animation> anim;
		real_t length = 0.0f;
		bool loop = false;
		LocalVector<int> rot; // Дорожка поворота кости (−1 — нет)
		LocalVector<int> pos; // Дорожка позиции кости (−1 — нет)
	};

	LocalVector<int> parent;
	LocalVector<Transform3D> rest;
	LocalVector<Quaternion> rest_rot;
	LocalVector<int> written; // Кости, которые пишет кадр (есть дорожка хоть в одном клипе; плечи рук; вращение)
	LocalVector<uint8_t> has_pos; // Позиция кости анимирована хоть в одном клипе
	LocalVector<uint8_t> anim_bone; // Кость пишет кадр (основа written)
	LocalVector<Clip> clips;
	HashMap<StringName, int> clip_index;
	Transform3D skel_xf; // Скелет в системе корня аватара
	int upper[ARMS] = { -1, -1 }; // Кость плеча
	int fore[ARMS] = { -1, -1 }; // Кость предплечья
	LocalVector<int> chain[ARMS]; // От корня скелета до предплечья
	bool built = false;
	// Наклон (set_lean): разница клипа направления с клипом стойки — по LEANS на кость (поворот, сдвиг, есть ли)
	LocalVector<Quaternion> lean_rot;
	LocalVector<Vector3> lean_pos;
	LocalVector<uint8_t> lean_has;
	real_t lean_w = 0.0f; // ω пружины наклона, 1/с (0 — наклона нет)
	// Вращение (set_spin): кость, темп (об/с), ось (кадр кости)
	LocalVector<int> spin_bone;
	LocalVector<real_t> spin_rate;
	Vector3 spin_axis = Vector3(0.0f, 1.0f, 0.0f);

	bool _load_clip(Node *p_anim_root, Skeleton3D *p_skeleton, AnimationPlayer *p_player, const StringName &p_name, Clip &r_clip);
	void _rebuild_written();
	real_t _clip_time(const Clip &p_c, real_t p_t) const;
	void _clip_pose(const Clip &p_c, real_t p_t, int p_bone, Quaternion &r_rot, Vector3 &r_pos) const;
	void _pose(const EnemyAnim &p_a, int p_bone, Quaternion &r_rot, Vector3 &r_pos) const;
	void _lean(int p_lean, real_t p_w, int p_bone, Quaternion &r_rot, Vector3 &r_pos) const;
	Quaternion _spin(int p_bone, double p_time) const; // Поворот вращения кости (единица — не вращается)

protected:
	static void _bind_methods();

public:
	// p_clips — имена в AnimationPlayer («библиотека/клип»): стойка, шаг, разовые; p_arm_bones — плечо и предплечье
	// левой, плечо и предплечье правой. Корень аватара и скелет — в дереве.
	bool build(Node3D *p_root, Skeleton3D *p_skeleton, AnimationPlayer *p_player, const PackedStringArray &p_clips, const PackedStringArray &p_arm_bones);
	bool is_built() const { return built; }
	int clip_find(const StringName &p_name) const; // −1 — нет
	int get_clip_count() const { return (int)clips.size(); }
	real_t clip_length(int p_clip) const;
	real_t clip_wrap(int p_clip, real_t p_time) const; // Повторяемый — по кругу; разовый — как есть (конец виден)
	// Кадры костей плеча и предплечья руки по анимации (система корня аватара, базисы ортонормированы)
	void arm_frames(const EnemyAnim &p_a, int p_side, Transform3D &r_upper, Transform3D &r_fore) const;
	Transform3D arm_rest(int p_side) const; // Покой кости предплечья в кадре кости плеча
	// Кадр: позы анимированных костей — в скелет; плечу руки — наклон p_tilt[сторона] поверх клипа (кадр кости); костям
	// вращения — поворот на время вращения p_spin_time (с)
	void apply(const EnemyAnim &p_a, const Quaternion p_tilt[ARMS], Skeleton3D *p_skeleton, double p_spin_time = 0.0) const;
	// Кадр без полной позы (уровень вида пропускает кадр): только кости вращения — поза их клипа (покой) × вращение
	void apply_spin(const EnemyAnim &p_a, Skeleton3D *p_skeleton, double p_spin_time) const;
	bool has_spin() const { return !spin_bone.is_empty(); }
	// Наклон по ходу: клипы направлений («библиотека/клип» в p_player) — вперёд, назад, влево, вправо; p_time — вход в
	// наклон до 95%, с. После build. false — нет клипа / кости
	bool set_lean(Skeleton3D *p_skeleton, AnimationPlayer *p_player, const PackedStringArray &p_clips, real_t p_time);
	bool has_lean() const { return lean_w > 0.0f; }
	real_t lean_omega() const { return lean_w; }
	// Вращение костей p_bones вокруг оси p_axis (кадр кости) с темпом p_rates (об/с; знак — сторона). После build
	bool set_spin(Skeleton3D *p_skeleton, const PackedStringArray &p_bones, const Vector3 &p_axis, const PackedFloat32Array &p_rates);
};
