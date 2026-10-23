#pragma once

#include "nav_column_index.h"

#include "core/math/math_funcs.h"
#include "core/math/transform_3d.h"
#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"
#include "scene/resources/navigation_mesh.h"
#include "servers/navigation_3d/navigation_path_query_parameters_3d.h"
#include "servers/navigation_3d/navigation_path_query_result_3d.h"
#include "servers/physics_3d/physics_server_3d.h"

// Связи сетки навигации карты (spec docs/superpowers/specs/2026-10-22-enemy-nav-links-design.md §3.1–§3.2; порт
// NavNarrow.gd и NavLinkGen.gd, AGENTS §2.9). prepare — чистые данные (можно в потоке WorkerThreadPool): узкие места из
// сетки без запаса от краёв (многоугольник, у которого центр, кольцо RING_N точек R RING_R и вершины, сдвинутые к
// центру на VERT_IN, не накрыты основной сеткой по вертикали ±COLUMN_H), объединённая сетка (основная, затем узкая —
// номера основной прежние), индекс колонн по ней, связные куски узкой сетки (общие вершины), точки краёв основной.
// step — главный поток, кусками не дольше бюджета за вызов: на точку края (нормаль наружу) лучи вниз вперёд через
// SCAN_STEP до SCAN_MAX → приземление на сетке (индекс колонн: накрыта или край не дальше SNAP_H, высота ±SNAP_V) →
// одно на уровень (±STEP_H), ближнее: вид, кому из профилей по силам, тело (капсула, cast_motion), не пешком рядом
// (путь, поиск ограничен длиной), повторы. Узкие куски обходятся, только когда на них ведёт принятая связь, — до
// неподвижной точки. Координаты — корня карты; физика — в мире по позе корня на каждом шаге.
class EnemyLinkGen : public RefCounted {
	GDCLASS(EnemyLinkGen, RefCounted);

public:
	enum Kind { // = NavLink.Kind
		KIND_DROP,
		KIND_CLIMB,
		KIND_JUMP,
	};

	static constexpr real_t RING_R = 0.6f;
	static constexpr int RING_N = 8;
	static constexpr real_t VERT_IN = 0.1f;
	static constexpr real_t COLUMN_H = 0.5f; // = EnemyWorld::COLUMN_H (окно колонны толпы)
	static constexpr real_t SAMPLE_STEP = 0.5f;
	static constexpr real_t SCAN_STEP = 0.25f;
	static constexpr real_t SCAN_MAX = 6.0f;
	static constexpr real_t RAY_UP = 1.8f; // Луч вниз — от высоты края + столько (выше залезания)
	static constexpr real_t RAY_DOWN = 200.0f;
	static constexpr real_t STEP_H = 0.35f; // Перепад не больше — шаг ногами (= NavWorld.agent_max_climb)
	static constexpr real_t NEAR_D = 1.0f; // Ближе — спрыгнуть / залезть у края; дальше — прыжок
	static constexpr real_t CLIMB_FAR_D = 2.0f; // Прыжок не по силам — залезть и дальше NEAR_D (круглая / скошенная стена), без провала под ходом
	static constexpr real_t SNAP_H = 0.35f; // Край сетки дальше от попадания луча — мимо сетки
	static constexpr real_t SNAP_V = 0.3f;
	static constexpr real_t SHORTCUT_MUL = 2.0f; // Пешком не длиннее max(MUL × по прямой, по прямой + EXTRA) — связь не нужна
	static constexpr real_t SHORTCUT_EXTRA = 2.0f;
	static constexpr real_t REACH_TOL = 0.3f;
	static constexpr real_t DEDUP_R = 0.9f; // Связь того же вида с началом и концом ближе — повтор
	static constexpr int DEDUP_WINDOW = 32; // Повторы ищутся среди последних принятых (соседние точки края)
	static constexpr int ARC_STEPS = 10;
	static constexpr real_t BODY_R = 0.3f; // Тело ходока для проверки (ноги — шар R 0.32)
	static constexpr real_t BODY_H = 1.7f;
	static constexpr real_t LIFT = 0.1f; // Тело над точкой траектории (не цеплять пол)
	static constexpr uint32_t WORLD_MASK = 1;

	// Подготовка (поток): full — сетка без запаса от краёв, main — основная. Сбрасывает связи и счётчики.
	void prepare(const Ref<NavigationMesh> &p_full, const Ref<NavigationMesh> &p_main);
	Ref<NavigationMesh> get_narrow() const { return narrow; }
	Ref<NavigationMesh> get_merged() const { return merged; }
	Vector3 get_probe_point() const { return probe_point; } // Центр первого узкого многоугольника (регион в карте?)

	void clear_profiles();
	void add_profile(int p_layer_bit, real_t p_max_drop, real_t p_jump_impulse, real_t p_jump_speed, real_t p_gravity, real_t p_climb, bool p_narrow);
	// После prepare и профилей, регионы в карте: walk_layers — слои регионов для пешего пути (связей в карте ещё нет).
	void start(RID p_map, int p_walk_layers);
	// Кусок генерации не дольше бюджета (по шагам вперёд от точки края); true — готово.
	bool step(Object *p_space, const Transform3D &p_root, int p_budget_usec);

	int get_link_count() const { return (int)links.size(); }
	PackedVector3Array get_starts() const;
	PackedVector3Array get_ends() const;
	PackedInt32Array get_kinds() const;
	PackedInt32Array get_layers() const;
	PackedInt32Array get_start_narrow() const;
	PackedVector3Array get_trajectory(int p_index) const;

	int get_samples() const { return samples; }
	int get_candidates() const { return candidates; } // Кому-то по силам
	int get_shortcuts() const { return shortcuts; } // Отброшено: рядом пешком
	int get_blocked() const { return blocked; } // Отброшено: тело задевает
	int get_narrow_scanned() const { return narrow_scanned; } // Узких многоугольников обойдено (достижимые куски)
	int get_steps() const { return steps; }
	int64_t get_prepare_usec() const { return (int64_t)prepare_usec; }
	int64_t get_gen_usec() const { return (int64_t)gen_usec; }
	int64_t get_max_step_usec() const { return (int64_t)max_step_usec; }

	~EnemyLinkGen();

protected:
	static void _bind_methods();

private:
	struct Profile {
		uint32_t bit = 0;
		real_t max_drop = 0.0f;
		real_t impulse = 0.0f;
		real_t speed = 0.0f;
		real_t gravity = 9.8f;
		real_t climb = 0.0f;
		bool walks_narrow = false;

		real_t apex() const { return impulse * impulse / (2.0f * gravity); }
		// Время полёта прыжка до перепада dh (на спуске дуги); -1 — не допрыгнуть.
		real_t land_time(real_t p_dh) const {
			const real_t disc = impulse * impulse - 2.0f * gravity * p_dh;
			if (impulse <= 0.0f || disc < 0.0f) {
				return -1.0f;
			}
			return (impulse + Math::sqrt(disc)) / gravity;
		}
		real_t reach(real_t p_dh) const {
			const real_t t = land_time(p_dh);
			return t > 0.0f ? speed * t : 0.0f;
		}
		bool can(int p_kind, real_t p_dh, real_t p_dist, bool p_touches_narrow) const {
			if (p_touches_narrow && !walks_narrow) {
				return false;
			}
			switch (p_kind) {
				case KIND_DROP:
					return -p_dh <= max_drop;
				case KIND_CLIMB:
					return p_dh <= climb;
				case KIND_JUMP:
					return p_dh <= apex() - 0.05f && p_dist <= reach(p_dh);
				default:
					return false;
			}
		}
	};

	struct Sample {
		Vector3 p;
		Vector3 n; // Нормаль края наружу (горизонтальная)
		bool narrow = false;
	};

	struct Link {
		Kind kind = KIND_DROP;
		Vector3 start;
		Vector3 end;
		uint32_t layers = 0;
		bool start_narrow = false;
		LocalVector<Vector3> traj;
	};

	Ref<NavigationMesh> narrow;
	Ref<NavigationMesh> merged;
	NavColumnIndex index; // Объединённая сетка
	int main_polys = 0; // Номера узких в объединённой — с этого
	Vector3 probe_point;
	LocalVector<int> comp; // Кусок (корень) на узкий многоугольник
	LocalVector<uint8_t> comp_reached; // По корню: на кусок ведёт принятая связь
	LocalVector<uint8_t> comp_scanned;
	LocalVector<Sample> queue; // Точки краёв: основная, затем достижимые узкие куски
	uint32_t cursor = 0;
	int scan_k = 0; // Шаг вперёд текущей точки края (0 — точка не начата): бюджет — по шагам, точка продолжается в следующем step
	int scan_n = 0; // Шагов вперёд на точку (start)
	bool done = false;

	LocalVector<Profile> profiles;
	bool can_jump = false;
	RID map;
	RID capsule;
	Ref<NavigationPathQueryParameters3D> query;
	Ref<NavigationPathQueryResult3D> query_result;
	LocalVector<Link> links;
	LocalVector<real_t> levels; // Уровни приземления точки края (рабочий)
	LocalVector<Vector3> traj; // Траектория кандидата (рабочая)

	PhysicsDirectSpaceState3D *space = nullptr; // Только на время step
	Transform3D root;
	Transform3D root_inv;

	int samples = 0;
	int candidates = 0;
	int shortcuts = 0;
	int blocked = 0;
	int narrow_scanned = 0;
	int steps = 0;
	uint64_t prepare_usec = 0;
	uint64_t gen_usec = 0;
	uint64_t max_step_usec = 0;

	static Vector3 _centroid(const Vector<Vector3> &p_verts, const Vector<int> &p_poly);
	static bool _near_main(const NavColumnIndex &p_main, const Vector<Vector3> &p_verts, const Vector<int> &p_poly);
	static Ref<NavigationMesh> _rebuild(const Ref<NavigationMesh> &p_src, const Vector<Vector3> &p_verts, const Vector<Vector<int>> &p_polys);
	static Ref<NavigationMesh> _merge(const Ref<NavigationMesh> &p_main, const Ref<NavigationMesh> &p_narrow);
	static uint64_t _edge_key(int p_a, int p_b);
	static int _kind(real_t p_dh, real_t p_dist);
	static void _trajectory(int p_kind, const Vector3 &p_a, const Vector3 &p_b, const Profile &p_prof, LocalVector<Vector3> &r_out);

	int _root_of(int p_i);
	void _components();
	void _add_samples(const Ref<NavigationMesh> &p_mesh, bool p_narrow, const LocalVector<uint8_t> *p_only);
	void _add_edge(const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_center, bool p_narrow);
	bool _expand_narrow();
	void _scan_step(const Sample &p_s, int p_k);
	bool _level_taken(real_t p_y) const;
	bool _ground(const Vector3 &p_q, real_t p_base_y, Vector3 &r_hit) const;
	bool _no_gap(const Vector3 &p_from, const Vector3 &p_to) const;
	uint32_t _layers(int p_kind, real_t p_dh, real_t p_dist, bool p_touches_narrow) const;
	const Profile &_strongest(uint32_t p_layers) const;
	bool _clear(const LocalVector<Vector3> &p_traj) const;
	bool _walkable(const Vector3 &p_from, const Vector3 &p_to);
	bool _repeat(int p_kind, const Vector3 &p_from, const Vector3 &p_to) const;
};
