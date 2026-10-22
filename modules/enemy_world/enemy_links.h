#pragma once

#include "core/math/vector3.h"
#include "core/templates/local_vector.h"
#include "core/variant/variant.h"

// Связи карты для тел ходоков (план docs/superpowers/plans/2026-10-22-nav-links-b.md; spec
// docs/superpowers/specs/2026-10-22-enemy-nav-links-design.md §4): таблица (начало, конец, вид — координаты корня
// карты; номер — NavLinkSet.links), занятость (на связи один), закрытие (у ждущего место приземления занято
// CLOSE_S подряд — связь закрыта для новых маршрутов) и открытие (место свободно OPEN_S подряд), движение тела по
// связи (время и точка траектории профиля: прыжок — дуга, залезть — вверх CLIMB_SPEED и вперёд шагом, спрыгнуть —
// вперёд шагом и падение).
class EnemyLinks {
public:
	enum Kind { // = NavLink.Kind
		KIND_DROP,
		KIND_CLIMB,
		KIND_JUMP,
	};

	static constexpr real_t CLOSE_S = 2.0f;
	static constexpr real_t OPEN_S = 1.0f;
	static constexpr real_t CLIMB_SPEED = 2.0f; // м/с вверх (залезть)

	struct Motion { // Движение тела по связи (координаты корня карты)
		int kind = KIND_DROP;
		Vector3 a; // Поза тела на входе
		Vector3 b; // Конец связи
		real_t walk = 3.0f; // Скорость шага тела (вперёд у залезть / спрыгнуть)
		real_t impulse = 4.2f; // Профиль тела: толчок прыжка
		real_t gravity = 9.8f;
		real_t duration = 0.0f;
	};

	void set(const PackedVector3Array &p_starts, const PackedVector3Array &p_ends, const PackedInt32Array &p_kinds);
	bool valid(int p_l) const { return p_l >= 0 && p_l < (int)links.size(); }
	const Vector3 &end(int p_l) const { return links[p_l].end; }
	int kind(int p_l) const { return links[p_l].kind; }
	bool closed(int p_l) const { return links[p_l].closed; }
	int busy(int p_l) const { return links[p_l].busy; }
	void claim(int p_l, int p_body);
	void release(int p_l, int p_body);
	// Ждущий у входа видит занятое место приземления: раз за тик копит время; true — связь закрылась сейчас.
	bool blocked(int p_l, uint64_t p_tick, real_t p_dt);
	void unblocked(int p_l) { links[p_l].blocked_t = 0.0f; }
	const LocalVector<int> &closed_list() const { return closed_ids; }
	// Закрытая связь: место свободно — копит OPEN_S; true — открыта сейчас (убрана из закрытых).
	bool open_tick(int p_l, bool p_free, real_t p_dt);
	int closed_count() const { return (int)closed_ids.size(); }

	static void begin(Motion &r_m); // Время движения (прыжок, который профилю не по силам, — шагом)
	static Vector3 at(const Motion &p_m, real_t p_t); // Точка траектории в момент p_t

private:
	struct Link {
		Vector3 start;
		Vector3 end;
		int kind = KIND_DROP;
		int busy = -1; // Тело на связи
		real_t blocked_t = 0.0f;
		uint64_t blocked_tick = UINT64_MAX;
		bool closed = false;
		real_t open_t = 0.0f;
	};

	LocalVector<Link> links;
	LocalVector<int> closed_ids;
};
