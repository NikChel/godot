#include "enemy_anim.h"

#include "enemy_rig.h"

#include "core/math/math_funcs.h"

real_t EnemyAnim::walk_rate(real_t p_speed) {
	const real_t s = Math::abs(p_speed);
	const real_t k = MAX(s - BLEND_SPEED, (real_t)0.0) / (RUN_SPEED - BLEND_SPEED);
	const real_t r = MIN(Math::lerp((real_t)1.0, RUN_RATE, k), MAX_RATE);
	return p_speed < 0.0f ? -r : r;
}

// Новый клип с начала; уходящим становится текущий. Смешивание ещё идёт и текущий легче уходящего — уходящий
// остаётся (скачок — только у лёгкого клипа).
void EnemyAnim::play(int p_clip, real_t p_blend, bool p_action) {
	if (p_blend > 0.0f) {
		if (from < 0 || weight() >= 0.5f) {
			from = clip;
			from_time = time;
			from_rate = rate;
		}
		blend = 0.0f;
		blend_len = p_blend;
	} else {
		from = -1;
	}
	clip = p_clip;
	time = 0.0f;
	rate = 1.0f;
	acting = p_action;
}

void EnemyAnim::step(const EnemyRig &p_rig, real_t p_dt, real_t p_speed) {
	*this = advanced(p_dt);
	time = p_rig.clip_wrap(clip, time);
	if (from >= 0) {
		from_time = p_rig.clip_wrap(from, from_time);
	}
	if (acting) {
		if (time < p_rig.clip_length(clip)) {
			return;
		}
		play(Math::abs(p_speed) > WALK_ON ? CLIP_WALK : CLIP_STAND, CLIP_BLEND, false);
	} else {
		const int c = Math::abs(p_speed) > (clip == CLIP_WALK ? WALK_OFF : WALK_ON) ? CLIP_WALK : CLIP_STAND;
		if (c != clip) {
			play(c, CLIP_BLEND, false);
		}
	}
	rate = clip == CLIP_WALK ? walk_rate(p_speed) : 1.0f;
}

EnemyAnim EnemyAnim::advanced(real_t p_dt) const {
	EnemyAnim a = *this;
	a.time += p_dt * rate;
	if (from >= 0) {
		a.from_time += p_dt * from_rate;
		a.blend += p_dt;
		if (a.blend >= a.blend_len) {
			a.from = -1;
		}
	}
	return a;
}

real_t EnemyAnim::weight() const {
	if (from < 0 || blend_len <= 0.0f) {
		return 1.0f;
	}
	return CLAMP(blend / blend_len, (real_t)0.0, (real_t)1.0);
}
