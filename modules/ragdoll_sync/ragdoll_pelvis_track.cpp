#include "ragdoll_pelvis_track.h"

#include "core/math/math_funcs.h"
#include "scene/3d/physics/rigid_body_3d.h"

static Transform3D _call_xf(const Callable &p_fn, int p_space) {
	if (!p_fn.is_valid()) {
		return Transform3D();
	}
	const Variant v = p_fn.call(p_space);
	return v.operator Transform3D();
}

void RagdollSpaceXforms::clear() {
	for (uint8_t &h : have) {
		h = 0;
	}
}

Transform3D RagdollSpaceXforms::get(int p_space) {
	if (p_space < 0) {
		return _call_xf(fn, p_space);
	}
	if ((uint32_t)p_space >= have.size()) {
		const uint32_t from = have.size();
		have.resize((uint32_t)p_space + 1);
		xf.resize((uint32_t)p_space + 1);
		for (uint32_t i = from; i < have.size(); i++) {
			have[i] = 0;
		}
	}
	if (!have[p_space]) {
		xf[p_space] = _call_xf(fn, p_space);
		have[p_space] = 1;
	}
	return xf[p_space];
}

void RagdollPelvisTrack::feed(double p_t, int p_space, const Transform3D &p_xf, bool p_final, const Params &p_params) {
	if (p_space != space) {
		t.clear();
		x.clear();
		space = p_space;
		if (following) {
			catch_t = 0.0; // разница с позой в новом пространстве (мир тот же) тает заново
			catch_pending = true;
		}
	}
	uint32_t i = t.size();
	while (i > 0 && t[i - 1] > p_t) {
		i--;
	}
	t.insert(i, p_t);
	x.insert(i, p_xf);
	fed = true;
	if (p_final) {
		final_t = MAX(final_t, p_t);
	} else if (p_t > final_t) {
		final_t = -1.0; // кукла владельца снова движется
	}
	while (t.size() > 1 && t[0] < t[t.size() - 1] - p_params.keep_s) {
		t.remove_at(0);
		x.remove_at(0);
	}
}

void RagdollPelvisTrack::reset() {
	t.clear();
	x.clear();
	space = -1;
	following = false;
	resting = false;
	lag = -1.0;
	catch_t = -1.0;
	driven = false;
	final_t = -1.0;
	rest_ticks = 0;
}

bool RagdollPelvisTrack::idle(bool p_asleep) const {
	return t.is_empty() || (resting && p_asleep && space == 0 && !fed);
}

bool RagdollPelvisTrack::tick(RigidBody3D *p_pelvis, double p_now, double p_one_way, double p_target,
		RagdollSpaceXforms &p_spaces, bool p_asleep, int p_tps, const Params &p_params) {
	if (t.is_empty()) {
		return false;
	}
	fed = false;
	const double dt = 1.0 / (double)p_tps;
	const double want = MAX(p_target, p_one_way + p_params.buffer_s);
	if (lag < 0.0) {
		lag = want;
	} else if (want < lag) {
		lag = MAX(want, lag - p_params.rate_up * dt);
	} else {
		lag = MIN(want, lag + p_params.rate_down * dt);
	}
	const double seek = p_now + p_one_way - lag;
	if (!following) {
		if (seek < t[0]) {
			return false; // поток ещё не дошёл до момента показа — своя физика
		}
		following = true;
		p_pelvis->set_freeze_enabled(true); // кинематика (FREEZE_MODE_KINEMATIC части): поза — цель шага физики
		catch_t = 0.0;
		catch_pending = true;
	}
	const Transform3D world = p_spaces.get(space) * sample(seek);
	if (catch_t >= 0.0 && catch_pending) { // разница своей позы и буфера — в первый тик следования
		const Transform3D cur = p_pelvis->get_global_transform();
		catch_p = cur.origin - world.origin;
		catch_q = (cur.basis.get_rotation_quaternion() * world.basis.get_rotation_quaternion().inverse()).normalized();
		if (catch_q.w < 0.0) {
			catch_q = -catch_q;
		}
		catch_pending = false;
	}
	Transform3D to = world;
	if (catch_t >= 0.0) {
		const double k = 1.0 - Math::smoothstep(0.0, 1.0, catch_t / p_params.catchup_s);
		to = Transform3D(Basis(Quaternion().slerp(catch_q, (real_t)k)) * world.basis, world.origin + catch_p * (real_t)k);
		catch_t += dt;
		if (k <= 0.0) {
			catch_t = -1.0;
		}
	}
	resting = final_t >= 0.0 && seek >= final_t && catch_t < 0.0;
	bool doze = false;
	if (resting && !p_asleep) {
		rest_ticks++;
		doze = rest_ticks >= (int)(p_params.rest_s * (double)p_tps);
		if (doze) {
			rest_ticks = 0;
		}
	} else {
		rest_ticks = 0;
	}
	if (driven && to.is_equal_approx(last)) {
		prev = last; // скорость таза — 0 (release не толкнёт его старым шагом)
		return doze; // таз стоит — тело не трогается (кукла засыпает)
	}
	prev = driven ? last : to;
	last = to;
	driven = true;
	p_pelvis->set_global_transform(to);
	return doze;
}

Vector3 RagdollPelvisTrack::velocity(int p_tps) const {
	return driven ? (last.origin - prev.origin) * (real_t)p_tps : Vector3();
}

Vector3 RagdollPelvisTrack::angular_velocity(int p_tps) const {
	if (!driven) {
		return Vector3();
	}
	Quaternion q = (last.basis.get_rotation_quaternion() * prev.basis.get_rotation_quaternion().inverse()).normalized();
	if (q.w < 0.0) {
		q = -q;
	}
	const real_t a = q.get_angle();
	return (double)a > 1e-6 ? q.get_axis() * a * (real_t)p_tps : Vector3();
}

Transform3D RagdollPelvisTrack::sample(double p_seek) const {
	const uint32_t n = t.size();
	if (p_seek <= t[0]) {
		return x[0];
	}
	if (p_seek >= t[n - 1]) {
		return x[n - 1];
	}
	uint32_t i = 1;
	while (t[i] < p_seek) {
		i++;
	}
	const double f = (p_seek - t[i - 1]) / MAX(t[i] - t[i - 1], 1e-6);
	return Transform3D(slerp_safe(x[i - 1].basis, x[i].basis, f), x[i - 1].origin.lerp(x[i].origin, (real_t)f));
}

Basis RagdollPelvisTrack::slerp_safe(const Basis &p_from, const Basis &p_to, double p_factor) {
	const Quaternion q1 = p_from.orthonormalized().get_rotation_quaternion().normalized();
	Quaternion q2 = p_to.orthonormalized().get_rotation_quaternion().normalized();
	if (q1.dot(q2) < 0.0) { // кратчайшая дуга
		q2 = -q2;
	}
	return Basis(q1.slerp(q2, (real_t)CLAMP(p_factor, 0.0, 1.0)).normalized());
}
