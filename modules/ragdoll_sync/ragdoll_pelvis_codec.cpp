#include "ragdoll_pelvis_codec.h"

#include "core/io/marshalls.h"
#include "core/math/math_funcs.h"

namespace RagdollPelvisCodec {

// Арифметика — в double, как у GDScript (PackedFloat64Array компонент, roundi).
uint32_t encode_rotation(const Quaternion &p_q) {
	const double c[4] = { (double)p_q.x, (double)p_q.y, (double)p_q.z, (double)p_q.w };
	int big = 0;
	for (int i = 0; i < 4; i++) {
		if (Math::abs(c[i]) > Math::abs(c[big])) {
			big = i;
		}
	}
	const double s = c[big] < 0.0 ? -1.0 : 1.0; // q и −q — один поворот: наибольшая — положительная
	int64_t bits = big;
	for (int i = 0; i < 4; i++) {
		if (i == big) {
			continue;
		}
		const int64_t u = (int64_t)Math::round((CLAMP(c[i] * s / QRANGE, -1.0, 1.0) * 0.5 + 0.5) * QMAX);
		bits = (bits << QBITS) | u;
	}
	return (uint32_t)bits;
}

Quaternion decode_rotation(uint32_t p_bits) {
	const int64_t bits = p_bits;
	const int big = (int)((bits >> (3 * QBITS)) & 3);
	double c[4] = { 0.0, 0.0, 0.0, 0.0 };
	double sum = 0.0;
	int shift = 2 * QBITS;
	for (int i = 0; i < 4; i++) {
		if (i == big) {
			continue;
		}
		const double v = (((double)((bits >> shift) & QMAX) / QMAX) - 0.5) * 2.0 * QRANGE;
		c[i] = v;
		sum += v * v;
		shift -= QBITS;
	}
	c[big] = Math::sqrt(MAX(1.0 - sum, 0.0));
	return Quaternion((real_t)c[0], (real_t)c[1], (real_t)c[2], (real_t)c[3]).normalized();
}

void write(uint8_t *w, int64_t p_key, int p_space, const Transform3D &p_xf) {
	encode_uint32((uint32_t)p_key, w);
	w[4] = (uint8_t)CLAMP(p_space, 0, 255);
	encode_float((float)p_xf.origin.x, w + 5);
	encode_float((float)p_xf.origin.y, w + 9);
	encode_float((float)p_xf.origin.z, w + 13);
	encode_uint32(encode_rotation(p_xf.basis.get_rotation_quaternion()), w + 17);
}

void read(const uint8_t *r, int64_t &r_key, int &r_space, Transform3D &r_xf) {
	r_key = (int64_t)decode_uint32(r);
	r_space = r[4];
	const Vector3 o(decode_float(r + 5), decode_float(r + 9), decode_float(r + 13));
	r_xf = Transform3D(Basis(decode_rotation(decode_uint32(r + 17))), o);
}

} // namespace RagdollPelvisCodec
