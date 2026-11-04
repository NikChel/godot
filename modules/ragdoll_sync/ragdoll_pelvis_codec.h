#pragma once

#include "core/math/transform_3d.h"

// Поза таза куклы в потоке владельца (spec docs/superpowers/specs/2026-11-02-walker-ragdoll-design.md §2.7) — перенос
// RagdollPelvisPose.pack / unpack (GDScript) один в один, формат тот же: BYTES на позу — номер куклы (u32), номер
// пространства (u8, зажат в 0..255), точка таза в системе пространства (3 × f32), поворот — smallest-three (2 бита —
// индекс наибольшей компоненты, 3 × QBITS бит — остальные; ошибка поворота ≤ 0.1°). Порядок байтов — little-endian
// (StreamPeerBuffer по умолчанию).
namespace RagdollPelvisCodec {

constexpr int BYTES = 4 + 1 + 12 + 4;
constexpr int QBITS = 10;
constexpr int QMAX = (1 << QBITS) - 1;
constexpr double QRANGE = 0.70710678; // |компонента| без наибольшей ≤ 1/√2

uint32_t encode_rotation(const Quaternion &p_q);
Quaternion decode_rotation(uint32_t p_bits);
// Поза в BYTES байт по адресу w.
void write(uint8_t *w, int64_t p_key, int p_space, const Transform3D &p_xf);
// Поза из BYTES байт по адресу r.
void read(const uint8_t *r, int64_t &r_key, int &r_space, Transform3D &r_xf);

} // namespace RagdollPelvisCodec
