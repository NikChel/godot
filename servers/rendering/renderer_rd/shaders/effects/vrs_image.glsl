#[compute]

#version 450

#VERSION_DEFINES

// Shading rate image for APIs that take one 2D image per pass (D3D12): a render target can't be a shading rate
// image, so it's written here instead of by vrs.glsl; with several views it holds the finest rate of all of them.

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

#ifdef SOURCE_ARRAY
layout(set = 0, binding = 0) uniform sampler2DArray source_density;
#else
layout(set = 0, binding = 0) uniform sampler2D source_density;
#endif

layout(r8ui, set = 1, binding = 0) uniform restrict writeonly uimage2D dest_rate;

layout(push_constant, std430) uniform Params {
	ivec2 size;
	int view_count;
	float max_texel_factor;
}
params;

void main() {
	ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(pos, params.size))) {
		return;
	}
	vec2 uv = (vec2(pos) + 0.5) / vec2(params.size);

	// Input is standardized. R for X, G for Y, 0.0 (0) = 1, 0.33 (85) = 2, 0.66 (170) = 3, 1.0 (255) = 8
#ifdef SOURCE_ARRAY
	vec2 density = vec2(1.0);
	for (int v = 0; v < params.view_count; v++) {
		density = min(density, textureLod(source_density, vec3(uv, float(v)), 0.0).rg);
	}
#else
	vec2 density = textureLod(source_density, uv, 0.0).rg;
#endif

	// Output according to D3D12_SHADING_RATE (same encoding as VK_KHR_fragment_shading_rate).
	vec2 rate = clamp(floor(density * params.max_texel_factor + 0.1), 0.0, params.max_texel_factor);
	// Note 1x4, 4x1, 1x8, 8x1, 2x8 and 8x2 are not supported:
	if (rate.x < (rate.y - 1.0)) {
		rate.x = rate.y - 1.0;
	}
	if (rate.y < (rate.x - 1.0)) {
		rate.y = rate.x - 1.0;
	}
	imageStore(dest_rate, pos, uvec4((uint(rate.x + 0.1) << 2) | uint(rate.y + 0.1)));
}
