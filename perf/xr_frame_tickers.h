#ifndef XR_FRAME_TICKERS_H
#define XR_FRAME_TICKERS_H

// Branch perf/xr-frame-timers: per-frame serial-segment instrumentation for the
// VR performance investigation (game repo: docs/superpowers/research/
// 2026-09-22-vr-perf-ab.md). Header-only: no build-system changes, no
// behaviour change. Valid for the safe-thread model (default: main thread ==
// render thread; in create_thread mode the fence/present/end segments belong
// to the render thread and the printed frame mixes threads).
//
// Segments (main thread, usec), printed as [XRT] line every 10th frame:
//   wait    - xrWaitFrame (OpenXRAPI::process): align to 120 Hz tick
//   cpu     - physics + _process between XR wait and draw (main loop)
//   draw    - RenderingServer::draw: scene update + draw_viewports +
//             rasterizer end_frame (D3D12 execute) + xrEndFrame
//   fence   -   D3D12 command_queue Wait (GPU-fence of the previous frame)
//   present -   D3D12 swap-chain Present (desktop mirror; the XR swapchain is
//               presented by the Oculus runtime inside xrEndFrame)
//   end     -   xrEndFrame (Oculus runtime: composite + upscale + Link encode)

#include <atomic>

#include "core/os/os.h"
#include "core/string/print_string.h"
#include "core/string/ustring.h"

namespace XRPT {

inline std::atomic<uint64_t> seg_wait{0};
inline std::atomic<uint64_t> seg_cpu{0};
inline std::atomic<uint64_t> seg_draw{0};
inline std::atomic<uint64_t> seg_fence{0};
inline std::atomic<uint64_t> seg_present{0};
inline std::atomic<uint64_t> seg_end{0};
inline std::atomic<int> frame_counter{0};

inline uint64_t now_usec() {
	return (uint64_t)OS::get_singleton()->get_ticks_usec();
}

class Scoper {
	uint64_t t0 = 0;
	std::atomic<uint64_t> *acc = nullptr;

public:
	explicit Scoper(std::atomic<uint64_t> *p_acc) : t0(now_usec()), acc(p_acc) {}
	// Stop (or stop-and-continue) the measurement; returns accumulated us.
	uint64_t stop() {
		if (t0 == 0) {
			return 0;
		}
		uint64_t dt = now_usec() - t0;
		t0 = 0;
		if (acc) {
			acc->fetch_add(dt);
		}
		return dt;
	}
	~Scoper() {
		stop();
	}
};

// End of main-loop frame (call after RenderingServer::draw): prints the [XRT]
// line every 10th frame and resets the accumulators.
inline void frame_done() {
	int n = frame_counter.fetch_add(1) + 1;
	bool emit = (n % 10 == 0);
	double w = seg_wait.load() / 1000.0;
	double c = seg_cpu.load() / 1000.0;
	double d = seg_draw.load() / 1000.0;
	double f = seg_fence.load() / 1000.0;
	double p = seg_present.load() / 1000.0;
	double e = seg_end.load() / 1000.0;
	if (emit) {
		print_line(vformat("[XRT] f=%05d wait=%.1f cpu=%.1f draw=%.1f (fence=%.2f present=%.2f end=%.1f)", n, w, c, d, f, p, e));
	}
	seg_wait = 0;
	seg_cpu = 0;
	seg_draw = 0;
	seg_fence = 0;
	seg_present = 0;
	seg_end = 0;
}

} // namespace XRPT

#endif // XR_FRAME_TICKERS_H
