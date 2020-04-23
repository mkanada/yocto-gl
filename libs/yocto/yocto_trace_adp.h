//
// # Yocto/Trace: Tiny path tracer
//
//
// Yocto/Trace is a simple path tracing library with support for microfacet
// materials, area and environment lights, and advacned sampling.
//
//
// ## Physically-based Path Tracing
//
// Yocto/Trace includes a tiny, but fully featured, path tracer with support for
// textured mesh area lights, GGX materials, environment mapping. The algorithm
// makes heavy use of MIS for fast convergence.
// The interface supports progressive parallel execution both synchronously,
// for CLI applications, and asynchronously for interactive viewing.
//
// Materials are represented as sums of an emission term, a diffuse term and
// a specular microfacet term (GGX or Phong), and a transmission term for
// this sheet glass.
// Lights are defined as any shape with a material emission term. Additionally
// one can also add environment maps. But even if you can, you might want to
// add a large triangle mesh with inward normals instead. The latter is more
// general (you can even more an arbitrary shape sun). For now only the first
// environment is used.
//
// 1. prepare the ray-tracing acceleration structure with `build_bvh()`
// 2. prepare lights for rendering with `init_trace_lights()`
// 3. create the random number generators with `init_trace_state()`
// 4. render blocks of samples with `trace_samples()`
// 5. you can also start an asynchronous renderer with `trace_asynch_start()`
//
//

//
// LICENSE:
//
// Copyright (c) 2016 -- 2020 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//

#ifndef _YOCTO_TRACE_ADP_H_
#define _YOCTO_TRACE_ADP_H_

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------

#include <atomic>
#include <future>
#include <memory>

#include "yocto_image.h"
#include "yocto_matht.h"
#include "yocto_trace.h"


#ifdef YOCTO_EMBREE
#include <embree3/rtcore.h>
#endif

namespace yocto::trace_adp {

// Namespace aliases
namespace trc = yocto::trace;
namespace img = yocto::image;

// Math defitions
using math::bbox3f;
using math::byte;
using math::frame3f;
using math::identity3x4f;
using math::ray3f;
using math::rng_state;
using math::vec2f;
using math::vec2i;
using math::vec3b;
using math::vec3f;
using math::vec3d;
using math::vec3i;
using math::vec4f;
using math::vec4i;
using math::vec4b;
using math::vec4d;


// State of a pixel during tracing
struct trace_info {
  vec3d     radiance = {0, 0, 0};
  int       hits     = 0;
  int       samples  = 0;
};

struct pixel {
  trace_info actual         = {};
  trace_info odd            = {};
  rng_state  rng            = {};
  double     q              = -1.0f;
  int        sample_budget  = 0;
  long       time_in_sample = 0;
};

struct statistic {
  long       samples    = 0;
  long       pixels     = 0;
  float      min_q      = std::numeric_limits<float>::max();
  float      max_q      = -std::numeric_limits<float>::max();
  int        min_spp    = 0;
  double     avg_spp    = 0;
  int        max_spp    = 0;
  std::string stat_text = "";
};

using time_point = std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<long, std::ratio<1, 1000000000>>>;

// [experimental] Asynchronous state
struct state {
  img::image<vec4f>  render          = {};
  img::image<vec4f>  odd_render      = {};
  img::image<pixel>  pixels          = {};
  float              min_q           = -1.0f;
  float              curr_q          = -1.0f;
  time_point         start_time      = {};
  long               sample_count    = 0;
  std::vector<vec2i> ij_by_q         = {};
  std::vector<vec2i> ij_by_proximity = {};
  std::atomic<bool>  stop            = {};
  std::future<void>  worker          = {};  
};
  
struct adp_params {
  trc::trace_params trc_params      = {};
  int               min_samples     = 32;
  int               max_samples     = 262144;
  int               sample_step     = 8;
  float             desired_q       = 5.0f;
  int               desired_spp     = 0;
  int               desired_seconds = 0;
  float             step_q          = 1.0/4.0;
  float             batch_step      = 1;
  bool              save_final_log  = false;
};


using batch_callback = std::function<void(state*, float curr_q, float desired_q)>;
using progress_callback = std::function<void(state*, std::string msg, int actual, int final)>;    
    
img::image<vec4f> trace_image(state* state, const trc::scene* scene,
    const trc::camera* camera, const adp_params& params,
    progress_callback progress_cb = {},
    batch_callback batch_cb = {});

// Format duration std::string from nanoseconds
inline std::string format_q_step(float current_q, float final_q) {
  char buffer[256];
  
  if (final_q > 0) {
    if (current_q >= 0) {
      sprintf(buffer, "%03d-%03d", (int) (current_q * 100.0f), (int) (final_q * 100.0f));
    } else {
      sprintf(buffer, "----%03d", (int) (final_q * 100.0f));
    }
  } else {
      sprintf(buffer, "----%03d", (int) (final_q * 100.0f));
  }
  
  return buffer;
}

inline int get_max_progress(const adp_params& params) {
  if (params.desired_seconds > 0) {
    return params.desired_seconds * 1000;
  }
  if (params.desired_spp > 0) {
    return params.desired_spp * 1000;
  }
  if (params.desired_q > 0) {
    return params.desired_q * 1000;
  }
  return 100;
}

inline int get_actual_progress(const state* state, const adp_params& params) {
  if (state->curr_q == -2) {
    // first status;
    return 0;
  }
  
  if (params.desired_seconds == 0 && params.desired_seconds == 0 && state->curr_q == -1) {
    // in initial_samples
    return 1;
  }
  
  if (params.desired_seconds > 0) {
    auto elapsed = std::chrono::system_clock::now() - state->start_time;
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    
    return milliseconds - 1;
  }
  
  if (params.desired_spp > 0) {
    auto img_size = state->render.size().x * state->render.size().y;
    auto image_spp = double(state->sample_count) / double(img_size);
    return int(image_spp * 1000) - 1;
  }
    
  if (state->curr_q == 0) {
    return 2;
  }
  
  if (state->curr_q > 0) {
    return (state->curr_q * 1000) - 1;
  }
  return 1;
}

img::image<vec4b> sample_density_img(const state* state, statistic& stat);
img::image<vec4b> q_img(const state* state);
img::image<vec4b> time_density_img(const state* state);
void collect_statistics(statistic& stat, const state* state);

void trace_start(state* state, const trc::scene* scene,
    const trc::camera* camera, const adp_params& params,
    progress_callback progress_cb = {}, batch_callback image_cb = {});


void trace_stop(state* state);

}

#endif
