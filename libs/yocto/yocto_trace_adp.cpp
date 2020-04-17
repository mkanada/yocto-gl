//
// Implementation for Yocto/Trace.
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

#include "yocto_trace_adp.h"

#include <atomic>
#include <cstring>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
using namespace std::string_literals;

#ifdef YOCTO_EMBREE
#include <embree3/rtcore.h>
#endif

namespace yocto::trace_adp {

// import math symbols for use
using math::abs;
using math::acos;
using math::atan2;
using math::clamp;
using math::cos;
using math::exp;
using math::flt_max;
using math::fmod;
using math::fresnel_conductor;
using math::fresnel_dielectric;
using math::identity3x3f;
using math::invalidb3f;
using math::log;
using math::make_rng;
using math::max;
using math::min;
using math::pif;
using math::pow;
using math::rng_state;
using math::sample_discrete_cdf;
using math::sample_discrete_cdf_pdf;
using math::sample_uniform;
using math::sample_uniform_pdf;
using math::sign;
using math::sin;
using math::sqrt;
using math::zero2f;
using math::zero2i;
using math::zero3f;
using math::zero3i;
using math::zero4f;
using math::zero4i;
using math::zero4b;

}  // namespace yocto::trace


namespace yocto::trace_adp {
  
void init_state(state* state, const trc::scene* scene,
    const trc::camera* camera, const trc::trace_params& params) {
  auto image_size =
      (camera->film.x > camera->film.y)
          ? vec2i{params.resolution,
                (int)round(params.resolution * camera->film.y / camera->film.x)}
          : vec2i{
                (int)round(params.resolution * camera->film.x / camera->film.y),
                params.resolution};
  if (!state->stop) state->start_time = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  if (!state->stop) state->pixels.assign(image_size, pixel{});
  if (!state->stop) state->render.assign(image_size, zero4f);
  if (!state->stop) state->odd_render.assign(image_size, zero4f);
  auto rng = make_rng(1301081);
  for (auto& pixel : state->pixels) {
    pixel.rng = make_rng(params.seed, rand1i(rng, 1 << 31) / 2 + 1);
    if (state->stop) break;
  }
}  
  
// Trace a block of samples
void trace_sample(state* state, const trc::scene* scene,
    const trc::camera* camera, const vec2i& ij, const int num_samples, 
    const adp_params& params) {
  auto& pixel   = state->pixels[ij];
  
  int samples = num_samples;
  
  if ((pixel.actual.samples + num_samples) > params.max_samples) {
    samples = params.max_samples - pixel.actual.samples;
  }
  
  for(auto s = 0; s < samples; s++) {
    auto sampler = trc::get_trace_sampler_func(params.trc_params);
    
    if (!state->stop.load()) {
      auto  ray = trc::sample_camera(camera, ij, state->pixels.size(), rand2f(pixel.rng),
        rand2f(pixel.rng), params.trc_params.tentfilter);
      auto [radiance, hit] = sampler(scene, ray, pixel.rng, params.trc_params);
    
      if (!hit) {
        if (params.trc_params.envhidden || scene->environments.empty()) {
          radiance = zero3f;
          hit      = false;
        } else {
          hit = true;
        }
      }
      if (!isfinite(radiance)) radiance = zero3f;
      if (max(radiance) >= params.trc_params.clamp)
        radiance = radiance * (params.trc_params.clamp / max(radiance));

      pixel.actual.radiance = { pixel.actual.radiance.x + radiance.x, 
                                pixel.actual.radiance.y + radiance.y, 
                                pixel.actual.radiance.z + radiance.z};
      pixel.actual.hits += hit ? 1 : 0;
      pixel.actual.samples += 1;
  
      if (pixel.actual.samples % 2 == 1) {
        pixel.odd.radiance = { pixel.odd.radiance.x + radiance.x, 
                               pixel.odd.radiance.y + radiance.y, 
                               pixel.odd.radiance.z + radiance.z};
        pixel.odd.hits += hit ? 1 : 0;
        pixel.odd.samples += 1;
      }
    }
    
    if (state->stop.load()) {
      return;
    }
  }
  
  state->render[ij]     = {pixel.actual.hits ? 
    vec3f{float(pixel.actual.radiance.x / pixel.actual.hits), 
          float(pixel.actual.radiance.y / pixel.actual.hits), 
          float(pixel.actual.radiance.z / pixel.actual.hits)} : zero3f,
                                   (float)pixel.actual.hits / (float)pixel.actual.samples};
  state->odd_render[ij] = {pixel.odd.hits ? 
    vec3f{float(pixel.odd.radiance.x / pixel.odd.hits), 
          float(pixel.odd.radiance.y / pixel.odd.hits), 
          float(pixel.odd.radiance.z / pixel.odd.hits)} : zero3f,
                                   (float)pixel.odd.hits / (float)pixel.odd.samples};
                                   
  if (pixel.actual.samples < params.max_samples) {
                                   
    vec4f srgb     = math::rgb_to_srgb(state->render[ij]);
    vec4f srgb_odd = math::rgb_to_srgb(state->odd_render[ij]);
  
    double err_px = 0;
    double div = sqrt(srgb.x + srgb.y + srgb.z);
           
    if (div >= 0.0001f) {
      err_px = (double) (abs(double(srgb.x) - double(srgb_odd.x)) + abs(double(srgb.y) - double(srgb_odd.y)) + abs(double(srgb.z) - double(srgb_odd.z))) / (double) sqrt(double(srgb.x) + double(srgb.y) + double(srgb.z));
    } else {
      err_px = (double) (abs(double(srgb.x) - double(srgb_odd.x)) + abs(double(srgb.y) - double(srgb_odd.y)) + abs(double(srgb.z) - double(srgb_odd.z))) / 0.01;
    }

    pixel.q = -math::log2(err_px);
    if (pixel.q > 10) {
      pixel.q = 10;
    }
  } else {
    pixel.q = 10;
  }
}

void trace_until_quality(state* state, const trc::scene* scene,
    const trc::camera* camera, const vec2i& ij, const adp_params& params, float q) {
  auto& pixel   = state->pixels[ij];
  
  trace_sample(state, scene, camera, ij, params.sample_step, params);
  if (state->stop.load()) {
    return;
  }
  
  while(pixel.q < q) {
    trace_sample(state, scene, camera, ij, params.sample_step, params);
    if (state->stop.load()) {
      return;
    }
  }    
}

void trace_by_budget(state* state, const trc::scene* scene,
    const trc::camera* camera, const vec2i& ij, const adp_params& params) {
  auto& pixel   = state->pixels[ij];
  
  trace_sample(state, scene, camera, ij, pixel.sample_budget, params);
  pixel.sample_budget = 0;
}

void trace_by_budget_or_q_below(state* state, const trc::scene* scene,
    const trc::camera* camera, const vec2i& ij, const adp_params& params, float step_q) {
  auto& pixel   = state->pixels[ij];

  int sample_max = pixel.actual.samples + pixel.sample_budget;
  while (pixel.actual.samples < sample_max && pixel.q >= step_q) {
    trace_sample(state, scene, camera, ij, params.sample_step, params);
    if (state->stop.load()) {
      return;
    }
  }    
  pixel.sample_budget = 0;
}


struct sample_spread {
  signed char   x = 0;
  signed char   y = 0;
  float       div = 0;
};

void create_sample_spread(std::vector<sample_spread> &spread_vec, const float step_q) {  
  int radius = 10;
  if(step_q <= 1.99) {
    radius = 4;
  } else if (step_q <= 3.99) {
    radius = 2;
  } else {
    radius = 1;
  }

  spread_vec.clear();
  for(auto i = -radius; i <= radius; i++) {
    for(auto j = -radius; j <= radius; j++) {
      
      if (i == 0 && j == 0) continue;
      
      sample_spread spread = {};
      spread.x = i;
      spread.y = j;
      spread.div = 1;
      if (radius == 1) {
        spread_vec.emplace_back(spread);
      } else {
        float dist = sqrtf((float) (i * i) + (j * j));
        if (dist <= radius) spread_vec.emplace_back(spread);
      }
    }
  }
}


std::vector<vec2i> all_image_ij(state* state) {
  std::vector<vec2i> to_return = std::vector<vec2i>{};

  auto size = state->render.size();
  
  for(int j = 0; j < size.y; j++) {
    for(int i = 0; i < size.x; i++) {
      to_return.push_back({i, j});
    }
  }
  
  return to_return;
}

template <typename Func>
inline void parallel_pixels_in_list(const state* state_ptr, const std::vector<vec2i>& ij_list, Func&& func) {
  auto  nthreads     = std::thread::hardware_concurrency();
  auto  futures      = std::vector<std::future<void>>(nthreads);
  std::atomic<int> next_idx(0);
  
    // sample cada pixel selecionado
    futures.clear();
    next_idx.store(0);
    for (auto thread_id = 0; thread_id < nthreads; thread_id++) {
      if (state_ptr->stop) {
         break;
      }
      
      futures.emplace_back(
          std::async(std::launch::async, [state_ptr, &next_idx, &ij_list, &func]() {
            while(true) {
              if (state_ptr->stop) break;
              auto idx = next_idx.fetch_add(1);
              if (idx >= ij_list.size()) break;
              func(ij_list[idx]);
            }
        }));
    }
    
    for (auto& f : futures) f.get();
}

void collect_statistics(statistic& stat, const state* state) {
  auto size = state->render.size();
  
  int    samples     = 0;
  int    pixels      = 0;
  float  min_q       = std::numeric_limits<float>::max();
  float  max_q       = -std::numeric_limits<float>::max();
  int    min_spp     = std::numeric_limits<int>::max();
  double avg_spp     = 0;
  int    max_spp     = 0;
  
  for(auto i = 0; i < size.x; i++) {
    for(auto j = 0; j < size.y; j++) {
      auto& pixel = state->pixels[{i,j}];
      
      samples += pixel.actual.samples;
      pixels++;
      
      if (pixel.q < min_q) {
        min_q = pixel.q;
      }
      
      if (pixel.q > max_q) {
        max_q = pixel.q;
      }
        
      if (pixel.actual.samples < min_spp) {
        min_spp = pixel.actual.samples;
      }
      
      if (pixel.actual.samples > max_spp) {
        max_spp = pixel.actual.samples;
      }
      
      if (state->stop.load()) {
        return;
      }
    }
  }
  
  avg_spp = (double) samples / (double) pixels;
  
  stat.samples        = samples;
  stat.pixels         = pixels;
  stat.min_q          = min_q;
  stat.max_q          = max_q;
  stat.min_spp        = min_spp;
  stat.avg_spp        = avg_spp;
  stat.max_spp        = max_spp;
  
  
  static auto pad = [](const std::string& str, int n) -> std::string {
    return std::string(std::max(0, n - (int)str.size()), '0') + str;
  };
  
  auto elapsed = std::chrono::high_resolution_clock::now().time_since_epoch().count() - state->start_time;
  elapsed /= 1000000;  // millisecs
  auto mins  = pad(std::to_string(elapsed / 60000), 2);
  auto secs  = pad(std::to_string((elapsed % 60000) / 1000), 2);
  auto msecs = pad(std::to_string((elapsed % 60000) % 1000), 3);
  
  
  stat.stat_text = "    Current q: " + std::to_string(state->curr_q) + "\n" +
                   "      min_spp: " + std::to_string(stat.min_spp) + "\n" +
                   "      avg_spp: " + std::to_string(stat.avg_spp) + "\n" +
                   "      max_spp: " + std::to_string(stat.max_spp) + "\n" +
                   "sampling time: " + mins + ":" + secs + "." + msecs + "\n";  
}

img::image<vec4f> trace_image(state* state_ptr, const trc::scene* scene, const trc::camera* camera, const adp_params& params,
                              progress_callback progress_cb, batch_callback batch_cb) {
  
  std::unique_ptr<state> state_guard;
  
  if (state_ptr == nullptr) {
    state_guard = std::make_unique<state>();
    state_ptr = state_guard.get();
  }

  std::vector<sample_spread> spread_vec;
  float step_q  =  0.0f;
  state_ptr->curr_q = -1.0f;
  
  
  // Somewhat expensive actions.
  init_state(state_ptr, scene, camera, params.trc_params);
  spread_vec = std::vector<sample_spread>();
  create_sample_spread(spread_vec, step_q);
  
  vec2i size = state_ptr->render.size();

  if (progress_cb) progress_cb(state_ptr, "initial samples", state_ptr->curr_q, params.desired_q);

  // initial samples
  if (state_ptr->stop) return state_ptr->render;

  for(auto sampled = 0; sampled < params.min_samples; sampled += params.sample_step) {
    parallel_pixels_in_list(state_ptr, all_image_ij(state_ptr), [state_ptr, scene, camera, &params](const vec2i& ij) {
        if (!state_ptr->stop) trace_sample(state_ptr, scene, camera, ij, params.sample_step, params);
    });
  }
  
  
  if (batch_cb) batch_cb(state_ptr, state_ptr->curr_q, params.desired_q);
  float next_batch = state_ptr->curr_q + params.batch_step;
  while (state_ptr->min_q < params.desired_q) {
    if (state_ptr->stop) return state_ptr->render;
    
    // select pixels that are below the actual quality step.
    state_ptr->ij_by_q.clear();    
    for(int j = 0; j < size.y; j++) {
      for(int i = 0; i < size.x; i++) {
        auto& pixel = state_ptr->pixels[{i,j}];
        pixel.sample_budget = 0;
        if (pixel.q < step_q) {
          state_ptr->ij_by_q.push_back({i, j});
        }
        
        if (state_ptr->stop) return state_ptr->render;
        
      }      
    }
    
    // trace samples for each pixel until it reaches the actual quality step.
    if (progress_cb) progress_cb(state_ptr, "samples by quality", state_ptr->curr_q, params.desired_q);
    parallel_pixels_in_list(state_ptr, state_ptr->ij_by_q, [state_ptr, scene, camera, &params, step_q](const vec2i& ij) {
              trace_until_quality(state_ptr, scene, camera, ij, params, step_q);
    });

    // here is supposed that every pixel in image has quality > step_q
    // trace pixels in neighborhood until pixel budget is reached or if quality drops below step_q to
    // restart de process
    state_ptr->ij_by_proximity.clear();
    // find pixels near pixels sampled by quality using indexes definded in 'sample_spread' vector.
    for(auto& ij_sampled : state_ptr->ij_by_q) {
      auto& pixel = state_ptr->pixels[ij_sampled];
      
      for(auto& neigh_idx : spread_vec) {
        int   k   = ij_sampled.x + neigh_idx.x;
        int   l   = ij_sampled.y + neigh_idx.y;
        float div = neigh_idx.div;
        
        if (k >= 0 && l >= 0 && k < size.x && l < size.y) {          
          auto& pix_neighbor = state_ptr->pixels[{k, l}];
          
          if ((pix_neighbor.actual.samples + pix_neighbor.sample_budget) < (pixel.actual.samples / div)) {
            pix_neighbor.sample_budget = ((float) pixel.actual.samples / div) - pix_neighbor.actual.samples;
          }
        }
        
        if (state_ptr->stop) return state_ptr->render;        
      }
    }
    
    // Now, find every pixel with budget > 0
    for(int j = 0; j < size.y; j++) {
      for(int i = 0; i < size.x; i++) {
        auto& pixel = state_ptr->pixels[{i,j}];
        if (pixel.sample_budget > 0) {
          state_ptr->ij_by_proximity.push_back({i, j});
        }
        
        if (state_ptr->stop) return state_ptr->render;
      }
    }
    
    
    // trace samples for each pixel near pixels sampled by quality
    if (progress_cb) progress_cb(state_ptr, "samples by proximity", state_ptr->curr_q, params.desired_q);
    parallel_pixels_in_list(state_ptr, state_ptr->ij_by_proximity, [state_ptr, scene, camera, &params](const vec2i& ij) {
            trace_by_budget(state_ptr, scene, camera, ij, params);
    });    

    // collect important statistis..
    float tmp_min_q = ::std::numeric_limits<float>::max();        
    for(int j = 0; j < size.y; j++) {
      for(int i = 0; i < size.x; i++) {
        auto& pixel = state_ptr->pixels[{i, j}];
      
        if (tmp_min_q > pixel.q) {
          tmp_min_q = pixel.q;
        }
        
        if (state_ptr->stop) return state_ptr->render;
      }
    }
    
    state_ptr->min_q = tmp_min_q;
    if (state_ptr->min_q >= step_q) {
      state_ptr->curr_q = step_q;
      
      if (state_ptr->curr_q >= next_batch) {
        if (batch_cb) batch_cb(state_ptr, state_ptr->curr_q, params.desired_q);
        next_batch = state_ptr->curr_q + params.batch_step;
      }
      step_q += params.step_q;
      create_sample_spread(spread_vec, step_q);
      
      if (step_q > params.desired_q) {
        step_q = params.desired_q;
      }
    }
  }
  
  if (!state_ptr->stop.load() && progress_cb) {
    progress_cb(state_ptr, "samples by proximity", state_ptr->curr_q, params.desired_q);
  }
  if (!state_ptr->stop.load() && batch_cb) batch_cb(state_ptr, state_ptr->curr_q, params.desired_q);
  
  return state_ptr->render;
}

img::image<vec4b> sample_density_img(const state* state, statistic& stat) {
  img::image<vec4b> img = {};
  img.assign({state->render.size().x, state->render.size().y}, math::vec4b{0, 0, 0, 255});

  float step = 255.0f / sqrt(float(stat.max_spp - stat.min_spp));
  
  for(auto i = 0; i < state->render.size().x; i++) {
    for(auto j = 0; j < state->render.size().y; j++) {
      auto& px = state->pixels[{i,j}];
      byte  p = byte(sqrt(px.actual.samples - stat.min_spp) * step);
      img[{i,j}] = vec4b{p, p, p, 255};
    }
  }

  return img;
}

img::image<vec4b> q_img(const state* state) {
  img::image<vec4b> img = {};
  img.assign({state->render.size().x, state->render.size().y}, math::vec4b{ 0, 0, 0, 255});

  float step = 20.0f;
  
  for(auto i = 0; i < state->render.size().x; i++) {
    for(auto j = 0; j < state->render.size().y; j++) {
      auto& px = state->pixels[{i,j}];
      int p = int(px.q * step);
      if (p > 255) p = 255;
      img[{i,j}] = vec4b{byte(p), byte(p), byte(p), 255};
    }
  }

  return img;
}

void trace_start(state* state, const trc::scene* scene,
    const trc::camera* camera, const adp_params& params,
    progress_callback progress_cb, batch_callback image_cb) {

  state->stop.store(false);
  
  state->worker = std::async(std::launch::async, [=]() {    
    trace_image(state, scene, camera, params, progress_cb, image_cb);
  });
}

void trace_stop(state* state) {
  if (!state) return;
  state->stop.store(true);
  if (state->worker.valid()) {
    state->worker.get();
  }
}

}  // namespace yocto::trace_adp
