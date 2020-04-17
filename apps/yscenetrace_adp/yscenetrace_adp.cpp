//
// LICENSE:
//
// Copyright (c) 2016 -- 2020 Fabio Pellacini
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <yocto/yocto_commonio.h>
#include <yocto/yocto_image.h>
#include <yocto/yocto_matht.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_trace_adp.h>
using namespace yocto::math;
namespace sio = yocto::sceneio;
namespace img = yocto::image;
namespace cli = yocto::commonio;
namespace adp = yocto::trace_adp;
namespace trc = yocto::trace;

#include <map>
#include <memory>
#include <thread>
using namespace std::string_literals;

#include "ext/filesystem.hpp"
namespace sfs = ghc::filesystem;

// construct a scene from io
void init_scene(trc::scene* scene, sio::model* ioscene, trc::camera*& camera,
    sio::camera* iocamera, sio::progress_callback progress_cb = {}) {
  // handle progress
  auto progress = vec2i{
      0, (int)ioscene->cameras.size() + (int)ioscene->environments.size() +
             (int)ioscene->materials.size() + (int)ioscene->textures.size() +
             (int)ioscene->shapes.size() + (int)ioscene->subdivs.size() +
             (int)ioscene->instances.size() + (int)ioscene->objects.size()};

  auto camera_map     = std::unordered_map<sio::camera*, trc::camera*>{};
  camera_map[nullptr] = nullptr;
  for (auto iocamera : ioscene->cameras) {
    if (progress_cb) progress_cb("convert camera", progress.x++, progress.y);
    auto camera = add_camera(scene);
    set_frame(camera, iocamera->frame);
    set_lens(camera, iocamera->lens, iocamera->aspect, iocamera->film,
        iocamera->orthographic);
    set_focus(camera, iocamera->aperture, iocamera->focus);
    camera_map[iocamera] = camera;
  }

  auto texture_map     = std::unordered_map<sio::texture*, trc::texture*>{};
  texture_map[nullptr] = nullptr;
  for (auto iotexture : ioscene->textures) {
    if (progress_cb) progress_cb("convert texture", progress.x++, progress.y);
    auto texture = add_texture(scene);
    if (!iotexture->colorf.empty()) {
      set_texture(texture, iotexture->colorf);
    } else if (!iotexture->colorb.empty()) {
      set_texture(texture, iotexture->colorb);
    } else if (!iotexture->scalarf.empty()) {
      set_texture(texture, iotexture->scalarf);
    } else if (!iotexture->scalarb.empty()) {
      set_texture(texture, iotexture->scalarb);
    }
    texture_map[iotexture] = texture;
  }

  auto material_map     = std::unordered_map<sio::material*, trc::material*>{};
  material_map[nullptr] = nullptr;
  for (auto iomaterial : ioscene->materials) {
    if (progress_cb) progress_cb("convert material", progress.x++, progress.y);
    auto material = add_material(scene);
    set_emission(material, iomaterial->emission,
        texture_map.at(iomaterial->emission_tex));
    set_color(
        material, iomaterial->color, texture_map.at(iomaterial->color_tex));
    set_specular(material, iomaterial->specular,
        texture_map.at(iomaterial->specular_tex));
    set_ior(material, iomaterial->ior);
    set_metallic(material, iomaterial->metallic,
        texture_map.at(iomaterial->metallic_tex));
    set_transmission(material, iomaterial->transmission, iomaterial->thin,
        iomaterial->trdepth, texture_map.at(iomaterial->transmission_tex));
    set_roughness(material, iomaterial->roughness,
        texture_map.at(iomaterial->roughness_tex));
    set_opacity(
        material, iomaterial->opacity, texture_map.at(iomaterial->opacity_tex));
    set_thin(material, iomaterial->thin);
    set_normalmap(material, texture_map.at(iomaterial->normal_tex));
    set_scattering(material, iomaterial->scattering, iomaterial->scanisotropy,
        texture_map.at(iomaterial->scattering_tex));
    material_map[iomaterial] = material;
  }

  for (auto iosubdiv : ioscene->subdivs) {
    if (progress_cb) progress_cb("convert subdiv", progress.x++, progress.y);
    tesselate_subdiv(ioscene, iosubdiv);
  }

  auto shape_map     = std::unordered_map<sio::shape*, trc::shape*>{};
  shape_map[nullptr] = nullptr;
  for (auto ioshape : ioscene->shapes) {
    if (progress_cb) progress_cb("convert shape", progress.x++, progress.y);
    auto shape = add_shape(scene);
    set_points(shape, ioshape->points);
    set_lines(shape, ioshape->lines);
    set_triangles(shape, ioshape->triangles);
    set_quads(shape, ioshape->quads);
    set_positions(shape, ioshape->positions);
    set_normals(shape, ioshape->normals);
    set_texcoords(shape, ioshape->texcoords);
    set_colors(shape, ioshape->colors);
    set_radius(shape, ioshape->radius);
    set_tangents(shape, ioshape->tangents);
    shape_map[ioshape] = shape;
  }

  auto instance_map     = std::unordered_map<sio::instance*, trc::instance*>{};
  instance_map[nullptr] = nullptr;
  for (auto ioinstance : ioscene->instances) {
    if (progress_cb) progress_cb("convert instance", progress.x++, progress.y);
    auto instance = add_instance(scene);
    set_frames(instance, ioinstance->frames);
    instance_map[ioinstance] = instance;
  }

  for (auto ioobject : ioscene->objects) {
    if (progress_cb) progress_cb("convert object", progress.x++, progress.y);
    auto object = add_object(scene);
    set_frame(object, ioobject->frame);
    set_shape(object, shape_map.at(ioobject->shape));
    set_material(object, material_map.at(ioobject->material));
    set_instance(object, instance_map.at(ioobject->instance));
  }

  for (auto ioenvironment : ioscene->environments) {
    if (progress_cb)
      progress_cb("convert environment", progress.x++, progress.y);
    auto environment = add_environment(scene);
    set_frame(environment, ioenvironment->frame);
    set_emission(environment, ioenvironment->emission,
        texture_map.at(ioenvironment->emission_tex));
  }

  // done
  if (progress_cb) progress_cb("convert done", progress.x++, progress.y);

  // get camera
  camera = camera_map.at(iocamera);
}

void save_batch_img(std::string imfilename, img::image<vec4f> img, std::string part_name, float step_q, float max_q) {
  auto str_q_step = adp::format_q_step(step_q, max_q);
  auto ext = part_name + "." + str_q_step + sfs::path(imfilename).extension().string();
  auto actual_filename = sfs::path(imfilename).replace_extension(ext).string();            
  auto ioerror = ""s;
        
  if (!save_image(actual_filename, img, ioerror))
          cli::print_fatal(ioerror);
}

void save_batch_img(std::string imfilename, img::image<vec4b> img, std::string part_name, float step_q, float max_q) {
  auto str_q_step = adp::format_q_step(step_q, max_q);
  auto ext = part_name + "." + str_q_step + sfs::path(imfilename).extension().string();
  auto actual_filename = sfs::path(imfilename).replace_extension(ext).string();            
  auto ioerror = ""s;
  
  if (!save_image(actual_filename, img, ioerror))
          cli::print_fatal(ioerror);
}

void save_log(std::string imfilename, std::string log) {
  auto log_filename = sfs::path(imfilename).replace_extension("log").string();            
  auto ioerror = ""s;

  auto previous_log = ""s;
  auto new_log = ""s;

  if (!yocto::commonio::load_text(log_filename, previous_log, ioerror)) {
    previous_log = ""s;
  }

  new_log = previous_log + log;
        
  if (!yocto::commonio::save_text(log_filename, new_log, ioerror))
      cli::print_fatal(ioerror);
}

void print_status(adp::state* state, std::string current_action, float curr_q, float desired_q) {
  char buffer[255];
  auto stats = adp::statistic{};
  
  collect_statistics(stats, state);
  
  sprintf(buffer, "tracing. spp: %8.2f, q: %4.2f, %s", stats.avg_spp, curr_q, current_action.c_str());
  int actual = 0;
  if (curr_q <= 0) {
    actual = 1;
  } else {
    actual = curr_q * 100;
  }
  
  int total = desired_q * 100;
  
  cli::print_progress(buffer, actual, total);    
}



int main(int argc, const char* argv[]) {
  // options
  auto params      = adp::adp_params{};
  auto save_batch  = false;
  auto add_skyenv  = false;
  auto camera_name = ""s;
  auto imfilename  = "out.hdr"s;
  auto filename    = "scene.json"s;

  // parse command line
  auto cli = cli::make_cli("yscntrace_adp", " Offline adaptive path tracing");
  add_option(cli, "--quality,-q", params.desired_q, "Quality.");
  add_option(cli, "--camera", camera_name, "Camera name.");
  add_option(cli, "--resolution,-r", params.trc_params.resolution, "Image resolution.");
  add_option(
      cli, "--tracer,-t", params.trc_params.sampler, "Trace type.", trc::sampler_names);
  add_option(cli, "--falsecolor,-F", params.trc_params.falsecolor,
      "Tracer false color type.", trc::falsecolor_names);
  add_option(cli, "--bounces,-b", params.trc_params.bounces, "Maximum number of bounces.");
  add_option(cli, "--clamp", params.trc_params.clamp, "Final pixel clamping.");
  add_option(cli, "--filter/--no-filter", params.trc_params.tentfilter, "Filter image.");
  add_option(cli, "--env-hidden/--no-env-hidden", params.trc_params.envhidden,
      "Environments are hidden in renderer");
  add_option(cli, "--save-batch", save_batch, "Save images progressively");
  add_option(cli, "--bvh", params.trc_params.bvh, "Bvh type", trc::bvh_names);
  add_option(cli, "--skyenv/--no-skyenv", add_skyenv, "Add sky envmap");
  add_option(cli, "--output-image,-o", imfilename, "Image filename");
  add_option(cli, "scene", filename, "Scene filename", true);
  parse_cli(cli, argc, argv);

  // scene loading
  auto ioscene_guard = std::make_unique<sio::model>();
  auto ioscene       = ioscene_guard.get();
  auto ioerror       = ""s;
  if (!load_scene(filename, ioscene, ioerror, cli::print_progress))
    cli::print_fatal(ioerror);

  // add sky
  if (add_skyenv) add_sky(ioscene);

  // get camera
  auto iocamera = get_camera(ioscene, camera_name);

  // convert scene
  auto scene_guard = std::make_unique<trc::scene>();
  auto scene       = scene_guard.get();
  auto camera      = (trc::camera*)nullptr;
  init_scene(scene, ioscene, camera, iocamera, cli::print_progress);

  // cleanup
  if (ioscene_guard) ioscene_guard.reset();

  // build bvh
  trc::init_bvh(scene, params.trc_params, cli::print_progress);

  // init renderer
  trc::init_lights(scene, cli::print_progress);

  // fix renderer type if no lights
  if (scene->lights.empty() && is_sampler_lit(params.trc_params)) {
    cli::print_info("no lights presents, switching to eyelight shader");
    params.trc_params.sampler = trc::sampler_type::eyelight;
  }

  std::atomic<bool> running  = false;
  adp::state* last_state     = nullptr;
  float       last_curr_q    = 0;
  float       last_desired_q = 0;
  std::string last_act       = {};
  
  running.store(true);
  
  cli::print_progress("tracing...", 0, 100);
  
  auto print_state = [&last_state, &last_curr_q, &last_desired_q, &last_act]
    (adp::state* state, std::string current_action, float curr_q, float desired_q) {
    
      last_state     = state;
      last_curr_q    = curr_q;
      last_desired_q = desired_q;
      last_act       = current_action;
    
      print_status(last_state, last_act, last_curr_q, last_desired_q);
  };
  
  std::thread([&last_state, &last_curr_q, &last_desired_q, &last_act, &running]() {
    while(true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      if (running.load() == false) {
        break;
      }
      if (last_state != nullptr) {
        print_status(last_state, last_act, last_curr_q, last_desired_q);
      }
    }
  }).detach();

  // render
  auto render = adp::trace_image(nullptr, scene, camera, params, print_state,
      [save_batch, imfilename](const adp::state* state, float curr_q, float desired_q) {
        if (!save_batch) return;
                                 
        adp::statistic stat = {};
        
        adp::collect_statistics(stat, state);
                                 
        save_batch_img(imfilename, state->render, "actual", curr_q, desired_q);
        save_batch_img(imfilename, adp::sample_density_img(state, stat), "spl", curr_q, desired_q);
        save_batch_img(imfilename, adp::q_img(state), "q", curr_q, desired_q);
        save_log(imfilename, stat.stat_text);
      });
  
  running.store(false);  

  // save image
  cli::print_progress("save image", 0, 1);
  if (!save_image(imfilename, render, ioerror)) cli::print_fatal(ioerror);
  cli::print_progress("save image", 1, 1);

  // done
  return 0;
}
