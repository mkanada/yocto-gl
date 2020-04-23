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
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto/yocto_trace_adp.h>
#include <yocto_gui/yocto_gui.h>
using namespace yocto::math;
namespace sio = yocto::sceneio;
namespace img = yocto::image;
namespace cli = yocto::commonio;
namespace adp = yocto::trace_adp;
namespace trc = yocto::trace;
namespace gui = yocto::gui;

#include <future>
#include <memory>
using namespace std::string_literals;

// Application state
struct app_state {
  // loading options
  std::string filename  = "scene.yaml";
  std::string imagename = "out.png";
  std::string name      = "";

  // options
  adp::adp_params params = {};

  // scene
  trc::scene*              scene        = new trc::scene{};
  trc::camera*             camera       = nullptr;
  std::vector<std::string> camera_names = {};

  // rendering state
  img::image<vec4f> render   = {};
  img::image<vec4f> display  = {};
  float             exposure = 0;

  // view scene
  gui::image*       glimage  = new gui::image{};
  gui::image_params glparams = {};

  // computation
  int             render_sample  = 0;
  int             render_counter = 0;
  adp::state*     render_state   = new adp::state{};
  adp::statistic  stats          = adp::statistic{};
  
  std::future<void>  display_thread = {};
  
  std::string    current_msg = "";

  // status
  std::atomic<float> progress = 0;
  bool               show_image       = true;
  bool               show_samples     = false;
  bool               show_sample_time = false;
  bool               show_q_image     = false;
  bool               highlight_q      = false;
  bool               highlight_prox   = false;

  ~app_state() {
    if (render_state) {
      adp::trace_stop(render_state);
      delete render_state;
    }
    if (scene) delete scene;
    if (glimage) delete glimage;
  }
};

// construct a scene from io
void init_scene(trc::scene* scene, sio::model* ioscene, trc::camera*& camera,
    sio::camera* iocamera, sio::progress_callback print_progress = {}) {
  // handle progress
  auto progress = vec2i{
      0, (int)ioscene->cameras.size() + (int)ioscene->environments.size() +
             (int)ioscene->materials.size() + (int)ioscene->textures.size() +
             (int)ioscene->shapes.size() + (int)ioscene->subdivs.size() +
             (int)ioscene->instances.size() + (int)ioscene->objects.size()};

  auto camera_map     = std::unordered_map<sio::camera*, trc::camera*>{};
  camera_map[nullptr] = nullptr;
  for (auto iocamera : ioscene->cameras) {
    if (print_progress)
      print_progress("convert camera", progress.x++, progress.y);
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
    if (print_progress)
      print_progress("convert texture", progress.x++, progress.y);
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
    if (print_progress)
      print_progress("convert material", progress.x++, progress.y);
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
    if (print_progress)
      print_progress("convert subdiv", progress.x++, progress.y);
    tesselate_subdiv(ioscene, iosubdiv);
  }

  auto shape_map     = std::unordered_map<sio::shape*, trc::shape*>{};
  shape_map[nullptr] = nullptr;
  for (auto ioshape : ioscene->shapes) {
    if (print_progress)
      print_progress("convert shape", progress.x++, progress.y);
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
    if (print_progress)
      print_progress("convert instance", progress.x++, progress.y);
    auto instance = add_instance(scene);
    set_frames(instance, ioinstance->frames);
    instance_map[ioinstance] = instance;
  }

  for (auto ioobject : ioscene->objects) {
    if (print_progress)
      print_progress("convert object", progress.x++, progress.y);
    auto object = add_object(scene);
    set_frame(object, ioobject->frame);
    set_shape(object, shape_map.at(ioobject->shape));
    set_material(object, material_map.at(ioobject->material));
    set_instance(object, instance_map.at(ioobject->instance));
  }

  for (auto ioenvironment : ioscene->environments) {
    if (print_progress)
      print_progress("convert environment", progress.x++, progress.y);
    auto environment = add_environment(scene);
    set_frame(environment, ioenvironment->frame);
    set_emission(environment, ioenvironment->emission,
        texture_map.at(ioenvironment->emission_tex));
  }

  // done
  if (print_progress) print_progress("convert done", progress.x++, progress.y);

  // get camera
  camera = camera_map.at(iocamera);
}

// init camera names
void init_camera_names(std::vector<std::string>& names,
    const std::vector<sio::camera*>&             iocameras) {
  for (auto iocamera : iocameras) {
    names.push_back(iocamera->name);
  }
}

void reset_display(app_state* app) {
  // stop render
  adp::trace_stop(app->render_state);
  if (app->display_thread.valid()) {
    app->display_thread.get();
  }

  // start render
  app->render_counter = 0;
  adp::trace_start(
      app->render_state, app->scene, app->camera, app->params,
      [app](adp::state* state, const std::string& message, float curr_q, float desired_q) {
        app->progress = curr_q / desired_q;
        app->current_msg = message;
      },
      [app](adp::state* state, float curr_q, float desired_q) {
        std::string ioerror = "";
        if (curr_q == desired_q) {
          img::save_image(app->imagename, app->render_state->render, ioerror);
        }
      });
  
  app->display_thread = std::async(std::launch::async, [app]() {
    while(!app->render_state->stop) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      adp::collect_statistics(app->stats, app->render_state);          
      if (app && !app->render_state->stop) {
        if (app->show_image) {
          app->render  = app->render_state->render;
          app->display = tonemap_image(app->render, app->exposure);
        }
        if (app->show_samples) {
          app->display = img::byte_to_float(adp::sample_density_img(app->render_state, app->stats));
        }
        if (app->show_sample_time) {
          app->display = img::byte_to_float(adp::time_density_img(app->render_state));
        }
        if (app->show_q_image) {
          app->display = img::byte_to_float(adp::q_img(app->render_state));
        }
        
        if (app->highlight_prox) {
          for(auto ij : app->render_state->ij_by_proximity) {
            app->display[ij] = { 1, 0, 0, 1 };
          }
        }
        if (app->highlight_q) {
          for(auto ij : app->render_state->ij_by_q) {
            app->display[ij] = { 0.5, 1, 0, 1 };
          }
        }
      }
    }
  });
  
}

int main(int argc, const char* argv[]) {
  // application
  auto app_guard = std::make_unique<app_state>();
  auto app       = app_guard.get();

  // command line options
  auto camera_name = ""s;
  auto add_skyenv  = false;

  // parse command line
  auto cli = cli::make_cli("yscnitraces_adp", "progressive adaptive path tracing");
  add_option(cli, "--quality,-q", app->params.desired_q, "Quality.");  
  add_option(cli, "--camera", camera_name, "Camera name.");
  add_option(
      cli, "--resolution,-r", app->params.trc_params.resolution, "Image resolution.");
  add_option(cli, "--tracer,-t", app->params.trc_params.sampler, "Tracer type.",
      trc::sampler_names);
  add_option(cli, "--falsecolor,-F", app->params.trc_params.falsecolor,
      "Tracer false color type.", trc::falsecolor_names);
  add_option(
      cli, "--bounces,-b", app->params.trc_params.bounces, "Maximum number of bounces.");
  add_option(cli, "--clamp", app->params.trc_params.clamp, "Final pixel clamping.");
  add_option(
      cli, "--filter/--no-filter", app->params.trc_params.tentfilter, "Filter image.");
  add_option(cli, "--env-hidden/--no-env-hidden", app->params.trc_params.envhidden,
      "Environments are hidden in renderer");
  add_option(cli, "--bvh", app->params.trc_params.bvh, "Bvh type", trc::bvh_names);
  add_option(cli, "--skyenv/--no-skyenv", add_skyenv, "Add sky envmap");
  add_option(cli, "--output,-o", app->imagename, "Image output");
  add_option(cli, "scene", app->filename, "Scene filename", true);
  parse_cli(cli, argc, argv);

  // scene loading
  auto ioscene_guard = std::make_unique<sio::model>();
  auto ioscene       = ioscene_guard.get();
  auto ioerror       = ""s;
  if (!load_scene(app->filename, ioscene, ioerror, cli::print_progress))
    cli::print_fatal(ioerror);

  // add sky
  if (add_skyenv) add_sky(ioscene);

  // get camera
  auto iocamera = get_camera(ioscene, camera_name);

  // conversion
  init_scene(app->scene, ioscene, app->camera, iocamera, cli::print_progress);

  // camera names
  init_camera_names(app->camera_names, ioscene->cameras);

  // cleanup
  if (ioscene_guard) ioscene_guard.reset();

  // build bvh
  init_bvh(app->scene, app->params.trc_params, cli::print_progress);

  // init renderer
  init_lights(app->scene, cli::print_progress);

  // fix renderer type if no lights
  if (app->scene->lights.empty() && is_sampler_lit(app->params.trc_params)) {
    cli::print_info("no lights presents, switching to eyelight shader");
    app->params.trc_params.sampler = trc::sampler_type::eyelight;
  }

  // allocate buffers
  reset_display(app);

  // callbacks
  auto callbacks     = gui::ui_callbacks{};
  callbacks.clear_cb = [app](gui::window* win, const gui::input& input) {
    clear_image(app->glimage);
  };
  callbacks.draw_cb = [app](gui::window* win, const gui::input& input) {
    if (!is_initialized(app->glimage)) init_image(app->glimage);
    if (!app->render_counter)
      set_image(app->glimage, app->display, false, false);
    app->glparams.window      = input.window_size;
    app->glparams.framebuffer = input.framebuffer_viewport;
    update_imview(app->glparams.center, app->glparams.scale,
        app->display.size(), app->glparams.window, app->glparams.fit);
    draw_image(app->glimage, app->glparams);
    app->render_counter++;
    if (app->render_counter > 10) app->render_counter = 0;
  };
  callbacks.widgets_cb = [app](gui::window* win, const gui::input& input) {
    auto  edited  = 0;
    draw_progressbar(win, "render", app->progress);
    edited +=draw_slider(win, "desired q", app->params.desired_q, 0.0, 7.0);
    if (edited) reset_display(app);
    draw_label(win, "", app->current_msg);
    draw_label(win, "avg spp",   std::to_string(app->stats.avg_spp));
    draw_label(win, "max spp",   std::to_string(app->stats.max_spp));
    draw_label(win, "q",         std::to_string(app->render_state->curr_q));
    draw_label(win, "desired q", std::to_string(app->params.desired_q));
    
    if (draw_checkbox(win, "show image", app->show_image)) {
      if (app->show_image) {
        app->show_samples     = false;
        app->show_sample_time = false;
        app->show_q_image     = false;
      }
    }
    if (draw_checkbox(win, "show samples", app->show_samples)) {
      if (app->show_samples) {
        app->show_image       = false;
        app->show_sample_time = false;        
        app->show_q_image     = false;
      }
    }
    if (draw_checkbox(win, "show sample time", app->show_sample_time)) {
      if (app->show_sample_time) {
        app->show_image       = false;
        app->show_samples     = false;
        app->show_q_image     = false;
      }
    }
    if (draw_checkbox(win, "show q image", app->show_q_image)) {
      if (app->show_q_image) {
        app->show_image       = false;
        app->show_samples     = false;
        app->show_sample_time = false;
      }
    }
    
    draw_checkbox(win, "show samples by prox", app->highlight_prox);    
    draw_checkbox(win, "show samples by q", app->highlight_q);    
  };
  // run ui
  run_ui({1280 + 320, 720}, "yscnitraces_adp", callbacks);
  

  // done
  return 0;
}
