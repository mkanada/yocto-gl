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
#include <yocto/yocto_shape.h>
using namespace yocto::math;
namespace shp = yocto::shape;
namespace img = yocto::image;
namespace cli = yocto::commonio;

#include "ext/filesystem.hpp"
namespace sfs = ghc::filesystem;

using std::string;
using std::vector;
using namespace std::string_literals;

int main(int argc, const char* argv[]) {
  // command line parameters
  auto smooth    = false;
  auto scale     = vec3f{1};
  auto uscale    = 1.0f;
  auto height    = 1.0f;
  auto rotate    = zero3f;
  auto translate = zero3f;
  auto info      = false;
  auto output    = "out.ply"s;
  auto filename  = "heightfield.png"s;

  // parse command line
  auto cli = cli::make_cli(
      "yheightfieldproc", "Makes a mesh from a heightfield");
  add_option(cli, "--height,-h", height, "Height scale");
  add_option(cli, "--smooth", smooth, "Compute smooth normals");
  add_option(cli, "--rotatey,-ry", rotate.y, "Rotate around y axis");
  add_option(cli, "--rotatex,-rx", rotate.x, "Rotate around x axis");
  add_option(cli, "--rotatez,-rz", rotate.z, "Rotate around z axis");
  add_option(cli, "--translatey,-ty", translate.y, "Translate along y axis");
  add_option(cli, "--translatex,-tx", translate.x, "Translate along x axis");
  add_option(cli, "--translatez,-tz", translate.z, "Translate along z axis");
  add_option(cli, "--scale,-s", uscale, "Scale along xyz axes");
  add_option(cli, "--scaley,-sy", scale.y, "Scale along y axis");
  add_option(cli, "--scalex,-sx", scale.x, "Scale along x axis");
  add_option(cli, "--scalez,-sz", scale.z, "Scale along z axis");
  add_option(cli, "--info,-i", info, "print mesh info");
  add_option(cli, "--output,-o", output, "output mesh");
  add_option(cli, "mesh", filename, "input heightfield", true);
  parse_cli(cli, argc, argv);

  // mesh data
  auto positions = std::vector<vec3f>{};
  auto normals   = std::vector<vec3f>{};
  auto texcoords = std::vector<vec2f>{};
  auto quads     = std::vector<vec4i>{};

  // image data
  auto heightfield = img::image<float>{};

  // load mesh
  auto ioerror = ""s;
  cli::print_progress("load image", 0, 1);
  if (img::is_hdr_filename(filename)) {
    if (!img::load_image(filename, heightfield, ioerror))
      cli::print_fatal(ioerror);
  } else {
    auto heightfield16 = img::image<ushort>{};
    if (!img::load_image(filename, heightfield16, ioerror))
      cli::print_fatal(ioerror);
    heightfield = img::ushort_to_float(heightfield16);
  }
  cli::print_progress("load shape", 1, 1);

  // adjust height
  if (height != 1) {
    for (auto& pixel : heightfield) pixel *= height;
  }

  // create heightfield
  shp::make_heightfield(quads, positions, normals, texcoords,
      heightfield.size(), heightfield.data_vector());
  if (!smooth) normals.clear();

  // print info
  if (info) {
    cli::print_info("shape stats ------------");
    auto stats = shp::shape_stats(
        {}, {}, {}, quads, {}, {}, {}, positions, normals, texcoords, {}, {});
    for (auto& stat : stats) cli::print_info(stat);
  }

  // transform
  if (uscale != 1) scale *= uscale;
  if (translate != zero3f || rotate != zero3f || scale != vec3f{1}) {
    cli::print_progress("transform shape", 0, 1);
    auto xform = translation_frame(translate) * scaling_frame(scale) *
                 rotation_frame({1, 0, 0}, radians(rotate.x)) *
                 rotation_frame({0, 0, 1}, radians(rotate.z)) *
                 rotation_frame({0, 1, 0}, radians(rotate.y));
    for (auto& p : positions) p = transform_point(xform, p);
    for (auto& n : normals)
      n = transform_normal(xform, n, max(scale) != min(scale));
    cli::print_progress("transform shape", 1, 1);
  }

  // save mesh
  cli::print_progress("save shape", 0, 1);
  if (!shp::save_shape(output, {}, {}, {}, quads, positions, normals, texcoords,
          {}, {}, ioerror))
    cli::print_fatal(ioerror);
  cli::print_progress("save shape", 1, 1);

  // done
  return 0;
}
