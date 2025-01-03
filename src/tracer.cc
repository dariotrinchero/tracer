#include <stdexcept>

#include "hittable.h"
#include "material.h"
#include "camera.h"
#include "sphere.h"
#include "shape2d.h"
#include "volume.h"
#include "mesh.h"

// scene selection
#include "scenes/12_dragon.inc"

// default to empty scene if none is loaded
#ifndef SCENE_INC
void build_scene(HittableList&, Camera&) {}
#endif

int main(int argc, char* argv[]) {
	// default camera parameters
	Camera cam;
	cam.aspect_ratio = 16.0 / 9.0;
	cam.vert_fov = 20;

	cam.center = Point3(0, 0, 1);
	cam.facing = Point3(0, 0, 0);
	cam.v_up = Vec3(0, 1, 0);

#ifdef DEBUG_TRACER
	// fast low-res debug render
	cam.image_width = 400;
	cam.samples_per_pixel = 100;
	cam.max_depth = 40;
#else
	// high quality render
	cam.image_width = 1200;
	cam.samples_per_pixel = 500;
	cam.max_depth = 50;
#endif

	// build scene & render
	HittableList scene;
	build_scene(scene, cam);

	int start_pixel = 0;
	if (argc > 1) {
		// resume prior render if starting pixel given
		try {
			start_pixel = std::stoi(argv[1]);
		} catch (std::invalid_argument&) {
			std::cerr << "ERROR: Invalid starting pixel.\n";
			exit(EXIT_FAILURE);
		}
	}
	cam.render(scene, start_pixel);
}
