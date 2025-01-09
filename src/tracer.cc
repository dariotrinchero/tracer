#include <stdexcept>
#include <csignal>

#include "hittable.h"
#include "material.h"
#include "camera.h"
#include "sphere.h"
#include "shape2d.h"
#include "volume.h"
#include "mesh.h"
#include "bvh.h"

void register_interrupt_handler();

/* --- scene selection -------------------------------------------------------------------------- */

#include "scenes/07_cornell_box.inc"
// TODO update all other scenes to construct lights also

// default to empty scene if none is loaded
#ifndef SCENE_INC
void build_scene(HittableList&, HittableList&, Camera&) {}
#endif

/* --- main method ------------------------------------------------------------------------------ */

int main(int argc, char* argv[]) {
	register_interrupt_handler();

	// set up default camera
	Camera cam;
	cam.aspect_ratio = 16.0 / 9.0;
	cam.vert_fov = 20;
	cam.center = Point3(0, 0, 1);
	cam.facing = Point3(0, 0, 0);
	cam.v_up = Vec3(0, 1, 0);
#ifdef DEBUG_TRACER // fast low quality render
	cam.image_width = 400;
	cam.subpixel_grid_size = 10;
	cam.max_depth = 40;
#else // slow high quality render
	cam.image_width = 1200;
	cam.subpixel_grid_size = 23;
	cam.max_depth = 50;
#endif

	// build scene
	HittableList scene;
	HittableList lights;
	build_scene(scene, lights, cam);
	BVHNode scene_tree(scene);

	// begin render
	int start_pixel = 0;
	if (argc > 1) { // resume prior render if start pixel given
		try {
			start_pixel = std::stoi(argv[1]);
			std::clog << "Resuming render from pixel: " << start_pixel << '\n';
		} catch (std::invalid_argument&) {
			std::cerr << "ERROR: Invalid starting pixel.\n";
			exit(EXIT_FAILURE);
		}
	}
	cam.render(scene_tree, *lights.objects[0], start_pixel); // TODO TEMPORARY until HittableList supports pdf_value()
	//cam.render(scene_tree, lights, start_pixel);
}

/* --- helper methods --------------------------------------------------------------------------- */

void register_interrupt_handler() {
	struct sigaction sig_int_handler;
	sig_int_handler.sa_handler = [](int) {
		std::cout.flush();
		std::clog << "\n\033[93mResume interrupted render:\n"
			<< "> \033[2m./tracer $(($(wc -l < render.ppm) - 3)) >> render.ppm\n";
		exit(EXIT_SUCCESS);
	};
	sigemptyset(&sig_int_handler.sa_mask);
	sig_int_handler.sa_flags = 0;
	sigaction(SIGINT, &sig_int_handler, NULL);
}
