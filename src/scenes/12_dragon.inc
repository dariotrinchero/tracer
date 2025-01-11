#ifndef SCENE_INC
#define SCENE_INC

void build_scene(HittableList& scene, HittableList&, Camera& cam) {
	// checkered sphere floor
	auto checker = make_shared<CheckerTexture>(30.0, Color(.2, .21, .27), Color(.9, .9, .9));
	scene.add(make_shared<Sphere>(Point3(-5000, 0, 0), 5000, make_shared<Lambertian>(checker)));

	// dragon
	shared_ptr<Material> mat = make_shared<Metal>(Color(0.65, 0.21, 0.16), 0.3);
	scene.add(Mesh("dragon.stl", mat).get_model());

	// camera
	cam.center = Point3(150, 270, 70);
	cam.facing = Point3(90, 0, 0);
	cam.v_up = Vec3(1, 0, 0);

#ifdef DEBUG_TRACER // really low-fidelity debug, as render is slow
	cam.subpixel_grid_size = 3;
	cam.max_depth = 3;
#endif

	cam.focus_dist = 285;
	cam.defocus_angle = 0.6;
}

#endif
