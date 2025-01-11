#ifndef SCENE_INC
#define SCENE_INC

void build_scene(HittableList& scene, HittableList& lights, Camera& cam) {
	// floor of randomly-displaced boxes
	HittableList boxes1;
	auto ground = make_shared<Lambertian>(Color(0.48, 0.83, 0.53));
	int boxes_per_side = 20;

	for (int i = 0; i < boxes_per_side; i++) {
		for (int j = 0; j < boxes_per_side; j++) {
			auto w = 100.0;
			auto x0 = -1000.0 + i*w;
			auto z0 = -1000.0 + j*w;
			auto y0 = 0.0;
			auto x1 = x0 + w;
			auto y1 = rnd_double(1, 101);
			auto z1 = z0 + w;

			boxes1.add(box(Point3(x0, y0, z0), Point3(x1, y1, z1), ground));
		}
	}

	scene.add(make_shared<BVHNode>(boxes1));

	// ceiling light
	auto light = make_shared<DiffuseLight>(Color(7, 7, 7));
	auto light_quad = make_shared<Parallelogram>(Point3(123, 554, 147), Vec3(300, 0, 0), Vec3(0, 0, 265), light);
	scene.add(light_quad);
	lights.add(light_quad);

	// moving matte sphere
	auto center1 = Point3(400, 400, 200);
	auto center2 = center1 + Vec3(30, 0, 0);
	auto sphere_material = make_shared<Lambertian>(Color(0.7, 0.3, 0.1));
	scene.add(make_shared<Sphere>(center1, center2, 50, sphere_material));

	// glass & metal spheres
	scene.add(make_shared<Sphere>(Point3(260, 150, 45), 50, make_shared<Dielectric>(1.5)));
	scene.add(make_shared<Sphere>(Point3(0, 150, 145), 50, make_shared<Metal>(Color(0.8, 0.8, 0.9), 1.0)));

	// subsurface material = volume inside dielectric
	auto boundary = make_shared<Sphere>(Point3(360, 150, 145), 70, make_shared<Dielectric>(1.5));
	scene.add(boundary);
	scene.add(make_shared<ConstantMedium>(boundary, 0.2, Color(0.2, 0.4, 0.9)));

	// smoke filling room
	boundary = make_shared<Sphere>(Point3(0, 0, 0), 5000, make_shared<Dielectric>(1.5));
	scene.add(make_shared<ConstantMedium>(boundary, .0001, Color(1, 1, 1)));

	// globe
	auto emat = make_shared<Lambertian>(make_shared<ImageTexture>("earthmap.ppm"));
	scene.add(make_shared<Sphere>(Point3(400, 200, 400), 100, emat));

	// noisy banded sphere
	auto pertext = make_shared<NoiseTexture>(0.2);
	scene.add(make_shared<Sphere>(Point3(220, 280, 300), 80, make_shared<Lambertian>(pertext)));

	// box of randomly-located spheres
	HittableList boxes2;
	auto white = make_shared<Lambertian>(Color(.73, .73, .73));
	for (int j = 0; j < 1000; j++) boxes2.add(make_shared<Sphere>(rnd_vec(0, 165), 10, white));

	shared_ptr<Hittable> boxes2_trns = make_shared<BVHNode>(boxes2);
	boxes2_trns = make_shared<Rotate>(boxes2_trns, Axis::Y, 15 * PI / 180);
	boxes2_trns = make_shared<Translate>(boxes2_trns, Vec3(-100, 270, 395));
	scene.add(boxes2_trns);

	// camera settings
	cam.aspect_ratio       = 1.0;
	cam.image_width        = 800;
#ifdef DEBUG_TRACER
	cam.subpixel_grid_size = 5;
#else
	cam.subpixel_grid_size = 100;
#endif

	cam.set_solid_bg(black);

	cam.center   = Point3(478, 278, -600);
	cam.facing   = Point3(278, 278, 0);
	cam.shutter_speed = 1.0;
}

#endif
