#ifndef SCENE_INC
#define SCENE_INC

void build_scene(HittableList& scene, HittableList& lights, Camera& cam) {
	// materials
	auto red   = make_shared<Lambertian>(Color(.65, .05, .05));
	auto white = make_shared<Lambertian>(Color(.73, .73, .73));
	auto green = make_shared<Lambertian>(Color(.12, .45, .15));
	auto light = make_shared<DiffuseLight>(Color(15, 15, 15));

	// Cornell box sides
	scene.add(make_shared<Parallelogram>(Point3(555, 0, 0), Vec3(0, 0, 555), Vec3(0, 555, 0), green));
	scene.add(make_shared<Parallelogram>(Point3(0, 0, 555), Vec3(0, 0, -555), Vec3(0, 555, 0), red));
	scene.add(make_shared<Parallelogram>(Point3(0, 555, 0), Vec3(555, 0, 0), Vec3(0, 0, 555), white));
	scene.add(make_shared<Parallelogram>(Point3(0, 0, 555), Vec3(555, 0, 0), Vec3(0, 0, -555), white));
	scene.add(make_shared<Parallelogram>(Point3(555, 0, 555), Vec3(-555, 0, 0), Vec3(0, 555, 0), white));

	// light
	auto light_quad = make_shared<Parallelogram>(Point3(213, 554, 227), Vec3(130, 0, 0), Vec3(0, 0, 105), light);
	scene.add(light_quad);
	lights.add(light_quad);

	// box 1
	shared_ptr<Hittable> box1 = box(Point3(0, 0, 0), Point3(165, 330, 165), white);
	box1 = make_shared<Rotate>(box1, Axis::Y, 15 * PI / 180);
	box1 = make_shared<Translate>(box1, Vec3(265, 0, 295));
	scene.add(box1);

	// box 2
	shared_ptr<Hittable> box2 = box(Point3(0, 0, 0), Point3(165, 165, 165), white);
	box2 = make_shared<Rotate>(box2, Axis::Y, -18 * PI / 180);
	box2 = make_shared<Translate>(box2, Vec3(130, 0, 65));
	scene.add(box2);

	// camera settings
	cam.aspect_ratio       = 1.0;
	cam.image_width        = 600;
	cam.set_solid_bg(black);

#ifdef DEBUG_TRACER // very few rays to stress test importance sampling
	cam.subpixel_grid_size = 3;
#else
	cam.subpixel_grid_size = 32;
#endif
	cam.max_depth          = 50;

	cam.vert_fov = 40;
	cam.center   = Point3(278, 278, -800);
	cam.facing   = Point3(278, 278, 0);
}

#endif
