#ifndef SCENE_INC
#define SCENE_INC

void build_scene(HittableList& scene, Camera& cam) {
	auto red   = make_shared<Lambertian>(Color(.65, .05, .05));
	auto white = make_shared<Lambertian>(Color(.73, .73, .73));
	auto green = make_shared<Lambertian>(Color(.12, .45, .15));
	auto light = make_shared<DiffuseLight>(Color(15, 15, 15));

	scene.add(make_shared<Parallelogram>(Point3(555, 0, 0), Vec3(0, 555, 0), Vec3(0, 0, 555), green));
	scene.add(make_shared<Parallelogram>(Point3(0, 0, 0), Vec3(0, 555, 0), Vec3(0, 0, 555), red));
	scene.add(make_shared<Parallelogram>(Point3(343, 554, 332), Vec3(-130, 0, 0), Vec3(0, 0, -105), light));
	scene.add(make_shared<Parallelogram>(Point3(0, 0, 0), Vec3(555, 0, 0), Vec3(0, 0, 555), white));
	scene.add(make_shared<Parallelogram>(Point3(555, 555, 555), Vec3(-555, 0, 0), Vec3(0, 0, -555), white));
	scene.add(make_shared<Parallelogram>(Point3(0, 0, 555), Vec3(555, 0, 0), Vec3(0, 555, 0), white));	

	shared_ptr<Hittable> box1 = box(Point3(0, 0, 0), Point3(165, 330, 165), white);
	box1 = make_shared<Rotate>(box1, Axis::Y, 15 * PI / 180);
	box1 = make_shared<Translate>(box1, Vec3(265, 0, 295));
	scene.add(box1);

	shared_ptr<Hittable> box2 = box(Point3(0, 0, 0), Point3(165, 165, 165), white);
	box2 = make_shared<Rotate>(box2, Axis::Y, -18 * PI / 180);
	box2 = make_shared<Translate>(box2, Vec3(130, 0, 65));
	scene.add(box2);

	cam.aspect_ratio      = 1.0;
	cam.image_width       = 600;
	cam.set_solid_bg(black);

	cam.samples_per_pixel = 200;
	cam.max_depth         = 50;

	cam.vert_fov = 40;
	cam.center   = Point3(278, 278, -800);
	cam.facing   = Point3(278, 278, 0);
}

#endif
