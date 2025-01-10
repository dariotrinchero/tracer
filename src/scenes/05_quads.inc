#ifndef SCENE_INC
#define SCENE_INC

void build_scene(HittableList& scene, HittableList&, Camera& cam) {
	auto left_red     = make_shared<Lambertian>(Color(1.0, 0.2, 0.2));
	auto back_green   = make_shared<Lambertian>(Color(0.2, 1.0, 0.2));
	auto right_blue   = make_shared<Lambertian>(Color(0.2, 0.2, 1.0));
	auto upper_orange = make_shared<Lambertian>(Color(1.0, 0.5, 0.0));
	auto lower_teal   = make_shared<Lambertian>(Color(0.2, 0.8, 0.8));
	scene.add(make_shared<Parallelogram>(Point3(-3, -2, 5), Vec3(0, 0, -4), Vec3(0, 4, 0), left_red));
	scene.add(make_shared<Parallelogram>(Point3(-2, -2, 0), Vec3(4, 0, 0), Vec3(0, 4, 0), back_green));
	scene.add(make_shared<Parallelogram>(Point3( 3, -2, 1), Vec3(0, 0, 4), Vec3(0, 4, 0), right_blue));
	scene.add(make_shared<Parallelogram>(Point3(-2, 3, 1), Vec3(4, 0, 0), Vec3(0, 0, 4), upper_orange));
	scene.add(make_shared<Parallelogram>(Point3(-2, -3, 5), Vec3(4, 0, 0), Vec3(0, 0, -4), lower_teal));

	cam.aspect_ratio = 1.0;
	cam.vert_fov = 80;
	cam.center = Point3(0, 0, 9);
}

#endif
