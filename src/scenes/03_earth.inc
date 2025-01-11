#ifndef SCENE_INC
#define SCENE_INC

void build_scene(HittableList& scene, HittableList&, Camera& cam) {
	auto earth_texture = make_shared<ImageTexture>("earthmap.ppm");
	auto earth_surface = make_shared<Lambertian>(earth_texture);
	shared_ptr<Hittable> earth = make_shared<Sphere>(Point3(0, 0, 0), 2, earth_surface);
	earth = make_shared<Rotate>(earth, Axis::Y, -90 * PI / 180);
	scene.add(earth);
	cam.center = Point3(0, 0, 12);
	cam.vert_fov = 20;
}

#endif
