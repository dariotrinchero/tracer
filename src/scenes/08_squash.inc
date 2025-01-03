#ifndef SCENE_INC
#define SCENE_INC

void build_scene(HittableList& scene, Camera& cam) {
	auto pertext = make_shared<NoiseTexture>(4);
	shared_ptr<Hittable> top_sphere = make_shared<Sphere>(Point3(0, 2, 0), 2, make_shared<Metal>(Color(.9, .5, .5), 0.07));
	top_sphere = make_shared<Scale>(top_sphere, 1, .6, .8);
	top_sphere = make_shared<Rotate>(top_sphere, Axis::X, 30 * PI / 180);
	top_sphere = make_shared<Translate>(top_sphere, Vec3(0, 1.5, 0));
	scene.add(top_sphere);
	scene.add(make_shared<Sphere>(Point3(0, -1000, 0), 1000, make_shared<Lambertian>(pertext)));
	cam.center = Point3(17, 3.5, 3);
}

#endif
