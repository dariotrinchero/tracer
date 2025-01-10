#ifndef SCENE_INC
#define SCENE_INC

void build_scene(HittableList& scene, HittableList&, Camera& cam) {
	auto pertext = make_shared<NoiseTexture>(4);
	scene.add(make_shared<Sphere>(Point3(0, -1000, 0), 1000, make_shared<Lambertian>(pertext)));
	scene.add(make_shared<Sphere>(Point3(0, 2, 0), 2, make_shared<Lambertian>(pertext)));
	cam.center = Point3(13, 2, 3);
}

#endif
