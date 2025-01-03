#ifndef SCENE_INC
#define SCENE_INC

void build_scene(HittableList& scene, Camera& cam) {
	auto pertext = make_shared<NoiseTexture>(4);
	scene.add(make_shared<Sphere>(Point3(0, -1000, 0), 1000, make_shared<Lambertian>(pertext)));
	scene.add(make_shared<Sphere>(Point3(0, 2, 0), 2, make_shared<Lambertian>(pertext)));

	auto difflight = make_shared<DiffuseLight>(4 * white); // note: 4*white is brighter than white
	scene.add(make_shared<Parallelogram>(Point3(3, 1, -2), Vec3(2, 0, 0), Vec3(0, 2, 0), difflight));

	cam.center = Point3(26, 3, 6);
	cam.facing = Point3(0, 2, 0);
	cam.set_solid_bg(black);
}

#endif
