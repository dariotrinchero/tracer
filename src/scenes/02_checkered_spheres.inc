#ifndef SCENE_INC
#define SCENE_INC

void build_scene(HittableList& scene, HittableList&, Camera& cam) {
	auto checker = make_shared<CheckerTexture>(0.32, Color(.2, .3, .1), Color(.9, .9, .9));
	scene.add(make_shared<Sphere>(Point3(0, -10, 0), 10, make_shared<Lambertian>(checker)));
	scene.add(make_shared<Sphere>(Point3(0, 10, 0), 10, make_shared<Lambertian>(checker)));
	cam.center = Point3(13, 2, 3);
	cam.vert_fov = 20;
}

#endif
