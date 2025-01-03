#ifndef SCENE_INC
#define SCENE_INC

void build_scene(HittableList& scene, Camera& cam) {
	// checkered sphere floor
	auto checker = make_shared<CheckerTexture>(9.0, Color(.2, .3, .1), Color(.9, .9, .9));
	scene.add(make_shared<Sphere>(Point3(0, -4000, 0), 4000, make_shared<Lambertian>(checker)));

	// fox
	shared_ptr<Material> mat = make_shared<Metal>(Color(0.65, 0.21, 0.16), 0.3);
	scene.add(Mesh("fox.stl", mat).get_model());

	// camera
	cam.center = Point3(-20, 100, -220);
	cam.facing = Point3(-10, 50, 0);
	cam.vert_fov = 40;
}

#endif
