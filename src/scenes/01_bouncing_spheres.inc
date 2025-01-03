#ifndef SCENE_INC
#define SCENE_INC

void build_scene(HittableList& scene, Camera& cam) {
	auto checker = make_shared<CheckerTexture>(0.32, Color(.2, .3, .1), Color(.9, .9, .9));
	scene.add(make_shared<Sphere>(Point3(0, -1000, 0), 1000, make_shared<Lambertian>(checker)));

	for (int a = -11; a < 11; a++) {
		for (int b = -11; b < 11; b++) {
			auto choose_mat = rnd_double();
			Point3 center(a + 0.9 * rnd_double(), 0.2, b + 0.9 * rnd_double());

			if ((center - Point3(4, 0.2, 0)).length() > 0.9) {
				shared_ptr<Material> sphere_material;

				if (choose_mat < 0.8) {
					// diffuse
					auto albedo = rnd_vec() * rnd_vec();
					sphere_material = make_shared<Lambertian>(albedo);
					auto center2 = center + Vec3(0, rnd_double(0, 0.3), 0);
					scene.add(make_shared<Sphere>(center, center2, 0.2, sphere_material));
				} else if (choose_mat < 0.95) {
					// metal
					auto albedo = rnd_vec(0.5, 1);
					auto fuzz = rnd_double(0, 0.5);
					sphere_material = make_shared<Metal>(albedo, fuzz);
					scene.add(make_shared<Sphere>(center, 0.2, sphere_material));
				} else {
					// glass
					sphere_material = make_shared<Dielectric>(1.5);
					scene.add(make_shared<Sphere>(center, 0.2, sphere_material));
				}
			}
		}
	}

	auto material1 = make_shared<Dielectric>(1.5);
	scene.add(make_shared<Sphere>(Point3(0, 1, 0), 1.0, material1));

	auto material2 = make_shared<Lambertian>(Color(0.4, 0.2, 0.1));
	scene.add(make_shared<Sphere>(Point3(-4, 1, 0), 1.0, material2));

	auto material3 = make_shared<Metal>(Color(0.7, 0.6, 0.5), 0.0);
	scene.add(make_shared<Sphere>(Point3(4, 1, 0), 1.0, material3));

	cam.center = Point3(13, 2, 3);
	cam.defocus_angle = 0.6;
	cam.focus_dist = 10;
	cam.shutter_speed = 1.0;
}

#endif
