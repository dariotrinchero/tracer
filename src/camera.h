#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <cmath>

#include "hittable.h"
#include "material.h"
#include "color.h"
#include "random.h"
#include "bvh.h"
#include "threads.h"
#include "image.h"

#define PI 3.1415926535897932385

using BgFn = std::function<Color(const Ray&)>;

class Camera {
  public:
	// image geometry
	double aspect_ratio      = 1.0;
	int    image_width       = 100;
	double vert_fov          = 90;                // vertical FOV (deg)

	// camera positioning
	Point3 center            = Point3(0, 0, 0);   // camera center location
	Point3 facing            = Point3(0, 0, -1);  // direction faced by camera
	Vec3   v_up              = Vec3(0, 1, 0);     // defines camera roll

	// quality & performance
	int    samples_per_pixel = 10;                // rays per pixel
	int    max_depth         = 10;                // max ray bounces

	// aesthetic features
	double gamma             = 2.0;
	double defocus_angle     = 0;                 // variation in incident ray angle (deg)
	double focus_dist        = 10;                // aka. 'depth-of-field'
	double shutter_speed     = 0;                 // aka. 'exposure time' (seconds); 0 = instantaneous
	BgFn   background        = [](const Ray& r) { // background skybox
			// default sky background
			double a = 0.5 * (r.direction().unit().y() + 1);
			return (1 - a) * white + a * Color(.5, .7, 1);
		};

	void set_solid_bg(const Color& bg) {
		background = [bg](const Ray&) { return bg; };
	}

	void render(const BVHNode& scene, int start_pixel = 0) {
		initialize(start_pixel);

		ThreadPool pool;
		int samples_per_thread = samples_per_pixel / pool.num_threads();
		int sample_remainder = samples_per_pixel % pool.num_threads();

		bool resumed = false;
		if (start_pixel == 0) PPMImage::write_header(std::cout, image_width, image_height);
		else {
			std::clog << "Resuming render from pixel: " << start_pixel << '\n';
			resumed = true;
		}

		int first_row = start_pixel / image_width;
		for (int j = first_row; j < image_height; j++) {
			std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;

			for (int i = resumed && j == first_row ? start_pixel % image_width : 0; i < image_width; i++) {
				// spawn threads to collect samples
				std::vector<std::future<Color>> futures;
				for (int t = 0; t < pool.num_threads(); t++) {
					int samples = samples_per_thread + (t < sample_remainder ?  1 : 0);
					if (samples > 0) {
						futures.push_back(pool.enqueue(&Camera::pixel_color, this, std::ref(scene), i, j, samples));
					}
				}

				// join threads & aggregate results
				Color pixel_col = black;
				for (auto& future : futures) pixel_col += future.get();
				PPMImage::write_color(std::cout, pixel_col / samples_per_pixel, gamma);
			}
		}

		std::clog << "\rDone.                 \n";
	}

	void render(const HittableList& scene, int start_pixel = 0) {
		render(BVHNode(scene), start_pixel);
	}

  private:
	int    image_height;
	Point3 pixel00_loc;    // top left pixel

	// basis vectors
	Vec3   u, v, w;        // camera frame basis
	Vec3   pixel_delta_u;  // horizontal pixel offset
	Vec3   pixel_delta_v;  // vertical pixel offset
	Vec3   defocus_disk_u; // defocus disk horizontal basis vec
	Vec3   defocus_disk_v; // defocus disk vertical basis vec

	void initialize(int start_pixel) {
		// image & viewport dimensions
		image_height = int(image_width / aspect_ratio);
		if (image_height < 1) image_height = 1;
		auto viewport_height = 2.0 * std::tan(vert_fov * PI / 360) * focus_dist;
		auto viewport_width = viewport_height * (double(image_width) / image_height);

		// camera frame basis vectors
		w = (center - facing).unit();
		u = cross(v_up, w).unit();
		v = cross(w, u);

		// directions down & along viewport left & top edges
		auto viewport_u = viewport_width * u;
		auto viewport_v = viewport_height * -v;

		// delta vectors between adjacent pixels
		pixel_delta_u = viewport_u / image_width;
		pixel_delta_v = viewport_v / image_height;

		// upper left pixel
		auto viewport_upper_left = center - (focus_dist * w) - viewport_u/2 - viewport_v/2;
		pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

		// defocus disk basis vectors
		auto defocus_radius = std::tan(defocus_angle * PI / 360) * focus_dist;
		defocus_disk_u = defocus_radius * u;
		defocus_disk_v = defocus_radius * v;

		// ensure start pixel is valid
		if (start_pixel < 0 || start_pixel >= image_width * image_height) {
			std::cerr << "ERROR: Invalid starting pixel.\n";
			exit(EXIT_FAILURE);
		}
	}

	Ray get_ray(int i, int j) const {
		auto offset = 0.5 * rnd_vec_unit_disk();
		auto pixel_sample = pixel00_loc + ((i + offset.x()) * pixel_delta_u)
			+ ((j + offset.y()) * pixel_delta_v);
		
		auto ray_origin = center;
		if (defocus_angle > 0) {
			auto p = rnd_vec_unit_disk();
			ray_origin += (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
		}

		double ray_time = 0;
		if (shutter_speed > 0) ray_time = rnd_double(0, shutter_speed);

		return Ray(ray_origin, pixel_sample - ray_origin, ray_time);
	}

	Color ray_color(const Ray& r, int depth, const Hittable& scene) const {
		if (depth <= 0) return black;

		HitRecord rec;
		// bound t>1e-3 prevents scattered rays colliding immediately with the same surface
		// due to rounding errors, causing 'shadow acne'
		if (!scene.hit(r, Interval(1e-3, infinity), rec)) return background(r);

		// color from emission
		Color ray_col = rec.mat->emitted(rec.u, rec.v, rec.p);

		// color from scattered rays
		Ray scattered;
		Color attenuation;
		if (rec.mat->scatter(r, rec, attenuation, scattered))
			ray_col += attenuation * ray_color(scattered, depth - 1, scene);

		return ray_col;
	}

	Color pixel_color(const Hittable& scene, int i, int j, int samples) {
		Color pixel_col = black;
		for (int s = 0; s < samples; s++) pixel_col += ray_color(get_ray(i, j), max_depth, scene);
		return pixel_col;
	}
};

#endif
