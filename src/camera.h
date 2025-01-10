#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <cmath>

#include "hittable.h"
#include "material.h"
#include "color.h"
#include "random.h"
#include "threads.h"
#include "image.h"

using BgFn = std::function<Color(const Ray&)>;

class Camera {
  public:
	// image geometry
	double aspect_ratio       = 1.0;
	int    image_width        = 100;
	double vert_fov           = 90;                // vertical FOV (deg)

	// camera positioning
	Point3 center             = Point3(0, 0, 0);   // camera center location
	Point3 facing             = Point3(0, 0, -1);  // direction faced by camera
	Vec3   v_up               = Vec3(0, 1, 0);     // defines camera roll

	// quality & performance
	int    subpixel_grid_size = 10;                // sqrt of # rays per pixel
	int    max_depth          = 10;                // max ray bounces

	// aesthetic features
	double gamma              = 2.0;
	double defocus_angle      = 0;                 // variation in incident ray angle (deg)
	double focus_dist         = 10;                // aka. 'depth-of-field'
	double shutter_speed      = 0;                 // aka. 'exposure time' (seconds); 0 = instantaneous
	BgFn   background         = [](const Ray& r) { // background skybox
			// default sky background
			double a = 0.5 * (r.direction().unit().y() + 1);
			return (1 - a) * white + a * Color(.5, .7, 1);
		};

	void set_solid_bg(const Color& bg) {
		background = [bg](const Ray&) { return bg; };
	}

	void render(const Hittable& scene, const HittableList& lights, int start_pixel=0) {
		initialize(start_pixel);

		ThreadPool pool;
		int samples_per_thread = samples_per_pixel / pool.num_threads();
		int sample_remainder = samples_per_pixel % pool.num_threads();

		if (start_pixel == 0) PPMImage::write_header(std::cout, image_width, image_height);

		int start_row = start_pixel / image_width;
		for (int j = start_row; j < image_height; j++) {
			std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;

			for (int i = j == start_row ? start_pixel % image_width : 0; i < image_width; i++) {
				// spawn threads to collect samples
				std::vector<std::future<Color>> futures;
				int subpixel = 0;

				for (int t = 0; t < pool.num_threads(); t++) {
					int samples = samples_per_thread + (t < sample_remainder ?  1 : 0);
					if (samples <= 0) break;
					futures.push_back(pool.enqueue(&Camera::sample_pixel, this, std::ref(scene),
						std::ref(lights), i, j, subpixel, samples));
					subpixel += samples;
				}

				// join threads & aggregate results
				Color color_total = black;
				for (auto& future : futures) color_total += future.get();
				PPMImage::write_color(std::cout, color_total / samples_per_pixel, gamma);
			}
		}

		std::clog << "\rDone.                 \n";
	}

  private:
	// dimensions
	int    image_height;
	int    samples_per_pixel; // # rays averaged per pixel
	double inv_sp_grid_size;  // intermediate: 1 / subpixel_grid_size

	// basis vectors
	Vec3   u, v, w;           // camera frame basis
	Point3 pixel00_loc;       // top left pixel
	Vec3   pixel_delta_u;     // horizontal pixel offset
	Vec3   pixel_delta_v;     // vertical pixel offset
	Vec3   defocus_disk_u;    // defocus disk horizontal basis vec
	Vec3   defocus_disk_v;    // defocus disk vertical basis vec

	void initialize(int start_pixel) {
		// dimensions of image, viewport, & sub-pixel grid
		image_height = int(image_width / aspect_ratio);
		if (image_height < 1) image_height = 1;
		samples_per_pixel = subpixel_grid_size * subpixel_grid_size;
		inv_sp_grid_size = 1.0 / subpixel_grid_size;
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

	/**
	 * Get ray from camera (ie. from random point on camera defocus disk) towards given
	 * pixel & sub-pixel (ie. to random point within sub-pixel). Ray is also assigned
	 * random time in range [0,shutter_speed).
	 *
	 * @param i    column index of pixel
	 * @param j    row index of pixel
	 * @param sp_i column index of sub-pixel
	 * @param sp_j row index of sub-pixel
	 * @returns ray with source at camera & direction towards given sub-pixel
	 */
	Ray get_ray(int i, int j, int sp_i, int sp_j) const {
		auto offset = inv_sp_grid_size * (0.5 * rnd_vec_unit_disk()
			+ Vec3(sp_i + 0.5, sp_j + 0.5, 0)) - Vec3(0.5, 0.5, 0);
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

	/**
	 * Trace given ray towards given scene, recursing for scattered rays up to given depth.
	 * Return resulting color of ray.
	 *
	 * @param scene  the scene at which to fire ray
	 * @param lights light emitters in scene (otherwise objects toward which to bias rays)
	 * @param r      the ray to trace
	 * @param depth  maximum recursion depth (number of scattered rays followed)
	 * @returns color acquired by ray along its path through scene
	 */
	Color ray_color(const Hittable& scene, const HittableList& lights, const Ray& r, int depth) const {
		if (depth <= 0) return black;

		HitRecord rec;
		/* t>1e-3: 'bias' to prevent scattered rays re-colliding with same surface
		 * (by rounding error) & causing 'shadow acne' */
		if (!scene.hit(r, Interval(1e-3, infinity), rec)) return background(r);

		// color from emission
		Color pixel_col = rec.mat->emitted(rec);

		// get scattering data
		ScatterRecord srec;
		if (!rec.mat->scatter(r, rec, srec)) return pixel_col;

		Ray scattered;
		double weight = 1;

		if (!srec.pdf) { // use hard-coded scattered ray
			scattered = srec.override_ray;
		} else { // sample scattered ray direction from PDF
			MixturePDF mixed_pdf{srec.pdf};
			if (lights.objects.size() > 0) mixed_pdf.add(make_shared<HittablePDF>(lights, rec.p));
			scattered = Ray(rec.p, mixed_pdf.sample(), r.time());

			double sample_pdf_val = mixed_pdf.density(scattered.direction());
			if (sample_pdf_val == 0) return pixel_col; // scattering is impossible
			double scatter_pdf_val = srec.pdf->density(scattered.direction());
			weight = scatter_pdf_val / sample_pdf_val;
		}

		// color from scattering
		pixel_col += weight * srec.attenuation * ray_color(scene, lights, scattered, depth - 1);
		return pixel_col;
	}

	/**
	 * Fire specified number of samples at the scene, all within the bounds of given pixel,
	 * but spread out along the sub-pixel grid (from given starting index). Return the sum
	 * of the colors of all rays fired.
	 *
	 * @param scene    the scene at which to fire rays
	 * @param lights   light emitters in scene (otherwise objects toward which to bias rays)
	 * @param i        column index of pixel to sample
	 * @param j        row index of pixel to sample
	 * @param subpixel index of sub-pixel at which to begin sampling (converted to 2D coords)
	 * @param samples  number of samples to gather
	 * @returns sum of colors returned by all rays fired
	 */
	Color sample_pixel(
		const Hittable& scene, const HittableList& lights, int i, int j, int subpixel, int samples
	) {
		Color color_sum = black;
		for (int sp = subpixel; sp < subpixel + samples; sp++) {
			int sp_i = sp % subpixel_grid_size;
			int sp_j = sp / subpixel_grid_size;
			color_sum += ray_color(scene, lights, get_ray(i, j, sp_i, sp_j), max_depth);
		}
		return color_sum;
	}
};

#endif
