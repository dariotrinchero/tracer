#ifndef SPHERE_H
#define SPHERE_H

#include <cmath>

#include "hittable.h"
#include "material.h"

class Sphere : public Hittable {
  public:
	/* stationary sphere */
	Sphere(const Point3& static_center, double radius, shared_ptr<Material> mat)
		: center(static_center, Vec3(0, 0, 0)), radius(std::fmax(0, radius)), mat(mat)
	{
		auto rvec = Vec3(radius, radius, radius);
		bbox = AABB(static_center - rvec, static_center + rvec);
	}

	/* linearly moving sphere - moves from center0 at t=0 to center1 at t=1
	 * (motion seen in render will depend on shutter speed) */
	Sphere(const Point3& center0, const Point3& center1, double radius, shared_ptr<Material> mat)
		: center(center0, center1 - center0), radius(std::fmax(0, radius)), mat(mat)
	{
		auto rvec = Vec3(radius, radius, radius);
		AABB box0(center0 - rvec, center0 + rvec);
		AABB box1(center1 - rvec, center1 + rvec);
		bbox = AABB(box0, box1);
	}

	bool hit(const Ray& r, Interval ray_t, HitRecord& rec) const override {
		Point3 curr_center = center.at(r.time());
		Vec3 oc = curr_center - r.source();

		// check if ray intersection exists
		auto a = r.direction().length_squared();
		auto h = dot(r.direction(), oc); // b = -2h
		auto c = oc.length_squared() - radius * radius;
		auto discriminant = h * h - a * c;
		if (discriminant < 0) return false;

		// find nearest root in acceptable t range
		auto sqrtd = std::sqrt(discriminant);
		auto root = (h - sqrtd) / a;
		if (!ray_t.surrounds(root)) {
			root = (h + sqrtd) / a;
			if (!ray_t.surrounds(root)) return false;
		}

		// record hit
		rec.t = root;
		rec.p = r.at(root);
		Vec3 outward_normal = (rec.p - curr_center) / radius;
		rec.set_face_normal(r, outward_normal);
		get_sphere_uv(outward_normal, rec.u, rec.v);
		rec.mat = mat;
		return true;
	}

	AABB bounding_box() const override { return bbox; }

	// TODO ONLY WORKS FOR STATIONARY SPHERE
	// maybe modify pdf_value to take Ray (or just Ray time) as parameter?
	double pdf_value(const Point3& origin, const Vec3& direction) const override {
		HitRecord rec;
		if (!this->hit(Ray(origin, direction), Interval(1e-3, infinity), rec)) return 0;

		double dist_sq = (center.at(0) - origin).length_squared();
		double cos_theta_max = std::sqrt(1 - radius * radius / dist_sq);
		double solid_angle = 2 * PI * (1 - cos_theta_max);

		return 1 / solid_angle;
	}

	Vec3 rnd_point(const Point3& origin) const override {
		Vec3 direction = center.at(0) - origin;
		double dist_sq = direction.length_squared();

		double z = 1 + rnd_double() * (std::sqrt(1 - radius * radius / dist_sq) - 1);
		double phi = 2 * PI * rnd_double();
		double xy_proj = std::sqrt(1 - z * z);
		double x = std::cos(phi) * xy_proj;
		double y = std::sin(phi) * xy_proj;

		return Mat3::orthog(direction) * Vec3(x, y, z);
	}

  private:
	Ray center;
	double radius;
	shared_ptr<Material> mat;
	AABB bbox;

	/**
	 * Get spherical coordinates of given point on unit sphere, normalized to lie in [0,1].
	 * These are the texture uv-coordinates on the sphere.
	 *
	 * @param[in] p  point on unit sphere
	 * @param[out] u azimuth (phi) coordinate of p, scaled from [-pi,pi] to [0,1]
	 * @param[out] v inclination (theta) coordinate of p, scaled from [0,pi] to [0,1]
	 */
	static void get_sphere_uv(const Point3& p, double& u, double& v) {
		u = 0.5 * (std::atan2(-p.z(), p.x()) / PI + 1);
		v = std::acos(-p.y()) / PI;
	}
};

#endif
