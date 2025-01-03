#ifndef SHAPE2D_H
#define SHAPE2D_H

#include <cmath>

#include "hittable.h"
#include "material.h"

class Shape2D : public Hittable {
  public:
	Shape2D(const Point3& anchor, const Vec3& normal, shared_ptr<Material> mat)
		: anchor(anchor), normal(normal), mat(mat)
	{
		// construct orthonormal basis u,v
		u = (normal[2] < normal[0] // test ensures stability for components near 0
			? Vec3(normal[1], -normal[0], 0)
			: Vec3(0, -normal[2], normal[1])).unit();
		v = cross(normal, u).unit();
		w = normal;

		D = dot(normal, anchor);
		set_bounding_box();
	}

	Shape2D(const Point3& anchor, const Vec3& u, const Vec3& v, shared_ptr<Material> mat)
		: anchor(anchor), u(u), v(v), mat(mat)
	{
		auto u_cross_v = cross(u, v);
		normal = u_cross_v.unit();
		w = u_cross_v / u_cross_v.length_squared();
		D = dot(normal, anchor);
		set_bounding_box();
	}

	AABB bounding_box() const override { return bbox; }

	virtual void set_bounding_box() {
		auto diag1 = AABB(anchor, anchor + u + v);
		auto diag2 = AABB(anchor + u, anchor + v);
		bbox = AABB(diag1, diag2);
	}

	bool hit(const Ray& r, Interval ray_t, HitRecord& rec) const override {
		// solve intersection with plane of shape
		auto ray_dot_normal = dot(normal, r.direction());
		if (std::fabs(ray_dot_normal) < 1e-8) return false; // ray parallel to plane

		auto t = (D - dot(normal, r.source())) / ray_dot_normal;
		if (!ray_t.contains(t)) return false;
		auto intersection = r.at(t);

		// express intersection as anchor + a*u + b*v
		Vec3 anchor_to_intersection = intersection - anchor;
		auto a = triple(w, anchor_to_intersection, v);
		auto b = triple(w, u, anchor_to_intersection);

		// check intersection is inside shape
		if (!is_interior(a, b)) return false;
		
		// record hit
		rec.t = t;
		rec.p = intersection;
		rec.set_face_normal(r, normal);
		rec.u = a;
		rec.v = b;
		rec.mat = mat;
		return true;
	}

	virtual bool is_interior(double a, double b) const = 0;

  protected:
	Point3 anchor; // anchor point on plane of shape
	Vec3   u, v;   // basis for plane of shape
	Vec3   normal; // normal to plane of shape
	double D;      // constant in plane equation: dot(r,normal)=D, for any r on plane
	Vec3   w;      // used in ray intersection logic: w=n/|uxv|
	shared_ptr<Material> mat;
	AABB bbox;
};

class Parallelogram : public Shape2D {
  public:
	Parallelogram(const Point3& corner, const Vec3& side1, const Vec3& side2, shared_ptr<Material> mat)
		: Shape2D(corner, side1, side2, mat) {}

	bool is_interior(double a, double b) const override {
		return Interval::unit.contains(a) && Interval::unit.contains(b);
	}
};

class Triangle : public Shape2D {
  public:
	Triangle(const Point3& corner, const Vec3& side1, const Vec3& side2, shared_ptr<Material> mat)
		: Shape2D(corner, side1, side2, mat) {}

	bool is_interior(double a, double b) const override {
		return a > 0 && b > 0 && a + b < 1;
	}

	static Triangle from_verts(const Point3& p1, const Point3& p2, const Point3& p3, shared_ptr<Material> mat) {
		return Triangle(p1, p2 - p1, p3 - p1, mat);
	}
};

class Annulus : public Shape2D {
  public:
	Annulus(const Point3& center, const Vec3& major_axis, const Vec3& minor_axis, shared_ptr<Material> mat)
		: Shape2D(center - major_axis / 2 - minor_axis / 2, major_axis, minor_axis, mat), inner_sq(0) {}

	Annulus(const Point3& center, const Vec3& major_axis, const Vec3& minor_axis, double inner, shared_ptr<Material> mat)
		: Shape2D(center - major_axis / 2 - minor_axis / 2, major_axis, minor_axis, mat)
	{
		inner = Interval::unit.clamp(inner);
		inner_sq = inner * inner / 4;
	}

	bool is_interior(double a, double b) const override {
		double a2 = a - 0.5, b2 = b - 0.5;
		double r_sq = a2 * a2 + b2 * b2;
		return inner_sq < r_sq && r_sq < 0.25;
	}

  private:
	double inner_sq;
};

class Ellipse : public Annulus {
  public:
	Ellipse(const Point3& center, const Vec3& major_axis, const Vec3& minor_axis, shared_ptr<Material> mat)
		: Annulus(center, major_axis, minor_axis, mat) {}
};

inline shared_ptr<HittableList> box(const Point3& a, const Point3& b, shared_ptr<Material> mat) {
	auto min = Point3(std::fmin(a[0], b[0]), std::fmin(a[1], b[1]), std::fmin(a[2], b[2]));
	auto max = Point3(std::fmax(a[0], b[0]), std::fmax(a[1], b[1]), std::fmax(a[2], b[2]));

	auto dx = Vec3(max.x() - min.x(), 0, 0);
	auto dy = Vec3(0, max.y() - min.y(), 0);
	auto dz = Vec3(0, 0, max.z() - min.z());

	auto sides = make_shared<HittableList>();
	sides->add(make_shared<Parallelogram>(Point3(min[0], min[1], max[2]),  dx,  dy, mat)); // front
	sides->add(make_shared<Parallelogram>(Point3(max[0], min[1], max[2]), -dz,  dy, mat)); // right
	sides->add(make_shared<Parallelogram>(Point3(max[0], min[1], min[2]), -dx,  dy, mat)); // back
	sides->add(make_shared<Parallelogram>(Point3(min[0], min[1], min[2]),  dz,  dy, mat)); // left
	sides->add(make_shared<Parallelogram>(Point3(min[0], max[1], max[2]),  dx, -dz, mat)); // top
	sides->add(make_shared<Parallelogram>(Point3(min[0], min[1], min[2]),  dx,  dz, mat)); // bottom
	return sides;
}

#endif
