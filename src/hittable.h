#ifndef HITTABLE_H
#define HITTABLE_H

#include <memory>
#include <vector>
#include <stdexcept>
#include <iostream>

#include "ray.h"
#include "interval.h"
#include "aabb.h"

using std::make_shared;
using std::shared_ptr;

// macro to mark arguments as unused (to silence warnings)
template <typename... Args> inline void unused(Args&&...) {}

// signatures from material.h & random.h
class Material;
inline int rnd_int(int min, int max);

/* --- container for intersection data ---------------------------------------------------------- */

class HitRecord {
  public:
	Point3               p;          // point hit
	double               t;          // distance along ray of hit
	Vec3                 normal;     // normal to object at hit point
	bool                 front_face; // whether ray hit front of surface
	shared_ptr<Material> mat;        // material of hit object
	double               u, v;       // u-v texture coordinates of p

	/**
	 * Record normal vector. Our convention is that the normal is opposed to ray, rather
	 * than facing outwards; thus, we record face orientation in front_face instead.
	 *
	 * @param r              the ray which intersected the surface
	 * @param outward_normal the outward-facing normal to the surface at intersection point
	 */
	void set_face_normal(const Ray& r, const Vec3& outward_normal) {
		front_face = dot(r.direction(), outward_normal) < 0;
		normal = front_face ? outward_normal : -outward_normal;
	}
};

/* --- superclass for hittable ------------------------------------------------------------------ */

class Hittable {
  public:
	virtual ~Hittable() = default;

	/**
	 * Test whether given ray hits this object within given time interval. If so, record
	 * details of intersection in given record.
	 *
	 * @param[in] r     ray to test for intersection
	 * @param[in] ray_t interval of times for which to consider ray intersections
	 * @param[out] rec  HitRecord for storing ray intersection details
	 * @returns whether given ray hits this object
	 */
	virtual bool hit(const Ray& r, Interval ray_t, HitRecord& rec) const = 0;

	/**
	 * Get bounding box of hittable object.
	 *
	 * @returns axis-aligned bounding box containing this
	 */
	virtual AABB bounding_box() const = 0;

	/**
	 * Get PDF on unit sphere of directions (around given origin) which, when sampled,
	 * produces rays whose intersection points with this hittable are uniformly-
	 * distributed across the surface of this hittable. Usually called on lights; used to
	 * bias rays towards hitting this object.
	 *
	 * @param origin    source of rays
	 * @param direction direction of rays
	 * @returns the PDF on unit sphere around given origin evaluated at given direction
	 */
	virtual double pdf_value(const Point3& origin, const Vec3& direction) const {
		unused(origin, direction);
		return 0.0;
	}

	/**
	 * Generate vector from given origin to uniformly-distributed random point on surface
	 * of hittable object. Usually called on lights; used to bias rays towards hitting this
	 * object.
	 *
	 * @param origin source of rays (tail of random vector)
	 * @returns random vector from origin to point on surface of this hittable
	 */
	virtual Vec3 rnd_point(const Point3& origin) const {
		unused(origin);
		return Vec3(1, 0, 0);
	}
};

/* --- class for groups of hittables ------------------------------------------------------------ */

class HittableList : public Hittable {
  public:
	std::vector<shared_ptr<Hittable>> objects;

	HittableList() {}
	HittableList(shared_ptr<Hittable> object) { add(object); }

	void clear() { objects.clear(); }

	void add(shared_ptr<Hittable> object) {
		objects.push_back(object);
		bbox = AABB(bbox, object->bounding_box());
	}

	bool hit(const Ray& r, Interval ray_t, HitRecord& rec) const override {
		HitRecord temp_rec;
		bool hit_anything = false;
		auto closest_yet = ray_t.max;

		for (const auto& object : objects) {
			if (object->hit(r, Interval(ray_t.min, closest_yet), temp_rec)) {
				hit_anything = true;
				closest_yet = temp_rec.t;
				rec = temp_rec;
			}
		}

		return hit_anything;
	}

	AABB bounding_box() const override { return bbox; }

	double pdf_value(const Point3& origin, const Vec3& direction) const override {
		double sum = 0;
		for (const auto& obj : objects) sum += obj->pdf_value(origin, direction);
		return sum / objects.size();
	}

	Vec3 rnd_point(const Point3& origin) const override {
		return objects[rnd_int(0, objects.size() - 1)]->rnd_point(origin);
	}

  private:
	AABB bbox;
};

/* --- affine transformations ------------------------------------------------------------------- */

// TODO unify with other transformations by using 4x4 matrices & length 4 vectors instead
// see pg 16+ of https://web.cs.hacettepe.edu.tr/~erkut/bco511.s12/w04-transformations.pdf
// This will allow us to squash a sequence: transform > translate > transform
// into a single transform, like we currently do with sequences of just transforms
class Translate : public Hittable {
  public:
	Translate(shared_ptr<Hittable> object, const Vec3& offset) : object(object), offset(offset) {
		bbox = object->bounding_box() + offset;
	}

	bool hit(const Ray& r, Interval ray_t, HitRecord& rec) const override {
		// transform ray from world to object space
		Ray shifted(r.source() - offset, r.direction(), r.time());

		// find intersection in object space
		if (!object->hit(shifted, ray_t, rec)) return false;

		// transform intersection back to world space
		rec.p += offset;
		return true;
	}

	AABB bounding_box() const override { return bbox; }

  private:
	shared_ptr<Hittable> object;
	Vec3 offset;
	AABB bbox;
};

class Transform : public Hittable {
  public:
	Transform(shared_ptr<Hittable> obj, const Mat3& t) : object(obj), trns(t) {
		bbox = object->bounding_box();

		try {
			trns_inv = trns.inv();

			// transform bounding box
			Point3 min(infinity, infinity, infinity);
			Point3 max(-infinity, -infinity, -infinity);

			for (auto x : { bbox.x.min, bbox.x.max }) {
				for (auto y : { bbox.y.min, bbox.y.max }) {
					for (auto z : { bbox.z.min, bbox.z.max }) {
						Vec3 tester = trns * Vec3(x, y, z);

						for (int c = 0; c < 3; c++) {
							min[c] = std::fmin(min[c], tester[c]);
							max[c] = std::fmax(max[c], tester[c]);
						}
					}
				}
			}

			bbox = AABB(min, max);
		} catch (std::domain_error&) {
			// gracefully ignore singular transformations
			std::cerr << "WARNING: Ignoring singular transformation by matrix:\n" << trns << '\n';
			trns = Mat3::id();
			trns_inv = Mat3::id();
		}

		// check if object is a Transform; if so, unwrap it & multiply matrices
		if (auto inner_transform = std::dynamic_pointer_cast<Transform>(object)) {
			trns = trns * inner_transform->trns;
			trns_inv = inner_transform->trns_inv * trns_inv;
			object = inner_transform->object;
		}

	}

	bool hit(const Ray& r, Interval ray_t, HitRecord& rec) const override {
		// transform ray from world to object space
		auto src = trns_inv * r.source();
		auto dir = trns_inv * r.direction();
		Ray transformed_r(src, dir, r.time());

		// find intersection in object space
		if (!object->hit(transformed_r, ray_t, rec)) return false;

		// transform intersection back to world space
		rec.p = trns * rec.p;
		rec.normal = trns_inv.transpose() * rec.normal; // cf. "surface normal transformation"
		return true;
	}

	AABB bounding_box() const override { return bbox; }

  private:
	shared_ptr<Hittable> object;
	Mat3 trns, trns_inv;
	AABB bbox;
};

class Scale : public Transform {
  public:
	Scale(shared_ptr<Hittable> obj, double sx, double sy, double sz)
		: Transform(obj, Mat3::diag(sx, sy, sz)) {}
};

class Rotate : public Transform {
  public:
	Rotate(shared_ptr<Hittable> obj, const Vec3& axis, double angle)
		: Transform(obj, Mat3::rotate(axis, angle)) {}
	
	Rotate(shared_ptr<Hittable> obj, Axis axis, double angle)
		: Transform(obj, Mat3::rotate(axis, angle)) {}
};

#endif
