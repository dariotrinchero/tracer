#pragma once

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

template <typename T>
inline T rnd_item(const std::vector<T>& items);

template <typename T, typename Callable>
inline double average(const std::vector<T>& items, const Callable&& f);

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

	void clear() {
		objects.clear();
		bbox = AABB();
	}

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
		return average(objects, [origin, direction] (shared_ptr<Hittable> obj) {
			return obj->pdf_value(origin, direction);
		});
	}

	Vec3 rnd_point(const Point3& origin) const override {
		return rnd_item(objects)->rnd_point(origin);
	}

  private:
	AABB bbox;
};

/* --- general affine transformations ----------------------------------------------------------- */

class Transform : public Hittable {
  public:
	Transform(shared_ptr<Hittable> obj, const Mat3& lin, const Vec3& shift)
		: object(obj), trns(lin), offset(shift)
	{
		try {
			trns_inv = trns.inv();
		} catch (std::domain_error&) { // gracefully ignore singular matrix
			std::cerr << "WARNING: Ignoring singular transformation by matrix:\n" << trns << '\n';
			trns = Mat3::id();
			trns_inv = Mat3::id();
		}

		bbox = trns * object->bounding_box() + offset;

		// check if object is a Transform; if so, combine matrices & offsets
		if (auto inner_transform = std::dynamic_pointer_cast<Transform>(object)) {
			offset = trns * inner_transform->offset + offset;
			trns = trns * inner_transform->trns;
			trns_inv = inner_transform->trns_inv * trns_inv;
			object = inner_transform->object;
		}
	}

	Transform(shared_ptr<Hittable> obj, const Mat3& lin)
		: Transform(obj, lin, Vec3()) {}

	Transform(shared_ptr<Hittable> obj, const Vec3& shift)
		: Transform(obj, Mat3::id(), shift) {}

	bool hit(const Ray& r, Interval ray_t, HitRecord& rec) const override {
		// find intersection in object space
		auto src = trns_inv * (r.source() - offset);
		auto dir = trns_inv * r.direction();
		Ray transformed_r(src, dir, r.time());
		if (!object->hit(transformed_r, ray_t, rec)) return false;

		// transform intersection back to world space
		rec.p = trns * rec.p + offset;
		rec.normal = trns_inv.transpose() * rec.normal; // cf. "surface normal transformation"
		return true;
	}

	AABB bounding_box() const override { return bbox; }

  private:
	shared_ptr<Hittable> object;
	Mat3 trns, trns_inv;
	Vec3 offset;
	AABB bbox;
};

/* --- assorted transformations ----------------------------------------------------------------- */

class Translate : public Transform {
  public:
	Translate(shared_ptr<Hittable> obj, const Vec3& shift) : Transform(obj, shift) {}
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
