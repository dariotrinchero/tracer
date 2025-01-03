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

class Material;

class HitRecord {
  public:
	Point3               p;          // point hit
	double               t;          // distance along ray of hit
	Vec3                 normal;     // normal to object at hit point
	bool                 front_face; // whether ray hit front of surface
	shared_ptr<Material> mat;        // material of hit object
	double               u,v;        // u-v texture coordinates of p

	void set_face_normal(const Ray& r, const Vec3& outward_normal) {
		// our convention: normal is opposed to ray, rather than being outward-facing;
		// record face orientation in front_face instead
		front_face = dot(r.direction(), outward_normal) < 0;
		normal = front_face ? outward_normal : -outward_normal;
	}
};

class Hittable {
  public:
	virtual ~Hittable() = default;

	virtual bool hit(const Ray& r, Interval ray_t, HitRecord& rec) const = 0;

	virtual AABB bounding_box() const = 0;
};

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

  private:
	AABB bbox;
};

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
 	Transform(shared_ptr<Hittable> object, const Mat3& t) : object(object), trns(t) {
		bbox = object->bounding_box();

		try {
			trns_inv = trns.inv();

			// compute bounding box around transformed object
			Point3 min(infinity, infinity, infinity);
			Point3 max(-infinity, -infinity, -infinity);

			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 2; j++) {
					for (int k = 0; k < 2; k++) {
						auto x = i * bbox.x.max + (1 - i) * bbox.x.min;
						auto y = j * bbox.y.max + (1 - j) * bbox.y.min;
						auto z = k * bbox.z.max + (1 - k) * bbox.z.min;
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
		// TODO THERE IS A BUG HERE! THIS BEHAVES DIFFERENTLY ON squash SCENE VS IF THIS
		// CODE IS ABSENT!!
		if (auto inner_transform = std::dynamic_pointer_cast<Transform>(object)) {
			trns = trns * inner_transform->trns;
			trns_inv = inner_transform->trns_inv * trns_inv;
			object = inner_transform->object;
		}
	}

	// TODO devise way to automatically combine chained transformations into single matrix;
	// otherwise each ray must undergo multiple transformations during rendering

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
		: Transform(obj, Mat3::scale(sx, sy, sz)) {}
};

class Rotate : public Transform {
  public:
	Rotate(shared_ptr<Hittable> obj, const Vec3& axis, double angle)
		: Transform(obj, Mat3::rotate(axis, angle)) {}
	
	Rotate(shared_ptr<Hittable> obj, Axis axis, double angle)
		: Transform(obj, Mat3::rotate(axis, angle)) {}
};

#endif
