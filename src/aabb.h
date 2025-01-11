#pragma once

#include "interval.h"
#include "ray.h"

#define MIN_DIMENSION 1e-4 // minimum dimension along any axis

/* --- class for axis-aligned bounding box ------------------------------------------------------ */

class AABB {
  public:
	Interval x, y, z;

	AABB() {} // empty by default (since intervals are)

	AABB(const Interval& x, const Interval& y, const Interval& z) : x(x), y(y), z(z) {
		pad_to_minima();
	}

	AABB(const Point3& a, const Point3& b) {
		x = a[0] <= b[0] ? Interval(a[0], b[0]) : Interval(b[0], a[0]);
		y = a[1] <= b[1] ? Interval(a[1], b[1]) : Interval(b[1], a[1]);
		z = a[2] <= b[2] ? Interval(a[2], b[2]) : Interval(b[2], a[2]);
		pad_to_minima();
	}

	AABB(const AABB& box0, const AABB& box1) {
		x = Interval(box0.x, box1.x);
		y = Interval(box0.y, box1.y);
		z = Interval(box0.z, box1.z);
	}

	const Interval& axis_interval(int n) const {
		return n == 0 ? x : (n == 1 ? y : z);
	}

	bool hit(const Ray& r, Interval ray_t) const {
		const Point3& ray_src = r.source();
		const Vec3& ray_dir = r.direction();

		for (int axis = 0; axis < 3; axis++) {
			const Interval& ax = axis_interval(axis);

			const double adinv = 1.0 / ray_dir[axis],
				t0 = (ax.min - ray_src[axis]) * adinv,
				t1 = (ax.max - ray_src[axis]) * adinv;

			if (t0 < t1) {
				if (t0 > ray_t.min) ray_t.min = t0;
				if (t1 < ray_t.max) ray_t.max = t1;
			} else {
				if (t1 > ray_t.min) ray_t.min = t1;
				if (t0 < ray_t.max) ray_t.max = t0;
			}

			if (ray_t.max <= ray_t.min) return false;
		}

		return true;
	}

	int longest_axis() const {
		if (x.size() > y.size()) return x.size() > z.size() ? 0 : 2;
		return y.size() > z.size() ? 1 : 2;
	}

	static const AABB empty, universe;

  private:
	void pad_to_minima() {
		if (x.size() < MIN_DIMENSION) x = x.expand(MIN_DIMENSION);
		if (y.size() < MIN_DIMENSION) y = y.expand(MIN_DIMENSION);
		if (z.size() < MIN_DIMENSION) z = z.expand(MIN_DIMENSION);
	}
};

/* --- bounding box operations ------------------------------------------------------------------ */

/**
 * Translate bounding box by offset vector.
 *
 * @param bbox   bounding box to translate
 * @param offset vector by which to offset bounding box
 * @returns new bounding box with all vertices offset
 */
inline AABB operator+(const AABB& bbox, const Vec3& offset) {
	return AABB(bbox.x + offset.x(), bbox.y + offset.y(), bbox.z + offset.z());
}

/**
 * Translate bounding box by offset vector.
 *
 * @overload
 */
inline AABB operator+(const Vec3& offset, const AABB& bbox) {
	return bbox + offset;
}

/**
 * Transform bounding box by general linear transformation.
 *
 * @param trns matrix giving transformation to apply
 * @param bbox bounding box to transform
 * @returns new axis-aligned bounding box enclosing the image of bbox under transformation
 */
inline AABB operator*(const Mat3& trns, const AABB& bbox) {
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

	return AABB(min, max);
}

/* --- global constants ------------------------------------------------------------------------- */

const AABB AABB::empty = AABB(Interval::empty, Interval::empty, Interval::empty);
const AABB AABB::universe = AABB(Interval::universe, Interval::universe, Interval::universe);
