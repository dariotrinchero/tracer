#ifndef RAY_H
#define RAY_H

#include "vec3.h"

class Ray {
  public:
	Ray() {}

	Ray(const Point3& source, const Vec3& direction, double time)
		: src(source), dir(direction), tm(time) {}

	Ray(const Point3& source, const Vec3& direction)
		: Ray(source, direction, 0) {}

	const Point3& source() const { return src; }
	const Vec3& direction() const { return dir; }
	double time() const { return tm; }

	Point3 at(double t) const {
		return src + t * dir;
	}

  private:
	Point3 src;
	Vec3   dir;
	double tm;
};

#endif
