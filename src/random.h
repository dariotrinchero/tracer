#ifndef RANDOM_H
#define RANDOM_H

#include <cstdlib>
#include <cmath>

#include "linalg.h"
#include "hittable.h"

#define PI 3.1415926535897932385

/* --- general RNG utility functions ------------------------------------------------------------ */

/**
 * Generate random uniformly-distributed fractional double.
 *
 * @returns random double in range [0,1)
 */
inline double rnd_double() {
	return std::rand() / (RAND_MAX + 1.0);
}

/**
 * Generate random double uniformly-distributed in specified range.
 *
 * @param min, max bounds of range
 * @returns random double in range [min,max)
 */
inline double rnd_double(double min, double max) {
	return min + (max - min) * rnd_double();
}

/**
 * Generate random int uniformly-distributed in specified range.
 *
 * @param min, max bounds of range
 * @returns random int in range [min,max]
 */
inline int rnd_int(int min, int max) {
	return int(rnd_double(min, max + 1));
}

/* --- random vector utility functions ---------------------------------------------------------- */

/**
 * Generate random vector uniformly-distributed in first octant.
 *
 * @returns random vector with all components in range [0,1)
 */
inline Vec3 rnd_vec() {
	return Vec3(rnd_double(), rnd_double(), rnd_double());
}

/**
 * Generate random vector uniformly-distributed in specified cube.
 *
 * @param min, max bounds of range of each component
 * @returns random vector with components all in range [min,max)
 */
inline Vec3 rnd_vec(double min, double max) {
	return Vec3(rnd_double(min, max), rnd_double(min, max), rnd_double(min, max));
}

/**
 * Generate (by rejection sampling) random unit vector, uniformly-distributed on unit sphere.
 *
 * @returns random vector of unit length
 */
inline Vec3 rnd_unit_vec() {
	while (true) {
		Vec3 r = rnd_vec(-1, 1);
		double lensq = r.length_squared();
		if (1e-160 < lensq && lensq <= 1) return r / std::sqrt(lensq);
	}
}

/**
 * Generate random vector uniformly-distributed on unit hemisphere lying on the same side
 * of the plane with given normal as that normal.
 *
 * @param normal the normal to plane bisecting unit sphere
 * @returns random unit vector having positive dot product with given normal
 */
inline Vec3 rnd_vec_hemisphere(const Vec3& normal) {
	Vec3 r = rnd_unit_vec();
	return dot(r, normal) > 0 ? r : -r;
}

/**
 * Generate (by rejection sampling) random vector uniformly-distributed within unit disk
 * on xy-plane.
 *
 * @returns random vector in unit disk on xy-plane
 */
inline Vec3 rnd_vec_unit_disk() {
	while (true) {
		Vec3 r = Vec3(rnd_double(-1, 1), rnd_double(-1, 1), 0);
		if (r.length_squared() < 1.0) return r;
	}
}

/**
 * Generate (by inverse transform sampling) random unit vector, distributed according to
 * PDF, p(v) = cos(theta)/pi, for unit vector v, and angle theta between v & the z-axis.
 * This is the scattering distribution of a Lambertian material.
 *
 * @returns random unit vector with cosine distribution centered on z-axis
 */
inline Vec3 rnd_cosine_direction() {
	auto r1 = rnd_double(), r2 = rnd_double();
	auto phi = 2 * PI * r1;
	auto sqrt_r2 = std::sqrt(r2);
	return Vec3(std::cos(phi) * sqrt_r2, std::sin(phi) * sqrt_r2, std::sqrt(1 - r2));
}

/* --- superclass for PDF on unit sphere -------------------------------------------------------- */

class SpherePDF {
  public:
	virtual ~SpherePDF() {}

	/**
	 * Get value of PDF at given point in sample space (ie. unit vector).
	 *
	 * @param direction point on unit sphere at which to evaluate PDF
	 * @returns value of PDF at given sample
	 */
	virtual double density(const Vec3& direction) const = 0;

	/**
	 * Generate random sample (ie. unit vector) from distribution given by PDF.
	 *
	 * @returns random unit vector distributed by this PDF
	 */
	virtual Vec3 sample() const = 0;
};

/* --- class for linear combination of PDFs ----------------------------------------------------- */

class MixturePDF : public SpherePDF {
  public:
	// TODO allow more than two PDFs?
	MixturePDF(shared_ptr<SpherePDF> p0, shared_ptr<SpherePDF> p1) : p{p0, p1} {}

	double density(const Vec3& direction) const override {
		// TODO allow variable relative weightings?
		return 0.5 * (p[0]->density(direction) + p[1]->density(direction));
	}

	Vec3 sample() const override {
		if (rnd_double() < 0.5) return p[0]->sample();
		return p[1]->sample();
	}

  private:
	shared_ptr<SpherePDF> p[2];
};

/* --- assorted PDFs on unit sphere ------------------------------------------------------------- */

class UniformPDF : public SpherePDF {
  public:
	UniformPDF() {}

	double density(const Vec3&) const override {
		return 1 / (4 * PI);
	}

	Vec3 sample() const override {
		return rnd_unit_vec();
	}
};

// TODO document this; see rnd_cosine_direction() above
// TODO maybe delete utility function above entirely, if it is only called here?
class CosinePDF : public SpherePDF {
  public:
	CosinePDF(const Vec3& w) {
		onb = Mat3::orthog(w);
	}

	double density(const Vec3& direction) const override {
		auto cos_theta = dot(direction.unit(), onb.col(2));
		return std::fmax(0, cos_theta / PI);
	}

	Vec3 sample() const override {
		return onb * rnd_cosine_direction();
	}

  private:
	Mat3 onb;
};

// TODO document this
class HittablePDF : public SpherePDF {
  public:
	HittablePDF(const Hittable& objects, const Point3& origin) : objects(objects), origin(origin) {}

	double density(const Vec3& direction) const override {
		return objects.pdf_value(origin, direction);
	}

	Vec3 sample() const override {
		return objects.rnd_point(origin);
	}

  private:
	const Hittable& objects;
	Point3 origin;
};

/* --- class for 3D Perlin noise ---------------------------------------------------------------- */

class Perlin {
  public:
	Perlin() {
		for (int i = 0; i < point_count; i++) rand_vec[i] = rnd_vec(-1, 1).unit();
		perlin_generate_perm(perm_x);
		perlin_generate_perm(perm_y);
		perlin_generate_perm(perm_z);
	}

	/**
	 * Get value of Perlin noise at given point.
	 *
	 * @param p point at which to sample noise
	 * @returns value of Perlin noise at given point
	 */
	double at(const Point3& p) const {
		auto i = int(std::floor(p.x()));
		auto j = int(std::floor(p.y()));
		auto k = int(std::floor(p.z()));
		auto u = p.x() - i, v = p.y() - j, w = p.z() - k;

		Vec3 c[2][2][2];
		for (int di = 0; di < 2; di++)
			for (int dj = 0; dj < 2; dj++)
				for (int dk = 0; dk < 2; dk++)
					c[di][dj][dk] = rand_vec[
						perm_x[(i + di) & 255] ^
						perm_y[(j + dj) & 255] ^
						perm_z[(k + dk) & 255]
					];

		return perlin_interp(c, u, v, w);
	}

	/**
	 * Get weighted sum of several layers of Perlin noise, each having double the frequency
	 * & half the amplitude of the previous. The resulting noise is marble-like.
	 *
	 * @param p     point at which to sample turbulence noise
	 * @param depth number of layers to sum
	 * @returns value of weighted sum of Perlin noise layers at given point
	 */
	double turbulence(const Point3& p, int depth) const {
		auto accum = 0.0;
		auto temp_p = p;
		auto weight = 1.0;

		for (int i = 0; i < depth; i++) {
			accum += weight * at(temp_p);
			weight *= 0.5;
			temp_p *= 2;
		}
		return std::fabs(accum);
	}

  private:
	static const int point_count = 256;
	Vec3 rand_vec[point_count];
	int perm_x[point_count], perm_y[point_count], perm_z[point_count];

	static void perlin_generate_perm(int* p) {
		for (int i = 0; i < point_count; i++) p[i] = i;

		// Fisher-Yates shuffle
		for (int i = point_count - 1; i > 0; i--) {
			int target = rnd_int(0, i);
			int tmp = p[i];
			p[i] = p[target];
			p[target] = tmp;
		}
	}

	static double perlin_interp(const Vec3 c[2][2][2], double u, double v, double w) {
		auto uu = u * u * (3 - 2 * u);
		auto vv = v * v * (3 - 2 * v);
		auto ww = w * w * (3 - 2 * w);
		auto accum = 0.0;

		for (int i = 0; i < 2; i++)
			for (int j = 0; j < 2; j++)
				for (int k = 0; k < 2; k++) {
					Vec3 weight_v(u - i, v - j, w - k);
					accum += (i * uu + (1 - i) * (1 - uu))
						* (j * vv + (1 - j) * (1 - vv))
						* (k * ww + (1 - k) * (1 - ww))
						* dot(c[i][j][k], weight_v);
				}
		return accum;
	}
};

#endif
