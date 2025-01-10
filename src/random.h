#ifndef RANDOM_H
#define RANDOM_H

#include <cstdlib>
#include <cmath>
#include <vector>
#include <initializer_list>
#include <stdexcept>
#include <functional>

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

/**
 * Get random item from given list.
 *
 * @param items list (vector) of items from which to draw
 * @returns random uniformly-selected item from given list
 */
template <typename T>
inline T rnd_item(const std::vector<T>& items) {
	if (items.size() == 0) throw std::runtime_error("Cannot draw random item from empty list.");
	return items[rnd_int(0, items.size() - 1)];
}

/**
 * Average outputs of given function over items of given list. Helper function for
 * MixturePDF class below, as well as HittableList.
 *
 * @param items list (vector) of items on which to call function
 * @param f     function (or callable) to evaluate on each item in list
 * @returns sum of outputs of f, divided by number of items in list
 */
template <typename T, typename Callable>
inline double average(const std::vector<T>& items, const Callable&& f) {
	if (items.size() == 0) throw std::runtime_error("Cannot average over empty list.");

	double sum = 0;
	for (const auto& item : items) sum += f(item);
	return sum / items.size();
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
	MixturePDF(std::initializer_list<shared_ptr<SpherePDF>> pdfs) : pdfs(pdfs) {}

	void add(shared_ptr<SpherePDF> pdf) { pdfs.push_back(pdf); }

	double density(const Vec3& direction) const override {
		return average(pdfs, [direction] (shared_ptr<SpherePDF> pdf) {
			return pdf->density(direction);
		});
	}

	Vec3 sample() const override { return rnd_item(pdfs)->sample(); }

  private:
	std::vector<shared_ptr<SpherePDF>> pdfs;
};

/* --- assorted PDFs on unit sphere ------------------------------------------------------------- */

class UniformPDF : public SpherePDF {
  public:
	UniformPDF() {}

	double density(const Vec3&) const override { return 1 / (4 * PI); }

	Vec3 sample() const override { return rnd_unit_vec(); }
};

/**
 * PDF given by p(v)=cos(theta)/pi, for angle theta between unit vector v & fixed axis w.
 * This is the scattering distribution of a Lambertian material.
 */
class CosinePDF : public SpherePDF {
  public:
	CosinePDF(const Vec3& w) : onb(Mat3::orthog(w)) {}

	double density(const Vec3& direction) const override {
		auto cos_theta = dot(direction.unit(), onb.col(2));
		return std::fmax(0, cos_theta / PI);
	}

	Vec3 sample() const override {
		double phi = 2 * PI * rnd_double();
		double r2 = rnd_double();
		double sqrt_r2 = std::sqrt(r2);
		Vec3 direction(std::cos(phi) * sqrt_r2, std::sin(phi) * sqrt_r2, std::sqrt(1 - r2));
		return onb * (direction.near_zero() ? Vec3(1, 0, 0) : direction);
	}

  private:
	Mat3 onb;
};

/**
 * PDF induced on unit sphere of directions by having them face towards a uniformly-
 * distributed random point on given Hittable object (that is visible from given origin).
 */
class HittablePDF : public SpherePDF {
  public:
	HittablePDF(const Hittable& obj, const Point3& origin) : obj(obj), origin(origin) {}

	double density(const Vec3& direction) const override {
		return obj.pdf_value(origin, direction);
	}

	Vec3 sample() const override { return obj.rnd_point(origin); }

  private:
	const Hittable& obj;
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

	/**
	 * Generate permutation of length point_count; ie. an array with random ordering of
	 * {0, 1, 2, ..., point_count-1}.
	 *
	 * @param[out] p pointer to array to populate with permutation
	 */
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
