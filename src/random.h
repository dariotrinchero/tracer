#ifndef RANDOM_H
#define RANDOM_H

#include <cstdlib>
#include <cmath>

#include "vec3.h"

inline double rnd_double() {
	// random double in range [0,1)
	return std::rand() / (RAND_MAX + 1.0);
}

inline double rnd_double(double min, double max) {
	// random double in range [min,max)
	return min + (max - min) * rnd_double();
}

inline int rnd_int(int min, int max) {
	// random int in range [min,max]
	return int(rnd_double(min, max + 1));
}

inline Vec3 rnd_vec() {
	return Vec3(rnd_double(), rnd_double(), rnd_double());
}

inline Vec3 rnd_vec(double min, double max) {
	return Vec3(rnd_double(min, max), rnd_double(min, max), rnd_double(min, max));
}

inline Vec3 rnd_unit_vec() {
	while (true) {
		Vec3 r = rnd_vec(-1, 1);
		double lensq = r.length_squared();
		if (1e-160 < lensq && lensq <= 1) return r / sqrt(lensq);
	}
}

inline Vec3 rnd_vec_hemisphere(const Vec3& normal) {
	Vec3 r = rnd_unit_vec();
	return dot(r, normal) > 0 ? r : -r;
}

inline Vec3 rnd_vec_unit_disk() {
	// rejection method: faster than polar because of avoided sin, cos, & sqrt
	while (true) {
		Vec3 r = Vec3(rnd_double(-1, 1), rnd_double(-1, 1), 0);
		if (r.length_squared() < 1.0) return r;
	}
}

class Perlin {
  public:
	Perlin() {
		for (int i = 0; i < point_count; i++) rand_vec[i] = rnd_vec(-1, 1).unit();
		perlin_generate_perm(perm_x);
		perlin_generate_perm(perm_y);
		perlin_generate_perm(perm_z);
	}

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
