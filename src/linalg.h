#ifndef LINALG_H
#define LINALG_H

#include <cmath>
#include <iostream>
#include <stdexcept>

#define NEAR_0_TOL 1e-8 // tolerance for testing if vectors are near 0

/* --- superclass for vector & matrix ----------------------------------------------------------- */

/* Operations like += inherited by subclasses must return those subtypes; hence, pass derived
 * class as generic to this base class (cf. 'curiously recurring template pattern'). */
template <typename T, typename Derived>
class Triple {
  public:
	T e[3];

	Triple(T e0, T e1, T e2) : e{e0, e1, e2} {}

	Derived operator-() const { return Derived(-e[0], -e[1], -e[2]); }
	T operator[](int i) const { return e[i]; }
	T& operator[](int i) { return e[i]; }

	Derived& operator+=(const Derived& v) {
		e[0] += v.e[0];
		e[1] += v.e[1];
		e[2] += v.e[2];
		return static_cast<Derived&>(*this);
	}

	Derived& operator*=(double t) {
		e[0] *= t;
		e[1] *= t;
		e[2] *= t;
		return static_cast<Derived&>(*this);
	}

	Derived& operator/=(double t) {
		return *this *= 1/t;
	}
};

/* --- arithmetic operators shared by vectors & matrices ---------------------------------------- */

template <typename T, typename Derived>
inline Derived operator+(const Triple<T, Derived>& u, const Triple<T, Derived>& v) {
	return Derived(u.e[0] + v.e[0], u.e[1] + v.e[1], u.e[2] + v.e[2]);
}

template <typename T, typename Derived>
inline Derived operator-(const Triple<T, Derived>& u, const Triple<T, Derived>& v) {
	return Derived(u.e[0] - v.e[0], u.e[1] - v.e[1], u.e[2] - v.e[2]);
}

template <typename T, typename Derived>
inline Derived operator*(double t, const Triple<T, Derived>& u) {
	return Derived(t*u.e[0], t*u.e[1], t*u.e[2]);
}

template <typename T, typename Derived>
inline Derived operator*(const Triple<T, Derived>& u, double t) {
	return t * u;
}

template <typename T, typename Derived>
inline Derived operator/(const Triple<T, Derived>& u, double t) {
	return (1/t) * u;
}

/* --- class for 3D vector ---------------------------------------------------------------------- */

class Vec3 : public Triple<double, Vec3> {
  public:
	Vec3() : Triple(0, 0, 0) {}
	Vec3(double e0, double e1, double e2) : Triple(e0, e1, e2) {}

	double x() const { return e[0]; }
	double y() const { return e[1]; }
	double z() const { return e[2]; }

	double length() const { return std::sqrt(length_squared()); }
	double length_squared() const {	return e[0] * e[0] + e[1] * e[1] + e[2] * e[2]; }
	inline Vec3 unit() const { return *this / length(); }

	bool near_zero() const {
		return std::fabs(e[0]) < NEAR_0_TOL && std::fabs(e[1]) < NEAR_0_TOL
			&& std::fabs(e[2]) < NEAR_0_TOL;
	}
};

using Point3 = Vec3;

/* --- vector operations ------------------------------------------------------------------------ */

inline std::ostream& operator<<(std::ostream& out, const Vec3& v) {
	return out << v.e[0] << ' ' << v.e[1] << ' ' << v.e[2];
}

inline double dot(const Vec3& u, const Vec3& v) {
	return u.e[0] * v.e[0] + u.e[1] * v.e[1] + u.e[2] * v.e[2];
}

inline Vec3 cross(const Vec3& u, const Vec3& v) {
	return Vec3(u.e[1] * v.e[2] - u.e[2] * v.e[1],
		u.e[2] * v.e[0] - u.e[0] * v.e[2],
		u.e[0] * v.e[1] - u.e[1] * v.e[0]);
}

/**
 * Compute the vector triple product of given three vectors (unrelated to Triple class
 * above).
 *
 * @param u, v, w input vectors
 * @returns triple product of given vectors
 */
inline double triple(const Vec3& u, const Vec3& v, const Vec3& w) {
	return dot(u, cross(v, w));
}

/**
 * Reflect given vector about the plane with given normal.
 *
 * @param u      the vector to reflect
 * @param normal the normal of the plane about which to reflect
 * @returns the reflected vector
 */
inline Vec3 reflect(const Vec3& u, const Vec3& normal) {
	return u - 2 * dot(u, normal) * normal;
}

/* --- class for 3x3 matrix --------------------------------------------------------------------- */

enum class Axis { X, Y, Z };

class Mat3 : public Triple<Vec3, Mat3> {
  public:
	Mat3() : Triple(Vec3(), Vec3(), Vec3()) {}
	Mat3(const Vec3& r1, const Vec3& r2, const Vec3 r3) : Triple(r1, r2, r3) {}
	Mat3(double e00, double e01, double e02,
		double e10, double e11, double e12,
		double e20, double e21, double e22)
		: Triple(Vec3(e00, e01, e02), Vec3(e10, e11, e12), Vec3(e20, e21, e22)) {}

	Vec3 col(int i) const { return Vec3(e[0][i], e[1][i], e[2][i]); }
	Mat3 transpose() const { return Mat3(col(0), col(1), col(2)); }

	double det() const { return triple(e[0], e[1], e[2]); }
	Mat3 inv() const {
		double determinant = det();
		if (std::fabs(determinant) < NEAR_0_TOL)
			throw std::domain_error("Attempt to invert singular matrix");
		return Mat3(cross(col(1), col(2)), cross(col(2), col(0)), cross(col(0), col(1))) / determinant;
	}

	static inline Mat3 from_cols(const Vec3& c1, const Vec3& c2, const Vec3& c3) {
		return Mat3(c1, c2, c3).transpose();
	}

	static inline Mat3 diag(double d1, double d2, double d3) {
		return Mat3(d1, 0, 0, 0, d2, 0, 0, 0, d3);
	}

	static inline Mat3 id() {
		return diag(1, 1, 1);
	}

	/**
	 * Get orthogonal matrix having given vector as final column. This amounts to extending
	 * the given vector to an orthonormal basis; vectors are expanded in this basis via
	 * matrix product: orthog(...) * vec.
	 *
	 * @param n vector to be used as final column of orthogonal matrix
	 * @returns orthogonal matrix with given vector as final column
	 */
	static inline Mat3 orthog(const Vec3& n) {
		Vec3 z_prime = n.unit();
		Vec3 a = std::fabs(z_prime.x()) > 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
		Vec3 y_prime = cross(z_prime, a).unit();
		Vec3 x_prime = cross(y_prime, z_prime);
		return Mat3::from_cols(x_prime, y_prime, z_prime);
	}

	/**
	 * Get matrix encoding 3D rotation of given angle about given axis. This fast
	 * implementation only accepts primary axes.
	 *
	 * @param axis  one of the three primary axes
	 * @param angle angle by which to rotate
	 * @returns rotation matrix with given axis & angle
	 */
	static inline Mat3 rotate(Axis axis, double angle) {
		auto c = std::cos(angle), s = std::sin(angle);
		switch (axis) {
			case Axis::X: return Mat3(1, 0, 0, 0, c, -s, 0, s, c);
			case Axis::Y: return Mat3(c, 0, s, 0, 1, 0, -s, 0, c);
			default: return Mat3(c, -s, 0, s, c, 0, 0, 0, 1);
		}
	}

	/**
	 * Get matrix encoding 3D rotation of given angle about given axis, using Rodrigues'
	 * rotation formula.
	 *
	 * @param axis  the axis about which to rotate
	 * @param angle angle by which to rotate
	 * @returns rotation matrix with given axis & angle
	 */
	static Mat3 rotate(const Vec3& axis, double angle) {
		double cos = std::cos(angle);
		Vec3 u = axis.unit();
		Vec3 ex(1, 0, 0), ey(0, 1, 0), ez(0, 0, 1);
		Mat3 outer_prod(u[0] * u, u[1] * u, u[2] * u);
		Mat3 cross_mat(cross(ex, u), cross(ey, u), cross(ez, u));
		return cos * id() + std::sin(angle) * cross_mat + (1 - cos) * outer_prod;
	}
};

/* --- matrix operations ------------------------------------------------------------------------ */

inline std::ostream& operator<<(std::ostream& out, const Mat3& m) {
	return out << m.e[0] << '\n' << m.e[1] << '\n' << m.e[2];
}

inline Vec3 operator*(const Mat3& m, const Vec3& v) {
	return Vec3(dot(m.e[0], v), dot(m.e[1], v), dot(m.e[2], v));
}

inline Mat3 operator*(const Mat3& m, const Mat3& n) {
	return Mat3::from_cols(m * n.col(0), m * n.col(1), m * n.col(2));
}

#endif
