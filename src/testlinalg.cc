#include <iostream>
#include <cassert>
#include <string>
#include <cmath>

#include "linalg.h"

#define PI 3.1415926535897932385

// test utility functions

#define BOLD_RED "\033[91m\033[1m"
#define UNBOLD   "\033[22m"

#define ASSERT_EQUAL(x, y) { assert_equal(x, y, __FUNCTION__, __LINE__); }

#define EXPECTED(type, lpad, rpad, end) std::cerr << BOLD_RED << fn << ':' << line << UNBOLD\
	<< ": expected " << type << lpad << expected << rpad << "but received" << lpad << test\
	<< rpad << end;

inline void assert_equal(double test, double expected, const std::string& fn, int line) {
	if (std::fabs(expected - test) > 1e-8) EXPECTED("double", ' ', ' ', '\n');
}

inline void assert_equal(const Vec3& test, const Vec3& expected, const std::string& fn, int line) {
	if (!(expected - test).near_zero()) EXPECTED("Vec3", " (", ") ", '\n');
}

inline void assert_equal(const Mat3& test, const Mat3& expected, const std::string& fn, int line) {
	Mat3 diff = expected - test;
	if (!diff[0].near_zero() || !diff[1].near_zero() || !diff[2].near_zero())
		EXPECTED("Mat3", '\n', '\n', "");
}

// test cases

void test_length() {
	Vec3 v(1, 3, -4), w(2, 5, 0);
	ASSERT_EQUAL(v.length_squared(), 26);
	ASSERT_EQUAL(v.length(), std::sqrt(26));
	ASSERT_EQUAL(w.length(), std::sqrt(29));
	ASSERT_EQUAL(v.unit(), (1 / std::sqrt(26)) * v);
}

void test_cross_product() {
	Vec3 v(1, 3, -4), w(2, 5, 0);
	ASSERT_EQUAL(cross(v, w), Vec3(20, -8, -1));
	ASSERT_EQUAL(triple(v, w, v), 0);
	ASSERT_EQUAL(triple(v, v, w), 0);
	ASSERT_EQUAL(triple(w, v, w), 0);
}

void test_mat3_constructors() {
	Vec3 u(1, 3, -4), v(2, 5, 0), w(-1, 0, 4);
	Mat3 m1(1, 3, -4, 2, 5, 0, -1, 0, 4);
	Mat3 m2(u, v, w);
	Mat3 m3 = Mat3::from_cols(Vec3(1, 2, -1), Vec3(3, 5, 0), Vec3(-4, 0, 4));
	ASSERT_EQUAL(m1, m2);
	ASSERT_EQUAL(m2, m3);
}

void test_det() {
	Vec3 u(1, 3, -4), v(2, 5, 0), w(-1, 0, 4);
	Mat3 m(u, v, w);
	ASSERT_EQUAL(m.det(), -24);
	Mat3 singular(u, v, u);
	ASSERT_EQUAL(singular.det(), 0);
}

void test_transpose() {
	Vec3 u(1, 3, -4), v(2, 5, 0), w(-1, 0, 4);
	Mat3 m(u, v, w);
	Mat3 mt = Mat3(Vec3(1, 2, -1), Vec3(3, 5, 0), Vec3(-4, 0, 4));
	ASSERT_EQUAL(m.transpose(), mt);
}

void test_matrix_product() {
	Mat3 m1(1, 3, -4, 2, 5, 0, -1, 0, 4);
	Mat3 m2(8, -3, 4, 0, 2, 5, 0.5, 1, 3);
	Mat3 prod(6, -1, 7, 16, 4, 33, -6, 7, 8);
	ASSERT_EQUAL(m1 * m2, prod);
}

void test_inverse() {
	Vec3 u(1, 3, -4), v(2, 5, 0), w(-1, 0, 4);
	Mat3 m(u, v, w);
	Mat3 m_inv(-5/6.0, 1/2.0, -5/6.0, 1/3.0, 0, 1/3.0, -5/24.0, 1/8.0, 1/24.0);
	ASSERT_EQUAL(m.inv(), m_inv);
	ASSERT_EQUAL(m * m.inv(), Mat3::id());
	ASSERT_EQUAL(m.inv() * m, Mat3::id());
}

void test_rotations() {
	Vec3 u(1, 3, -4), v(2, 5, 0), w(-1, 0, 4);

	Mat3 u_quart = Mat3::rotate(u, PI / 2);
	Mat3 v_half = Mat3::rotate(v, PI);
	Mat3 w_full = Mat3::rotate(w, 2 * PI);

	ASSERT_EQUAL(u_quart * u_quart * u_quart * u_quart, Mat3::id());
	ASSERT_EQUAL(v_half * v_half, Mat3::id());
	ASSERT_EQUAL(w_full, Mat3::id());

	ASSERT_EQUAL(Mat3::rotate(Vec3(1, 0, 0), 35.422), Mat3::rotate(Axis::X, 35.422));
	ASSERT_EQUAL(Mat3::rotate(Vec3(0, 1, 0), 35.422), Mat3::rotate(Axis::Y, 35.422));
	ASSERT_EQUAL(Mat3::rotate(Vec3(0, 0, 1), 35.422), Mat3::rotate(Axis::Z, 35.422));
}

int main() {
	test_length();
	test_cross_product();
	test_mat3_constructors();
	test_det();
	test_transpose();
	test_matrix_product();
	test_inverse();
	test_rotations();
}
