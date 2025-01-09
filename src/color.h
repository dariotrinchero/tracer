#ifndef COLOR_H
#define COLOR_H

#include <iostream>
#include <cmath>

#include "interval.h"
#include "linalg.h"

#define RGB_MAX 255.999

using byte = unsigned char;

/* --- class for color -------------------------------------------------------------------------- */

class Color : public Triple<double, Color> {
  public:
	Color() : Triple(0, 0, 0) {}

	Color(double r, double g, double b) : Triple(r, g, b) {}

	explicit Color(int hex)
		: Triple(((hex >> 16) & 0xFF) / 255.0, ((hex >> 8) & 0xFF) / 255.0, (hex & 0xFF) / 255.0) {}

	Color(const byte bytes[3]) : Triple(bytes[0] / 255.0, bytes[1] / 255.0, bytes[2] / 255.0) {}

	Color to_gamma(double gamma = 2.0) const {
		return Color(linear_to_gamma(e[0], gamma),
			linear_to_gamma(e[1], gamma),
			linear_to_gamma(e[2], gamma));
	}

	Color to_linear(double gamma = 2.0) const {
		return Color(gamma_to_linear(e[0], gamma),
			gamma_to_linear(e[1], gamma),
			gamma_to_linear(e[2], gamma));
	}

	// RBG values in range [0,255]
	int r() const { return int(RGB_MAX * e[0]); }
	int g() const { return int(RGB_MAX * e[1]); }
	int b() const { return int(RGB_MAX * e[2]); }

  private:
	static inline double linear_to_gamma(double linear_component, double gamma) {
		if (std::isnan(linear_component)) linear_component = 0;
		return std::pow(Interval::unit.clamp(linear_component), 1.0 / gamma);
	}

	static inline double gamma_to_linear(double gamma_component, double gamma) {
		if (std::isnan(gamma_component)) gamma_component = 0;
		return std::pow(Interval::unit.clamp(gamma_component), gamma);
	}
};

/* --- color operations ------------------------------------------------------------------------- */

inline Color operator*(const Color& col1, const Color& col2) {
	// point-wise (Hadamard) product (used for scattered ray attenuation)
    return Color(col1.e[0] * col2.e[0], col1.e[1] * col2.e[1], col1.e[2] * col2.e[2]);
}

/* --- global constants ------------------------------------------------------------------------- */

const Color white(1, 1, 1);
const Color black(0, 0, 0);

#endif
