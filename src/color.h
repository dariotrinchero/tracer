#ifndef COLOR_H
#define COLOR_H

#include <iostream>
#include <cmath>

#include "interval.h"
#include "vec3.h"

#define RGB_MAX 255.999

using Color = Vec3;

inline double linear_to_gamma(double linear_component, double gamma) {
	if (linear_component <= 0) return 0;
	return std::pow(linear_component, 1.0 / gamma);
}

inline Color linear_to_gamma(const Color& linear_color, double gamma) {
	return Color(linear_to_gamma(linear_color.x(), gamma),
		linear_to_gamma(linear_color.y(), gamma),
		linear_to_gamma(linear_color.z(), gamma));
}

void write_color(std::ostream& out, const Color& pixel_color, double gamma) {
	// scale from [0,1] to [0,255]
	Color col = linear_to_gamma(pixel_color, gamma);
	int r = int(RGB_MAX * Interval::unit.clamp(col.x()));
	int g = int(RGB_MAX * Interval::unit.clamp(col.y()));
	int b = int(RGB_MAX * Interval::unit.clamp(col.z()));

	// TODO does the flush here slow things down? benchmark this...
	out << r << ' ' << g << ' ' << b << '\n' << std::flush;
}

const Color white(1, 1, 1);
const Color black(0, 0, 0);

#endif
