#ifndef INTERVAL_H
#define INTERVAL_H

#include <limits>

/* --- class for interval ----------------------------------------------------------------------- */

class Interval {
  public:
	double min, max;

	Interval() : min(0), max(0) {} // empty by default

	Interval(double min, double max) : min(min), max(max) {}

	Interval(const Interval& a, const Interval& b) {
		// create interval tightly enclosing given intervals
		min = a.min <= b.min ? a.min : b.min;
		max = a.max >= b.max ? a.max : b.max;
	}

	double size() const {
		return min < max ? max - min : 0;
	}

	bool contains(double x) const {
		return min <= x && x <= max;
	}

	bool surrounds(double x) const {
		return min < x && x < max;
	}

	double clamp(double x) const {
		if (x < min) return min;
		if (x > max) return max;
		return x;
	}

	Interval expand(double delta) const {
		auto padding = delta / 2;
		return Interval(min - padding, max + padding);
	}

	static const Interval empty, universe, unit;
};

/* --- global constants ------------------------------------------------------------------------- */

inline Interval operator+(const Interval& ivl, double displacement) {
	return Interval(ivl.min + displacement, ivl.max + displacement);
}

inline Interval operator+(double displacement, const Interval& ivl) {
	return ivl + displacement;
}

/* --- global constants ------------------------------------------------------------------------- */

const double infinity = std::numeric_limits<double>::infinity();
const Interval Interval::empty = Interval();
const Interval Interval::universe = Interval(-infinity, infinity);
const Interval Interval::unit = Interval(0, 1);

#endif
