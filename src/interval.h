#pragma once

#include <limits>
#include <vector>
#include <initializer_list>

/* --- class for interval ----------------------------------------------------------------------- */

class Interval {
  public:
	double min, max;

	Interval() : min(1), max(-1) {} // empty by default

	Interval(double min, double max) : min(min), max(max) {}

	Interval(const Interval& a, const Interval& b) {
		// create interval tightly enclosing given intervals
		min = a.min <= b.min ? a.min : b.min;
		max = a.max >= b.max ? a.max : b.max;
	}

	double is_empty() const {
		return min > max;
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

/* --- interval operations ---------------------------------------------------------------------- */

// TODO document all these

inline Interval operator+(const Interval& ivl, double displacement) {
	return Interval(ivl.min + displacement, ivl.max + displacement);
}

inline Interval operator+(double displacement, const Interval& ivl) {
	return ivl + displacement;
}

inline Interval operator&(const Interval& i1, const Interval& i2) {
	return Interval(std::fmax(i1.min, i2.min), std::fmin(i1.max, i2.max));
}

/* --- class for union of intervals ------------------------------------------------------------- */

// TODO document; note assumption that intervals are sorted & non-overlapping
// TODO for the sake of CSG, should intervals not have entire HitRecords as endpoints?
// If not, how will we figure out what normal to use etc one we have intersected the ray t-intervals?
//
// Maybe I could make HitRecords support the <, >, <=, >= operators (by their .t fields), and make
// Interval<T> generic?

class IntervalUnion {
  public:
	std::vector<Interval> ivls;

	IntervalUnion(std::initializer_list<Interval> ivls) : ivls(ivls) {}

	Interval operator[](int i) const { return ivls[i]; }
	Interval& operator[](int i) { return ivls[i]; }

	void append(const Interval& ivl) { ivls.push_back(ivl); }
	size_t size() const { return ivls.size(); }
};

/* --- interval union operations ---------------------------------------------------------------- */

// TODO document all these

inline IntervalUnion operator&(const IntervalUnion& u1, const IntervalUnion& u2) {
	IntervalUnion intersection{};

	size_t i = 0, j = 0;
	while (i < u1.size() && j < u2.size()) {
		Interval overlap = u1[i] & u2[j];
		if (!overlap.is_empty()) intersection.append(overlap);

		if (u1[i].max < u2[j].max) i++;
		else j++;
	}

	return intersection;
}

inline IntervalUnion operator|(const Interval& i1, const Interval& i2) {
	// case 1: empty interval(s)
	if (i1.is_empty() && i2.is_empty()) return IntervalUnion{};
	if (i1.is_empty()) return IntervalUnion{i2};
	if (i2.is_empty()) return IntervalUnion{i1};

	// case 2: non-overlapping intervals
	if (i1.max < i2.min) return IntervalUnion{i1, i2};
	if (i2.max < i1.min) return IntervalUnion{i2, i1};

	// case 3: overlapping intervals
	return IntervalUnion{Interval(std::fmin(i1.min, i2.min), std::fmax(i1.max, i2.max))};
}

/* --- global constants ------------------------------------------------------------------------- */

const double infinity = std::numeric_limits<double>::infinity();
const Interval Interval::empty = Interval();
const Interval Interval::universe = Interval(-infinity, infinity);
const Interval Interval::unit = Interval(0, 1);
