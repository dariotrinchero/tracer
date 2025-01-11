#pragma once

#include "hittable.h"
#include "material.h"
#include "random.h"

class ConstantMedium : public Hittable {
  public:
	ConstantMedium(shared_ptr<Hittable> boundary, double density, shared_ptr<Texture> tex)
		: boundary(boundary), neg_inv_density(-1 / density), phase_function(make_shared<Isotropic>(tex)) {}

	ConstantMedium(shared_ptr<Hittable> boundary, double density, const Color& albedo)
		: boundary(boundary), neg_inv_density(-1 / density), phase_function(make_shared<Isotropic>(albedo)) {}
	
	bool hit(const Ray& r, Interval ray_t, HitRecord& rec) const override {
		HitRecord rec1, rec2;

		// find where ray enters (could be behind source) & exits medium
		if (!boundary->hit(r, Interval::universe, rec1)) return false;
		if (!boundary->hit(r, Interval(rec1.t + 1e-4, infinity), rec2)) return false;

		// shrink interval so [rec1.t,rec2.t] is largest considered duration spent in medium
		if (rec1.t < ray_t.min) rec1.t = ray_t.min;
		if (rec2.t > ray_t.max) rec2.t = ray_t.max;
		if (rec1.t >= rec2.t) return false;

		// check if ray passes through enough medium to interact
		auto ray_length = r.direction().length();
		auto distance_in_medium = (rec2.t - rec1.t) * ray_length;
		auto hit_distance = neg_inv_density * std::log(rnd_double());
		if (hit_distance > distance_in_medium) return false;

		// TODO this method only works for convex boundary; generalize?
		// I think we can recursively call this method instead of returning false above,
		// only now limit the rec1 hit to have t > rec2.t

		// record hit
		rec.t = rec1.t + hit_distance / ray_length;
		rec.p = r.at(rec.t);
		rec.mat = phase_function;
		// TODO We don't set u,v. How is isotropic material meant to use a general texture?
		// Also, should we set normal and front_face?
		return true;
	}

	AABB bounding_box() const override { return boundary->bounding_box(); }
	
  private:
	shared_ptr<Hittable> boundary;
	double               neg_inv_density;
	shared_ptr<Material> phase_function;
};
