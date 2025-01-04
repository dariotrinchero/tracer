#ifndef MATERIAL_H
#define MATERIAL_H

#include <cmath>

#include "hittable.h"
#include "random.h"
#include "color.h"
#include "texture.h"

/* --- superclass for material ------------------------------------------------------------------ */

class Material {
  public:
	virtual ~Material() = default;

	virtual Color emitted(double, double, const Point3&) const { return black; }

	virtual bool scatter(const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered) const = 0;
};

/* --- assorted materials ----------------------------------------------------------------------- */

class Lambertian : public Material { // aka. matte material
  public:
	Lambertian(const Color& albedo) : tex(make_shared<SolidColor>(albedo)) {}

	Lambertian(shared_ptr<Texture> tex) : tex(tex) {}

	bool scatter(const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered) const override {
		auto scatter_direction = rec.normal + rnd_unit_vec();

		// catch degenerate scatter direction
		if (scatter_direction.near_zero()) scatter_direction = rec.normal;

		scattered = Ray(rec.p, scatter_direction, r_in.time());
		attenuation = tex->value(rec.u, rec.v, rec.p);
		return true;
	}

  private:
	shared_ptr<Texture> tex;
};

class Metal : public Material { // aka. mirror
  public:
	Metal(const Color& albedo, double fuzz = 0)
		: albedo(albedo), fuzz(fuzz < 1 ? fuzz : 1) {}

	bool scatter(const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered) const override {
		Vec3 reflected = reflect(r_in.direction(), rec.normal);
		reflected = reflected.unit() + (fuzz * rnd_unit_vec());
		scattered = Ray(rec.p, reflected, r_in.time());
		attenuation = albedo;
		return dot(reflected, rec.normal) > 0;
	}

  private:
	Color albedo;
	double fuzz;
};

class Dielectric : public Material { // aka. glass
  public:
	Dielectric(double refractive_index) : refractive_index(refractive_index) {}

	bool scatter(const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered) const override {
		double ri = rec.front_face ? 1 / refractive_index : refractive_index;

		Vec3 in_dir = r_in.direction().unit();
		double cos_theta = std::fmin(dot(-in_dir, rec.normal), 1);
		double sin_theta = std::sqrt(1 - cos_theta * cos_theta);

		bool cannot_refract = ri * sin_theta > 1;
		Vec3 direction;

		if (cannot_refract || reflectance(cos_theta, ri) > rnd_double())
			direction = reflect(in_dir, rec.normal);
		else direction = refract(in_dir, rec.normal, ri, cos_theta);

		scattered = Ray(rec.p, direction, r_in.time());
		attenuation = white; // for tinted glass (subsurface scattering), render volume inside dielectric
		return true;
	}

  private:
	double refractive_index;

	static Vec3 refract(const Vec3& in, const Vec3& normal, double ri, double cos_theta) {
		// refract incident vector through boundary with given normal & refractive index
		Vec3 r_out_perp = ri * (in + cos_theta * normal);
		Vec3 r_out_parallel = -std::sqrt(std::fmax(1 - r_out_perp.length_squared(), 0)) * normal;
		return r_out_perp + r_out_parallel;
	}

	static double reflectance(double cosine, double refraction_index) {
		// Schlick's approximation of Fresnel equations
		auto r0 = (1 - refraction_index) / (1 + refraction_index);
		r0 *= r0;
		return r0 + (1 - r0) * std::pow(1 - cosine, 5);
	}
};

class DiffuseLight : public Material { // aka. emitter
  public:
	DiffuseLight(shared_ptr<Texture> tex) : tex(tex) {}
	DiffuseLight(const Color& emit) : tex(make_shared<SolidColor>(emit)) {}

	Color emitted(double u, double v, const Point3& p) const override {
		return tex->value(u, v, p);
	}

	bool scatter(const Ray&, const HitRecord&, Color&, Ray&) const override { return false; }

  private:
	shared_ptr<Texture> tex;
};

class Isotropic : public Material { // aka. smoke
  public:
	Isotropic(const Color& albedo) : tex(make_shared<SolidColor>(albedo)) {}
	Isotropic(shared_ptr<Texture> tex) : tex(tex) {}

	bool scatter(const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered) const override {
		scattered = Ray(rec.p, rnd_unit_vec(), r_in.time());
		attenuation = tex->value(rec.u, rec.v, rec.p);
		return true;
	}

  private:
	shared_ptr<Texture> tex;
};

#endif
