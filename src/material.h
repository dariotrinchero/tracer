#ifndef MATERIAL_H
#define MATERIAL_H

#include <cmath>

#include "hittable.h"
#include "random.h"
#include "color.h"
#include "texture.h"

/* --- container for scattering data ------------------------------------------------------------ */

class ScatterRecord {
  public:
	Color                 attenuation;  // ray color attenuation from scattering
	shared_ptr<SpherePDF> pdf;          // PDF from which to sample scattered ray (or nullptr)
	Ray                   override_ray; // hard-coded scattered ray to use if pdf is nullptr
};

/* --- superclass for material ------------------------------------------------------------------ */

class Material {
  public:
	virtual ~Material() = default;

	/**
	 * Get color of light emitted by material at given ray intersection point.
	 *
	 * @param rec HitRecord with ray intersection details
	 * @returns color of light emitted by material at given intersection point
	 */
	virtual Color emitted(const HitRecord& rec) const {
		unused(rec);
		return black;
	}

	/**
	 * Determine whether given incoming ray scatters upon collision with material. If so,
	 * record details of scattering event (attenutation, scattered ray, etc).
	 *
	 * @param r_in[in]  incoming ray
	 * @param rec[in]   HitRecord with incoming ray intersection details
	 * @param srec[out] populated with details of scattering if scattering occurs
	 * @returns whether incoming ray is scattered
	 */
	virtual bool scatter(const Ray& r_in, const HitRecord& rec, ScatterRecord& srec) const {
		unused(r_in, rec, srec);
		return false;
	}
};

/* --- assorted materials ----------------------------------------------------------------------- */

class Lambertian : public Material { // aka. matte material
  public:
	Lambertian(const Color& albedo) : tex(make_shared<SolidColor>(albedo)) {}

	Lambertian(shared_ptr<Texture> tex) : tex(tex) {}

	bool scatter(const Ray&, const HitRecord& rec, ScatterRecord& srec) const override {
		srec.attenuation = tex->value(rec.u, rec.v, rec.p);
		srec.pdf = make_shared<CosinePDF>(rec.normal);
		return true;
	}

  private:
	shared_ptr<Texture> tex;
};

class Metal : public Material { // aka. mirror
  public:
	Metal(const Color& albedo, double fuzz = 0)
		: albedo(albedo), fuzz(fuzz < 1 ? fuzz : 1) {}

	bool scatter(const Ray& r_in, const HitRecord& rec, ScatterRecord& srec) const override {
		Vec3 reflected = reflect(r_in.direction(), rec.normal);
		reflected = reflected.unit() + (fuzz * rnd_unit_vec());

		srec.attenuation = albedo;
		srec.pdf = nullptr;
		srec.override_ray = Ray(rec.p, reflected, r_in.time());

		return dot(reflected, rec.normal) > 0;
	}

  private:
	Color albedo;
	double fuzz;
};

class Dielectric : public Material { // aka. glass
  public:
	Dielectric(double refractive_index) : refractive_index(refractive_index) {}

	bool scatter(const Ray& r_in, const HitRecord& rec, ScatterRecord& srec) const override {
		double ri = rec.front_face ? 1 / refractive_index : refractive_index;

		Vec3 in_dir = r_in.direction().unit();
		double cos_theta = std::fmin(dot(-in_dir, rec.normal), 1);
		double sin_theta = std::sqrt(1 - cos_theta * cos_theta);

		bool cannot_refract = ri * sin_theta > 1; // check for total internal reflection
		Vec3 direction;

		if (cannot_refract || reflectance(cos_theta, ri) > rnd_double())
			direction = reflect(in_dir, rec.normal);
		else direction = refract(in_dir, rec.normal, ri, cos_theta);

		srec.attenuation = white; // for tint (subsurface scattering), render volume inside dielectric
		srec.pdf = nullptr;
		srec.override_ray = Ray(rec.p, direction, r_in.time());

		return true;
	}

  private:
	double refractive_index;

	/**
	 * Refract incident light ray through boundary with given normal & refractive index.
	 *
	 * @param in        incident light ray direction (assumed to be unit vector)
	 * @param normal    normal to boundary between media
	 * @param ri        relative refractive index (index of medium exited over that entered)
	 * @param cos_theta cosine of angle between incoming light ray & normal (optional)
	 * @returns direction of refracted light ray
	 */
	static Vec3 refract(const Vec3& in, const Vec3& normal, double ri, double cos_theta = 2) {
		if (cos_theta > 1) cos_theta = std::fmin(dot(-in, normal), 1);
		Vec3 r_out_perp = ri * (in + cos_theta * normal);
		Vec3 r_out_parallel = -std::sqrt(std::fmax(1 - r_out_perp.length_squared(), 0)) * normal;
		return r_out_perp + r_out_parallel;
	}

	/**
	 * Get reflectance of boundary between optical media as function of incoming ray angle,
	 * using Schlick's approximation of Fresnel equations.
	 *
	 * @param cos_theta cosine of angle between incoming light ray & normal to boundary
	 * @param ri        relative refractive index (index of medium exited over that entered)
	 * @returns approximate reflectance value in range [0,1)
	 */
	static double reflectance(double cos_theta, double ri) {
		auto r0 = (1 - ri) / (1 + ri);
		r0 *= r0;
		return r0 + (1 - r0) * std::pow(1 - cos_theta, 5);
	}
};

class DiffuseLight : public Material { // aka. emitter
  public:
	DiffuseLight(shared_ptr<Texture> tex) : tex(tex) {}
	DiffuseLight(const Color& emit) : tex(make_shared<SolidColor>(emit)) {}

	Color emitted(const HitRecord& rec) const override {
		if (!rec.front_face) return black; // only emit from front face
		return tex->value(rec.u, rec.v, rec.p);
	}

  private:
	shared_ptr<Texture> tex;
};

class Isotropic : public Material { // aka. smoke
  public:
	Isotropic(const Color& albedo) : tex(make_shared<SolidColor>(albedo)) {}
	Isotropic(shared_ptr<Texture> tex) : tex(tex) {}

	bool scatter(const Ray&, const HitRecord& rec, ScatterRecord& srec) const override {
		srec.attenuation = tex->value(rec.u, rec.v, rec.p);
		srec.pdf = make_shared<UniformPDF>();
		return true;
	}

  private:
	shared_ptr<Texture> tex;
};

#endif
