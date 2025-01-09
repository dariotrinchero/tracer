#ifndef MATERIAL_H
#define MATERIAL_H

#include <cmath>

#include "hittable.h"
#include "random.h"
#include "color.h"
#include "texture.h"

// TODO is this still needed after PDF class in random.h?
#define PI 3.1415926535897932385

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
	 * record the outgoing scattered ray, color attenuation, and value of scattering PDF,
	 * as evaluated for this instance of scattering.
	 *
	 * @param r_in[in]         incoming ray
	 * @param rec[in]          HitRecord with incoming ray intersection details
	 * @param attenuation[out] populated with ray color attenuation if scattering occurs
	 * @param scattered[out]   populated with outgoing scattered ray if scattering occurs
	 * @param pdf[out]         populated with scattering PDF sample if scattering occurs
	 * @returns whether incoming ray is scattered
	 */
	virtual bool scatter(
		const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered, double& pdf
	) const {
		unused(r_in, rec, attenuation, scattered, pdf);
		return false;
	}

	/**
	 * Probability distribution function (PDF) for scattering.
	 *
	 * @param r_in      incoming ray
	 * @param rec       HitRecord with incoming ray intersection details
	 * @param scattered a possible outgoing scattered ray
	 * @returns probability density for r_in to be scattered to given outgoing ray
	 */
	virtual double scatter_pdf(const Ray& r_in, const HitRecord& rec, const Ray& scattered) const {
		unused(r_in, rec, scattered);
		return 0;
	}
};

/* --- assorted materials ----------------------------------------------------------------------- */

class Lambertian : public Material { // aka. matte material
  public:
	Lambertian(const Color& albedo) : tex(make_shared<SolidColor>(albedo)) {}

	Lambertian(shared_ptr<Texture> tex) : tex(tex) {}

	bool scatter(
		const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered, double& pdf
	) const override {
		// Old method: TODO was this not faster? rnd_cosine_direction() is slow!
		//auto scatter_direction = rec.normal + rnd_unit_vec();
		// catch degenerate scatter direction
		//if (scatter_direction.near_zero()) scatter_direction = rec.normal;

		Mat3 onb = Mat3::orthog(rec.normal);
		auto scatter_direction = onb * rnd_cosine_direction();

		scattered = Ray(rec.p, scatter_direction, r_in.time());
		attenuation = tex->value(rec.u, rec.v, rec.p);
		// TODO this is the same as the value returned by scatter_pdf(...). Why is it also
		// being computed & returned here?
		pdf = dot(onb.col(2), scatter_direction) / PI;
		return true;
	}

	double scatter_pdf(const Ray&, const HitRecord& rec, const Ray& scattered) const override {
		auto cos_theta = dot(rec.normal, scattered.direction().unit());
		return cos_theta < 0 ? 0 : cos_theta / PI;
	}

  private:
	shared_ptr<Texture> tex;
};

class Metal : public Material { // aka. mirror
  public:
	Metal(const Color& albedo, double fuzz = 0)
		: albedo(albedo), fuzz(fuzz < 1 ? fuzz : 1) {}

	bool scatter(
		const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered, double&
	) const override {
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

	bool scatter(
		const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered, double&
	) const override {
		double ri = rec.front_face ? 1 / refractive_index : refractive_index;

		Vec3 in_dir = r_in.direction().unit();
		double cos_theta = std::fmin(dot(-in_dir, rec.normal), 1);
		double sin_theta = std::sqrt(1 - cos_theta * cos_theta);

		bool cannot_refract = ri * sin_theta > 1; // check for total internal reflection
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

	bool scatter(
		const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered, double& pdf
	) const override {
		scattered = Ray(rec.p, rnd_unit_vec(), r_in.time());
		attenuation = tex->value(rec.u, rec.v, rec.p);
		// TODO this is the same as the value returned by scatter_pdf(...). Why is it also
		// being computed & returned here?
		pdf = 1 / (4 * PI);
		return true;
	}

	double scatter_pdf(const Ray&, const HitRecord&, const Ray&) const override {
		return 1 / (4 * PI);
	}

  private:
	shared_ptr<Texture> tex;
};

#endif
