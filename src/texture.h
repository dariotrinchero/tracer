#ifndef TEXTURE_H
#define TEXTURE_H

#include <memory>
#include <string>
#include <cmath>

#include "color.h"
#include "image.h"
#include "interval.h"
#include "random.h"

using std::make_shared;
using std::shared_ptr;

/* --- superclass for texture ------------------------------------------------------------------- */

class Texture {
  public:
	virtual ~Texture() = default;

	virtual Color value(double u, double v, const Point3& p) const = 0;
};

class SolidColor : public Texture {
  public:
	SolidColor(const Color& albedo) : albedo(albedo) {}

	SolidColor(double r, double g, double b) : SolidColor(Color(r, g, b)) {}

	Color value(double, double, const Point3&) const override {
		return albedo;
	}

  private:
	Color albedo;
};

/* --- assorted textures ------------------------------------------------------------------------ */

class CheckerTexture : public Texture {
  public:
	CheckerTexture(double scale, shared_ptr<Texture> even, shared_ptr<Texture> odd)
		: inv_scale(1.0 / scale), even(even), odd(odd) {}
	
	CheckerTexture(double scale, const Color& c1, const Color& c2)
		: CheckerTexture(scale, make_shared<SolidColor>(c1), make_shared<SolidColor>(c2)) {}

	Color value(double u, double v, const Point3& p) const override {
		bool is_even = (int(std::floor(inv_scale * p[0])) 
			+ int(std::floor(inv_scale * p[1]))
			+ int(std::floor(inv_scale * p[2]))) % 2 == 0;
		return is_even ? even->value(u, v, p) : odd->value(u, v, p);
	}

  private:
	double inv_scale;
	shared_ptr<Texture> even;
	shared_ptr<Texture> odd;
};

class ImageTexture : public Texture {
  public:
	ImageTexture(const std::string& filename, double gamma = 2.2) // gamma=2.2 is typical
		: img(filename), gamma(gamma) {}

	Color value(double u, double v, const Point3&) const override {
		// if image fails to load, return solid cyan as debugging aid
		if (img.height() <= 0) return Color(0, 1, 1);

		// clamp texture coordinates
		Interval uv_range(0, 1);
		u = uv_range.clamp(u);
		v = 1 - uv_range.clamp(v); // flip v to img coords

		int i = int(u * img.width()), j = int(v * img.height());
		return img.pixel(i, j).to_linear(gamma); // scattering attenuation needs linear color
	}

  private:
	PPMImage img;
	double gamma;
};

class NoiseTexture : public Texture {
  public:
	NoiseTexture(double scale) : scale(scale) {}

	Color value(double, double, const Point3& p) const override {
		return white * 0.5 * (1 + std::sin(scale * p.z() + 10 * noise.turbulence(p, 7)));
	}

  private:
	Perlin noise;
	double scale;
};

#endif
