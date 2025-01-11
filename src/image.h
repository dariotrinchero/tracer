#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "color.h"

/* --- class for .ppm format images ------------------------------------------------------------- */

class PPMImage {
  public:
	PPMImage() {}

	PPMImage(const std::string& filename) {
		try {
			if (filename.substr(filename.length() - 4) != ".ppm") throw "Can only load .ppm images";

			// attempt to load image from some likely locations
			for (auto prefix : {"", "images/", "../images/", "../../images/", "../../../images/"}) {
				if (load_ppm(prefix + filename)) return;
			}

			throw "Could not load image file '" + filename + "'";
		} catch (char const* msg) {
			std::cerr << "ERROR: " << msg << ".\n";
		}
	}

	~PPMImage() { delete[] bdata; }

	int width() const { return bdata == nullptr ? 0 : image_width; }
	int height() const { return bdata == nullptr ? 0 : image_height; }

	Color pixel(int x, int y) const {
		if (bdata == nullptr) return Color(1, 0, 1); // return magenta as fallback
		x = clamp(x, 0, image_width - 1);
		y = clamp(y, 0, image_height - 1);
		return Color(bdata + y * bytes_per_scanline + x * bytes_per_pixel);
	}

	/**
	 * Output the file header of a (binary- or ASCII-encoded) PPM image file to given stream.
	 *
	 * @param out        the output stream to which to output file header
	 * @param width      the width of the PPM image
	 * @param height     the height of the PPM image
	 * @param binary_enc whether image uses binary encoding (else, use ASCII encoding)
	 */
	static void write_header(std::ostream& out, int width, int height, bool binary_enc=false) {
		out << (binary_enc ? "P6\n" : "P3\n") << width << ' ' << height << "\n255\n";
	}

	/**
	 * Output given (gamma-corrected) color to given stream in format required for PPM files.
	 *
	 * @param out        the output stream to which to output pixel color
	 * @param col        pixel color to output
	 * @param gamma      gamma correction to apply to pixel prior to output
	 * @param binary_enc whether image uses binary encoding (else, use ASCII encoding)
	 */
	static void write_color(std::ostream& out, const Color& col, double gamma, bool binary_enc=false) {
		Color col_gam = col.to_gamma(gamma);
		if (binary_enc) {
			const char rgb[3] = { (char) col_gam.r(), (char) col_gam.g(), (char) col_gam.b() };
			out.write(rgb, 3);
		} else {
			out << col_gam.r() << ' ' << col_gam.g() << ' ' << col_gam.b() << '\n' << std::flush;
		}
	}

  private:
  	static constexpr size_t bytes_per_pixel = 3;
	int image_width = 0, image_height = 0;
	size_t bytes_per_scanline = 0;
	byte *bdata = nullptr;

	/**
	 * Load PPM image file with given filename, reading bytes into local array, bdata.
	 * Return false if file does not exist, but throw exceptions for other failure cases.
	 *
	 * @param filename the path of the image PPM file to load
	 * @returns whether file exists
	 */
	bool load_ppm(const std::string& filename) {
		std::ifstream file(filename);
		if (!file.is_open()) return false;

		// check for .ppm magic number (P6)
		std::string line;
		if (get_line(file, line) != "P6") throw "Can only load binary .ppm images";

		// read width & height
		std::istringstream iss(get_line(file, line));
		if (!(iss >> image_width >> image_height) || image_width <= 0 || image_height <= 0)
			throw "Image size invalid";

		// check for max value 255
		if (get_line(file, line) != "255") throw "Can only read .ppm images with 8 bits per pixel";

		// read color bytes
		bytes_per_scanline = bytes_per_pixel * image_width;
		size_t total_bytes = image_height * bytes_per_scanline;
		bdata = new byte[total_bytes];
		if (!file.read(reinterpret_cast<char*>(bdata), total_bytes))
			throw "End of image reached unexpectedly";

		return true;
	}

	/**
	 * Clamp value x to range [low,high].
	 *
	 * @param x         value to clamp to an interval
	 * @param low, high inclusive bounds of interval to which to clamp x
	 * @returns the value of x restricted to lie within [low,high]
	 */
	static int clamp(int x, int low, int high) {
		if (x < low) return low;
		if (x > high) return high;
		return x;
	}

	/**
	 * Read next line from given file into given string, skipping lines beginning with '#'
	 * (PPM format comment token), & throwing exception if end-of-file unexpectedly reached.
	 *
	 * @param[in] file  file stream from which to read next line
	 * @param[out] line string into which to read from file
	 * @returns reference to line (for convenience)
	 */
	static std::string& get_line(std::ifstream& file, std::string& line) {
		do {
			if (!std::getline(file, line)) throw "End of image reached unexpectedly";
		} while (line.empty() || line[0] == '#'); // skip .ppm comments
		return line;
	}
};
