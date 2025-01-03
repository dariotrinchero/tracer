#ifndef MESH_H
#define MESH_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "hittable.h"
#include "shape2d.h"
#include "material.h"
#include "bvh.h"

// debug output
#ifdef DEBUG_MESH
#define DEBUG_STREAM std::clog << "\r[Mesh] "
#else
#define DEBUG_STREAM if constexpr (false) std::clog
#endif

namespace {
	// STL format for each facet
	struct STLFacet {
		float normal[3];
		float verts[3][3];
		unsigned short att;
	};

	// size of facet in bytes (differs from sizeof(STLFacet) due to padding)
	constexpr size_t facet_size = sizeof(float) * 3 * 4 + sizeof(unsigned short);
}

class Mesh {
  public:
	Mesh(const std::string& filename, shared_ptr<Material> mat) {
		try {
			if (filename.substr(filename.length() - 4) != ".stl") throw "Can only load .stl models";

			// attempt to load image from some likely locations
			for (auto prefix : {"", "models/", "../models/", "../../models/", "../../../models/"}) {
				if ((load_success = load_stl(prefix + filename, mat))) return;
			}

			throw "Could not load model '" + filename + "'";
		} catch (char const* msg) {
			std::cerr << "ERROR: " << msg << ".\n";
			load_success = false;
			model = make_shared<BVHNode>(HittableList()); // empty model as fallback
		}
	}

	bool model_loaded() const { return load_success; }
	shared_ptr<BVHNode> get_model() const { return model; }

  private:
	bool                 load_success;   // whether model successfully loaded
	shared_ptr<BVHNode>  model;          // BVH-subdivided model

	bool load_stl(const std::string& filename, shared_ptr<Material> mat) {
		DEBUG_STREAM << "Attempting to load model '" << filename << "' ...\n";
		std::ifstream file(filename, std::ios::binary);
		if (!file.is_open()) return false;

		// skip over header (fixed at 80 bytes)
		file.ignore(80);
		if (!file) throw "Missing header from .stl file";

		// read number of triangles
		int num_facets;
		file.read(reinterpret_cast<char*>(&num_facets), sizeof(int));
		if (!file || num_facets <= 0) throw "Invalid triangle count";

		// process triangles
		STLFacet facet;
		HittableList facets;
		for (int t = 0; t < num_facets; t++) {
			if (t % (num_facets / 100) == 0) {
				DEBUG_STREAM << "Reading facets: " << int((t + 1.0) / num_facets * 100)
					<< "% of " << num_facets;
			}

			if (!file.read(reinterpret_cast<char*>(&facet), facet_size))
				throw "End of mesh reached unexpectedly";

			Vec3 normal(facet.normal[0], facet.normal[1], facet.normal[2]);
			Point3 corner(facet.verts[0][0], facet.verts[0][1], facet.verts[0][2]);
			Vec3 s1 = Point3(facet.verts[1][0], facet.verts[1][1], facet.verts[1][2]) - corner;
			Vec3 s2 = Point3(facet.verts[2][0], facet.verts[2][1], facet.verts[2][2]) - corner;

			bool oriented = dot(cross(s1, s2), normal) >= 0;
			shared_ptr<Triangle> tri = oriented ? make_shared<Triangle>(corner, s1, s2, mat)
				: make_shared<Triangle>(corner, s2, s1, mat);
			facets.add(tri);
		}

		DEBUG_STREAM << "Reading facets: 100% of " << num_facets << '\n';
		DEBUG_STREAM << "Building BVH tree for mesh...\n";
		model = make_shared<BVHNode>(facets);

		DEBUG_STREAM << "Model successfully loaded.\n";
		return true;
	}
};


#endif
