#ifndef BVH_H
#define BVH_H

#include <algorithm>

#include "hittable.h"
#include "random.h"

class BVHNode : public Hittable {
  public:
	BVHNode(HittableList list) : BVHNode(list.objects, 0, list.objects.size()) {}

	BVHNode(std::vector<shared_ptr<Hittable>>& objects, size_t start, size_t end) {
		// compute bounding box
		bbox = AABB::empty;
		for (size_t obj = start; obj < end; obj++) bbox = AABB(bbox, objects[obj]->bounding_box());

		// set left & right child nodes
		size_t object_span = end - start;
		if (object_span == 1) left = right = objects[start];
		else if (object_span == 2) {
			left = objects[start];
			right = objects[start + 1];
		} else if (object_span > 0) {
			// sort object list along longest axis
			int axis = bbox.longest_axis();
			std::sort(objects.begin() + start, objects.begin() + end, box_cmp(axis));

			// split sorted list into left & right halves
			auto mid = start + object_span / 2;
			left = make_shared<BVHNode>(objects, start, mid);
			right = make_shared<BVHNode>(objects, mid, end);
		}
	}

	bool hit(const Ray& r, Interval ray_t, HitRecord& rec) const override {
		if (!bbox.hit(r, ray_t)) return false;

		bool hit_left = left->hit(r, ray_t, rec);
		bool hit_right = right->hit(r, Interval(ray_t.min, hit_left ? rec.t : ray_t.max), rec);
		return hit_left || hit_right;
	}

	AABB bounding_box() const override { return bbox; }

  private:
	AABB bbox;
	shared_ptr<Hittable> left;
	shared_ptr<Hittable> right;

	using HittableComparator = std::function<bool(shared_ptr<Hittable>, shared_ptr<Hittable>)>;

	static HittableComparator box_cmp(int axis) {
		return [axis](const shared_ptr<Hittable> a, const shared_ptr<Hittable> b) -> bool {
			auto a_axis_min = a->bounding_box().axis_interval(axis).min;
			auto b_axis_min = b->bounding_box().axis_interval(axis).min;
			return a_axis_min < b_axis_min;
		};
	}
};

#endif
