#pragma once

#include <Eigen/Eigen>
#include <generate_data.h>

namespace stamp {
	using namespace Eigen;



	Quaterniond csos_alg(std::vector<pointcloud_state_t>& state) {
		std::vector<pointcloud_state_t> velocity{};
		std::vector<pointcloud_state_t> acceleration{};
		std::vector<Vector3d> acceleration_vectors{};

		//generate velocity
		for (auto s1 = state.begin(), s2 = state.begin(); s2 != state.end(); s1 = s2++) {
			if (s1 == s2) continue;
			auto& entry = velocity.emplace_back();
			entry.time = s1->time;
			for (auto c1 = s1->points.begin(), c2 = s2->points.begin(); c1 == s1->points.end(); ++c1, ++c2) {
				Vector3d delta = (*c2 - *c1) / (s2->time - s1->time);
				entry.points.push_back(delta);
			}
		}
		//generate acceleration
		for (auto s1 = velocity.begin(), s2 = velocity.begin(); s2 != velocity.end(); s1 = s2++) {
			if (s1 == s2) continue;
			auto& entry = acceleration.emplace_back();
			entry.time = s1->time;
			for (auto c1 = s1->points.begin(), c2 = s2->points.begin(); c1 == s1->points.end(); ++c1, ++c2) {
				Vector3d delta = (*c2 - *c1) / (s2->time - s1->time);
				entry.points.push_back(delta);
				acceleration_vectors.push_back(delta);
			}
		}

		// assume no external acceleration


		std::vector<Vector3d> normal{};
		Vector3d tangent_vector{};
		
		//generate potential normals
		for (auto a1 = acceleration_vectors.begin(), a2 = acceleration_vectors.begin(); a2 != acceleration_vectors.end(); a1 = a2++) {
			if (a1 == a2) continue;
			Vector3d vec = a2->cross(*a1);
			vec.normalize();
			normal.push_back(vec);
		}
		// get average normal vector
		for (auto& n : normal) {
			double d = normal[0].dot(n);
			if (d < 0) {
				n = -n;
			}
			tangent_vector += n;
		}
		tangent_vector /= normal.size();
		tangent_vector.normalize();

		std::vector<pointcloud_state_t> mapping{};
		std::vector<pointcloud_state_t> mapping_vel{};
		for (auto& s : state) {
			auto& map = mapping.emplace_back();
			map.time = s.time;
			for (auto& p : s.points) {
				Vector3d proj = p - tangent_vector * tangent_vector.dot(p);
				map.points.push_back(proj);
			}
		}
		for (auto s1 = mapping.begin(), s2 = mapping.begin(); s2 != mapping.end(); s1 = s2++) {
			if (s1 == s2) continue;
			auto& entry = mapping_vel.emplace_back();
			entry.time = s1->time;
			for (auto c1 = s1->points.begin(), c2 = s2->points.begin(); c1 == s1->points.end(); ++c1, ++c2) {
				Vector3d delta = (*c2 - *c1) / (s2->time - s1->time);
				entry.points.push_back(delta);
			}
		}
	}
}