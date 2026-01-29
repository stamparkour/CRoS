#pragma once

#include <Eigen/Eigen>
#include <generate_data.h>
#include <iostream>

namespace stamp {
	using namespace Eigen;



	Quaterniond csos_alg(const std::vector<pointcloud_state_t>& state) {
		std::vector<pointcloud_state_t> velocity{};
		std::vector<pointcloud_state_t> acceleration{};
		std::vector<Vector3d> acceleration_vectors{};

		//generate velocity
		for (auto s1 = state.begin(), s2 = state.begin(); s2 != state.end(); s1 = s2++) {
			if (s1 == s2) continue;
			auto& entry = velocity.emplace_back();
			entry.time = s1->time;
			for (auto c1 = s1->points.begin(), c2 = s2->points.begin(); c1 != s1->points.end(); ++c1, ++c2) {
				Vector3d delta = (*c2 - *c1) / (s2->time - s1->time);
				entry.points.push_back(delta);
			}
		}
		//generate acceleration
		for (auto s1 = velocity.begin(), s2 = velocity.begin(); s2 != velocity.end(); s1 = s2++) {
			if (s1 == s2) continue;
			auto& entry = acceleration.emplace_back();
			entry.time = s1->time;
			for (auto c1 = s1->points.begin(), c2 = s2->points.begin(); c1 != s1->points.end(); ++c1, ++c2) {
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
			if (vec.norm() > 0.0001) {
				vec.normalize();
				normal.push_back(vec);
			}
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

		double deviation = 0;
		for (auto& n : normal) {
			deviation += (n - tangent_vector).squaredNorm();
		}
		deviation /= normal.size();

		
		std::cout << "rot vector: " << std::endl << tangent_vector << std::endl;
		std::cout << "deviation: " << deviation << std::endl;

		std::vector<pointcloud_state_t> mapping{};
		std::vector<pointcloud_state_t> mapping_vel{};
		std::vector<pointcloud_state_t> mapping_accel{};
		//map all points onto plane
		for (auto& s : state) {
			auto& map = mapping.emplace_back();
			map.time = s.time;
			for (auto& p : s.points) {
				Vector3d proj = p - tangent_vector * tangent_vector.dot(p);
				map.points.push_back(proj);
			}
		}
		//get velocity of mapped points
		for (auto s1 = mapping.begin(), s2 = mapping.begin(); s2 != mapping.end(); s1 = s2++) {
			if (s1 == s2) continue;
			auto& entry = mapping_vel.emplace_back();
			entry.time = s1->time;
			for (auto c1 = s1->points.begin(), c2 = s2->points.begin(); c1 != s1->points.end(); ++c1, ++c2) {
				Vector3d delta = (*c2 - *c1) / (s2->time - s1->time);
				entry.points.push_back(delta);
			}
		}
		//get accelration of mapped points
		for (auto s1 = mapping_vel.begin(), s2 = mapping_vel.begin(); s2 != mapping_vel.end(); s1 = s2++) {
			if (s1 == s2) continue;
			auto& entry = mapping_accel.emplace_back();
			entry.time = s1->time;
			for (auto c1 = s1->points.begin(), c2 = s2->points.begin(); c1 != s1->points.end(); ++c1, ++c2) {
				Vector3d delta = (*c2 - *c1) / (s2->time - s1->time);
				entry.points.push_back(delta);
			}
		}

		for (auto& s : mapping_accel) {
			for (auto a1 = acceleration_vectors.begin(), a2 = acceleration_vectors.begin(); a2 != acceleration_vectors.end(); a1 = a2++) {
				if (a1 == a2) continue;
				Vector3d vec = a2->cross(*a1);
				vec.normalize();
				normal.push_back(vec);
			}
		}
		//find the average center of rotation
		std::vector<Vector3d> center_of_rotation{};
		double cor_deviation = 0;
		int cor_deviation_total = 0;
		for (auto state = mapping.begin(), accel_state = mapping_accel.begin(); accel_state != mapping_accel.end(); ++state, ++accel_state) {
			Vector3d& cor = center_of_rotation.emplace_back();
			int total_cor = 0;
			std::vector<Vector3d> tmp_target_cor{};
			for (auto a1 = accel_state->points.begin(), a2 = accel_state->points.begin(), 
				p1 = state->points.begin(), p2 = state->points.begin();
				a2 != accel_state->points.end() && p2 != state->points.end(); //a2 will reach end first. len(a2) < len(p2)
				a1 = a2++, p1 = p2++) {
				if (a1 == a2) continue;
				Vector3d& u = *a1;
				Vector3d& v = *a2;
				Vector3d v_i{ 1 / v.x(), 1 / v.y(), 1 / v.z() };
				Vector3d& A = *p1;
				Vector3d& B = *p2;

				// A+ut=B+vg
				//solve for t

				Vector3d target = A + u * (B - A).dot(v_i) / u.dot(v_i);
				tmp_target_cor.push_back(target);
				cor += target;
				total_cor++;
			}
			cor /= total_cor;
			double dev = 0;
			int dev_total = 0;
			for (auto& t : tmp_target_cor) {
				dev += (cor - t).squaredNorm();
				dev_total++;
			}
			cor_deviation += dev;
			cor_deviation_total++;
		}
		cor_deviation /= cor_deviation_total;
		std::cout << "center of rotation deviation total: " << cor_deviation << std::endl;

		std::vector<pointcloud_state_t> local_vel{};
		std::vector<double> rotational_speed{};
		double av_rotational_speed = 0;
		int av_rotational_speed_total = 0;
		//get velocity of mapped points relative to center of rotation
		{
		auto cor_i = center_of_rotation.begin();
		auto s1 = mapping.begin(), s2 = mapping.begin();
		for (; s2 != mapping.end() && cor_i != center_of_rotation.end(); s1 = s2++, cor_i++) {
			if (s1 == s2) continue;
			auto& vel_state = local_vel.emplace_back();
			vel_state.time = s1->time;
			auto& cor = *cor_i;
			double& rot_speed = rotational_speed.emplace_back();
			int rot_speed_total = 0;
			for (auto c1 = s1->points.begin(), c2 = s2->points.begin(); c1 != s1->points.end(); ++c1, ++c2) {
				Vector3d p1 = *c1 - cor;
				Vector3d p2 = *c2 - cor;
				Vector3d velocity = (p2 - p1) / (s2->time - s1->time); // velocity should be perpindicular
				vel_state.points.push_back(velocity);

				rot_speed += (p1.cross(velocity) / p1.squaredNorm()).dot(tangent_vector);
				rot_speed_total++;
			}
			rot_speed /= rot_speed_total;
			av_rotational_speed += rot_speed;
			av_rotational_speed_total++;
		}}
		
		av_rotational_speed /= av_rotational_speed_total;

		return Quaterniond{ AngleAxisd(av_rotational_speed,tangent_vector) };
	}
}