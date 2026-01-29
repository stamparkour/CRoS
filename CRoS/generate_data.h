#pragma once

#include <Eigen/Eigen>
#include <array>
#include <vector>
#include <numbers>
#include <cmath>

namespace stamp {
	using namespace Eigen;

	// points of the target object
	inline const std::array<Vector3d, 8> points = {
		Vector3d{-1,-1,-1},
		Vector3d{1,-1,-1},
		Vector3d{-1,1,-1},
		Vector3d{1,1,-1},
		Vector3d{-1,-1,1},
		Vector3d{1,-1,1},
		Vector3d{-1,1,1},
		Vector3d{1,1,1},
	};

	inline const Quaterniond rotation_start = Quaterniond{
		AngleAxisd(45 * std::numbers::pi,Vector3d::UnitZ()) *
		AngleAxisd(45 * std::numbers::pi,Vector3d::UnitX()) *
		AngleAxisd(0 * std::numbers::pi,Vector3d::UnitY())
	};
	inline const Quaterniond rotation_velocity = Quaterniond{
		AngleAxisd(0 * std::numbers::pi,Vector3d::UnitZ()) *
		AngleAxisd(0 * std::numbers::pi,Vector3d::UnitX()) *
		AngleAxisd(60 * std::numbers::pi,Vector3d::UnitY())
	};

	inline const Vector3d position_velocity = Vector3d{ 0.25, 1.5, 1 };
	inline const double time_end = 5;
	inline const double time_delta = 0.1;

	struct pointcloud_state_t {
		double time;
		std::vector<Vector3d> points;
	};

	std::vector<pointcloud_state_t> generate_data() {
		std::vector<pointcloud_state_t> arr{};
		Quaterniond rot = rotation_start;
		Vector3d pos{ 0,0,0 };
		for (double t = 0; t <= time_end; t += time_delta) {
			pointcloud_state_t& state = arr.emplace_back();
			state.time = t;
			for (const auto& p : points) {
				Vector3d v = rot * p + pos;
				state.points.push_back(v);
			}

			rot.normalize();
			rot *= Quaterniond{ 1,0,0,0 }.slerp(time_delta, rotation_velocity); // rot_vel^time_delta
			pos += position_velocity * time_delta;
		}
		return arr;
	}
}