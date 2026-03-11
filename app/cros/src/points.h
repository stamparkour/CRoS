#pragma once
#ifndef CROS_POINTS_H
#define CROS_POINTS_H

#include <string>
#include <string_view>
#include <Eigen/Dense>

namespace cros {
	class point_str {
		std::string buffer = "[";
		int points = 0;
	public:
		point_str() {}

		void append(const Eigen::Vector3d& point) {
			std::string p = "(";
			p += std::to_string(point.x());
			p += ",";
			p += std::to_string(point.y());
			p += ",";
			p += std::to_string(point.z());
			p += ")";

			if (points != 0) buffer += ",";
			buffer += p;
			points++;
		}
		void append(const Eigen::Vector4d& point) {
			std::string p = "(";
			p += std::to_string(point.x());
			p += ",";
			p += std::to_string(point.y());
			p += ",";
			p += std::to_string(point.z());
			p += ")";

			if (points != 0) buffer += ",";
			buffer += p;
			points++;
		}
		void finish() {
			buffer += "]";
		}
		const std::string& str() {
			return buffer;
		}
	};
}

#endif // CROS_POINTS_H