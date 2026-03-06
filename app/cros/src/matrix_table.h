#pragma once
#ifndef CROS_MATRIX_TABLE_H
#define CROS_MATRIX_TABLE_H

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <vector>

namespace cros {
	class matrix_table {
	public:
		std::vector<Eigen::Matrix4d> table{};

		matrix_table() = default;
	};
}

#endif // CROS_MATRIX_TABLE_H