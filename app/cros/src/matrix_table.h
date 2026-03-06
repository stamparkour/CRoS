#pragma once
#ifndef CROS_MATRIX_TABLE_H
#define CROS_MATRIX_TABLE_H

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <vector>
#include <istream>

namespace cros {
	class matrix_table {
	public:
		std::vector<Eigen::Matrix4d> table{};

		matrix_table() = default;
	};

	inline matrix_table parse_fit_data(std::istream& fin) {
		using json = nlohmann::json;
		json data = json::parse(fin);


	}
}

#endif // CROS_MATRIX_TABLE_H