#pragma once
#ifndef CROS_DATA_TABLE_H
#define CROS_DATA_TABLE_H

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <vector>
#include <istream>
#include <string>
#include <string_view>
#include <charconv>
#include <stdexcept>
#include "word_iterable.h"
#include <algorithm>
#include <iostream>
#include <regex>
#include "points.h"

namespace cros {
	constexpr double FRAME_RATE = 0.2;

	struct table_entry {
		Eigen::Matrix4d mat;
		double time;
		int id;
	};

	class data_table {
	public:
		std::vector<table_entry> table{};
		data_table() = default;

		template<typename UnaryFunc>
		void for_each(UnaryFunc f) {
			std::for_each(table.begin(), table.end(), f);
		}
		template<typename UnaryFunc>
		void for_each(UnaryFunc f) const {
			std::for_each(table.cbegin(), table.cend(), f);
		}

		void inverse() {
			for_each([](table_entry& entry) {
				entry.mat = entry.mat.inverse().eval();
			});
		}

		data_table& operator *=(const data_table& other) {
			if (table.size() > other.table.size()) {
				table.resize(other.table.size());
			}
			for (int i = 0; i < table.size(); i++) {
				table[i].mat *= other.table[i].mat;
			}
			return *this;
		}
		data_table operator *(const data_table& other) const {
			data_table o = *this;
			o *= other;
			return o;
		}
		data_table& operator +=(const data_table& other) {
			if (table.size() > other.table.size()) {
				table.resize(other.table.size());
			}
			for (int i = 0; i < table.size(); i++) {
				table[i].mat += other.table[i].mat;
			}
			return *this;
		}
		data_table operator +(const data_table& other) const {
			data_table o = *this;
			o += other;
			return o;
		}
	};


	static std::regex parse_frame_id{"(\\d+)"};
	inline data_table parse_fit_data(std::istream* _fin) {
		std::istream& fin = *_fin;
		using json = nlohmann::json;

		data_table table{};
		json data = json::parse(fin);

		int index = 0;
		for (auto& elm : data["frames"]) {
			auto mat_json = elm["transform_matrix"];
			std::string file_path = elm["file_path"].get<std::string>();
			Eigen::Matrix4d mat{};

			// retrieve matrix
			int y = 0;
			for (auto& row : mat_json) {
				int x = 0;
				for (auto& col : row) {
					double value = col.get<double>();
					mat(y, x) = value;
					x++;
				}
				y++;
			}

			std::cmatch result{};
			std::regex_search(file_path.data(), result, parse_frame_id);
			std::string id_str = result[1].str();
			int id = std::stoi(id_str);

			table.table.emplace_back(table_entry{
				mat, // matrix
				id / FRAME_RATE, // times
				id // index
			});

			index++;
		}
		return table;
	}

	// world to camera
	inline data_table parse_colmap_images_data(std::istream* _fin) {
		std::istream& fin = *_fin;
		std::string line{};
		std::vector<std::string_view> words{};
		data_table table{};

		int line_id = 0;
		while (fin) {
			std::getline(fin, line);
			if (line.starts_with("#") || line.empty()) continue;

			if (line_id % 2 == 0) { // image id line
				words.clear();
				Eigen::Quaterniond rot{};
				Eigen::Vector3d pos{};
				int frame_id{};

				for (const std::string_view& word : word_iterable(line)) {
					words.push_back(word);
				}

				if (words.size() < 10) {
					throw std::runtime_error("colmap image line contains less than 10 words.");
				}

				std::from_chars(words[0].data(), words[0].data() + words[0].size(), frame_id);

				// coord: (right,down,forward)

				// rot data
				std::from_chars(words[1].data(), words[1].data() + words[1].size(), rot.w());
				std::from_chars(words[2].data(), words[2].data() + words[2].size(), rot.x());
				std::from_chars(words[3].data(), words[3].data() + words[3].size(), rot.y());
				std::from_chars(words[4].data(), words[4].data() + words[4].size(), rot.z());

				// pos data
				std::from_chars(words[5].data(), words[5].data() + words[5].size(), pos.x());
				std::from_chars(words[6].data(), words[6].data() + words[6].size(), pos.y());
				std::from_chars(words[7].data(), words[7].data() + words[7].size(), pos.z());

				std::cmatch result{};
				std::regex_search(words[9].data(), result, parse_frame_id);
				std::string id_str = result[1].str();
				int id = std::stoi(id_str);

				

				Eigen::Affine3d translate{Eigen::Translation3d{pos}};
				Eigen::Affine3d rotation{Eigen::AngleAxisd{rot}};

				Eigen::Matrix4d correct_transform = Eigen::Matrix4d{
					{ 0, 1, 0, 0 },
					{ 0, 0, -1, 0 },
					{ -1, 0, 0, 0 },
					{ 0, 0, 0, 1 }
				};

				table.table.emplace_back(table_entry{
					translate.matrix() * rotation.matrix() * correct_transform, // matrix 
					id / FRAME_RATE, // time
					id // id
				});
			}

			line_id++;
		}

		return table;
	}


	static const Eigen::Matrix4d desmos_transform = Eigen::Matrix4d{
		{ 1, 0, 0, 0 },
		{ 0, 1, 0, 0 },
		{ 0, 0, 1, 0 },
		{ 0, 0, 0, 1 },
	};

	inline void print_desmos_formatted_points(const data_table& table, int jump = 5) {
		int index = 0;
		point_str str{};

		table.for_each([&](const table_entry& entry) {
			auto mat = entry.mat;
			if (index % jump == 0) {
				auto point = desmos_transform * mat * Eigen::Vector4d(0, 0, 0, 1);
				str.append(point.block<1,3>(0,0));
			}
			index++;
		});
		str.finish();
		std::cout << str.str() << std::endl;
	}

	inline void print_desmos_formatted_dir(const data_table& table, int jump = 5, int dir = 3) {
		int index = 0;
		point_str str{};

		table.for_each([&](const table_entry& entry) {
			auto mat = entry.mat;
			if (index % jump == 0) {
				auto point = desmos_transform * mat * Eigen::Vector4d(dir == 1 ? 1 : 0, dir == 2 ? 1 : 0, dir == 3 ? 1 : 0, 0);
				str.append(point.block<1, 3>(0, 0));
			}
			index++;
		});
		str.finish();
		std::cout << str.str() << std::endl;
	}
}

#endif // CROS_DATA_TABLE_H