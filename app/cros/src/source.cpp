#include <iostream>
#include "data_table.h"
#include "points.h"
#include <stdexcept>
#include <fstream>

using namespace cros;

static data_table calc_transforms(std::istream* _colmap, std::istream* _fit) {
	
	data_table colmap_table = parse_colmap_images_data(_colmap); // colmap world to camera space
	data_table fit_table = parse_fit_data(_fit); // camera space to earth space
	
	//colmap world space to earth space
	return fit_table * colmap_table;
}

static double calc_error(data_table& guess, data_table& target, int frame_delta = 100) {

	//guess: colmap world space to earth space
	//target: object space to earth space

	data_table delta_guess{};
	data_table delta_target{};

	
	for (int i = 0; i < guess.table.size() - frame_delta; i+= frame_delta) {
		delta_guess.table.emplace_back(table_entry{
			guess.table[i].mat.inverse().eval() * guess.table[i + frame_delta].mat,
			guess.table[i].time,
			guess.table[i].id
		});
	}
	for (int i = 0; i < target.table.size() - frame_delta; i+= frame_delta) {
		delta_target.table.emplace_back(table_entry{
			target.table[i].mat.inverse().eval() * target.table[i + frame_delta].mat,
			target.table[i].time,
			target.table[i].id
		});
	}

	// print_desmos_formatted_dir(delta_guess, 1, 3);
	// std::cout << std::endl;
	// print_desmos_formatted_dir(delta_target, 1, 3);
	// std::cout << std::endl;
	// char c;
	// std::cin >> c;

	//point_str rot_plane_guess{};
	//point_str rot_plane_target{};

	double error = 0;
	double total = 0;
	double total_count = 0;
	for (int i = 0; i < delta_guess.table.size() && i < delta_target.table.size(); i++) {
		// mean square error per element
		double e = 0;
		for (int y = 0; y < 3; y++) {
			for (int x = 0; x < 3; x++) {
				double v = delta_guess.table[i].mat(y, x) - delta_target.table[i].mat(y, x);
				// std::cout << delta_guess[i](y, x) << " - " << delta_target[i](y, x) << " = " << v << std::endl;
				e += std::sqrt(v * v);
				total += std::abs(delta_target.table[i].mat(y, x));
				total_count++;
			}
		}
		//std::cout << "Guess delta" << std::endl;
		//std::cout << delta_guess.table[i].mat.block(0,0,3,3) << std::endl;
		//std::cout << "Actual delta" << std::endl;
		//std::cout << delta_target.table[i].mat.block(0, 0, 3, 3) << std::endl;
		//std::cout << "A - B" << std::endl;
		//std::cout << (delta_guess.table[i].mat - delta_target.table[i].mat).block(0, 0, 3, 3) << std::endl;

		//Eigen::AngleAxisd g = Eigen::AngleAxisd(delta_guess.table[i].mat.block<3,3>(0,0));
		//Eigen::AngleAxisd t = Eigen::AngleAxisd(delta_target.table[i].mat.block<3,3>(0,0));

		//double e = g.angle() - t.angle();
		//e = e * e;
		//e = sqrt(e);
		//std::cout << delta_guess.table[i].id << " " << g.angle() << " : " << delta_target.table[i].id << " " << t.angle() << std::endl;
		//error += e;
		//total += std::abs(t.angle());

		//Eigen::Vector4d g_vec = delta_guess.table[i].mat * Eigen::Vector4d(0, 1, 0, 0);
		//Eigen::Vector4d t_vec = delta_target.table[i].mat * Eigen::Vector4d(0, 1, 0, 0);

		//rot_plane_guess.append(g_vec.block<3, 1>(0, 0));
		//rot_plane_target.append(t_vec.block<3, 1>(0, 0));

		//rot_plane_guess.append(g.axis());
		//rot_plane_target.append(t.axis());

		//std::cout << "error: " << e << std::endl;
		error += e;
	}

	std::cout << "table jump: " << frame_delta << std::endl;
	std::cout << "total error: " << error << std::endl;
	std::cout << "mean error: " << error / total_count << std::endl;
	std::cout << "normalized mean error: " << error / total << std::endl;

	//rot_plane_guess.finish();
	//rot_plane_target.finish();

	//std::cout << rot_plane_guess.str() << std::endl << std::endl;
	//std::cout << rot_plane_target.str() << std::endl << std::endl;

	return error;
}

int run(const char* fit_arg, const char* col_arg, const char* act_arg) {
	std::fstream fit{fit_arg, std::ios::beg | std::ios::in};
	std::fstream colmap{col_arg, std::ios::beg | std::ios::in};
	std::fstream actual{act_arg, std::ios::beg | std::ios::in};

	data_table table = calc_transforms(&colmap, &fit);
	data_table actual_table = parse_fit_data(&actual);
	for (int i = 20; i < 200; i += 20) {
		calc_error(table, actual_table, i);
	}

	return 0;
}

int main(int argc, char** argv) {
	/*if (argc == 1) {
		std::cout << "usage" << std::endl;
		std::cout << "cros.exe <path_to_fit_data> <path_to_colmap_data> <path_to_actual_output>" << std::endl;
		std::cout << std::endl;
		std::cout << "cros.exe 0 - run with console input" << std::endl;
		return 0;
	}*/

	std::string arg1;
	std::string arg2;
	std::string arg3;

	if (argc == 1) {
		std::cout << "path_to_fit_data: " << std::endl;
		std::cin >> arg1;
		std::cout << "path_to_colmap_data: " << std::endl;
		std::cin >> arg2;
		std::cout << "path_to_actual_output: " << std::endl;
		std::cin >> arg3;
	}
	else if (argc == 4) {
		arg1 = argv[1];
		arg2 = argv[2];
		arg3 = argv[3];
	}

	int return_code;

	try {
		return_code = run(arg1.data(), arg2.data(), arg3.data());
	}
	catch (std::exception& e) {
		std::cout << "error: " << e.what() << std::endl;
		return_code = 1;
	}

	return return_code;
}