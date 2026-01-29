#include <generate_data.h>
#include <cros_algorithm.h>
#include <iostream>

int main(int argc, char** argv) {
	Eigen::Quaterniond given_rot;
	auto data = stamp::generate_data(given_rot);

	Eigen::Quaterniond output_rot = stamp::csos_alg(data);

	std::cout << "given rot: " << std::endl
		<< given_rot.w() << "+" << given_rot.x() << "i+" << given_rot.y() << "j+" << given_rot.z() << "k" << std::endl;
	Eigen::AngleAxisd axis_given{ given_rot };
	std::cout << "angle axis: " << axis_given.axis().x() << " " << axis_given.axis().y() << " " << axis_given.axis().z() << std::endl;
	std::cout << "angle angle: " << axis_given.angle() << std::endl;


	std::cout << std::endl;

	std::cout << "returned rot: " << std::endl
		<< output_rot.w() << "+" << output_rot.x() << "i+" << output_rot.y() << "j+" << output_rot.z() << "k" << std::endl;
	Eigen::AngleAxisd axis{ output_rot };
	std::cout << "angle axis: " << axis.axis().x() << " " << axis.axis().y() << " " << axis.axis().z() << std::endl;
	std::cout << "angle angle: " << axis.angle() << std::endl;

	std::cout << std::endl;

	return 0;
}
