#include <Eigen/Dense>
#include <iostream>
#include <vector>

int main(int /* argc */, char ** /* argv */)
{
 Eigen::VectorXd q;
std::vector<double> a= { 0.02699, -0.66813, -0.32761, -0.0235,   0.02016,  0.};
q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(a.data(), a.size());

std::cout << "q: " << q << std::endl;
}