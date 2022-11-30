#include "Kin.cpp"
#include <iostream>


using namespace std;

int main(){

    Kin kin;
    Eigen::Vector<double, 6> Th(0.089159, 0, 0, 0.10915, 0.09465, 0.0823);
    Eigen::Matrix4d get_T0e = kin.get_T0e(Th);

    cout << "matrice: " << endl << get_T0e << endl;

    cout << "fine " << endl;
}