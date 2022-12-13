
#include "Kin.cpp"
#include <iostream>
#include "Helper.cpp"

using namespace std;

int main(){

    Kin kin;
    Eigen::Vector<double, 6> Th(0.089159, 0, 0, 0.10915, 0.09465, 0.0823);
    Eigen::Matrix4d get_T0e = kin.get_T0e(Th);

    cout << "matrice: " << endl << get_T0e << endl;

    Helper h;
    float start[6]={1,1,1,1,1,1};
    float end[6]={2,2,2,2,2,2};
   
    float** matrix = h.sin_square(start,end,10);

    for(int a = 0; a < 10; a++){
        for(int b = 0; b < 6; b++){
            cout << matrix[a][b] << " ";
        }
        cout << endl;
    } 


    cout << "----------------------------------------------" << endl;
    cout << "pinocchio" << endl << endl;

    Kin k;
    k.test_pinocchio();
    k.test_pinocchio2();
    k.test_pinocchio3();
    cout << "----------------------------------------------" << endl;
    cout << "pass" << endl << endl;

    //k.test_pinocchio2();


    cout << "fine " << endl;
}