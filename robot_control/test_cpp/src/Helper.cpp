
#include <complex>
#include <cmath>
//#include "robot.cpp"

using namespace std;


class Helper{
    public:
        Helper(){

        }

        float** sin_square(float start[], float end[], int steps){

            float** path =0;
            path   = new float*[steps];//altezza = steps

            for(int i =0;i < steps ; i++){
                path[i]= new float[6]; //larghezza = 6 per le coordinate
                //cout << "created: " << i << endl;
                for (int j = 0; j < 6; j++){
                    // q_i+ (math.sin((math.pi/2)*i/steps)**2)*diff
                    //cout << "created: " << i << " " << j << endl;
                    path[i][j]=start[j]+pow(sin((M_PI/2*i/steps)),2)*(end[j]-start[j]);
                    
                    //cout << "inserito: " << path[i][j] << endl;
                }

            }

            cout << "creato" << endl << endl;

            return path;
        };

        
};