#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <time.h>
#include <math.h>

int write_at_line(int n_block, int line_number);
int randomInt(int a, int b);
float randomFloat(float lower_bound, float upper_bound);

int main()
{
    srand(time(NULL));
    
    int n_blocks = randomInt(2, 5);
    n_blocks=4;

    write_at_line(n_blocks, 24);
	
    return 0;
}


int write_at_line(int n_block, int line_number)
{

  char line[100];
  char *str = "This is a string to append";

  int current_line = 1;

  FILE *file = fopen("../ros_impedance_controller/worlds/mio_tavolo_temp.world", "r+");
  FILE *file2 = fopen("../ros_impedance_controller/worlds/mio_tavolo.world", "w");

  if (file == NULL) {
    printf("Error opening file!\n");
    return 1;
  }
  
  if (file2 == NULL) {
    printf("Error opening file 2!\n");
    return 1;
  }
  
  
  while (fgets(line, sizeof(line), file)) {
    fprintf(file2, "%s", line);
    
    if (current_line == line_number) {
      // seek back to the beginning of the line
      fseek(file, -strlen(line), SEEK_CUR);
      
      for(int i = 0; i < n_block; i++)
      {
          int block_num = randomInt(0,10);
          float x = randomFloat(0.1, 0.65);
          float y = randomFloat(0.2, 0.65);
          float z = randomFloat(0.92, 1.1);
          z = 0.89;
          float r = randomFloat(0.3, 0.6);
          float p = randomFloat(0.3, 0.6);
          r=randomInt(-1,1)*M_PI_2;
          p=randomInt(-1,1)*M_PI_2;
          r=0;
          //p=0;
          float ya = randomFloat(0.01, 3.14);
          
      	  // write the line, including the string to append
      	  fprintf(file2, "%s""%d""%s""%d""%s""%.5f"" %.5f"" %.5f"" %.5f"" %.5f"" %.5f""%s", "    <include>\n     <name>block", block_num, "high</name>\n     <uri>model://block", block_num, "high</uri>\n     <pose>", x, y, z, r, p, ya, "</pose>\n    </include>\n\n");
      }
      
      break;
    }
    current_line++;
  }
  
  fprintf(file2, "%s", "    <gui>\n     <camera name=\"gzclient_camera\">\n     <pose>1. 3.2 2.2 0. 0.4 -1.75</pose>\n     </camera>\n    </gui>\n\n  </world>\n</sdf>");
  

  fclose(file);
  fclose(file2);
  

  return 0;

}

int randomInt(int a, int b) {
	return rand() % (b - a + 1) + a;
}

float randomFloat(float lower_bound, float upper_bound) {
	return ((float)rand() / RAND_MAX) * (upper_bound - lower_bound) + lower_bound;
}




