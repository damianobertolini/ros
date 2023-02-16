#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <time.h>

#define LINE_NUMEBR 24

//int write_at_line(int n_block);
int randomInt(int a, int b);
float randomFloat(float lower_bound, float upper_bound);
void task1();
void task2(int n_blocks);
void task3(int n_blocks);
void open_files();
void close_files();
int checkDuplicates(int already[], int n, int proposal);

int main(int argc, char *argv[]) {
  if (argc != 2) {
    printf("Usage: %s <integer>\n", argv[0]);
    return 1;
  }

  int task = atoi(argv[1]);
  srand(time(NULL));

  int n_blocks = randomInt(2, 5);

  switch (task) {
    case 0:
      printf("insert a task bwtween 1 and 4\n");
      break;
    case 1:
      task1();
      break;
    case 2:
      task2(n_blocks);
      break;
    case 3:
      task3(n_blocks);
      break;
    case 4:
      //task(n_blocks, 4);
      break;
  }

  //write_at_line(n_blocks);

  return 0;
}

void task1(){
  char line[100];
  char *str = "This is a string to append";
  int current_line = 1;

  FILE *file = fopen("../ros_impedance_controller/worlds/mio_tavolo_temp.world", "r+");
  FILE *file2 = fopen("../ros_impedance_controller/worlds/mio_tavolo.world", "w");

  if (file == NULL) {
    printf("Error opening file!\n");
    return;
  }

  if (file2 == NULL) {
    printf("Error opening file 2!\n");
    return;
  }

  int block_num;

  while (fgets(line, sizeof(line), file)) {
    fprintf(file2, "%s", line);
    
    if (current_line == LINE_NUMEBR) {
      // seek back to the beginning of the line
      fseek(file, -strlen(line), SEEK_CUR);
      
      block_num = randomInt(0,10);
      float x = randomFloat(0.05, 0.6);
      float y = randomFloat(0.22, 0.75);
      float z = 0.871;
      float r = 0.0;
      float p = 0.0;
      float ya = randomFloat(0.0, 6.28318);
      
      // write the line, including the string to append
      fprintf(file2, "%s""%d""%s""%d""%s""%.5f"" %.5f"" %.5f"" %.5f"" %.5f"" %.5f""%s", "    <include>\n     <name>block#", 1, "</name>\n     <uri>model://block", block_num, "high</uri>\n     <pose>", x, y, z, r, p, ya, "</pose>\n    </include>\n\n");
      
      break;
    }
    current_line++;
  }
  
  fprintf(file2, "%s", "    <gui>\n     <camera name=\"gzclient_camera\">\n     <pose>1. 3.2 2.2 0. 0.4 -1.75</pose>\n     </camera>\n    </gui>\n\n  </world>\n</sdf>");
  

  fclose(file);
  fclose(file2);
  
  printf("world for task1 created with block: %d\n", block_num);
  return;
}

void task2(int n_blocks) {

  char line[100];
  char *str = "This is a string to append";

  int current_line = 1;

  FILE *file = fopen("../ros_impedance_controller/worlds/mio_tavolo_temp.world", "r+");
  FILE *file2 = fopen("../ros_impedance_controller/worlds/mio_tavolo.world", "w");

  if (file == NULL) {
    printf("Error opening file!\n");
    return;
  }

  if (file2 == NULL) {
    printf("Error opening file 2!\n");
    return;
  }
  
  int taken[n_blocks];
  for(int j = 0; j < n_blocks; j++){
    taken[j] = -1;
  }

  while (fgets(line, sizeof(line), file)) {
    fprintf(file2, "%s", line);

    if (current_line == LINE_NUMEBR) {
      // seek back to the beginning of the line
      fseek(file, -strlen(line), SEEK_CUR);

      for (int i = 0; i < n_blocks; i++) {
        int block_num;
        do{
          block_num = randomInt(0, 10);
        } while(checkDuplicates(taken, n_blocks, block_num));
        taken[i] = block_num;
        
        float x = randomFloat(0.05, 0.6);
        float y = randomFloat(0.22, 0.75);
        float z = 0.871;
        float r = 0.0;
        float p = 0.0;
        float ya = randomFloat(0.0, 6.28318);

        // write the line, including the string to append
        fprintf(file2, "%s""%d""%s""%d""%s""%.5f"" %.5f"" %.5f"" %.5f"" %.5f"" %.5f""%s", "    <include>\n     <name>block#", i+1, "</name>\n     <uri>model://block", block_num, "high</uri>\n     <pose>", x, y, z, r, p, ya, "</pose>\n    </include>\n\n");
      }
      
      break;
    }
    current_line++;
  }

  fprintf(file2, "%s",
          "    <gui>\n     <camera name=\"gzclient_camera\">\n     <pose>1. "
          "3.2 2.2 0. 0.4 -1.75</pose>\n     </camera>\n    </gui>\n\n  "
          "</world>\n</sdf>");

  fclose(file);
  fclose(file2);

  printf("world for task2 created with %d blocks:", n_blocks);
  for(int i=0; i<n_blocks; i++){printf(" %d", taken[i]);} printf("\n");
  return;
}

void task3(int n_blocks) {

  char line[100];
  char *str = "This is a string to append";

  int current_line = 1;

  FILE *file = fopen("../ros_impedance_controller/worlds/mio_tavolo_temp.world", "r+");
  FILE *file2 = fopen("../ros_impedance_controller/worlds/mio_tavolo.world", "w");

  if (file == NULL) {
    printf("Error opening file!\n");
    return;
  }

  if (file2 == NULL) {
    printf("Error opening file 2!\n");
    return;
  }

  int taken[n_blocks];
  for(int j = 0; j < n_blocks; j++){
    taken[j] = -1;
  }

  while (fgets(line, sizeof(line), file)) {
    fprintf(file2, "%s", line);

    if (current_line == LINE_NUMEBR) {
      // seek back to the beginning of the line
      fseek(file, -strlen(line), SEEK_CUR);

      for (int i = 0; i < n_blocks; i++) {
        int block_num = randomInt(0, 10);
        taken[i] = block_num;
        int pose = randomInt(1, 4);
        
        float x = randomFloat(0.05, 0.6);
        float y = randomFloat(0.22, 0.75);
        float z = 0.871;
        float r = 0.0;
        float p = 0.0;
        float ya = randomFloat(0.0, 6.28318);
        switch(pose){
          case 1:
            r = 0.0;
            p = 0.0;
            break;
          case 2:
            r = 1.57079;
            p = 0.0;
            break;
          case 3:
            r = 1.57079;
            p = 1.57079;
            break;
          case 4:
            r = 3.14159;
            z += 0.2;
            break;
        }

        // write the line, including the string to append
        fprintf(file2, "%s""%d""%s""%d""%s""%.5f"" %.5f"" %.5f"" %.5f"" %.5f"" %.5f""%s", "    <include>\n     <name>block#", i+1, "</name>\n     <uri>model://block", block_num, "high</uri>\n     <pose>", x, y, z, r, p, ya, "</pose>\n    </include>\n\n");
      }
      
      break;
    }
    current_line++;
  }

  fprintf(file2, "%s",
          "    <gui>\n     <camera name=\"gzclient_camera\">\n     <pose>1. "
          "3.2 2.2 0. 0.4 -1.75</pose>\n     </camera>\n    </gui>\n\n  "
          "</world>\n</sdf>");

  fclose(file);
  fclose(file2);

  printf("world for task3 created with %d blocks:", n_blocks);
  for(int i=0; i<n_blocks; i++){printf(" %d", taken[i]);} printf("\n");
  return;
}

int randomInt(int a, int b) { return rand() % (b - a + 1) + a; }

float randomFloat(float lower_bound, float upper_bound) {
  return ((float)rand() / RAND_MAX) * (upper_bound - lower_bound) + lower_bound;
}

int checkDuplicates(int already[], int n, int proposal) {
    int i;
    for (i = 0; i < n; i++) {
        if (already[i] == proposal) {
            return 1; // il numero è già presente nell'array
        }
    }
    return 0; // il numero non è presente nell'array
}