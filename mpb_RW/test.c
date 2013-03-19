#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "config.h"
#include "RCCE_memcpy.c"

t_vcharp mpbs[48];

void readFromMpb(int core, int size){
  char *result = (char*) malloc (size);
  memcpy_get((void*)result, (void*)mpbs[core],size);
  result[size-1]='\0';
  printf("%s\n",result);
  free(result);
}

void writeToMpb(int core, int size,char *str){
  char *buf = (char*) malloc (size);
  int len = strlen(str);
  printf("%d %s\n\n",len,str);
  if(len == 1){
    memset(buf,(char)*str, size);
  }else{
    int i;
    for(i=0; i+len <=size; i+=len){
      memcpy(buf+i,str,len);
    } 
  }

  buf[size-1] = '\0';


  printf("%s\n\nCopying to mpb now\n",buf);

  memcpy_put((void*)mpbs[core],(void*)buf,size);

  free(buf);
}


int main(int argc, char **argv){

   int core,size;
   int x, y, z, address,node_location;
   unsigned char cpu;
  
   InitAPI(0);
   z = ReadConfigReg(CRB_OWN+MYTILEID);
   x = (z >> 3) & 0x0f; // bits 06:03
   y = (z >> 7) & 0x0f; // bits 10:07
   z = z & 7; // bits 02:00
   node_location = PID(x, y, z);
 
  for (cpu = 0; cpu < 48; cpu++) {
    x = X_PID(cpu);
    y = Y_PID(cpu);
    z = Z_PID(cpu);

    if (cpu == node_location) address = CRB_OWN;
    else address = CRB_ADDR(x, y);
    
    MPBalloc(&mpbs[cpu], x, y, z, cpu == node_location);
  }

    if (argc ==  3 && !strcmp("test", argv[1])) {
      core = atoi(argv[2]);
      //testMPB(core);       
    } else if (argc == 4 && !strcmp("read", argv[1])){
      core = atoi(argv[2]);
      size = atoi(argv[3]);
      readFromMpb(core, size);
    }else if (argc == 5 && !strcmp("write", argv[1])){//string
      core = atoi(argv[2]);
      size = atoi(argv[3]);
      printf("core=%d size=%d char=%s\n\n",core,size,argv[4]);
      writeToMpb(core,size,argv[4]);
    }else{
      fprintf(stderr, "Usage:\n"
          "%s test <destination core> \n"
          "read <source core ID> <data size> \n"
          "write <destination core ID> <data size> (<char to write> || <string to write>) \n", argv[0]);
      exit(1);
    }

  return 0;
}


