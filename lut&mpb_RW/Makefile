SHELL=sh

OBJ = includes/scc.o includes/distribution.o includes/sccmalloc.o includes/memfun.o 
SRC = $(OBJ:%.o=%.c)
HDR = $(OBJ:%.o=%.h)


default:
		@echo "Usage: make test"
		@echo "       make clean"

test: test.c config.o RCCE_memcpy.c 
	gcc -g -o test $(SRC) $(HDR) includes/configuration.h test.c config.o -lpthread
config.o: config.c config.h
	gcc -g -c config.c -o config.o

clean:
	@ rm -f *.o test
