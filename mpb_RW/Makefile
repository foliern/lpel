SHELL=sh

default:
		@echo "Usage: make test"
		@echo "       make clean"

test: test.c config.o RCCE_memcpy.c
	gcc -g -o test test.c config.o

config.o: config.c config.h
	gcc -g -c config.c -o config.o

clean:
	@ rm -f *.o test
