all: pso

pso: main.c OF_lib.h OF.c PSO.c utility.h
	gcc -o pso main.c OF_lib.h OF.c PSO.c utility.h -lm -O3 -Wall -Wextra -Wconversion -Wsign-conversion


clean:
	rm -f pso