CC = gcc
CFLAGS = -lm -O3 -Wall -Wextra -Wconversion -Wsign-conversion
SRC = main.c OF_lib.h OF.c PSO.c utility.h
TARGET = pso

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) -o $(TARGET) $(SRC) $(CFLAGS)

clean:
	rm -f $(TARGET)