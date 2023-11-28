all: pose6D
CC = g++
FCPPLAGS = -lrealsense2 `pkg-config --cflags opencv4` `pkg-config --libs opencv4`

pose6D: main.cpp
	$(CC) $^ -o $@ $(FCPPLAGS)
