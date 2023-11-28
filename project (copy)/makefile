CC=g++
EXE=object_detect

INC=$(wildcard ./inc/*.hpp)
SRC=$(wildcard ./src/*.cpp)
OBJ=$(SRC:./src/%.cpp=./obj/%.o)

include ./macros.mk
INC_DIR=./inc/
CPPFLAGS= -I$(INC_DIR) `pkg-config --cflags --libs opencv4` -O3  $(MACROS)  -lrealsense2 -lglut -lGL -lGLU -lpthread

all:$(EXE)

shared:./main.cpp ./libobj_detect.so $(INC)
	@echo "Linking ..."
	@$(CC) $< -o obj_detect $(CPPFLAGS) -lobj_detect  -L.
	@echo "Done"

$(EXE): ./obj/main.o $(OBJ) 
	@echo "Linking ..."
	@$(CC)  $^ -o $@ $(CPPFLAGS)
	@echo "Done"

./obj/main.o: ./main.cpp ./inc/main_inc.hpp
	@echo "Compiling main.o ..."
	@$(CC) -c $< -o $@ $(CPPFLAGS)
	
libobj_detect.so: $(OBJ) $(INC)
	@echo "Compiling shared lib " $@ " ..."
	@$(CC) -shared -o $@ $(OBJ) $(CPPFLAGS)

./obj/%.o: ./src/%.cpp ./inc/%.hpp $(include) ./macros.mk
	@echo "Compiling source file " $@ " ..."
	@$(CC) -c $< -o $@ $(CPPFLAGS) -fpic

clean:
	@echo "Cleaning.."
	@rm -rf ./obj/*
	@rm $(EXE)

execute_0:
	$(EXE) 0
	
execute_1:
	$(EXE) 1
	
	
