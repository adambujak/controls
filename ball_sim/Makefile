EIGEN_PATH = /usr/local/include/eigen3
APPLICATION = ball_sim
OUTPUT = _build
ARGS ?= ""

CPP_FILES = main.cpp LTI.cpp
CPP_INCLUDES = -I$(EIGEN_PATH) 

default: build

clean: 
	rm -rf $(OUTPUT)

build:
	@echo "Compiling..."
	@mkdir -p $(OUTPUT) 
	@g++ -Wno-c++11-extensions -Wno-c++11-long-long $(CPP_INCLUDES) $(CPP_FILES) -o "$(OUTPUT)/$(APPLICATION)_$(TARGET).out"

run: build
	rm -f output.dat
	@echo "Running:"
	@./$(OUTPUT)/$(APPLICATION)_$(TARGET).out $(ARGS)

