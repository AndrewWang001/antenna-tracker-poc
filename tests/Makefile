# Host unit tests for the hardware-independent firmware modules.
#   make -C tests          build + run
CC     ?= gcc
SRC_DIR = ../NUCLEO_F103RB_Antenna_Tracker/Core
CFLAGS  = -std=c11 -O2 -Wall -Wextra -Wpedantic -I$(SRC_DIR)/Inc -D_USE_MATH_DEFINES
SRCS    = test_main.c azel_float_legacy.c \
          $(SRC_DIR)/Src/azel.c $(SRC_DIR)/Src/pid.c $(SRC_DIR)/Src/rocket_can.c

run: test_runner
	./test_runner

test_runner: $(SRCS) $(wildcard $(SRC_DIR)/Inc/*.h)
	$(CC) $(CFLAGS) -o $@ $(SRCS) -lm

clean:
	rm -f test_runner test_runner.exe

.PHONY: run clean
