.PHONY: all, clean
.DEFAULT_GOAL := normal
CC = gcc
CCFLAGS = -Wall -Werror -Wextra -pedantic -O2 -std=gnu99
INCLUDE = include
INC_WIRINGPI = wiringPi
DEBUG = DEBUG


SINGLE_MESSAGE_DEPENDS = source/single_message.c source/spi_utils.c
RANGING_DEPENDS = source/ranging.c source/spi_utils.c
SPI_TEST_DEPENDS = source/tests/spi_comm_check.c

single_message: $(SINGLE_MESSAGE_DEPENDS)
	$(CC) $(CCFLAGS) $^ -I$(INCLUDE) -l$(INC_WIRINGPI) -o bin/$@

single_message_debug: $(SINGLE_MESSAGE_DEPENDS)
	$(CC) $(CCFLAGS) -Og -g $^ -DVERBOSITY=3 -I$(INCLUDE) -l$(INC_WIRINGPI) -o bin/$@

single_message_small: $(SINGLE_MESSAGE_DEPENDS)
	$(CC) $(CCFLAGS) -Os $^ -I$(INCLUDE) -l$(INC_WIRINGPI) -o bin/$@


ranging: $(RANGING_DEPENDS)
	$(CC) $(CCFLAGS) $^ -I$(INCLUDE) -l$(INC_WIRINGPI) -o bin/$@

ranging_debug: $(RANGING_DEPENDS)
	$(CC) $(CCFLAGS) -Og -g $^ -I$(INCLUDE) -l$(INC_WIRINGPI) -D$(DEBUG) -DVERBOSITY=3 -o bin/$@

ranging_small: $(RANGING_DEPENDS)
	$(CC) $(CCFLAGS) -Os $^ -I$(INCLUDE) -l$(INC_WIRINGPI) -o bin/$@

spi_test: $(SPI_TEST_DEPENDS)
	$(CC) $(CCFLAGS) $^ -I$(INCLUDE) -o bin/$@

spi_test_debug: $(SPI_TEST_DEPENDS)
	$(CC) $(CCFLAGS) -Og -g $^ -I$(INCLUDE) -DVERBOSITY=3 -o bin/$@

spi_test_small: $(SPI_TEST_DEPENDS)
	$(CC) $(CCFLAGS) -Os $^ -I$(INCLUDE) -o bin/$@


all: single_message single_message_debug single_message_small spi_test spi_test_debug spi_test_small ranging ranging_debug ranging_small

normal: single_message spi_test ranging

clean:
	-rm bin/*
