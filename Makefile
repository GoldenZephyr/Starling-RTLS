.PHONY: all, clean
.DEFAULT_GOAL := normal
CC = gcc
CCFLAGS = -Wall -Werror -Wextra -pedantic -O3 -std=gnu99
INCLUDE = include
INC_WIRINGPI = wiringPi

SINGLE_MESSAGE_DEPENDS = source/single_message.c source/spi_utils.c
SPI_TEST_DEPENDS = source/tests/spi_comm_check.c

single_message: $(SINGLE_MESSAGE_DEPENDS)
	$(CC) $(CCFLAGS) $^ -I$(INCLUDE) -o bin/$@

single_message_debug: $(SINGLE_MESSAGE_DEPENDS)
	$(CC) $(CCFLAGS) -Og -g $^ -I$(INCLUDE) -o bin/$@

single_message_small: $(SINGLE_MESSAGE_DEPENDS)
	$(CC) $(CCFLAGS) -Os $^ -I$(INCLUDE) -o bin/$@



spi_test: $(SPI_TEST_DEPENDS)
	$(CC) $(CCFLAGS) $^ -I$(INCLUDE) -o bin/$@

spi_test_debug: $(SPI_TEST_DEPENDS)
	$(CC) $(CCFLAGS) -Og -g $^ -I$(INCLUDE) -o bin/$@

spi_test_small: $(SPI_TEST_DEPENDS)
	$(CC) $(CCFLAGS) -Os $^ -I$(INCLUDE) -o bin/$@


all: single_message single_message_debug single_message_small spi_test spi_test_debug spi_test_small

normal: single_message spi_test

clean:
	-rm bin/*
