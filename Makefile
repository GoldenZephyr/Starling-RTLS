.PHONY: all, clean
.DEFAULT_GOAL := all
CC = gcc
CCFLAGS = -Wall -Werror -Wextra -pedantic -g -O2 -std=gnu99
INCLUDE = include

SINGLE_MESSAGE_DEPENDS = source/single_message.c

SPI_TEST_DEPENDS = source/tests/spi_comm_check.c

single_message: $(SINGLE_MESSAGE_DEPENDS)
	$(CC) $(CCFLAGS) $< -I$(INCLUDE) -o bin/$@

spi_test: $(SPI_TEST_DEPENDS)
	$(CC) $(CCFLAGS) $< -I$(INCLUDE) -o bin/$@

all: single_message spi_test

clean:
	-rm bin/single_message bin/spi_test
