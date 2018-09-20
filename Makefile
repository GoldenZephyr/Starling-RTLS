CC = gcc
CCFLAGS = -Wall -Werror -Wextra -pedantic -O2 -std=c99
TARGET = spi_test
DEPENDS = spi_test.c spi_test.h


$(TARGET): $(DEPENDS)
	$(CC) $(CCFLAGS) $^ -o bin/$@

clean:
	-rm $(TARGET)
