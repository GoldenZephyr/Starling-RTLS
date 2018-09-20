CC = gcc
CCFLAGS = -Wall -Werror -Wextra -pedantic -O2 -std=c99
TARGET = bin/single_message
DEPENDS = source/single_message.c 
INCLUDE = include

$(TARGET): $(DEPENDS)
	$(CC) $(CCFLAGS) $^ -I$(INCLUDE) -o $@

clean:
	-rm $(TARGET)
