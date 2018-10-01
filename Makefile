CC = gcc
CCFLAGS = -Wall -Werror -Wextra -pedantic -g -O2 -std=gnu99
TARGET = bin/single_message
DEPENDS = source/single_message.c 
INCLUDE = include

$(TARGET): $(DEPENDS)
	$(CC) $(CCFLAGS) $^ -I$(INCLUDE) -o $@

clean:
	-rm $(TARGET)
