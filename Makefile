CC = gcc
CCFLAGS = -Wall -Werror -Wextra -pedantic -g -O2 -std=gnu99
SINGLE_MESSAGE_TARGET = bin/singe_message
SINGLE_MESSAGE_DEPENDS = source/single_message.c
INCLUDE = include

$(SINGLE_MESSAGE_TARGET): $(SINGLE_MESSAGE_DEPENDS)
	$(CC) $(CCFLAGS) $< -I$(INCLUDE) -o $@

clean:
	-rm $(TARGET)
