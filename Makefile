CC = gcc
CFLAGS = -Wall -Wextra -O2
LIBS = -lpthread -lrt

# Default target builds from infra-dread_direct_gpio.c
all: infra-dread

# Build from infra-dread_direct_gpio.c (default)
infra-dread: infra-dread_direct_gpio.c
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)

# Build from infra-dread_direct_gpio.c
infra-dread-new: infra-dread_direct_gpio.c
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)

# Clean compiled binaries
clean:
	rm -f infra-dread infra-dread

# Install the binaries
install:
	install -m 755 infra-dread-direct /usr/local/bin/

# Phony targets
.PHONY: all clean install
