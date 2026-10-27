CC ?= cc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror
SANITIZE ?= -fsanitize=address,undefined -fno-omit-frame-pointer

build/libssd.a: src/ssd.c include/rightline/ssd.h | build
	$(CC) $(CFLAGS) -Iinclude -c src/ssd.c -o build/ssd.o
	ar rcs $@ build/ssd.o

build/test_ssd: tests/test_ssd.c src/ssd.c include/rightline/ssd.h | build
	$(CC) $(CFLAGS) -Wno-conversion -Wno-sign-conversion $(SANITIZE) -Iinclude tests/test_ssd.c src/ssd.c -o $@

test: build/test_ssd
	./build/test_ssd

build:
	mkdir -p build

clean:
	rm -rf build

.PHONY: test clean
