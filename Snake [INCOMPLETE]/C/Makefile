TUPLE         := x86_64-pc-linux-gnu-
CC            := $(TUPLE)gcc
OBJCPY        := $(TUPLE)objcopy
STRIP         := $(TUPLE)strip
CFLAGS        := -Wall -Wextra -std=c99 -O2 -march=native -fno-stack-check -fno-stack-protector
LDFLAGS       := -pthread
DEFINES       := -D _POSIX_C_SOURCE=200809L

UFILES        := 

# Programs
#  - Init
UFILES        := $(UFILES) snake.o

.PHONY: all rebuild clean

all: snake.elf.strip

rebuild: clean
	$(MAKE) all

clean:
	rm -f *.elf *.strip $(UFILES)

%.o: %.c
	$(CC) $(CFLAGS) $(DEFINES) $^ -c -o $@

snake.elf: $(UFILES)
	$(CC) $(CFLAGS) $(LDFLAGS) $(UFILES) -o $@

snake.elf.strip: snake.elf
	$(STRIP) -s -x -R .comment -R .text.startup $^ -o $@
