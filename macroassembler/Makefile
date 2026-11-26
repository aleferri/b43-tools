CC		?= cc
PREFIX		?= /usr/local
CFLAGS		?= -O2
CFLAGS		+= -std=c99 -D_POSIX_C_SOURCE=200809L \
		   -Wall -Wextra -Wformat=2 -Wundef -Wno-unused-parameter \
		   -Wshadow -Wpointer-arith

V		= @
Q		= $(V:1=)
QUIET_CC	= $(Q:@=@echo '     CC       '$@;)$(CC)

BIN		= b43-masm
SRCS		= $(wildcard src/*.c)
OBJS		= $(SRCS:src/%.c=obj/%.o)
DEPS		= $(OBJS:.o=.d)

all: $(BIN)

obj/%.o: src/%.c
	@mkdir -p obj
	$(QUIET_CC) $(CFLAGS) -MMD -MP -c -o $@ $<

$(BIN): $(OBJS)
	$(QUIET_CC) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)

install: all
	install -d $(DESTDIR)$(PREFIX)/bin
	install -m 755 $(BIN) $(DESTDIR)$(PREFIX)/bin/

check: all
	./tests/run.sh

clean:
	rm -rf obj $(BIN)

distclean: clean

.PHONY: all install check clean distclean

-include $(DEPS)
