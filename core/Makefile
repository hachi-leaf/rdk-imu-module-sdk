CC       = gcc
AR       = ar
CFLAGS   = -Wall -Wextra -O2 -I include -I src
LDFLAGS  =
LDLIBS   = -lpthread -lgpiod -lm

SRC_DIR  = src
INC_DIR  = include
TST_DIR  = examples
BLD_DIR  = build
OUT_DIR  = out
LIB_DIR  = lib

LIB_SRCS = $(SRC_DIR)/rdkimu.c $(SRC_DIR)/rdkimu_bus.c
TST_SRCS = $(TST_DIR)/test.c

VERSION  := $(shell cat ../version)

LIB_NAME  = rdkimu
STATIC_LIB = $(LIB_DIR)/lib$(LIB_NAME).a
SHARED_LIB = $(LIB_DIR)/lib$(LIB_NAME).so.$(VERSION)
SONAME     = lib$(LIB_NAME).so.1

OBJS_STATIC = $(patsubst $(SRC_DIR)/%.c, $(BLD_DIR)/static/%.o, $(LIB_SRCS))
OBJS_SHARED = $(patsubst $(SRC_DIR)/%.c, $(BLD_DIR)/shared/%.o, $(LIB_SRCS))

TEST_TARGET  = $(OUT_DIR)/test

PREFIX ?= /usr/local

.PHONY: all lib test clean install uninstall

all: lib $(TEST_TARGET)

lib: $(STATIC_LIB) $(SHARED_LIB)

$(BLD_DIR)/static/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

$(STATIC_LIB): $(OBJS_STATIC)
	@mkdir -p $(@D)
	$(AR) rcs $@ $^

$(BLD_DIR)/shared/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -fPIC -c $< -o $@

$(SHARED_LIB): $(OBJS_SHARED)
	@mkdir -p $(@D)
	$(CC) -shared -Wl,-soname,$(SONAME) -o $@ $^ $(LDLIBS)
	@cd $(LIB_DIR) && ln -sf lib$(LIB_NAME).so.$(VERSION) $(SONAME) && ln -sf $(SONAME) lib$(LIB_NAME).so

$(TEST_TARGET): $(TST_SRCS) $(SHARED_LIB)
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) $< -L$(LIB_DIR) -l$(LIB_NAME) -Wl,-rpath,'$$ORIGIN/../lib' -o $@

test: $(TEST_TARGET)
	@echo "Running $(notdir $<)..."
	./$<

clean:
	rm -rf $(BLD_DIR) $(OUT_DIR) $(LIB_DIR)

install: lib
	install -d $(DESTDIR)$(PREFIX)/include
	install -m 644 $(INC_DIR)/rdkimu.h $(DESTDIR)$(PREFIX)/include/
	install -d $(DESTDIR)$(PREFIX)/lib
	install -m 755 $(SHARED_LIB) $(DESTDIR)$(PREFIX)/lib/
	ln -sf lib$(LIB_NAME).so.$(VERSION) $(DESTDIR)$(PREFIX)/lib/$(SONAME)
	ln -sf $(SONAME) $(DESTDIR)$(PREFIX)/lib/lib$(LIB_NAME).so
	@echo "Library and header installed to $(PREFIX)"

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/include/rdkimu.h
	rm -f $(DESTDIR)$(PREFIX)/lib/lib$(LIB_NAME).so*
	@echo "Library and header removed from $(PREFIX)"