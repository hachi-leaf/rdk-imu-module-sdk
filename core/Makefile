CC       = gcc
CFLAGS	 = -Wall -Wextra -O2 -I include -I src
LDFLAGS  =
LDLIBS   = -lpthread -lgpiod -lm

SRC_DIR  = src
TEST_DIR = examples
BUILD_DIR = build
OUT_DIR   = out

SRCS     = $(SRC_DIR)/rdkimu.c $(SRC_DIR)/rdkimu_bus.c
TEST_SRC = $(TEST_DIR)/test.c

OBJS     = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRCS))
TEST_OBJ = $(patsubst $(TEST_DIR)/%.c, $(BUILD_DIR)/%.o, $(TEST_SRC))

TARGET   = $(OUT_DIR)/test

# 默认编译
.PHONY: all
all: $(TARGET)

# 链接
$(TARGET): $(OBJS) $(TEST_OBJ)
	@mkdir -p $(OUT_DIR)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

# 编译库源文件
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# 编译测试程序
$(BUILD_DIR)/%.o: $(TEST_DIR)/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# 彻底重新编译并运行测试
.PHONY: test
test: clean $(TARGET)
	@echo "Running $(TARGET)..."
	./$(TARGET)

# 清理所有生成文件
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR) $(OUT_DIR)