# ========================
# Compiler & flags
# ========================
CXX       := g++
CXXFLAGS  := -march=native -mtune=native -flto -g -DDEBUG \
             -std=c++20 -Wall -Wextra -Wpedantic 
INCLUDES  := -Iinclude
LDFLAGS   := -flto

# ========================
# Project structure
# ========================
SRC_DIR   := src
INC_DIR   := include
BUILD_DIR := build
OUT_DIR	  := out
BIN       := $(OUT_DIR)/app

# ========================
# Files
# ========================
SRCS := $(shell find $(SRC_DIR) -name '*.cpp')
OBJS := $(patsubst $(SRC_DIR)/%.cpp,build/%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)

# ========================
# Targets
# ========================
all: $(BIN)

$(OUT_DIR):
	mkdir -p $(OUT_DIR)

$(BIN): $(OBJS) | $(OUT_DIR)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -MMD -MP -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(OUT_DIR)

rebuild: clean all

-include $(DEPS)

.PHONY: all clean rebuild
