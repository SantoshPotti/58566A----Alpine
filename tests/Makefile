# Desktop test build. Deliberately seperate from the PROS Makefile at the repo
# root so that 'pros make' and 'make -C test' never fight over the same file
# 
# Run from the repo root: "make -C tests"
#
# Every source listed here must compile with plain g++, which is the mechanism 
# that enforces "no Pros headers in the core". If a file stops building here,
# that would therefore mean that something 'leaked', or that it got lost somehwere in the process

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -I../include
BUILD    := ../build

CORE := ../src/alpine/localization/field.cpp

TESTS := test_field

.PHONY: all test clean

all: test

test: $(TESTS)

test_field: test_field.cpp $(CORE)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $^ -o $(BUILD)/$@
	@$(BUILD)/$@

clean:
	rm -rf $(BUILD)