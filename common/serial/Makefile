#  Makefile
#  Copyright 2011 Iain Peet
#
#  Builds all common serial protocol sources, and a tester program.
# 
#  Root makefile.  Includes subdir makefiles to get a complete list of
#  available sources; generates deps and objects for those sources, 
#  and compiles binaries from the objects.
# 
#  This program is distributed under the of the GNU Lesser Public License. 
# 
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>

.DEFAULT_GOAL=all
.SECONDEXPANSION:

# Compile config
CC:= gcc
CCFLAGS:= -Wall -g -O0
INCLUDES:= -I. 
LDFLAGS:= -Wall -g -O0
LIBS:= -lm

# List of root-level c sources:
CC_SRCS:=$(wildcard *.c)

# Generated directories
GENDIR:=gen

# List of bins to link
BINS:=test

test_OBJS:= $(patsubst %.c,$(GENDIR)/%.o,$(CC_SRCS))

# Compilation of c objects
CC_OBJS:=$(patsubst %.c,$(GENDIR)/%.o,$(CC_SRCS))
$(CC_OBJS): $$(patsubst $(GENDIR)/%.o,%.c,$$@)
	@if test ! -e $(dir $@); then mkdir -p $(dir $@); fi
	@echo [CC] $@
	@$(CC) $(CCFLAGS) $(INCLUDES) $< -c -o $@

# .c dependency generation
CC_DEPS:=$(patsubst %.c,$(GENDIR)/%.d,$(CC_SRCS))
$(CC_DEPS): $$(patsubst $(GENDIR)/%.d,%.c,$$@)
	@if test ! -e $(dir $@); then mkdir -p $(dir $@); fi
	@echo [DEP] $@
	@$(CC) $(INCLUDES) -MM -MT $(<:%.c=$(GENDIR)/%.o) -MF $@ $<
deps: $(CC_DEPS)
include $(CC_DEPS)

# Binary link rules
$(BINS): $$($$@_OBJS)
	@echo [LINK] $@
	@$(CC) $(LDFLAGS) $^ $(LIBS) -o $@

tags: $(CC_SRCS)
	@echo Generate tags...
	@ctags $(CC_SRCS)

all: $(BINS) tags

clean:
	rm -rf $(GENDIR)
	rm -f $(BINS)
	rm -f tags

debugp:
	@echo "CC_SRCS: $(CC_SRCS)"
	@echo "CC_OBJS: $(CC_OBJS)"
	@echo "CC_DEPS: $(CC_DEPS)"

