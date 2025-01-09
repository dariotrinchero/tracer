# compiler flags
DEBUG    = -ggdb
OPTIMISE = -O0
WARNINGS = -Wall -Wextra -Wno-variadic-macros -Wno-overlength-strings -pedantic
CFLAGS   = $(DEBUG) $(OPTIMISE) $(WARNINGS)
DFLAGS   = -DDEBUG_MESH -DDEBUG_TRACER

# commands
# Note: "c++" should default to the main local C++ compiler; if it does not, on
# Linux, use "g++"
CC       = c++
RM       = rm -f
COMPILE  = $(CC) $(CFLAGS) $(DFLAGS)
INSTALL  = install

# directories
BINDIR   = ../bin
OUTDIR   = ../renders
LOCALBIN = ~/.local/bin

# files
EXES     = tracer
RENDER   = $(OUTDIR)/render.ppm

### RULES ######################################################################

.DEFAULT_GOAL := all

# executables

tracer: tracer.cc | $(BINDIR)
	$(COMPILE) -o $(BINDIR)/$@ $^

testlinalg: testlinalg.cc | $(BINDIR)
	$(COMPILE) -o $(BINDIR)/$@ $^

# units

# --- add if needed ---

# BINDIR & OUTDIR

$(BINDIR):
	mkdir $(BINDIR)

$(OUTDIR):
	mkdir $(OUTDIR)

### PHONY TARGETS ##############################################################

.PHONY: all clean install uninstall

all: tracer

render: tracer | $(OUTDIR)
	$(BINDIR)/tracer > $(RENDER)

resume: tracer | $(OUTDIR)
	if test -f $(RENDER); then $(BINDIR)/tracer $$(($$(wc -l < $(RENDER)) - 3)) >> $(RENDER); \
		else $(BINDIR)/tracer > $(RENDER); fi

clean:
	$(RM) $(foreach EXEFILE, $(EXES), $(BINDIR)/$(EXEFILE))
	$(RM) *.o
	$(RM) -rf $(BINDIR)/*.dSYM

# For your program to be in your PATH, ensure that the following is somewhere
# near the end of your ~/.profile (for macOS, this might actually be
# ~/.bash_profile) file:
#
# if [ -d "$HOME/bin" ] ; then
#     PATH="$HOME/bin:$PATH"
# fi
#
# If this is not set, the install rule will copy the executable to ~/bin, but
# it will not be resolved automatically by the shell.  If you update the
# profile, either close and reopen the terminal, or use the "source" command to
# reload the profile.
#
# Also, to enable bash completion (of filenames with .simpl extensions), add
# the following at the end of ~/.bashrc:
#
# # bash completion for SIMPL
# complete -f -X '!*.simpl' simplc

# Install all compiler-related binaries in the local bin.
install:
	mkdir -p $(LOCALBIN)
	$(INSTALL) $(foreach EXEFILE, $(EXES), $(wildcard $(BINDIR)/$(EXEFILE))) \
		$(LOCALBIN)

# Remove all compiler-related binaries from the local bin.
uninstall:
	$(RM) $(foreach EXEFILE, $(EXES), $(LOCALBIN)/$(EXEFILE))
