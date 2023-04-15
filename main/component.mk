#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)
COMPONENT_SRCDIRS := .
COMPONENT_ADD_INCLUDEDIRS := include

LIBS := algobsec

COMPONENT_ADD_LDFLAGS     := -lbt -L $(COMPONENT_PATH)/lib \
                           $(addprefix -l,$(LIBS))
ALL_LIB_FILES := $(patsubst %,$(COMPONENT_PATH)/lib/lib%.a,$(LIBS))
COMPONENT_ADD_LINKER_DEPS := $(ALL_LIB_FILES)
COMPONENT_SUBMODULES += lib