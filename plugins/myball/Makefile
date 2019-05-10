# makefile $Id: Makefile 539 2014-08-15 10:19:43Z jcan $
#
# libraryname (for shared library)
libname = myball.so
#
# preprocessor flags like -I(include dir)
CPPFLAGS = -I ../../include -I/usr/local/smr/include
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname)
#
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -pedantic -fPIC $(DEFINES)
# CXXFLAGS += -Wno-unused-but-set-variable
include /usr/local/smr/include/opencv.mk
include /usr/local/smr/include/opencv_flags.mk
# ifeq ($(OPENCV),2.1)
#   export OPENCV=
# endif
# ifeq ($(OPENCV),)
# else
#   CXXFLAGS += `pkg-config opencv-$(OPENCV) --cflags`
# endif
#
# Object files to produce before link
sourcescpp = ufuncball.cpp
#
sourcesh = $(sourcescpp:.cpp=.h)
OBJECTS = $(sourcescpp:.cpp=.o)

# shared library file name (version 0)
shlib = $(libname).0

all: $(shlib)
# compile all - all objects depend on all used files
$(shlib): $(OBJECTS) Makefile $(sources)
	c++ -o $(shlib) $(OBJECTS) $(LDFLAGS)

.o: $(sources) $(sourcesh)
	c++ $@



.PHONY : clean install
clean :
	rm -f $(shlib) $(OBJECTS)
	-@rm -fr *~ .deps


######################################################################
#
# Automatic dependencies

DEPS_MAGIC := $(shell mkdir -p .deps)


%.o: .deps/%.d

.deps/%.d: src/%.c
	@cc $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#' 

.deps/%.d: %.cpp
	@g++ $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#' > $@



######################################################################
#
# Include automatic dependencies

-include $(patsubst %.o, .deps/%.d, $(objects))
