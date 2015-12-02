OBJS= imuparameters.o laserparameters.o localizemap.o \
	localizemap_test.o localizer.o motionmodel.o planner.o

BINDIR=../bin/
LIBDIR=../lib/
LIB= localizer
LIBNAME=lib$(LIB).so

APPS= $(BINDIR)clf2lmap $(BINDIR)clfmerge $(BINDIR)pgm2lmap $(BINDIR)localize_test



LIB= localizer
LIBNAME=lib$(LIB).so

LDFLAGS+=-lm -lm -L$(LIBDIR) -lstuff
CXXFLAGS_COMPILE=-c -Wall -O3 -I../ -fPIC
CXXFLAGS_LIB=-shared
CXXFLAGS_BIN=-l$(LIB) -Wall -O3 -I../

.phony: clean all

all: $(LIBDIR)$(LIBNAME) $(APPS)

clean:
	rm -rf *~ $(OBJS) $(LIBNAME) $(APPS)

%.o:	%.cpp
	$(CXX) $(CXXFLAGS_COMPILE) -o $@ $^

$(LIBDIR)$(LIBNAME):	$(OBJS)
	$(CXX) $(CXXFLAGS_LIB) $(LDFLAGS) -o $(LIBDIR)$(LIBNAME) $^

$(BINDIR)clf2lmap: clf2lmap.cpp
	$(CXX)   -o $@ $^ $(LDFLAGS) -l$(LIB) $(CXXFLAGS_BIN)

$(BINDIR)clfmerge: clfmerge.cpp
	$(CXX)   -o $@ $^ $(LDFLAGS) -l$(LIB) $(CXXFLAGS_BIN)

$(BINDIR)pgm2lmap: pgm2lmap.cpp
	$(CXX)   -o $@ $^ $(LDFLAGS) -l$(LIB) $(CXXFLAGS_BIN)

$(BINDIR)localize_test: localize_test.cpp
	$(CXX)   -o $@ $^ $(LDFLAGS) -l$(LIB) $(CXXFLAGS_BIN)
