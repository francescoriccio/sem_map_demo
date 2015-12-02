OBJS= stat.o
BINDIR=../bin/
LIBDIR=../lib/
LIB= stuff
LIBNAME=lib$(LIB).so
APPS= $(BINDIR)stat_test $(BINDIR)transformation3_test


LDFLAGS+=-lm -L$(LIBDIR)
CXXFLAGS_COMPILE=-c -Wall -O3 -I../ -fPIC
CXXFLAGS_LIB=-shared
CXXFLAGS_BIN=-lstuff

.phony: clean all

all: $(LIBDIR)$(LIBNAME) $(APPS)

clean:
	rm -rf *~ $(OBJS) $(LIBDIR)$(LIBNAME) $(APPS)

%.o:	%.cpp
	$(CXX) $(CXXFLAGS_COMPILE) -o $@ $^

$(LIBDIR)$(LIBNAME):	$(OBJS)
	$(CXX) $(CXXFLAGS_LIB) $(LDFLAGS) -o $(LIBDIR)$(LIBNAME) $^

$(BINDIR)stat_test: stat_test.cpp	
	$(CXX) $(CXXFLAGS_BIN) $(LDFLAGS) -l$(LIB) -o $@ $^

$(BINDIR)transformation3_test: transformation3_test.cpp	
	$(CXX) $(CXXFLAGS_BIN) $(LDFLAGS) -l$(LIB) -o $@ $^

