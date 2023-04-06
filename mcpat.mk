TARGET = mcpat
SHELL = /bin/sh
.PHONY: all depend clean
.SUFFIXES: .cc .o

ifndef NTHREADS
  NTHREADS = 1
endif


LIBS = 
INCS = -lm

ifeq ($(TAG),dbg)
  DBG = -Wall 
  OPT = -ggdb -g -O0 -DNTHREADS=1 -Ifncacti
else
  DBG = 
  OPT = -O3 -msse2 -mfpmath=sse -DNTHREADS=$(NTHREADS) -Ifncacti
  #OPT = -O0 -DNTHREADS=$(NTHREADS)
endif

ifeq ($(ARCH),ia32)
  OPT += -m32
endif

ifneq ($(CACHE),)
  OPT += -DENABLE_CACHE
  LIBS += -ldb
endif

 #CXXFLAGS = -Wall -Wno-unknown-pragmas -Winline $(DBG) $(OPT)
 CXXFLAGS = -Wno-unknown-pragmas $(DBG) $(OPT)
#CXX = g++ -m32
#CC  = gcc -m32
 CXX = g++
 CC  = gcc
 
# VPATH = fncacti
vpath %.h fncacti
vpath %.cc fncacti 
 
SRCS  = \
  Ucache.cc \
  XML_Parse.cc \
  arbiter.cc \
  area.cc \
  array.cc \
  bank.cc \
  basic_circuit.cc \
  basic_components.cc \
  cacti_interface.cc \
  component.cc \
  core.cc \
  crossbar.cc \
  decoder.cc \
  htree2.cc \
  interconnect.cc \
  io.cc \
  iocontrollers.cc \
  logic.cc \
  main.cc \
  mat.cc \
  memoryctrl.cc \
  noc.cc \
  nuca.cc \
  parameter.cc \
  processor.cc \
  router.cc \
  sharedcache.cc \
  subarray.cc \
  technology.cc \
  uca.cc \
  wire.cc \
  xmlParser.cc
#  xmlParser.cc \
#  powergating.cc

OBJS = $(patsubst %.cc,obj_$(TAG)/%.o,$(SRCS))

all: obj_$(TAG)/$(TARGET)
	cp -f obj_$(TAG)/$(TARGET) $(TARGET)$(SUFFIX)

obj_$(TAG)/$(TARGET) : $(OBJS)
	$(CXX) $(OBJS) -o $@ $(INCS) $(CXXFLAGS) $(LIBS) -pthread

#obj_$(TAG)/%.o : %.cc
#	$(CXX) -c $(CXXFLAGS) $(INCS) -o $@ $<

obj_$(TAG)/%.o : %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	-rm -f *.o $(TARGET)


