
PROGRAMS=nvme/smartio_nvme
all: $(PROGRAMS)

BITS=$(shell sh -c 'getconf LONG_BIT || echo NA')
ifeq ($(BITS),NA)
LIB_DIRECTORY=lib
endif

ifeq ($(BITS),32)
LIB_DIRECTORY=lib
endif

ifeq ($(BITS),64)
LIB_DIRECTORY=lib64
endif

CFLAGS+=-I/opt/DIS/include -I/opt/DIS/include/dis
LDFLAGS+=-Wl,-L/opt/DIS/$(LIB_DIRECTORY),-rpath,/opt/DIS/$(LIB_DIRECTORY),-lsisci



clean:
	$(RM) $(PROGRAMS) **/*.o
