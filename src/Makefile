sources = mfg_test.c

headers = 

CFLAGS=-Wall

INC = -I./include

ifeq ($(DEBUG_DEF),1)
	DEBUG = -g -O0 -DDEBUG
else
	DEBUG = -O2
endif

all: mfg-test

mfg-test: $(sources) $(headers)
	$(CC) -DAPP_VERSION='"$(APP_VERSION)"' $(CFLAGS) $(INC) $(LDFLAGS) $(DEBUG) -o $@ $(sources)

clean:
	$(RM) mfg-test

.PHONY: all clean
