sources_mfg = mfg_test.c

sources_fbimage = fbimage.c

headers_mfg = 

headers_fbimage = 

CFLAGS=-Wall

ifeq ($(DEBUG_DEF),1)
	DEBUG = -g -O0 -DDEBUG
else
	DEBUG = -O2
endif

all: mfg-test fbimage

mfg-test: $(sources_mfg) $(headers_mfg)
	$(CC) -DAPP_VERSION='"$(APP_VERSION)"' $(CFLAGS) $(LDFLAGS) $(DEBUG) -o $@ $(sources_mfg)

fbimage: $(sources_fbimage) $(headers_fbimage)
	$(CC) -DAPP_VERSION='"$(APP_VERSION)"' $(CFLAGS) $(LDFLAGS) $(DEBUG) -o $@ $(sources_fbimage)


clean:
	$(RM) mfg-test fbimage

.PHONY: all clean
