package = mfg-test
version = 1.0.0
tarname = $(package)
distdir = $(tarname)-$(version)

all clean mfg-test:
	cd src && $(MAKE) $@ APP_VERSION=$(version)

dist: $(distdir).tar.gz

$(distdir).tar.gz: $(distdir)
	tar chof - $(distdir) | gzip -9 -c > $@
	rm -rf $(distdir)

$(distdir): FORCE
	mkdir -p $(distdir)/src
	cp Makefile $(distdir)
	cp src/Makefile $(distdir)/src
	cp src/mfg_test.c $(distdir)/src

FORCE:
	-rm $(distdir).tar.gz > /dev/null 2>&1
	-rm -rf $(distdir) > /dev/null 2>&1
        
.PHONY: FORCE all clean dist
