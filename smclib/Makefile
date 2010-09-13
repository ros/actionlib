all: installed

#include $(shell rospack find mk)/cmake.mk

TARBALL = build/smc_6_0_1.tgz
TARBALL_URL = http://downloads.sourceforge.net/project/smc/smc/6_0_1/smc_6_0_1.tgz
UNPACK_CMD = tar xzf
SOURCE_DIR = build/smc_6_0_1
MD5SUM_FILE = smc_6_0_1.tgz.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

LIBS = $(SOURCE_DIR)/lib

include/statemap.h: $(SOURCE_DIR)/unpacked
	mkdir -p include
	cp $(SOURCE_DIR)/lib/C++/statemap.h include/

src/statemap.py: $(SOURCE_DIR)/unpacked
	mkdir -p src
	cp $(SOURCE_DIR)/lib/Python/statemap.py src/

INSTALLED_FILES = include/statemap.h src/statemap.py
installed: $(INSTALLED_FILES)

$(INSTALL_DIR)/installed: $(SOURCE_DIR)/unpacked
	mkdir -p $(INSTALL_DIR)
	cp -r $(SOURCE_DIR)/Eigen $(INSTALL_DIR)
	touch $(INSTALL_DIR)/installed

clean:
	rm -rf $(SOURCE_DIR) $(INSTALLED_FILES)
	rm -rf include src

wipe: clean
	rm -rf build
