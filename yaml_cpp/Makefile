all: yaml-cpp

TARBALL = build/yaml-cpp-0.2.5.tar.gz
TARBALL_URL = http://yaml-cpp.googlecode.com/files/yaml-cpp-0.2.5.tar.gz
SOURCE_DIR = build/yaml-cpp-0.2.5
MD5SUM_FILE = yaml-cpp-0.2.5.tar.gz.md5sum
UNPACK_CMD = tar -xzf 


include $(shell rospack find mk)/download_unpack_build.mk



yaml-cpp: wiped $(SOURCE_DIR)/unpacked
	cd build && cmake -DCMAKE_INSTALL_PREFIX=../yaml-cpp yaml-cpp-0.2.5
ifneq ($(MAKE),)
	cd build && $(MAKE) && $(MAKE) install
else
	cd build && make && make install
endif
	if [ `uname` = Darwin ]; then \
		install_name_tool -id `rospack find yaml_cpp`/yaml-cpp/lib/libyaml-cpp.0.2.dylib yaml-cpp/lib/libyaml-cpp.0.2.dylib; \
	fi

clean:
	rm -rf yaml-cpp $(SOURCE_DIR)
ifneq ($(MAKE),)
	-cd build && $(MAKE) clean
else
	-cd build && make clean
endif

wipe: clean
	rm -rf build

wiped: Makefile
ifneq ($(MAKE),)
	$(MAKE) -f Makefile wipe
else
	make -f Makefile wipe
endif
	touch wiped


