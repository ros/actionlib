include $(shell rospack find mk)/cmake.mk

all: installed

FILENAME = bfl-31655.tar.gz
TARBALL = build/$(FILENAME)
TARBALL_URL = http://pr.willowgarage.com/downloads/$(FILENAME)
SOURCE_DIR = build/bfl-tar
UNPACK_CMD = tar xzf
MD5SUM_FILE = $(FILENAME).md5sum
include $(shell rospack find mk)/download_unpack_build.mk

PATCH = total.patch
INSTALL_DIR = `rospack find bfl`
CMAKE = cmake
BOOST_INCLUDE =$(shell rosboost-cfg --include_dirs)

CMAKE_ARGS = -DBUILD_TESTS=OFF \
             -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR)/ \
	     -DCMAKE_INCLUDE_PATH=$(BOOST_INCLUDE) \
	     -DMATRIX_LIB=boost -DRNG_LIB=boost \
	     -DCMAKE_BUILD_TYPE="Release"

installed: wiped $(SOURCE_DIR)/unpacked 
	cd $(SOURCE_DIR) && patch -p0 < ../../$(PATCH)
	mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..

ifneq ($(MAKE),)
	cd $(SOURCE_DIR)/build && $(MAKE) $(ROS_PARALLEL_JOBS) && touch src/orocos-bfl.pc && make install
else
	cd $(SOURCE_DIR)/build && make $(ROS_PARALLEL_JOBS) && touch src/orocos-bfl.pc && make install
endif
	if [ `uname` = Darwin ]; then \
		install_name_tool -id `rospack find bfl`/lib/liborocos-bfl.dylib lib/liborocos-bfl.dylib; \
	fi
	touch installed


docs: 
	cd $(SOURCE_DIR)/build && make docs
	ln -s $(SOURCE_DIR)/build/doc doc

wiped: Makefile $(PATCH) $(MD5SUM_FILE)
ifneq ($(MAKE),)
	$(MAKE) wipe
else
	make wipe
endif
	touch wiped

clean:
	rm -rf bin
	rm -rf lib
	rm -rf include
	rm -rf share
	rm -rf build
	rm -rf installed

wipe: 	clean
	touch wiped
