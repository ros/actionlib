all: installed

FILENAME = bfl-31655.tar.gz
TARBALL = build/$(FILENAME)
TARBALL_URL = http://pr.willowgarage.com/downloads/$(FILENAME)
SOURCE_DIR = build/bfl-tar
UNPACK_CMD = tar xzf
MD5SUM_FILE = $(FILENAME).md5sum
include $(shell rospack find mk)/download_unpack_build.mk

TARBALL_PATCH = total.patch
INSTALL_DIR = bfl-boost
CMAKE = cmake
BOOST_INCLUDE =$(shell rosboost-cfg --include_dirs)
CMAKE_ARGS = -DBUILD_TESTS=OFF \
             -DCMAKE_INSTALL_PREFIX=`rospack find bfl`/$(INSTALL_DIR)/ \
	     -DCMAKE_INCLUDE_PATH=$(BOOST_INCLUDE) \
	     -DMATRIX_LIB=boost -DRNG_LIB=boost


installed: $(SOURCE_DIR)/unpacked patched
	mkdir -p $(SOURCE_DIR)/build-boost
	cd $(SOURCE_DIR)/build-boost && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SOURCE_DIR)/build-boost && make $(ROS_PARALLEL_JOBS) && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf $(INSTALL_DIR)
	rm -rf build
	rm -rf installed

wipe: 	clean

