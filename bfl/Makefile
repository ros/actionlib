all: installed

SVN_REVISION= -r 30395
SVN_DIR = bfl-svn
SVN_URL = http://svn.mech.kuleuven.be/repos/orocos/trunk/bfl
SVN_PATCH = total.patch
INSTALL_DIR = bfl-boost
CMAKE = cmake
BOOST_INCLUDE =$(shell rosboost-cfg --include_dirs)
CMAKE_ARGS = -DBUILD_TESTS=OFF \
						 -DCMAKE_INSTALL_PREFIX=`rospack find bfl`/$(INSTALL_DIR)/ \
						 -DCMAKE_INCLUDE_PATH=$(BOOST_INCLUDE) \
						 -DMATRIX_LIB=boost -DRNG_LIB=boost

include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	mkdir -p $(SVN_DIR)/build-boost
	cd $(SVN_DIR)/build-boost && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build-boost && make $(ROS_PARALLEL_JOBS) && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf $(INSTALL_DIR)
	rm -rf $(SVN_DIR)/build
	rm -rf installed

wipe: 	clean
	rm -rf $(SVN_DIR)
