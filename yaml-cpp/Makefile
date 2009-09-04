all: installed

SVN_CMDLINE = svn
SVN_REVISION = -r175
SVN_DIR = yaml-cpp-svn
SVN_URL = http://yaml-cpp.googlecode.com/svn/trunk
SVN_PATCH = yaml_cmake.patch
INSTALL_DIR = yaml-cpp
CMAKE = cmake
CMAKE_ARGS = 	-DCMAKE_INSTALL_PREFIX=$(CURDIR)/$(INSTALL_DIR)/ \
		-DOROCOS_TARGET=gnulinux \
		-DBOOST_DIR=$(BOOST_INCLUDE)

include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched 
#	$(SVN_DIR) Makefile
#	@if [ ! -d yaml-cpp-svn ]; then $(SVN_CMDLINE) co -r $(SVN_REVISION) http://yaml-cpp.googlecode.com/svn/trunk  yaml-cpp-svn; fi;
#	@if [ ! -n "`svn info yaml-cpp | grep 'Revision: $(SVN_REVISION)'`" ]; then cd yaml-cpp-svn && $(SVN_CMDLINE) up -r $(SVN_REVISION); fi;
	mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make && make install
	touch installed
	
wiped: Makefile
	make wipe
	touch wipe

clean:
	rm -rf $(INSTALL_DIR)
	rm -rf $(SVN_DIR)
	rm -rf installed

wipe:   clean
	rm -rf $(SVN_DIR)/build
