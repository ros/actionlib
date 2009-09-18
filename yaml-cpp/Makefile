all: yaml-cpp

TARBALL = build/yaml-cpp-0.2.2.tar.gz
TARBALL_URL = http://yaml-cpp.googlecode.com/files/yaml-cpp-0.2.2.tar.gz
SOURCE_DIR = build/yaml-cpp-0.2.2

UNPACK_CMD = mkdir -p yaml-cpp-0.2.2 && tar -C yaml-cpp-0.2.2 -xzf 


include $(shell rospack find mk)/download_unpack_build.mk



yaml-cpp: $(SOURCE_DIR)/unpacked
	cd build && cmake -DCMAKE_INSTALL_PREFIX=../yaml-cpp yaml-cpp-0.2.2
	cd build && make && make install

clean:
	rm -rf yaml-cpp $(SOURCE_DIR)

wipe: clean
	rm -rf build



