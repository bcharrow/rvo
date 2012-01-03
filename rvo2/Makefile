all: rvo2

TARBALL = build/rvo2-2.0.1.src.zip
TARBALL_URL = http://gamma.cs.unc.edu/RVO2/downloads/2.0/rvo2-2.0.1.src.zip
SOURCE_DIR = build/RVO2
include $(shell rospack find mk)/download_unpack_build.mk
UNPACK_CMD=unzip

rvo2: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && cmake -D CMAKE_INSTALL_PREFIX=$(SOURCE_DIR)/../.. src  && make install
	mkdir -p include  && mv build/include include/RVO
	mv build/lib lib
	touch rvo2

clean:
	rm -rf rvo2 $(SOURCE_DIR) include lib

wipe: clean
	rm -rf build
