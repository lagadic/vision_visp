all: visp

TARBALL     = build/ViSP-2.6.0.tar.gz
TARBALL_URL = http://gforge.inria.fr/frs/download.php/27536/ViSP-2.6.0.tar.gz
UNPACK_CMD  = tar xfz
SOURCE_DIR  = build/ViSP-2.6.0
TARBALL_PATCH =
MD5SUM_FILE = ViSP-2.6.0.tar.gz.md5sum

CMAKE_FLAGS = -DCMAKE_INSTALL_PREFIX=`rospack find visp`/$(INSTALL_DIR)/ \
	      -DCMAKE_CXX_FLAGS="-DNDEBUG -march=native -O3 -g"         \
	      -DBUILD_DEMOS=ON -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON   \
	      -DBUILD_SHARED_LIBS=ON -DBUILD_DEPRECATED=ON              \
	      -DUSE_DC1394_1=OFF -DUSE_FFMPEG=ON                        \
	      -DUSE_DC1394_2=ON -DUSE_V4L2=ON -DUSE_OPENCV=ON           \
	      -DUSE_COIN_SIMULATOR=ON -DUSE_GSL=ON                      \
	      -DUSE_XML2=ON -DUSE_GTK2=ON -DUSE_LIBJPEG=ON              \
	      -DUSE_LIBPNG=ON -DUSE_LIBPLOTTER=ON                       \
	      -DDESIRED_QT_VERSION=4

include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = install

visp: $(INSTALL_DIR)/installed

$(INSTALL_DIR)/installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR)	\
	&& cmake ${CMAKE_FLAGS} \
	&& make			\
	&& make install
	touch $(INSTALL_DIR)/installed

clean:
	rm -rf $(SOURCE_DIR) $(INSTALL_DIR)

wipe: clean
	rm -rf build
