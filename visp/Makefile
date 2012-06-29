all: visp

VERSION     = 2.6.2
TARBALL     = build/ViSP-$(VERSION).tar.gz
TARBALL_URL = \
https://gforge.inria.fr/frs/download.php/31248/ViSP-$(VERSION).tar.gz
UNPACK_CMD  = tar xfz
SOURCE_DIR  = build/ViSP-$(VERSION)
MD5SUM_FILE = ViSP-$(VERSION).tar.gz.md5sum

INSTALL_DIR = install

# Gdk include path is required for Fedora 15.
CMAKE_CXX_FLAGS=-DNDEBUG -march=native -O3 -g `pkg-config --cflags gtk+-2.0`

CMAKE_FLAGS = -DCMAKE_INSTALL_PREFIX=`rospack find visp`/$(INSTALL_DIR)/ \
	      -DCMAKE_CXX_FLAGS:STRING="$(CMAKE_CXX_FLAGS)"              \
	      -DBUILD_DEMOS=OFF -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
	      -DBUILD_SHARED_LIBS=ON -DBUILD_DEPRECATED=ON               \
	      -DUSE_DC1394_1=OFF -DUSE_FFMPEG=OFF                        \
	      -DUSE_DC1394_2=ON -DUSE_V4L2=ON -DUSE_OPENCV=ON            \
	      -DUSE_COIN_SIMULATOR=ON -DUSE_LAPACK=ON -DUSE_GSL=OFF      \
	      -DUSE_XML2=ON -DUSE_GTK2=ON -DUSE_LIBJPEG=ON               \
	      -DUSE_LIBPNG=ON -DUSE_LIBPLOTTER=ON                        \
	      -DDESIRED_QT_VERSION=4					 \
	      -DOpenCV_DIR=`rospack find opencv2`/opencv		 \
	      -DBOOST_ROOT=`rosboost-cfg --root`

include $(shell rospack find mk)/download_unpack_build.mk

visp: $(INSTALL_DIR)/installed

$(INSTALL_DIR)/installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR)	  \
	&& cmake . ${CMAKE_FLAGS} \
	&& make			  \
	&& make install
	touch $(INSTALL_DIR)/installed

clean:
	rm -rf $(SOURCE_DIR) $(INSTALL_DIR)

wipe: clean
	rm -rf build
