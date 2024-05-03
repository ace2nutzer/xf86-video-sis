#! /bin/sh

if [ -f Makefile ] ; then
	make -j2 clean
	make -j2 distclean
fi

srcdir=`dirname "$0"`
test -z "$srcdir" && srcdir=.

ORIGDIR=`pwd`
cd "$srcdir"

autoreconf -v --install || exit 1
cd "$ORIGDIR" || exit $?

CFLAGS_CPU="-march=core2 -mcpu=core2 -mtune=core2 -mhard-float -mfpmath=sse"

if test -z "$NOCONFIGURE"; then
    exec "$srcdir"/configure \
		CFLAGS="$CFLAGS_CPU -O2 -ftree-vectorize -fomit-frame-pointer -fno-strict-aliasing \
		-Werror-implicit-function-declaration -Wno-stringop-overflow -Wno-redundant-decls \
		-Wno-unused-variable -Wno-unused-function -Wno-unused-but-set-variable -DNDEBUG -pipe" \
		--prefix=/usr "$@"
fi
