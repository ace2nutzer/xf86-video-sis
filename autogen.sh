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

CFLAGS_ABI="-march=core2 -mcpu=core2 -mtune=core2"

if test -z "$NOCONFIGURE"; then
    exec "$srcdir"/configure \
		CFLAGS="$CFLAGS_ABI -O2 -fomit-frame-pointer -fno-strict-aliasing -Werror-implicit-function-declaration \
		-Wno-stringop-overflow -DNDEBUG -pipe" \
		--prefix=/usr "$@"
fi
