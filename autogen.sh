#! /bin/sh

N=$(nproc)

if [ -f Makefile ] ; then
	make -j$N clean
	make -j$N distclean
fi

autoupdate

srcdir=`dirname "$0"`
test -z "$srcdir" && srcdir=.

ORIGDIR=`pwd`
cd "$srcdir"

autoreconf --force -v --install || exit 1
cd "$ORIGDIR" || exit $?

CPUFLAGS="-march=native -mcpu=native -mtune=native"

if test -z "$NOCONFIGURE"; then
    exec "$srcdir"/configure \
		CFLAGS="$CPUFLAGS -O2 -ftree-vectorize \
		-fvect-cost-model=dynamic -flto=auto -fomit-frame-pointer \
		-Werror-implicit-function-declaration \
		-Werror=incompatible-pointer-types \
		-Wno-unused-variable -Wno-unused-but-set-variable \
		-Wno-declaration-after-statement -Wno-redundant-decls \
		-DNDEBUG -pipe" \
		LDFLAGS="-flto=auto" \
		--prefix=/usr \
		"$@"
fi
