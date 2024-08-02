/* $XFree86$ */
/* $XdotOrg$ */
/*
 * Configurable compile-time options
 *
 * Copyright (C) 2001-2005 by Thomas Winischhofer, Vienna, Austria
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1) Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2) Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3) The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:   Thomas Winischhofer <thomas@winischhofer.net>
 *
 */

#undef XORG_NEW
#undef SISDUALHEAD
#undef SISMERGED
#undef SISXINERAMA
#undef SIS_USE_XAA
#undef SIS_USE_EXA
#undef DEBUG
#undef ACCELDEBUG
#undef _3DACCELDEBUG
#undef TWDEBUG
#undef XVDEBUG
#undef TWDEBUG_VID

/* Configurable stuff: ------------------------------------- */

#if 1
#define XORG_NEW		/* Toggle this if you get compile errors */
#endif

#if 0
#define SISDUALHEAD		/* Include Dual Head support  */
#endif

#if 0
#define SISMERGED		/* Include Merged-FB support */
#endif

#if 0
#ifdef SISMERGED
#define SISXINERAMA		/* Include SiS Pseudo-Xinerama for MergedFB mode */
#endif
#endif

#if 1
#ifdef HAVE_XAA_H
#define SIS_USE_XAA		/* Include support for XAA */
#endif
#endif

#if 1
#if defined(SIS_HAVE_EXA) || (defined(USE_EXA) && (USE_EXA != 0))
#define SIS_USE_EXA		/* Include support for EXA */
#endif
#endif

#if 0
#define SIS761MEMFIX		/* Does the 761 need the same special care as the 760? */
#endif				/* (apparently not) */

#if 0
#define SIS770MEMFIX		/* Does the 770 need the same special care as the 760? */
#endif				/* (Hopefully no) */

#if 1
#define SIS671MEMFIX		/* Fix wrong DRAM clock for SIS_671 */
#endif

#if 0
#define DEBUG			/* DEBUG */
#define ACCELDEBUG
#define _3DACCELDEBUG
#define TWDEBUG
#define XVDEBUG
#define TWDEBUG_VID
#endif

/* End of configurable stuff --------------------------------- */
