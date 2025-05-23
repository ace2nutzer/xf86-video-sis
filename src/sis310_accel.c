/*
 * 2D Acceleration for SiS 315, 330 and 340 series
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
 *
 * Author:  	Thomas Winischhofer <thomas@winischhofer.net>
 *
 * 2003/08/18: Rewritten for using VRAM command queue
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "sis.h"
#define SIS_NEED_MYMMIO
#define SIS_NEED_ACCELBUF
#include "sis_regs.h"
#include "sis310_accel.h"

#define FBOFFSET 	(pSiS->dhmOffset)

#define DEV_HEIGHT	0xfff	/* "Device height of destination bitmap" */

#undef SIS_NEED_ARRAY

/* For XAA */

#ifdef SIS_USE_XAA

#undef TRAP		/* Use/Don't use Trapezoid Fills
			 * DOES NOT WORK. XAA sometimes provides illegal
			 * trapezoid data (left and right edges cross each
			 * other) which causes drawing errors. Since
			 * checking the trapezoid for such a case is very
			 * time-intensive, it is faster to let it be done
			 * by the generic polygon functions.
			 * Does not work on 330 series at all, hangs the engine.
			 * Even with correct trapezoids, this is slower than
			 * doing it by the CPU.
                         */

#undef CTSCE		/* Use/Don't use CPUToScreenColorExpand. Disabled
			 * because it is slower than doing it by the CPU.
			 * Indirect mode does not work in VRAM queue mode.
			 * Does not work on 330 series (even in MMIO mode).
			 */

#undef STSCE		/* Use/Don't use ScreenToScreenColorExpand - does not work,
			 * see comments below.
			 */

#define INCL_RENDER	/* Use/Don't use RENDER extension acceleration */

#ifdef INCL_RENDER
# ifdef RENDER
#  include "mipict.h"
#  include "dixstruct.h"
#  define SIS_NEED_ARRAY
#  define SISNEWRENDER
# endif
#endif

#endif /* XAA */


/* For EXA */
#ifdef SIS_USE_EXA
#define EXA_HAVE_UPLOAD_TO_SCRATCH	0
#if 0
#define SIS_HAVE_COMPOSITE		/* Have our own EXA composite */
#endif
#ifdef SIS_HAVE_COMPOSITE
#ifndef SIS_NEED_ARRAY
#define SIS_NEED_ARRAY
#endif
#endif
#endif

#ifdef SIS_USE_XAA		/* XAA */
#ifdef INCL_RENDER
#ifdef RENDER
static CARD32 SiSAlphaTextureFormats[2] = { PICT_a8      , 0 };
static CARD32 SiSTextureFormats[2]      = { PICT_a8r8g8b8, 0 };
static CARD32 SiS3DAlphaTextureFormats[2] = { PICT_a8r8g8b8 , 0 };
#ifdef SISNEWRENDER
static CARD32 SiSDstTextureFormats16[2] = { PICT_r5g6b5  , 0 };
static CARD32 SiSDstTextureFormats32[3] = { PICT_x8r8g8b8, PICT_a8r8g8b8, 0 };
#endif
#endif /* RENDER */
#endif /* INCL_RENDER */
#endif /* XAA */

#ifdef SIS_USE_EXA		/* EXA */
void SiSScratchSave(ScreenPtr pScreen, ExaOffscreenArea *area);
Bool SiSUploadToScratch(PixmapPtr pSrc, PixmapPtr pDst);
#endif /* EXA */

void SISWriteBlitPacket(SISPtr pSiS, CARD32 *packet);

extern unsigned char SiSGetCopyROP(int rop);
extern unsigned char SiSGetPatternROP(int rop);
int *CmdQueBusy;  /* flag to show command Quese state*/
int *_2DCmdFlushing;  /* flag to reflect 2D commands are being flushed */

static void SiSOccpyCmdQue(SISPtr pSiS){
	if(pSiS->DRIEnabled){
		*_2DCmdFlushing = TRUE; 
		while(*CmdQueBusy){xf86DrvMsg(0,X_INFO,"--garbage--");}
		*CmdQueBusy = TRUE;
	}

	return;
}

static void SiSReleaseCmdQue(SISPtr pSiS){
	if(pSiS->DRIEnabled)
		*CmdQueBusy = *_2DCmdFlushing = FALSE;
	return;
}

/* 3D-engine accel */
extern Bool
SiSSetupForCPUToScreenAlphaTexture3D (ScrnInfoPtr pScrn,
	int op, CARD16 red, CARD16 green,
	CARD16 blue, CARD16 alpha,
#ifdef SISNEWRENDER			
	CARD32 alphaType, CARD32 dstType,
#else			
	int alphaType,
#endif			
	CARD8 *alphaPtr,
	int alphaPitch, int width,
	int height, int	flags);

extern void SiSSubsequentCPUToScreenTexture3D(ScrnInfoPtr pScrn,
	int dst_x, int dst_y,
	int src_x, int src_y,
	int width, int height);

volatile CARD32 dummybuf;
	
#ifdef SIS_NEED_ARRAY
#if XF86_VERSION_CURRENT >= XF86_VERSION_NUMERIC(4,2,0,0,0)
#define SiSRenderOpsMAX 0x2b
#else
#define SiSRenderOpsMAX 0x0f
#endif
#if defined(RENDER) && defined(INCL_RENDER)
static const CARD8 SiSRenderOps[] = {	/* PictOpXXX 1 = supported, 0 = unsupported */
     1, 1, 1, 1,
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 0,
     1, 1, 1, 0,
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 0,
     1, 1, 1, 0,
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 0
};
#endif
#endif /* NEED ARRAY */

#ifdef SIS_NEED_ARRAY
static void
SiSCalcRenderAccelArray(ScrnInfoPtr pScrn)
{
	SISPtr  pSiS = SISPTR(pScrn);
#ifdef SISDUALHEAD
	SISEntPtr pSiSEnt = pSiS->entityPrivate;;
#endif

	if(((pScrn->bitsPerPixel == 16) || (pScrn->bitsPerPixel == 32)) && pSiS->doRender) {
	   int i, j;
#ifdef SISDUALHEAD
	   if(pSiSEnt) pSiS->RenderAccelArray = pSiSEnt->RenderAccelArray;
#endif
	   if(!pSiS->RenderAccelArray) {
	      if((pSiS->RenderAccelArray = xnfcalloc(1, 65536))) {
#ifdef SISDUALHEAD
	         if(pSiSEnt) pSiSEnt->RenderAccelArray = pSiS->RenderAccelArray;
#endif
		 for(i = 0; i < 256; i++) {
		    for(j = 0; j < 256; j++) {
		       pSiS->RenderAccelArray[(i << 8) + j] = (i * j) / 255;
		    }
		 }
	      }
	   }
	}
}
#endif

#ifdef SIS_USE_EXA
void
SiSScratchSave(ScreenPtr pScreen, ExaOffscreenArea *area)
{
	SISPtr pSiS = SISPTR(xf86ScreenToScrn(pScreen));

	pSiS->exa_scratch = NULL;
}
#endif

static void
SiSSync(ScrnInfoPtr pScrn)
{
	SISPtr pSiS = SISPTR(pScrn);

#ifdef SIS_USE_XAA
	if(!pSiS->useEXA) {
	   pSiS->DoColorExpand = FALSE;
	}
#endif

	pSiS->alphaBlitBusy = FALSE;

	SiSIdle;
}

static void
SiSSyncAccel(ScrnInfoPtr pScrn)
{
	SISPtr pSiS = SISPTR(pScrn);

	if(!pSiS->NoAccel) SiSSync(pScrn);
}

static void
SiSInitializeAccelerator(ScrnInfoPtr pScrn)
{
	SISPtr  pSiS = SISPTR(pScrn);

#ifdef SIS_USE_XAA
	pSiS->DoColorExpand = FALSE;
#endif
	pSiS->alphaBlitBusy = FALSE;

	if(!pSiS->NoAccel) {

	   if(pSiS->ChipFlags & SiSCF_DualPipe) {
	      SiSSync(pScrn);
	      SiSDualPipe(1);	/* 1 = disable, 0 = enable */
	      SiSSync(pScrn);
	   }

	}
}

static void
SiSSetupForScreenToScreenCopy(ScrnInfoPtr pScrn,
			int xdir, int ydir, int rop,
			unsigned int planemask, int trans_color)
{
	SISPtr  pSiS = SISPTR(pScrn);

	SiSOccpyCmdQue(pSiS);
#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, "XAA calling ScreenToScreenCopy\n");
#endif

	SiSSetupDSTColorDepth(pSiS->SiS310_AccelDepth);
	SiSCheckQueue(16 * 2);
	SiSSetupSRCPitchDSTRect(pSiS->scrnOffset, pSiS->scrnOffset, DEV_HEIGHT);

	if(trans_color != -1) {
	   SiSSetupROP(0x0A);
	   SiSSetupSRCTrans(trans_color);
	   SiSSetupCMDFlag(TRANSPARENT_BITBLT);
	} else {
	   SiSSetupROP(SiSGetCopyROP(rop));
	   /* Set command - not needed, both 0 */
	   /* SiSSetupCMDFlag(BITBLT | SRCVIDEO) */
	}

	SiSSyncWP;
	SiSReleaseCmdQue(pSiS);

	/* The chip is smart enough to know the direction */
}

static void
SiSSubsequentScreenToScreenCopy(ScrnInfoPtr pScrn,
			int src_x, int src_y, int dst_x, int dst_y,
			int width, int height)
{
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 srcbase, dstbase;
	int    mymin, mymax;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, "XAA calling SubsequentScreenToScreenCopy\n");
#endif

	srcbase = dstbase = 0;
	mymin = min(src_y, dst_y);
	mymax = max(src_y, dst_y);

	/* Libxaa.a has a bug: The tilecache cannot operate
	 * correctly if there are 512x512 slots, but no 256x256
	 * slots. This leads to catastrophic data fed to us.
	 * Filter this out here and warn the user.
	 * Fixed in 4.3.99.10 (?) and Debian's 4.3.0.1
	 */
#if (XF86_VERSION_CURRENT < XF86_VERSION_NUMERIC(4,3,99,10,0)) && (XF86_VERSION_CURRENT != XF86_VERSION_NUMERIC(4,3,0,1,0))
	if((src_x < 0)  ||
	   (dst_x < 0)  ||
	   (src_y < 0)  ||
	   (dst_y < 0)  ||
	   (width <= 0) ||
	   (height <= 0)) {
	   xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		"BitBlit fatal error: Illegal coordinates:\n");
	   xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
	        "Source x %d y %d, dest x %d y %d, width %d height %d\n",
			  src_x, src_y, dst_x, dst_y, width, height);
	   xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		"This is very probably caused by a known bug in libxaa.a.\n");
	   xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
		"Please update libxaa.a to avoid this error.\n");
	   return;
	}
#endif

	/* Although the chip knows the direction to use
	 * if the source and destination areas overlap,
	 * that logic fails if we fiddle with the bitmap
	 * addresses. Therefore, we check if the source
	 * and destination blitting areas overlap and
	 * adapt the bitmap addresses synchronously
	 * if the coordinates exceed the valid range.
	 * The the areas do not overlap, we do our
	 * normal check.
	 */
	if((mymax - mymin) < height) {
	   if((src_y >= 2048) || (dst_y >= 2048)) {
	      srcbase = pSiS->scrnOffset * mymin;
	      dstbase = pSiS->scrnOffset * mymin;
	      src_y -= mymin;
	      dst_y -= mymin;
	   }
	} else {
	   if(src_y >= 2048) {
	      srcbase = pSiS->scrnOffset * src_y;
	      src_y = 0;
	   }
	   if((dst_y >= pScrn->virtualY) || (dst_y >= 2048)) {
	      dstbase = pSiS->scrnOffset * dst_y;
	      dst_y = 0;
	   }
	}

	srcbase += FBOFFSET;
	dstbase += FBOFFSET;

	SiSOccpyCmdQue(pSiS);
	SiSCheckQueue(16 * 3);
	SiSSetupSRCDSTBase(srcbase, dstbase);
	SiSSetupSRCDSTXY(src_x, src_y, dst_x, dst_y);
	SiSSetRectDoCMD(width,height);
	SiSReleaseCmdQue(pSiS);
}

static void
SiSSetupForSolidFill(ScrnInfoPtr pScrn, int color,
			int rop, unsigned int planemask)
{
	SISPtr  pSiS = SISPTR(pScrn);

	if(pSiS->disablecolorkeycurrent || pSiS->nocolorkey) {
	   if((CARD32)color == pSiS->colorKey) {
	      rop = 5;  /* NOOP */
	   }
	}

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, "XAA calling SolidFill\n");
#endif

	SiSOccpyCmdQue(pSiS);
	SiSSetupDSTColorDepth(pSiS->SiS310_AccelDepth);
	SiSCheckQueue(16 * 1);
	SiSSetupPATFGDSTRect(color, pSiS->scrnOffset, DEV_HEIGHT);
	SiSSetupROP(SiSGetPatternROP(rop));
	SiSSetupCMDFlag(PATFG);
	SiSSyncWP;
	SiSReleaseCmdQue(pSiS);
}

static void
SiSSubsequentSolidFillRect(ScrnInfoPtr pScrn,
			int x, int y, int w, int h)
{
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 dstbase = 0;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, "XAA calling SubsequentSolidFillRect\n");
#endif

	if(y >= 2048) {
	   dstbase = pSiS->scrnOffset * y;
	   y = 0;
	}

	dstbase += FBOFFSET;

	pSiS->CommandReg &= ~(T_XISMAJORL | T_XISMAJORR |
	                      T_L_X_INC | T_L_Y_INC |
	                      T_R_X_INC | T_R_Y_INC |
			      TRAPAZOID_FILL);

	/* SiSSetupCMDFlag(BITBLT)  - BITBLT = 0 */

	SiSOccpyCmdQue(pSiS);
	SiSCheckQueue(16 * 2);
	SiSSetupDSTXYRect(x, y, w, h);
	SiSSetupDSTBaseDoCMD(dstbase);
	SiSReleaseCmdQue(pSiS);
}

#ifdef SIS_USE_XAA  /* ---------------------------- XAA -------------------------- */

static void
SiSSetupForSolidLine(ScrnInfoPtr pScrn, int color, int rop,
			unsigned int planemask)
{
	SISPtr pSiS = SISPTR(pScrn);

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, "XAA calling SolidLine\n");
#endif

	SiSOccpyCmdQue(pSiS);
	SiSSetupDSTColorDepth(pSiS->SiS310_AccelDepth);
	SiSCheckQueue(16 * 3);
	SiSSetupLineCountPeriod(1, 1);
	SiSSetupPATFGDSTRect(color, pSiS->scrnOffset, DEV_HEIGHT);
	SiSSetupROP(SiSGetPatternROP(rop));
	SiSSetupCMDFlag(PATFG | LINE);
	SiSSyncWP;
	SiSReleaseCmdQue(pSiS);
}

static void
SiSSubsequentSolidTwoPointLine(ScrnInfoPtr pScrn,
			int x1, int y1, int x2, int y2, int flags)
{
	SISPtr pSiS = SISPTR(pScrn);
	int    miny, maxy;
	CARD32 dstbase = 0;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, "XAA calling SubsequentSolidTwoPointLine\n");
#endif

	miny = (y1 > y2) ? y2 : y1;
	maxy = (y1 > y2) ? y1 : y2;
	if(maxy >= 2048) {
	   dstbase = pSiS->scrnOffset*miny;
	   y1 -= miny;
	   y2 -= miny;
	}

	dstbase += FBOFFSET;

	if(flags & OMIT_LAST) {
	   SiSSetupCMDFlag(NO_LAST_PIXEL);
	} else {
	   pSiS->CommandReg &= ~(NO_LAST_PIXEL);
	}

	SiSOccpyCmdQue(pSiS);
	SiSCheckQueue(16 * 2);
	SiSSetupX0Y0X1Y1(x1, y1, x2, y2);
	SiSSetupDSTBaseDoCMD(dstbase);
	SiSReleaseCmdQue(pSiS);
}

static void
SiSSubsequentSolidHorzVertLine(ScrnInfoPtr pScrn,
			int x, int y, int len, int dir)
{
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 dstbase = 0;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, " XAA calling SubsequentSolidTwoPointLine\n");
#endif

	len--; /* starting point is included! */

	if((y >= 2048) || ((y + len) >= 2048)) {
	   dstbase = pSiS->scrnOffset * y;
	   y = 0;
	}

	dstbase += FBOFFSET;

	SiSOccpyCmdQue(pSiS);
	SiSCheckQueue(16 * 2);
	if(dir == DEGREES_0) {
	   SiSSetupX0Y0X1Y1(x, y, (x + len), y);
	} else {
	   SiSSetupX0Y0X1Y1(x, y, x, (y + len));
	}
	SiSSetupDSTBaseDoCMD(dstbase);
	SiSReleaseCmdQue(pSiS);
}

static void
SiSSetupForDashedLine(ScrnInfoPtr pScrn,
			int fg, int bg, int rop, unsigned int planemask,
			int length, unsigned char *pattern)
{
	SISPtr pSiS = SISPTR(pScrn);

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, " XAA calling SetupForDashedLine\n");
#endif

	SiSOccpyCmdQue(pSiS);
	SiSSetupDSTColorDepth(pSiS->SiS310_AccelDepth);
	SiSCheckQueue(16 * 3);
	SiSSetupLineCountPeriod(1, (length - 1));
	SiSSetupStyle(*pattern,*(pattern + 4));
	SiSSetupPATFGDSTRect(fg, pSiS->scrnOffset, DEV_HEIGHT);
	SiSSetupROP(SiSGetPatternROP(rop));

	SiSSetupCMDFlag(LINE | LINE_STYLE);

	if(bg != -1) {
	   SiSSetupPATBG(bg);
	} else {
	   SiSSetupCMDFlag(TRANSPARENT);
	}
        SiSSyncWP;
	SiSReleaseCmdQue(pSiS);
}

static void
SiSSubsequentDashedTwoPointLine(ScrnInfoPtr pScrn,
			int x1, int y1, int x2, int y2,
			int flags, int phase)
{
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 dstbase, miny, maxy;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, " XAA calling SubsequentDashedTwoPointLine\n");
#endif

	dstbase = 0;
	miny = (y1 > y2) ? y2 : y1;
	maxy = (y1 > y2) ? y1 : y2;
	if(maxy >= 2048) {
	   dstbase = pSiS->scrnOffset * miny;
	   y1 -= miny;
	   y2 -= miny;
	}

	dstbase += FBOFFSET;

	if(flags & OMIT_LAST) {
	   SiSSetupCMDFlag(NO_LAST_PIXEL)
	} else {
	   pSiS->CommandReg &= ~(NO_LAST_PIXEL);
	}

	SiSOccpyCmdQue(pSiS);
	SiSCheckQueue(16 * 2);
	SiSSetupX0Y0X1Y1(x1, y1, x2, y2);
	SiSSetupDSTBaseDoCMD(dstbase);
	SiSReleaseCmdQue(pSiS);
}

static void
SiSSetupForMonoPatternFill(ScrnInfoPtr pScrn,
			int patx, int paty, int fg, int bg,
			int rop, unsigned int planemask)
{
	SISPtr pSiS = SISPTR(pScrn);

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, " XAA calling SetupForMonoPatternFill\n");
#endif

	SiSOccpyCmdQue(pSiS);
	SiSSetupDSTColorDepth(pSiS->SiS310_AccelDepth);
	SiSCheckQueue(16 * 3);
	SiSSetupPATFGDSTRect(fg, pSiS->scrnOffset, DEV_HEIGHT);
	SiSSetupMONOPAT(patx,paty);
	SiSSetupROP(SiSGetPatternROP(rop));
	SiSSetupCMDFlag(PATMONO);

	if(bg != -1) {
	   SiSSetupPATBG(bg);
	} else {
	   SiSSetupCMDFlag(TRANSPARENT);
	}

	SiSSyncWP;
	SiSReleaseCmdQue(pSiS);

}

static void
SiSSubsequentMonoPatternFill(ScrnInfoPtr pScrn,
			int patx, int paty,
			int x, int y, int w, int h)
{
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 dstbase = 0;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, " XAA calling SubsequentMonoPatternFill\n");

#endif

	if(y >= 2048) {
	   dstbase = pSiS->scrnOffset * y;
	   y = 0;
	}

	dstbase += FBOFFSET;

	/* Clear commandReg because Setup can be used for Rect and Trap */
	pSiS->CommandReg &= ~(T_XISMAJORL | T_XISMAJORR |
			      T_L_X_INC | T_L_Y_INC |
			      T_R_X_INC | T_R_Y_INC |
			      TRAPAZOID_FILL);
	
	SiSOccpyCmdQue(pSiS);
	SiSCheckQueue(16 * 2);
	SiSSetupDSTXYRect(x,y,w,h);
	SiSSetupDSTBaseDoCMD(dstbase);
	SiSReleaseCmdQue(pSiS);

}

static void
SiSSetupForColor8x8PatternFill(ScrnInfoPtr pScrn, int patternx, int patterny,
			int rop, unsigned int planemask, int trans_col)
{
	SISPtr pSiS = SISPTR(pScrn);
	int j = pSiS->CurrentLayout.bytesPerPixel;
	CARD32 *patadr = (CARD32 *)(pSiS->FbBase + (patterny * pSiS->scrnOffset) +
				(patternx * j));


#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, " XAA calling ForColor8x8PatternFill\n");
#endif

	SiSOccpyCmdQue(pSiS);
	SiSSetupDSTColorDepth(pSiS->SiS310_AccelDepth);
	SiSCheckQueue(16 * 3);

	SiSSetupDSTRectBurstHeader(pSiS->scrnOffset, DEV_HEIGHT,
			PATTERN_REG, (pSiS->CurrentLayout.bitsPerPixel << 1))

	while(j--) {
	   SiSSetupPatternRegBurst(patadr[0],  patadr[1],  patadr[2],  patadr[3]);
	   SiSSetupPatternRegBurst(patadr[4],  patadr[5],  patadr[6],  patadr[7]);
	   SiSSetupPatternRegBurst(patadr[8],  patadr[9],  patadr[10], patadr[11]);
	   SiSSetupPatternRegBurst(patadr[12], patadr[13], patadr[14], patadr[15]);
	   patadr += 16;  /* = 64 due to (CARD32 *) */
	}

	SiSSetupROP(SiSGetPatternROP(rop))

	SiSSetupCMDFlag(PATPATREG)

	SiSSyncWP

	SiSReleaseCmdQue(pSiS);
}

static void
SiSSubsequentColor8x8PatternFillRect(ScrnInfoPtr pScrn, int patternx,
			int patterny, int x, int y, int w, int h)
{
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 dstbase = 0;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, " XAA calling SubsequentColor8x8PatternFillRect\n");
#endif



	if(y >= 2048) {
	   dstbase = pSiS->scrnOffset * y;
	   y = 0;
	}

	dstbase += FBOFFSET;

	/* SiSSetupCMDFlag(BITBLT)  - BITBLT = 0 */

	SiSOccpyCmdQue(pSiS);

	SiSCheckQueue(16 * 2)
	SiSSetupDSTXYRect(x, y, w, h)
	SiSSetupDSTBaseDoCMD(dstbase)

	SiSReleaseCmdQue(pSiS);
}

#ifdef SISDUALHEAD
static void
SiSRestoreAccelState(ScrnInfoPtr pScrn)
{
	SISPtr pSiS = SISPTR(pScrn);

	pSiS->ColorExpandBusy = FALSE;
	pSiS->alphaBlitBusy = FALSE;
	SiSIdle
}
#endif

/* ---- RENDER ---- */

#if defined(RENDER) && defined(INCL_RENDER)
static void
SiSRenderCallback(ScrnInfoPtr pScrn)
{
	SISPtr pSiS = SISPTR(pScrn);

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, " XAA calling SiSRenderCallback\n");
#endif

	if((currentTime.milliseconds > pSiS->RenderTime) && pSiS->AccelLinearScratch) {
	   xf86FreeOffscreenLinear(pSiS->AccelLinearScratch);
	   pSiS->AccelLinearScratch = NULL;
	}

	if(!pSiS->AccelLinearScratch) {
	   pSiS->RenderCallback = NULL;
	}
}

#define RENDER_DELAY 15000


/* for extern referred, we remove the  type "static"*/
Bool
SiSAllocateLinear(ScrnInfoPtr pScrn, int sizeNeeded)
{
	SISPtr pSiS = SISPTR(pScrn);

	pSiS->RenderTime = currentTime.milliseconds + RENDER_DELAY;
	pSiS->RenderCallback = SiSRenderCallback;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, " XAA calling SiSAllocateLinear\n");
#endif

	if(pSiS->AccelLinearScratch) {
	   if(pSiS->AccelLinearScratch->size >= sizeNeeded) {
	      return TRUE;
	   } else {
	      if(pSiS->alphaBlitBusy) {
	         pSiS->alphaBlitBusy = FALSE;
	         SiSIdle
	      }
	      if(xf86ResizeOffscreenLinear(pSiS->AccelLinearScratch, sizeNeeded)) {
		 return TRUE;
	      }
	      xf86FreeOffscreenLinear(pSiS->AccelLinearScratch);
	      pSiS->AccelLinearScratch = NULL;
	   }
	}

	pSiS->AccelLinearScratch = xf86AllocateOffscreenLinear(
				 	pScrn->pScreen, sizeNeeded, 32,
				 	NULL, NULL, NULL);

	return(pSiS->AccelLinearScratch != NULL);
}

static Bool
SiSSetupForCPUToScreenAlphaTexture(ScrnInfoPtr pScrn,
			int op, CARD16 red, CARD16 green,
			CARD16 blue, CARD16 alpha,
#ifdef SISNEWRENDER
			CARD32 alphaType, CARD32 dstType,
#else
			int alphaType,
#endif
			CARD8 *alphaPtr,
			int alphaPitch, int width,
			int height, int	flags)
{
	SISPtr pSiS = SISPTR(pScrn);
	unsigned char *renderaccelarray;
	CARD32 *dstPtr;
	int    x, pitch, sizeNeeded;
	int    sbpp = pSiS->CurrentLayout.bytesPerPixel;
	int    sbppshift = sbpp >> 1;	/* 8->0, 16->1, 32->2 */
	CARD8  myalpha;
	Bool   docopy = TRUE;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, "AT(1): op %d t %x ARGB %x %x %x %x, w %d h %d pch %d\n",
		op, alphaType, /*dstType, */alpha, red, green, blue, width, height, alphaPitch);
#endif

	if((width > 2048) || (height > 2048)) return FALSE;

	if(op > SiSRenderOpsMAX) return FALSE;
	if(!SiSRenderOps[op])    return FALSE;

	if(!((renderaccelarray = pSiS->RenderAccelArray)))
	   return FALSE;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, "AT(2): op %d t %x ARGB %x %x %x %x, w %d h %d pch %d\n",
		op, alphaType, alpha, red, green, blue, width, height, alphaPitch);
#endif

	pitch = (width + 31) & ~31;
	sizeNeeded = (pitch << 2) * height; /* Source a8 (=8bit), expand to A8R8G8B8 (=32bit) */

	if(!SiSAllocateLinear(pScrn, (sizeNeeded + sbpp - 1) >> sbppshift))
	   return FALSE;

	red &= 0xff00;
	green &= 0xff00;
	blue &= 0xff00;

	SiSOccpyCmdQue(pSiS);

	SiSSetupDSTColorDepth(pSiS->SiS310_AccelDepth);
	switch(op) {
	case PictOpClear:
#if XF86_VERSION_CURRENT >= XF86_VERSION_NUMERIC(4,2,0,0,0)
	case PictOpDisjointClear:
	case PictOpConjointClear:
#endif
	   SiSSetupPATFGDSTRect(0, pSiS->scrnOffset, DEV_HEIGHT)
	   /* SiSSetupROP(0x00) - is already 0 */
	   SiSSetupCMDFlag(PATFG)
	   docopy = FALSE;
	   break;
	case PictOpSrc:
#if XF86_VERSION_CURRENT >= XF86_VERSION_NUMERIC(4,2,0,0,0)
	case PictOpDisjointSrc:
	case PictOpConjointSrc:
#endif
	   SiSSetupSRCPitchDSTRect((pitch << 2), pSiS->scrnOffset, DEV_HEIGHT);
	   SiSSetupAlpha(0xff)
	   SiSSetupCMDFlag(ALPHA_BLEND | SRCVIDEO | A_NODESTALPHA)
	   break;
	case PictOpDst:
#if XF86_VERSION_CURRENT >= XF86_VERSION_NUMERIC(4,2,0,0,0)
	case PictOpDisjointDst:
	case PictOpConjointDst:
#endif
	   SiSSetupSRCPitchDSTRect((pitch << 2), pSiS->scrnOffset, DEV_HEIGHT);
	   SiSSetupAlpha(0x00)
	   SiSSetupCMDFlag(ALPHA_BLEND | SRCVIDEO | A_CONSTANTALPHA)
	   docopy = FALSE;
	   break;
	case PictOpOver:
	   SiSSetupSRCPitchDSTRect((pitch << 2), pSiS->scrnOffset, DEV_HEIGHT);
	   SiSSetupCMDFlag(ALPHA_BLEND | SRCVIDEO | A_PERPIXELALPHA)
	   break;
	}
        SiSSyncWP
	SiSReleaseCmdQue(pSiS);

	/* Don't need source for clear and dest */
	if(!docopy) return TRUE;

	dstPtr = (CARD32*)(pSiS->FbBase + (pSiS->AccelLinearScratch->offset << sbppshift));

	if(pSiS->alphaBlitBusy) {
	   pSiS->alphaBlitBusy = FALSE;
	   SiSIdle
	}

	if(alpha == 0xffff) {

	   while(height--) {
	      for(x = 0; x < width; x++) {
	         myalpha = alphaPtr[x];
	         dstPtr[x] = (renderaccelarray[red + myalpha] << 16)  |
			     (renderaccelarray[green + myalpha] << 8) |
			     renderaccelarray[blue + myalpha]         |
			     myalpha << 24;
	      }
	      dstPtr += pitch;
	      alphaPtr += alphaPitch;
	   }

	} else {

	   alpha &= 0xff00;

	   while(height--) {
	      for(x = 0; x < width; x++) {
	         myalpha = alphaPtr[x];
	         dstPtr[x] = (renderaccelarray[alpha + myalpha] << 24) |
			     (renderaccelarray[red + myalpha] << 16)   |
			     (renderaccelarray[green + myalpha] << 8)  |
			     renderaccelarray[blue + myalpha];
	      }
	      dstPtr += pitch;
	      alphaPtr += alphaPitch;
	   }

	}

	return TRUE;
}

static Bool
SiSSetupForCPUToScreenTexture(ScrnInfoPtr pScrn,
			int op,
#ifdef SISNEWRENDER
			CARD32 texType, CARD32 dstType,
#else
			int texType,
#endif
			CARD8 *texPtr,
			int texPitch, int width,
			int height, int	flags)
{
	SISPtr  pSiS = SISPTR(pScrn);
	CARD8   *dst;
	int     pitch, sizeNeeded;
	int     sbpp = pSiS->CurrentLayout.bytesPerPixel;
	int     sbppshift = sbpp >> 1;	          	  /* 8->0, 16->1, 32->2 */
	int     bppshift = PICT_FORMAT_BPP(texType) >> 4; /* 8->0, 16->1, 32->2 */
	Bool    docopy = TRUE;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, "T: type %x op %d w %d h %d T-pitch %d\n",
		texType, op, width, height, texPitch);
#endif

	SiSOccpyCmdQue(pSiS);
	if(op > SiSRenderOpsMAX) return FALSE;
	if(!SiSRenderOps[op])    return FALSE;
	if((width > 2048) || (height > 2048)) return FALSE;

	pitch = (width + 31) & ~31;
	sizeNeeded = (pitch << bppshift) * height;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, "T: %x op %x w %d h %d T-pitch %d size %d (%d %d %d)\n",
		texType, op, width, height, texPitch, sizeNeeded, sbpp, sbppshift, bppshift);
#endif

	if(!SiSAllocateLinear(pScrn, (sizeNeeded + sbpp - 1) >> sbppshift))
	   return FALSE;

	width <<= bppshift;  /* -> bytes (for engine and memcpy) */
	pitch <<= bppshift;  /* -> bytes */

	SiSSetupDSTColorDepth(pSiS->SiS310_AccelDepth);
	switch(op) {
	case PictOpClear:
#if XF86_VERSION_CURRENT >= XF86_VERSION_NUMERIC(4,2,0,0,0)
	case PictOpDisjointClear:
	case PictOpConjointClear:
#endif
	   SiSSetupPATFGDSTRect(0, pSiS->scrnOffset, DEV_HEIGHT);
	   /* SiSSetupROP(0x00) - is already zero */
	   SiSSetupCMDFlag(PATFG);
	   docopy = FALSE;
	   break;
	case PictOpSrc:
#if XF86_VERSION_CURRENT >= XF86_VERSION_NUMERIC(4,2,0,0,0)
	case PictOpDisjointSrc:
	case PictOpConjointSrc:
#endif
	   SiSSetupSRCPitchDSTRect(pitch, pSiS->scrnOffset, DEV_HEIGHT);
	   SiSSetupAlpha(0xff)
	   SiSSetupCMDFlag(ALPHA_BLEND | SRCVIDEO | A_NODESTALPHA)
	   break;
	case PictOpDst:
#if XF86_VERSION_CURRENT >= XF86_VERSION_NUMERIC(4,2,0,0,0)
	case PictOpDisjointDst:
	case PictOpConjointDst:
#endif
	   SiSSetupSRCPitchDSTRect(pitch, pSiS->scrnOffset, DEV_HEIGHT);
	   SiSSetupAlpha(0x00)
	   SiSSetupCMDFlag(ALPHA_BLEND | SRCVIDEO | A_CONSTANTALPHA)
	   docopy = FALSE;
	   break;
	case PictOpOver:
	   SiSSetupSRCPitchDSTRect(pitch, pSiS->scrnOffset, DEV_HEIGHT);
	   SiSSetupAlpha(0x00)
	   SiSSetupCMDFlag(ALPHA_BLEND | SRCVIDEO | A_PERPIXELALPHA)
	   break;
	default:
	   return FALSE;
 	}
        SiSSyncWP
	SiSReleaseCmdQue(pSiS);
	/* Don't need source for clear and dest */
	if(!docopy) return TRUE;

	dst = (CARD8*)(pSiS->FbBase + (pSiS->AccelLinearScratch->offset << sbppshift));

	if(pSiS->alphaBlitBusy) {
	   pSiS->alphaBlitBusy = FALSE;
	   SiSIdle
	}

	while(height--) {
	   memcpy(dst, texPtr, width);
	   texPtr += texPitch;
	   dst += pitch;
	}

	return TRUE;
}

static void
SiSSubsequentCPUToScreenTexture(ScrnInfoPtr pScrn,
			int dst_x, int dst_y,
			int src_x, int src_y,
			int width, int height)
{
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 srcbase, dstbase;

	srcbase = pSiS->AccelLinearScratch->offset << 1;
	if(pScrn->bitsPerPixel == 32) srcbase <<= 1;

#ifdef ACCELDEBUG
	xf86DrvMsg(0, X_INFO, "FIRE: scrbase %x dx %d dy %d w %d h %d\n",
		srcbase, dst_x, dst_y, width, height);
#endif

	dstbase = 0;
	if((dst_y >= pScrn->virtualY) || (dst_y >= 2048)) {
	   dstbase = pSiS->scrnOffset * dst_y;
	   dst_y = 0;
	}

	srcbase += FBOFFSET;
	dstbase += FBOFFSET;

	SiSOccpyCmdQue(pSiS);
	SiSCheckQueue(16 * 3);
	if(pSiS->ChipType == SIS_770)
		SiSSetupSafeReg(0x26a90000);
	SiSSetupSRCDSTBase(srcbase,dstbase);
	SiSSetupSRCDSTXY(src_x, src_y, dst_x, dst_y);
	SiSSetRectDoCMD(width,height);
	SiSReleaseCmdQue(pSiS);

	pSiS->alphaBlitBusy = TRUE;
}
#endif  /* RENDER && INCL_RENDER */

#endif /* XAA */


#ifdef SIS_USE_EXA  /* ---------------------------- EXA -------------------------- */

static void
SiSEXASync(ScreenPtr pScreen, int marker)
{
	SISPtr pSiS = SISPTR(xf86ScreenToScrn(pScreen));
	SiSIdle;
}

static Bool
SiSPrepareSolid(PixmapPtr pPixmap, int alu, Pixel planemask, Pixel fg)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);

	/* Planemask not supported */
	if((planemask & ((1 << pPixmap->drawable.depth) - 1)) !=
				(1 << pPixmap->drawable.depth) - 1) {
	   return FALSE;
	}

	if((pPixmap->drawable.bitsPerPixel != 8) &&
	   (pPixmap->drawable.bitsPerPixel != 16) &&
	   (pPixmap->drawable.bitsPerPixel != 32))
	   return FALSE;

	if(pSiS->disablecolorkeycurrent) {
	   if((CARD32)fg == pSiS->colorKey) {
	      alu = 5;  /* NOOP */
	   }
	}

	/* Check that the pitch matches the hardware's requirements. Should
	 * never be a problem due to pixmapPitchAlign and fbScreenInit.
	 */
	if(exaGetPixmapPitch(pPixmap) & 3)
	   return FALSE;

	SiSSetupDSTColorDepth((pPixmap->drawable.bitsPerPixel >> 4) << 16);
	SiSCheckQueue(16 * 1);
	SiSSetupPATFGDSTRect(fg, exaGetPixmapPitch(pPixmap), DEV_HEIGHT)
	SiSSetupROP(SiSGetPatternROP(alu))
	SiSSetupCMDFlag(PATFG)
	SiSSyncWP

	pSiS->fillDstBase = (CARD32)exaGetPixmapOffset(pPixmap) + FBOFFSET;

	return TRUE;
}

static void
SiSSolid(PixmapPtr pPixmap, int x1, int y1, int x2, int y2)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);

	/* SiSSetupCMDFlag(BITBLT)  - BITBLT = 0 */

	SiSCheckQueue(16 * 2);
	SiSSetupDSTXYRect(x1, y1, x2-x1, y2-y1);
	SiSSetupDSTBaseDoCMD(pSiS->fillDstBase);
}

static void
SiSDoneSolid(PixmapPtr pPixmap)
{
}

static Bool
SiSPrepareCopy(PixmapPtr pSrcPixmap, PixmapPtr pDstPixmap, int xdir, int ydir,
					int alu, Pixel planemask)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pDstPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);
	CARD32 srcbase, dstbase;

	/* Planemask not supported */
	if((planemask & ((1 << pSrcPixmap->drawable.depth) - 1)) !=
				(1 << pSrcPixmap->drawable.depth) - 1) {
	   return FALSE;
	}

	if((pDstPixmap->drawable.bitsPerPixel != 8) &&
	   (pDstPixmap->drawable.bitsPerPixel != 16) &&
	   (pDstPixmap->drawable.bitsPerPixel != 32))
	   return FALSE;

	/* Check that the pitch matches the hardware's requirements. Should
	 * never be a problem due to pixmapPitchAlign and fbScreenInit.
	 */
	if(exaGetPixmapPitch(pSrcPixmap) & 3)
	   return FALSE;
	if(exaGetPixmapPitch(pDstPixmap) & 3)
	   return FALSE;

	srcbase = (CARD32)exaGetPixmapOffset(pSrcPixmap) + FBOFFSET;

	dstbase = (CARD32)exaGetPixmapOffset(pDstPixmap) + FBOFFSET;

	/* TODO: Will there eventually be overlapping blits?
	 * If so, good night. Then we must calculate new base addresses
	 * which are identical for source and dest, otherwise
	 * the chips direction-logic will fail. Certainly funny
	 * to re-calculate x and y then...
	 */

	SiSSetupDSTColorDepth((pDstPixmap->drawable.bitsPerPixel >> 4) << 16);
	SiSCheckQueue(16 * 3);
	SiSSetupSRCPitchDSTRect(exaGetPixmapPitch(pSrcPixmap),
					exaGetPixmapPitch(pDstPixmap), DEV_HEIGHT);
	SiSSetupROP(SiSGetCopyROP(alu));
	SiSSetupSRCDSTBase(srcbase, dstbase);
	SiSSyncWP;

	return TRUE;
}

static void
SiSCopy(PixmapPtr pDstPixmap, int srcX, int srcY, int dstX, int dstY, int width, int height)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pDstPixmap->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);

	SiSCheckQueue(16 * 2);
	SiSSetupSRCDSTXY(srcX, srcY, dstX, dstY);
	SiSSetRectDoCMD(width, height);
}

static void
SiSDoneCopy(PixmapPtr pDstPixmap)
{
}

#ifdef SIS_HAVE_COMPOSITE
static Bool
SiSCheckComposite(int op, PicturePtr pSrcPicture, PicturePtr pMaskPicture,
				PicturePtr pDstPicture)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pDstPicture->pDrawable->pScreen);
	SISPtr pSiS = SISPTR(pScrn);

#ifdef ACCELDEBUG
	xf86DrvMsg(0, 0, "CC: %d Src %x (fi %d ca %d) Msk %x (%d %d) Dst %x (%d %d)\n",
		op, pSrcPicture->format, pSrcPicture->filter, pSrcPicture->componentAlpha,
		pMaskPicture ? pMaskPicture->format : 0x2011, pMaskPicture ? pMaskPicture->filter : -1,
			pMaskPicture ? pMaskPicture->componentAlpha : -1,
		pDstPicture->format, pDstPicture->filter, pDstPicture->componentAlpha);

	if(pSrcPicture->transform || (pMaskPicture && pMaskPicture->transform) || pDstPicture->transform) {
		xf86DrvMsg(0, 0, "CC: src tr %p msk %p dst %p  !!!!!!!!!!!!!!!\n",
			pSrcPicture->transform,
			pMaskPicture ? pMaskPicture->transform : 0,
			pDstPicture->transform);
        }
#endif

	return FALSE;
}

static Bool
SiSPrepareComposite(int op, PicturePtr pSrcPicture, PicturePtr pMaskPicture,
				PicturePtr pDstPicture, PixmapPtr pSrc, PixmapPtr pMask, PixmapPtr pDst)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pDst->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);

	return FALSE;
}

static void
SiSComposite(PixmapPtr pDst, int srcX, int srcY, int maskX, int maskY, int dstX, int dstY,
				int width, int height)
{
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pDst->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);
}

static void
SiSDoneComposite(PixmapPtr pDst)
{
}
#endif

Bool
SiSUploadToScratch(PixmapPtr pSrc, PixmapPtr pDst)
{
#if EXA_HAVE_UPLOAD_TO_SCRATCH
	ScrnInfoPtr pScrn = xf86ScreenToScrn(pSrc->drawable.pScreen);
	SISPtr pSiS = SISPTR(pScrn);
	unsigned char *src, *dst;
	int src_pitch = exaGetPixmapPitch(pSrc);
	int dst_pitch, size, w, h;

	w = pSrc->drawable.width;

	dst_pitch = ((w * (pSrc->drawable.bitsPerPixel >> 3)) +
		     pSiS->EXADriverPtr->pixmapPitchAlign - 1) &
		    ~(pSiS->EXADriverPtr->pixmapPitchAlign - 1);

	size = dst_pitch * pSrc->drawable.height;

	if(size > pSiS->exa_scratch->size)
	   return FALSE;

	pSiS->exa_scratch_next = (pSiS->exa_scratch_next +
				  pSiS->EXADriverPtr->pixmapOffsetAlign - 1) &
				  ~(pSiS->EXADriverPtr->pixmapOffsetAlign - 1);

	if(pSiS->exa_scratch_next + size >
	   pSiS->exa_scratch->offset + pSiS->exa_scratch->size) {
	   (pSiS->EXADriverPtr->WaitMarker)(pSrc->drawable.pScreen, 0);
	   pSiS->exa_scratch_next = pSiS->exa_scratch->offset;
	}

	memcpy(pDst, pSrc, sizeof(*pDst));
	pDst->devKind = dst_pitch;
	pDst->devPrivate.ptr = pSiS->EXADriverPtr->memoryBase + pSiS->exa_scratch_next;

	pSiS->exa_scratch_next += size;

	src = pSrc->devPrivate.ptr;
	src_pitch = exaGetPixmapPitch(pSrc);
	dst = pDst->devPrivate.ptr;

	h = pSrc->drawable.height;

	(pSiS->SyncAccel)(pScrn);

	while(h--) {
	   SiSMemCopyToVideoRam(pSiS, dst, src, size);
	   src += src_pitch;
	   dst += dst_pitch;
	}

	return TRUE;
#else
	return FALSE;
#endif
}
#endif /* EXA */

/* Helper for xv video blitter and rotation */

void
SISWriteBlitPacket(SISPtr pSiS, CARD32 *packet)
{
	SiSWritePacketPart(packet[0], packet[1], packet[2], packet[3]);
	SiSWritePacketPart(packet[4], packet[5], packet[6], packet[7]);
	SiSWritePacketPart(packet[8], packet[9], packet[10], packet[11]);
	SiSWritePacketPart(packet[12], packet[13], packet[14], packet[15]);
	SiSWritePacketPart(packet[16], packet[17], packet[18], packet[19]);
	SiSSyncWP;
}


/* For DGA usage */

static void
SiSDGAFillRect(ScrnInfoPtr pScrn, int x, int y, int w, int h, int color)
{
	SiSSetupForSolidFill(pScrn, color, GXcopy, ~0);
	SiSSubsequentSolidFillRect(pScrn, x, y, w, h);
}

static void
SiSDGABlitRect(ScrnInfoPtr pScrn, int srcx, int srcy, int dstx, int dsty, int w, int h, int color)
{
	/* Don't need xdir, ydir */
	SiSSetupForScreenToScreenCopy(pScrn, 0, 0, GXcopy, (CARD32)~0, color);
	SiSSubsequentScreenToScreenCopy(pScrn, srcx, srcy, dstx, dsty, w, h);
}

/* Initialisation */

Bool
SiS315AccelInit(ScreenPtr pScreen)
{
	ScrnInfoPtr     pScrn = xf86ScreenToScrn(pScreen);
	SISPtr          pSiS = SISPTR(pScrn);
#ifdef SIS_USE_XAA
	XAAInfoRecPtr   infoPtr = NULL;
	int		topFB, reservedFbSize, usableFbSize;
	BoxRec          Avail;
#endif /* XAA */

	pSiS->ColorExpandBufferNumber = 0;
	pSiS->PerColorExpandBufferSize = 0;
	pSiS->RenderAccelArray = NULL;
#ifdef SIS_USE_XAA
	pSiS->AccelInfoPtr = NULL;
#endif
#ifdef SIS_USE_EXA
	pSiS->EXADriverPtr = NULL;
	pSiS->exa_scratch = NULL;
#endif

	if((pScrn->bitsPerPixel != 8)  &&
	   (pScrn->bitsPerPixel != 16) &&
	   (pScrn->bitsPerPixel != 32)) {
	   pSiS->NoAccel = TRUE;
	}

	if(!pSiS->NoAccel) {
#ifdef SIS_USE_XAA
	   if(!pSiS->useEXA) {
	      pSiS->AccelInfoPtr = infoPtr = XAACreateInfoRec();
	      if(!infoPtr) pSiS->NoAccel = TRUE;
	   }
#endif
#ifdef SIS_USE_EXA
	   if(pSiS->useEXA) {
	      if(!(pSiS->EXADriverPtr = exaDriverAlloc())) {
		 pSiS->NoAccel = TRUE;
		 pSiS->NoXvideo = TRUE; /* No fbmem manager -> no xv */
	      }
	   }
#endif
	}

	if(!pSiS->NoAccel) {

	   SiSInitializeAccelerator(pScrn);

	   pSiS->InitAccel = SiSInitializeAccelerator;
	   pSiS->SyncAccel = SiSSyncAccel;
	   pSiS->FillRect  = SiSDGAFillRect;
	   pSiS->BlitRect  = SiSDGABlitRect;


#ifdef SIS_USE_XAA	/* ----------------------- XAA ----------------------- */
	   if(!pSiS->useEXA) {

	      infoPtr->Flags = LINEAR_FRAMEBUFFER |
			       OFFSCREEN_PIXMAPS |
			       PIXMAP_CACHE;

	      /* sync */
	      infoPtr->Sync = SiSSync;

	      /* BitBlt */
	      infoPtr->SetupForScreenToScreenCopy = SiSSetupForScreenToScreenCopy;
	      infoPtr->SubsequentScreenToScreenCopy = SiSSubsequentScreenToScreenCopy;
	      infoPtr->ScreenToScreenCopyFlags = NO_PLANEMASK | TRANSPARENCY_GXCOPY_ONLY;

	      /* solid fills */
	      infoPtr->SetupForSolidFill = SiSSetupForSolidFill;
	      infoPtr->SubsequentSolidFillRect = SiSSubsequentSolidFillRect;
	      infoPtr->SolidFillFlags = NO_PLANEMASK;

	      /* solid line */
	      infoPtr->SetupForSolidLine = SiSSetupForSolidLine;
	      infoPtr->SubsequentSolidTwoPointLine = SiSSubsequentSolidTwoPointLine;
	      infoPtr->SubsequentSolidHorVertLine = SiSSubsequentSolidHorzVertLine;
	      infoPtr->SolidLineFlags = NO_PLANEMASK;

	      /* dashed line */
	      infoPtr->SetupForDashedLine = SiSSetupForDashedLine;
	      infoPtr->SubsequentDashedTwoPointLine = SiSSubsequentDashedTwoPointLine;
	      infoPtr->DashPatternMaxLength = 64;
	      infoPtr->DashedLineFlags = NO_PLANEMASK |
					 LINE_PATTERN_MSBFIRST_LSBJUSTIFIED;

	      /* 8x8 mono pattern fill */
	      infoPtr->SetupForMono8x8PatternFill = SiSSetupForMonoPatternFill;
	      infoPtr->SubsequentMono8x8PatternFillRect = SiSSubsequentMonoPatternFill;
	      infoPtr->Mono8x8PatternFillFlags = NO_PLANEMASK |
						 HARDWARE_PATTERN_SCREEN_ORIGIN |
						 HARDWARE_PATTERN_PROGRAMMED_BITS |
						 BIT_ORDER_IN_BYTE_MSBFIRST;



	      /* 8x8 color pattern fill (MMIO support not implemented) */
	      infoPtr->SetupForColor8x8PatternFill = SiSSetupForColor8x8PatternFill;
	      infoPtr->SubsequentColor8x8PatternFillRect = SiSSubsequentColor8x8PatternFillRect;
	      infoPtr->Color8x8PatternFillFlags = NO_PLANEMASK |
						  HARDWARE_PATTERN_SCREEN_ORIGIN |
						  NO_TRANSPARENCY;


#if defined(RENDER) && defined(INCL_RENDER)
	      /* Render */
	      SiSCalcRenderAccelArray(pScrn);
	      if(pSiS->RenderAccelArray) {
	         pSiS->AccelLinearScratch = NULL;

#ifdef SISNEWRENDER
		 infoPtr->SetupForCPUToScreenAlphaTexture2 = (pSiS->ChipType == SIS_671)?
				SiSSetupForCPUToScreenAlphaTexture3D : SiSSetupForCPUToScreenAlphaTexture;
		 infoPtr->CPUToScreenAlphaTextureDstFormats = (pScrn->bitsPerPixel == 16) ?
				SiSDstTextureFormats16 : SiSDstTextureFormats32;
#else
		 infoPtr->SetupForCPUToScreenAlphaTexture = (pSiS->ChipType == SIS_671)? 
		 		SiSSetupForCPUToScreenAlphaTexture3D : SiSSetupForCPUToScreenAlphaTexture;

#endif
		 infoPtr->SubsequentCPUToScreenAlphaTexture = (pSiS->ChipType == SIS_671)?
				SiSSubsequentCPUToScreenTexture3D : SiSSubsequentCPUToScreenTexture;
		 infoPtr->CPUToScreenAlphaTextureFormats = (pSiS->ChipType == SIS_671)? 
				SiS3DAlphaTextureFormats : SiSAlphaTextureFormats;  /*3D accelerator needs a8r8g8b8 format*/
		 infoPtr->CPUToScreenAlphaTextureFlags = XAA_RENDER_NO_TILE;

#ifdef SISNEWRENDER
		 infoPtr->SetupForCPUToScreenTexture2 = SiSSetupForCPUToScreenTexture;
		 infoPtr->CPUToScreenTextureDstFormats = (pScrn->bitsPerPixel == 16) ?
				SiSDstTextureFormats16 : SiSDstTextureFormats32;
#else
		 infoPtr->SetupForCPUToScreenTexture = SiSSetupForCPUToScreenTexture;
#endif
		 infoPtr->SubsequentCPUToScreenTexture = (pSiS->ChipType == SIS_671)?
				SiSSubsequentCPUToScreenTexture3D : SiSSubsequentCPUToScreenTexture;
		 infoPtr->CPUToScreenTextureFormats = SiSTextureFormats;
		 infoPtr->CPUToScreenTextureFlags = XAA_RENDER_NO_TILE;

	      }
#endif /* RENDER && INCL_RENDER */

#ifdef SISDUALHEAD
	      if(pSiS->DualHeadMode) {
		 infoPtr->RestoreAccelState = SiSRestoreAccelState;
	      }
#endif
	   }  /* !EXA */
#endif /* XAA */

#ifdef SIS_USE_EXA	/* ----------------------- EXA ----------------------- */
	   if(pSiS->useEXA) {
#ifndef XORG_NEW

	      int obase = 0;

	      /* data */
	      pSiS->EXADriverPtr->card.memoryBase = pSiS->FbBase;
	      pSiS->EXADriverPtr->card.memorySize = pSiS->maxxfbmem;

	      obase = pScrn->displayWidth * pScrn->virtualY * (pScrn->bitsPerPixel >> 3);

	      pSiS->EXADriverPtr->card.offScreenBase = obase;
	      if(pSiS->EXADriverPtr->card.memorySize > pSiS->EXADriverPtr->card.offScreenBase) {
		 pSiS->EXADriverPtr->card.flags = EXA_OFFSCREEN_PIXMAPS;
	      } else {
		 pSiS->NoXvideo = TRUE;
		 xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
			"Not enough video RAM for offscreen memory manager. Xv disabled\n");
	      }

#if  XORG_VERSION_CURRENT < XORG_VERSION_NUMERIC(6,8,2,0,0)
	      pSiS->EXADriverPtr->card.offscreenByteAlign = 16;	/* src/dst: double quad word boundary */
	      pSiS->EXADriverPtr->card.offscreenPitch = 4;	/* pitch:   double word boundary      */
#else
	      pSiS->EXADriverPtr->card.pixmapOffsetAlign = 16;	/* src/dst: double quad word boundary */
	      pSiS->EXADriverPtr->card.pixmapPitchAlign = 4;	/* pitch:   double word boundary      */
#endif
	      pSiS->EXADriverPtr->card.maxX = 4095;
	      pSiS->EXADriverPtr->card.maxY = 4095;

	      /* Sync */
	      pSiS->EXADriverPtr->accel.WaitMarker = SiSEXASync;

	      /* Solid fill */
	      pSiS->EXADriverPtr->accel.PrepareSolid = SiSPrepareSolid;
	      pSiS->EXADriverPtr->accel.Solid = SiSSolid;
	      pSiS->EXADriverPtr->accel.DoneSolid = SiSDoneSolid;

	      /* Copy */
	      pSiS->EXADriverPtr->accel.PrepareCopy = SiSPrepareCopy;
	      pSiS->EXADriverPtr->accel.Copy = SiSCopy;
	      pSiS->EXADriverPtr->accel.DoneCopy = SiSDoneCopy;

	      /* Composite */
#ifdef SIS_HAVE_COMPOSITE
	      SiSCalcRenderAccelArray(pScrn);
	      if(pSiS->RenderAccelArray) {
		 pSiS->EXADriverPtr->accel.CheckComposite = SiSCheckComposite;
		 pSiS->EXADriverPtr->accel.PrepareComposite = SiSPrepareComposite;
		 pSiS->EXADriverPtr->accel.Composite = SiSComposite;
		 pSiS->EXADriverPtr->accel.DoneComposite = SiSDoneComposite;
	      }
#endif


#else /*Xorg>= 7.0*/

	      pSiS->EXADriverPtr->exa_major = 2;
	      pSiS->EXADriverPtr->exa_minor = 0;

	      /* data */
	      pSiS->EXADriverPtr->memoryBase = pSiS->FbBase;
	      pSiS->EXADriverPtr->memorySize = pSiS->maxxfbmem;
	      pSiS->EXADriverPtr->offScreenBase = pScrn->virtualX * pScrn->virtualY
						* ((pScrn->bitsPerPixel + 7) / 8);
	      if(pSiS->EXADriverPtr->memorySize > pSiS->EXADriverPtr->offScreenBase) {
		 pSiS->EXADriverPtr->flags = EXA_OFFSCREEN_PIXMAPS;
	      } else {
		 pSiS->NoXvideo = TRUE;
		 xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
			"Not enough video RAM for offscreen memory manager. Xv disabled\n");
	      }
	      pSiS->EXADriverPtr->pixmapOffsetAlign = 16;	/* src/dst: double quad word boundary */
	      pSiS->EXADriverPtr->pixmapPitchAlign = 4;	/* pitch:   double word boundary      */
	      pSiS->EXADriverPtr->maxX = 4095;
	      pSiS->EXADriverPtr->maxY = 4095;

	      /* Sync */
	      pSiS->EXADriverPtr->WaitMarker = SiSEXASync;

	      /* Solid fill */
	      pSiS->EXADriverPtr->PrepareSolid = SiSPrepareSolid;
	      pSiS->EXADriverPtr->Solid = SiSSolid;
	      pSiS->EXADriverPtr->DoneSolid = SiSDoneSolid;

	      /* Copy */
	      pSiS->EXADriverPtr->PrepareCopy = SiSPrepareCopy;
	      pSiS->EXADriverPtr->Copy = SiSCopy;
	      pSiS->EXADriverPtr->DoneCopy = SiSDoneCopy;

	      /* Composite */
#ifdef SIS_HAVE_COMPOSITE
	      SiSCalcRenderAccelArray(pScrn);
	      if(pSiS->RenderAccelArray) {
		 pSiS->EXADriverPtr->CheckComposite = SiSCheckComposite;
		 pSiS->EXADriverPtr->PrepareComposite = SiSPrepareComposite;
		 pSiS->EXADriverPtr->Composite = SiSComposite;
		 pSiS->EXADriverPtr->DoneComposite = SiSDoneComposite;
	      }
#endif

#endif /*end of Xorg>=7.0*/ 
	   
	   }
#endif /*end of EXA acceleration*/
	}  /* NoAccel */

	/* Init framebuffer memory manager */

	/* Traditional layout:
	 *   |-----------------++++++++++++++++++++^************==========~~~~~~~~~~~~|
	 *   |  UsableFbSize    ColorExpandBuffers |  DRI-Heap   HWCursor  CommandQueue
	 * FbBase                                topFB
	 *   +-------------maxxfbmem---------------+
	 *
	 * On SiS76x with UMA+LFB:
	 * |UUUUUUUUUUUUUUU--------------++++++++++++++++++++^==========~~~~~~~~~~~~|
	 *     DRI heap    |UsableFbSize  ColorExpandBuffers | HWCursor  CommandQueue
	 *  (in UMA and   FbBase                           topFB
	 *   eventually    +---------- maxxfbmem ------------+
	 *  beginning of
	 *      LFB)
	 */

#ifdef SIS_USE_XAA
	if(!pSiS->useEXA) {
	   topFB = pSiS->maxxfbmem; /* relative to FbBase */

	   reservedFbSize = pSiS->ColorExpandBufferNumber * pSiS->PerColorExpandBufferSize;


	   usableFbSize = topFB - reservedFbSize;
	   Avail.x1 = 0;
	   Avail.y1 = 0;
	   Avail.x2 = pScrn->displayWidth;
	   Avail.y2 = (usableFbSize / (pScrn->displayWidth * pScrn->bitsPerPixel / 8)) - 1;


	   if(Avail.y2 < 0) Avail.y2 = 32767;
	   if(Avail.y2 < pScrn->currentMode->VDisplay) {
	      xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
			"Not enough video RAM for accelerator. "
			"%dKB needed, %dKB available\n",
			((((pScrn->displayWidth * pScrn->bitsPerPixel / 8)   /* +8 for make it sure */
			     * pScrn->currentMode->VDisplay) + reservedFbSize) / 1024) + 8,
			pSiS->maxxfbmem/1024);
	      pSiS->NoAccel = TRUE;
	      pSiS->NoXvideo = TRUE;
	      XAADestroyInfoRec(pSiS->AccelInfoPtr);
	      pSiS->AccelInfoPtr = NULL;
	      return FALSE;   /* Don't even init fb manager */
	   }

	   xf86DrvMsg(pScrn->scrnIndex, X_INFO,
		   "Framebuffer from (%d,%d) to (%d,%d)\n",
		   Avail.x1, Avail.y1, Avail.x2 - 1, Avail.y2 - 1);


	   xf86InitFBManager(pScreen, &Avail);


	   if(!pSiS->NoAccel) {
	      return XAAInit(pScreen, infoPtr);
	   }
	} /* !EXA */
#endif /* XAA */

#ifdef SIS_USE_EXA
	if(pSiS->useEXA) {

	   if(!pSiS->NoAccel) {

	      if(!exaDriverInit(pScreen, pSiS->EXADriverPtr)) {
		 pSiS->NoAccel = TRUE;
		 pSiS->NoXvideo = TRUE; /* No fbmem manager -> no xv */
		 return FALSE;
	      }

	      /* Reserve locked offscreen scratch area of 128K for glyph data */
	      pSiS->exa_scratch = exaOffscreenAlloc(pScreen, 128 * 1024, 16, TRUE,
						SiSScratchSave, pSiS);
	      if(pSiS->exa_scratch) {
		 pSiS->exa_scratch_next = pSiS->exa_scratch->offset;
#ifdef XORG_NEW
		 pSiS->EXADriverPtr->UploadToScratch = SiSUploadToScratch;
#else
		 pSiS->EXADriverPtr->accel.UploadToScratch = SiSUploadToScratch;
#endif
	      }

	   } else {

	      pSiS->NoXvideo = TRUE; /* No fbmem manager -> no xv */

	   }

	}
#endif /* EXA */

	return TRUE;
}




