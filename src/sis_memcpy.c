/*
 * Copyright (C) 2004 Thomas Hellstrom, All Rights Reserved.
 * Copyright (C) 2004 Thomas Winischhofer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE CODE SUPPLIER(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */

#include "xf86.h"
#include "sis.h"
#include "compiler.h"

extern FBLinearPtr SISAllocateOverlayMemory(ScrnInfoPtr pScrn, FBLinearPtr linear, int size);

#define FL_LIBC  0x01
#define FL_BI    0x02
#define FL_SSE   0x04
#define FL_MMX   0x08
#define FL_3DNOW 0x10
#define FL_MMX2  0x20

#define CPUBUFSIZE 2048      /* Size of /proc/cpuinfo buffer */
#define BUFSIZ (576 * 1152)  /* Matches 720x576 YUV420 */


/************************************************************************/
/*                   arch specific memcpy() routines                    */
/************************************************************************/

/* i386 */

#define SSE_PREFETCH "  prefetchnta "

#define FENCE __asm__ __volatile__ ("sfence":::"memory");

#define FENCEMMS __asm__ __volatile__ ("\t"		\
				       "sfence\n\t"	\
				       "emms\n\t"	\
				       :::"memory");
				       
#define FEMMS __asm__ __volatile__("femms":::"memory");

#define EMMS __asm__ __volatile__("emms":::"memory");

#define NOW_PREFETCH "  prefetch "

#define PREFETCH1(arch_prefetch,from)			\
    __asm__ __volatile__ (				\
		  "1:  " 				\
		  arch_prefetch "(%0)\n"		\
		  arch_prefetch "32(%0)\n"		\
		  arch_prefetch "64(%0)\n"		\
		  arch_prefetch "96(%0)\n"		\
		  arch_prefetch "128(%0)\n"		\
		  arch_prefetch "160(%0)\n"		\
		  arch_prefetch "192(%0)\n"		\
		  arch_prefetch "256(%0)\n"		\
		  arch_prefetch "288(%0)\n"		\
		  "2:\n"				\
		  : : "r" (from) );

#define PREFETCH2(arch_prefetch,from)			\
    __asm__ __volatile__ (				\
		  arch_prefetch "320(%0)\n"		\
		  : : "r" (from) );
		  
#define PREFETCH3(arch_prefetch,from)			\
    __asm__ __volatile__ (				\
		  arch_prefetch "288(%0)\n"		\
		  : : "r" (from) );

#define small_memcpy(to,from,n)						\
    {									\
	__asm__ __volatile__(						\
		  "movl %2,%%ecx\n\t"					\
                  "sarl $2,%%ecx\n\t"					\
		  "rep ; movsl\n\t"					\
		  "testb $2,%b2\n\t"					\
		  "je 1f\n\t"						\
		  "movsw\n"						\
		  "1:\ttestb $1,%b2\n\t"				\
		  "je 2f\n\t"						\
		  "movsb\n"						\
		  "2:"							\
		  :"=&D" (to), "=&S" (from)				\
		  :"q" (n),"0" ((long) to),"1" ((long) from) 		\
		  : "%ecx","memory");					\
    }

#define SSE_CPY(prefetch,from,to,dummy,lcnt)				\
    if((unsigned long) from & 15) {					\
	__asm__ __volatile__ (						\
		  "1:\n"						\
                  prefetch "320(%1)\n"					\
		  "  movups (%1), %%xmm0\n"				\
		  "  movups 16(%1), %%xmm1\n"				\
		  "  movntps %%xmm0, (%0)\n"				\
		  "  movntps %%xmm1, 16(%0)\n"				\
                  prefetch "352(%1)\n"					\
		  "  movups 32(%1), %%xmm2\n"				\
		  "  movups 48(%1), %%xmm3\n"				\
		  "  movntps %%xmm2, 32(%0)\n"				\
		  "  movntps %%xmm3, 48(%0)\n"				\
		  "  addl $64,%0\n"					\
		  "  addl $64,%1\n"					\
		  "  decl %2\n"						\
		  "  jne 1b\n"						\
		  :"=&D"(to), "=&S"(from), "=&r"(dummy)			\
		  :"0" (to), "1" (from), "2" (lcnt): "memory"); 	\
    } else {								\
	__asm__ __volatile__ (						\
		  "2:\n"						\
		  prefetch "320(%1)\n"					\
		  "  movaps (%1), %%xmm0\n"				\
		  "  movaps 16(%1), %%xmm1\n"				\
		  "  movntps %%xmm0, (%0)\n"				\
		  "  movntps %%xmm1, 16(%0)\n"				\
                  prefetch "352(%1)\n"					\
		  "  movaps 32(%1), %%xmm2\n"				\
		  "  movaps 48(%1), %%xmm3\n"				\
		  "  movntps %%xmm2, 32(%0)\n"				\
		  "  movntps %%xmm3, 48(%0)\n"				\
		  "  addl $64,%0\n"					\
		  "  addl $64,%1\n"					\
		  "  decl %2\n"						\
		  "  jne 2b\n"						\
		  :"=&D"(to), "=&S"(from), "=&r"(dummy)			\
		  :"0" (to), "1" (from), "2" (lcnt): "memory"); 	\
    }

#define MMX_CPY(prefetch,from,to,dummy,lcnt)				\
    __asm__ __volatile__ (						\
		  "1:\n"						\
		  prefetch "320(%1)\n"					\
		  "2:  movq (%1), %%mm0\n"				\
		  "  movq 8(%1), %%mm1\n"				\
		  "  movq 16(%1), %%mm2\n"				\
		  "  movq 24(%1), %%mm3\n"				\
		  "  movq %%mm0, (%0)\n"				\
		  "  movq %%mm1, 8(%0)\n"				\
		  "  movq %%mm2, 16(%0)\n"				\
		  "  movq %%mm3, 24(%0)\n"				\
		  prefetch "352(%1)\n"					\
		  "  movq 32(%1), %%mm0\n"				\
		  "  movq 40(%1), %%mm1\n"				\
		  "  movq 48(%1), %%mm2\n"				\
		  "  movq 56(%1), %%mm3\n"				\
		  "  movq %%mm0, 32(%0)\n"				\
		  "  movq %%mm1, 40(%0)\n"				\
		  "  movq %%mm2, 48(%0)\n"				\
		  "  movq %%mm3, 56(%0)\n"				\
		  "  addl $64,%0\n"					\
		  "  addl $64,%1\n"					\
		  "  decl %2\n"						\
		  "  jne 1b\n"						\
		  :"=&D"(to), "=&S"(from), "=&r"(dummy)			\
		  :"0" (to), "1" (from), "2" (lcnt) : "memory"); 

#define MMXEXT_CPY(prefetch,from,to,dummy,lcnt)				\
    __asm__ __volatile__ (						\
		  ".p2align 4,,7\n"					\
		  "1:\n"						\
		  prefetch "320(%1)\n"					\
		  "  movq (%1), %%mm0\n"				\
		  "  movq 8(%1), %%mm1\n"				\
		  "  movq 16(%1), %%mm2\n"				\
		  "  movq 24(%1), %%mm3\n"				\
		  "  movntq %%mm0, (%0)\n"				\
		  "  movntq %%mm1, 8(%0)\n"				\
		  "  movntq %%mm2, 16(%0)\n"				\
		  "  movntq %%mm3, 24(%0)\n"				\
		  prefetch "352(%1)\n"					\
		  "  movq 32(%1), %%mm0\n"				\
		  "  movq 40(%1), %%mm1\n"				\
		  "  movq 48(%1), %%mm2\n"				\
		  "  movq 56(%1), %%mm3\n"				\
		  "  movntq %%mm0, 32(%0)\n"				\
		  "  movntq %%mm1, 40(%0)\n"				\
		  "  movntq %%mm2, 48(%0)\n"				\
		  "  movntq %%mm3, 56(%0)\n"				\
		  "  addl $64,%0\n"					\
		  "  addl $64,%1\n"					\
		  "  decl %2\n"						\
		  "  jne 1b\n"						\
		  :"=&D"(to), "=&S"(from), "=&r"(dummy)			\
		  :"0" (to), "1" (from), "2" (lcnt) : "memory"); 

#define PREFETCH_FUNC(prefix,itype,ptype,begin,fence)			\
									\
    static void prefix##_memcpy(unsigned char *to,			\
				const unsigned char *from,		\
				int size)				\
    {									\
	int lcnt = size >> 6;						\
	int rest = size & 63;						\
	register int dummy;						\
									\
	PREFETCH1(ptype##_PREFETCH,from);				\
									\
	begin;								\
	if(lcnt) {							\
	   itype##_CPY(ptype##_PREFETCH,from,to,dummy,lcnt);		\
	}								\
	if(rest) {							\
	   PREFETCH2(ptype##_PREFETCH,from);				\
	   small_memcpy(to, from, rest);				\
	   PREFETCH3(ptype##_PREFETCH,from);				\
	}								\
	fence;								\
    }

#define NOPREFETCH_FUNC(prefix,itype,begin,fence)			\
									\
    static void prefix##_memcpy(unsigned char *to,			\
				const unsigned char *from,		\
				int size)				\
    {									\
	int lcnt = size >> 6;						\
	int rest = size & 63;						\
	register int dummy;						\
									\
	begin;								\
	if(lcnt) {							\
	   itype##_CPY("#",from,to,dummy,lcnt);				\
	}								\
	if(rest) {							\
	   small_memcpy(to, from, rest);				\
	}								\
	fence;								\
    }									

/* Other archs */

/* ... */

    
/* Type for table for benchmark */    
    
typedef struct {
    vidCopyFunc mFunc;
    char *mName;
    char **cpuFlag;
    int mycpuflag;
} SISMCFuncData;

/************************************************************************/
/*                   libc memcpy() wrapper - generic                    */
/************************************************************************/

static void SiS_libc_memcpy(unsigned char *dst, const unsigned char *src, int size)
{
    memcpy(dst, src, size);
}

/************************************************************************/
/*                   Built-in memcpy() - arch specific                  */
/************************************************************************/

#ifdef __i386__   

/* Built-in memcpy for i386 */
static __inline void * __memcpy(void * to, const void * from, size_t n)
{
    int d1,d2,d3;

    __asm__ __volatile__(
		 "rep ; movsl\n\t"
		 "testb $2,%b4\n\t"
		 "je 1f\n\t"
		 "movsw\n"
		 "1:\ttestb $1,%b4\n\t"
		 "je 2f\n\t"
		 "movsb\n"
		 "2:"
		 : "=&c" (d1), "=&D" (d2), "=&S" (d3)	
		 :"0" (n >> 2), "q" (n),"1" ((long) to),"2" ((long) from)
		 : "memory");
    return(to);
}

#else /* Other archs */

static __inline void * __memcpy(void * to, const void * from, size_t n)
{
    memcpy(dst, src, size);  /* placeholder */
}

#endif

/************************************************************************/
/*                   Generic built-in memcpy wrapper                    */
/************************************************************************/

static void SiS_builtin_memcpy(unsigned char *dst, const unsigned char *src, int size) 
{
    __memcpy(dst, src, size);
}

/************************************************************************/
/*                    Definitions for archs and OSes                    */
/************************************************************************/

#undef SiS_canBenchmark
#undef SiS_haveProc

#ifdef __i386__  /* i386 */

#define SiS_canBenchmark		/* Can we perform a benchmark? */
#ifdef linux
#define SiS_haveProc			/* Do we have /proc/cpuinfo or similar? */
#endif

static unsigned int taketime(void)	/* get current time (for benchmarking) */
{
    unsigned eax;
    __asm__ volatile ("\t"
		"cpuid\n\t"
		".byte 0x0f, 0x31" 
		: "=a" (eax)
		: "0"(0)
		: "ebx","ecx","edx","cc");     
		
    return (unsigned int)eax; 
}

#else		/* Other archs */

/* 1. Can we do a benchmark?  		*/
/* #define SiS_canBenchmark		*/

/* 2. Do we have /proc filesystem or similar for CPU information? */
/* #define SiS_haveproc			*/

/* 3. Function for getting current time (for benchmarking)  */
/*
static unsigned int taketime(void)
{
}
*/

#endif

/************************************************************************/
/* Generic routines if Benchmark can be performed (all archs, all OSes) */
/************************************************************************/

#ifdef SiS_canBenchmark

/* Get time (unsigned int) */
static unsigned int time_function(vidCopyFunc mf, unsigned char *buf1, unsigned char *buf2, int size) 
{
    unsigned int t1, t2;

    t1 = taketime();

    (*mf)(buf1, buf2, size);

    t2 = taketime(); 
    
    return((t1 <  t2) ? t2 - t1 : 0xFFFFFFFFU - (t1 - t2 - 1)); 
}

/* Allocate an area of offscreen FB memory (buf1), a simulated video
 * player buffer (buf2) and a pool of uninitialized "video" data (buf3). 
 */      
static FBLinearPtr SiS_AllocBuffers(ScrnInfoPtr pScrn, unsigned char **buf1, 
                                         unsigned char **buf2, unsigned char **buf3)    
{   
    SISPtr pSiS = SISPTR(pScrn);
    int depth = pSiS->CurrentLayout.bitsPerPixel >> 3;
    unsigned alignSize = (BUFSIZ + depth - 1) / depth;
    FBLinearPtr tmpFbBuffer = NULL;
    
    if(!(tmpFbBuffer = SISAllocateOverlayMemory(pScrn, tmpFbBuffer, alignSize + 31))) {
       return NULL;
    }
    (*buf1) = (unsigned char *)pSiS->FbBase + (tmpFbBuffer->offset * depth);
    (*buf1) = (unsigned char *)(((unsigned long)(*buf1) + 31) & ~31);
	
    if(!((*buf2) = (unsigned char *)xalloc(BUFSIZ))) {
       xf86FreeOffscreenLinear(tmpFbBuffer);
       return NULL;
    }
    
    if(!((*buf3) = (unsigned char *)xalloc(BUFSIZ))) {
       xfree((*buf2));
       xf86FreeOffscreenLinear(tmpFbBuffer);
       return NULL;
    }
    
    return tmpFbBuffer;
}

/* Perform Benchmark */
static int SiS_BenchmarkMemcpy(ScrnInfoPtr pScrn, SISMCFuncData *MCFunctions, 
                               unsigned int myCPUflags, 
			       unsigned char *buf1, unsigned char *buf2, 
			       unsigned char *buf3, char *frqBuf, double cpuFreq)    
{    
    SISMCFuncData *curData;
    int j = 0, bestSoFar = 0;
    unsigned int tmp1, tmp2, best = 0xFFFFFFFFU;
  
    /* Make probable buf1 and buf2 are not paged out by referencing them */
    SiS_libc_memcpy(buf1, buf2, BUFSIZ);

    xf86DrvMsg(pScrn->scrnIndex, X_INFO,
	       "Benchmarking memcpy() methods:\n");
	       
    j = 0;       
    while(MCFunctions[j].mFunc) {
    
	curData = MCFunctions + j;   

	if(myCPUflags & curData->mycpuflag) {

	   /* Simulate setup of the video buffer and copy result to framebuffer */
	   /* Do this 4 times to verify results */
	   SiS_builtin_memcpy(buf2, buf3, BUFSIZ);
	   tmp1 = time_function(curData->mFunc, buf1, buf2, BUFSIZ);
	   SiS_builtin_memcpy(buf2, buf3, BUFSIZ);
	   tmp2 = time_function(curData->mFunc, buf1, buf2, BUFSIZ);
	   tmp1 = (tmp2 < tmp1) ? tmp2 : tmp1;
	   SiS_builtin_memcpy(buf2, buf3, BUFSIZ);
	   tmp2 = time_function(curData->mFunc, buf1, buf2, BUFSIZ);
	   tmp1 = (tmp2 < tmp1) ? tmp2 : tmp1;
	   SiS_builtin_memcpy(buf2, buf3, BUFSIZ);
	   tmp2 = time_function(curData->mFunc, buf1, buf2, BUFSIZ);
	   tmp1 = (tmp2 < tmp1) ? tmp2 : tmp1;

	   if(!frqBuf) {
	      xf86DrvMsg(pScrn->scrnIndex, X_PROBED,
			   "\tChecked %s memcpy()... \t%u\n",curData->mName, tmp1);
	   } else {
	      xf86DrvMsg(pScrn->scrnIndex, X_PROBED,
			   "\tChecked %s memcpy()... \t%.1f MiB/s\n",
			   curData->mName, 
			   cpuFreq * 1.e6 * (double)BUFSIZ / ((double)(tmp1) * (double)(0x100000)));
	   }		
	    
	   if(tmp1 < best) {
	      best = tmp1;
	      bestSoFar = j;
	   }
	    
	} 
	
	j++;
    }
    
    return bestSoFar;
}
#endif

/**********************************************************************/
/*      Generic routines if /proc filesystem is available (Linux)     */
/**********************************************************************/

#ifdef SiS_haveProc
/* Linux: Read file (/proc/cpuinfo) into buffer */
static int SiS_ReadProc(char *buf, char *filename)
{    
    FILE *cpuInfoFile;
    int count;
    
    if((cpuInfoFile = fopen(filename, "r")) == NULL) {
       return 0;
    }
    
    count = fread(buf, 1, CPUBUFSIZE, cpuInfoFile);
    if(ferror(cpuInfoFile)) {
       fclose(cpuInfoFile);
       return 0;
    }
    
    fclose(cpuInfoFile);
    
    if(count >= CPUBUFSIZE - 2) {
       return 0;
    }
    
    buf[count] = 0;
    
    return count;
}

/* Linux: Extract CPU speed from /proc/cpuinfo */
static char *SiS_GetCPUFreq(ScrnInfoPtr pScrn, char *buf, double *cpuFreq)
{    
    char *frqBuf, *endBuf;
    
    (*cpuFreq) = 0.0;
    
    if((frqBuf = strstr(buf,"cpu MHz\t\t:"))) {
       frqBuf += 11;
       (*cpuFreq) = strtod(frqBuf, &endBuf);
       if(endBuf == frqBuf) frqBuf = NULL;
       else {
          xf86DrvMsg(pScrn->scrnIndex, X_PROBED, "CPU frequency %.2fMhz\n", (*cpuFreq));
       }
    }
    
    return frqBuf;
}

/* Linux: Parse "flags:" field from /proc/cpuinfo */
static unsigned int SiS_ParseCPUFlags(char *cpuinfo, SISMCFuncData *MCFunctions)
{
   unsigned int flags = 0;
   int i = 0, flagIdx, flagEnd;
   unsigned char bufbackup, bufbackup2; 
   char **sflags;
   char *flagLoc;
   
   if(!(cpuinfo = strstr(cpuinfo, "processor\t:"))) return FALSE;
   if(!(flagLoc = strstr(cpuinfo, "flags\t\t:"))) return FALSE;
   flagLoc += 8;
   flagIdx = flagEnd = (int)(flagLoc - cpuinfo);
   while((cpuinfo[flagEnd] != '\n') && (cpuinfo[flagEnd] != 0)) flagEnd++;
   bufbackup = cpuinfo[flagEnd];    cpuinfo[flagEnd]   = ' ';
   bufbackup2 = cpuinfo[flagEnd+1]; cpuinfo[flagEnd+1] =  0;
   
   while(MCFunctions[i].mFunc) {
      if((sflags = MCFunctions[i].cpuFlag)) {
         for(; *sflags != 0; sflags++) {
	    if(strstr(&cpuinfo[flagIdx], *sflags)) flags |= MCFunctions[i].mycpuflag;
	 }
      } else {
         flags |= MCFunctions[i].mycpuflag;
      }
      i++;
   }
   
   cpuinfo[flagEnd] = bufbackup; cpuinfo[flagEnd+1] = bufbackup2;
   
   return(flags);
}
#endif

/* Arch-specific routines */

#ifdef __i386__   /* i386 *************************************/

PREFETCH_FUNC(SiS_sse,SSE,SSE,,FENCE) 
PREFETCH_FUNC(SiS_mmxext,MMXEXT,SSE,EMMS,FENCEMMS)
PREFETCH_FUNC(SiS_now,MMX,NOW,FEMMS,FEMMS)
NOPREFETCH_FUNC(SiS_mmx,MMX,EMMS,EMMS)

char *sse_cpuflags[]    = {" sse ", 0};
char *mmx_cpuflags[]    = {" mmx ", 0};
char *now_cpuflags[]    = {" 3dnow ", 0};
char *mmx2_cpuflags[]   = {" mmxext ", " sse ", 0};

static SISMCFuncData MCFunctions_i386[] = {
    {SiS_libc_memcpy,   "libc",    NULL,         FL_LIBC},
    {SiS_builtin_memcpy,"built-in",NULL,         FL_BI},    
    {SiS_sse_memcpy,    "SSE",     sse_cpuflags, FL_SSE}, 
    {SiS_mmx_memcpy,    "MMX",     mmx_cpuflags, FL_MMX}, 
    {SiS_now_memcpy,    "3DNow!",  now_cpuflags, FL_3DNOW}, 
    {SiS_mmxext_memcpy, "MMX2",    mmx2_cpuflags,FL_MMX2},
    {NULL,              "",        NULL,         0}
}; 

static Bool cpuIDSupported(ScrnInfoPtr pScrn)
{
    int eax, ecx;
       
    /* Check for cpuid instruction */
    __asm__ __volatile__ ("\t"
    		"pushf\n\t"
		"popl %0\n\t"
		"movl %0, %1\n\t"
		"xorl $0x200000, %0\n\t"
		"push %0\n\t"
		"popf\n\t"
		"pushf\n\t"
		"popl %0"
		: "=a" (eax), "=c" (ecx)
		:
		: "cc");
		
    if(eax == ecx) {
       xf86DrvMsg(pScrn->scrnIndex, X_INFO, "CPU does not support CPUID instruction\n");
       return FALSE;
    }
    
    /* Check for RDTSC */
    __asm__ __volatile__ ("\t"
    		"pushl %%edx\n\t"
    		"movl $1, %%eax\n\t"
		"cpuid\t\n"
		"movl %%edx, %%eax\t\n"
		"popl %%edx"
		: "=a" (eax)
		:  
		: "ebx","ecx","cc");

    if(!(eax & 0x10)) {
       xf86DrvMsg(pScrn->scrnIndex, X_INFO, "CPU does not support RDTSC instruction\n");
       return FALSE;
    }
    
    return TRUE;
}

#else  /* Other archs ************************************* */

/* Fill in here */

#endif

#ifdef SiS_canBenchmark
/* Main: Benchmark the video copy routines and choose the fastest */
static vidCopyFunc SiSVidCopyInitGen(ScreenPtr pScreen, SISMCFuncData *MCFunctions) 
{
    ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];  
    SISPtr pSiS = SISPTR(pScrn);
    FBLinearPtr tmpFbBuffer = NULL;
    char *frqBuf = NULL;
    unsigned char *buf1, *buf2, *buf3;
    double cpuFreq;
    unsigned int myCPUflags;
    int best;
#ifdef SiS_haveProc   
    char buf[CPUBUFSIZE];
#endif    
    
    /* Bail out if user disabled benchmarking */
    if(!pSiS->BenchMemCpy) {
       return SiS_libc_memcpy;
    }

#ifdef SiS_haveProc   	
 
    /* Read /proc/cpuinfo into buf */
    if(SiS_ReadProc(buf, "/proc/cpuinfo")) {
       
       /* Extract CPU frequency */ 
       frqBuf = SiS_GetCPUFreq(pScrn, buf, &cpuFreq);
   
       /* Parse the CPU flags and convert them to our internal value */
       myCPUflags = SiS_ParseCPUFlags(buf, MCFunctions);
       
    } else {
    
       xf86DrvMsg(pScrn->scrnIndex, X_INFO, "/proc/cpuinfo not found\n");
      
       /* Default: no CPU freq, only libc and built-in memcpy() */
       cpuFreq = 0.0;
       frqBuf = NULL;
       myCPUflags = FL_LIBC | FL_BI;
    
    }
    
#else 			
    
    /* Default: no CPU freq, only libc and built-in memcpy() */
    cpuFreq = 0.0;
    frqBuf = NULL;
    myCPUflags = FL_LIBC | FL_BI;

#endif    		
	
    /* Allocate buffers */
    if(!(tmpFbBuffer = SiS_AllocBuffers(pScrn, &buf1, &buf2, &buf3))) {
       return SiS_libc_memcpy;
    }
      
    /* Perform Benchmark */
    best = SiS_BenchmarkMemcpy(pScrn, MCFunctions, myCPUflags, 
    					buf1, buf2, buf3, frqBuf, cpuFreq);
        
    /* Free buffers */			
    xf86FreeOffscreenLinear(tmpFbBuffer);
    xfree(buf2);
    xfree(buf3);   
    
    xf86DrvMsg(pScrn->scrnIndex, X_PROBED,
	       "Using %s memcpy() for video data transfers\n", MCFunctions[best].mName);
	       
    return MCFunctions[best].mFunc; 
}

#ifdef __i386__

vidCopyFunc SiSVidCopyInit(ScreenPtr pScreen)
{
    ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
    
    /* Check if cpuid and rdtsc instructions are supported */
    if(!cpuIDSupported(pScrn)) {
       return SiS_libc_memcpy;
    }
    
    return(SiSVidCopyInitGen(pScreen, MCFunctions_i386));
}

#else /* Other archs: For now, use libc memcpy() */

vidCopyFunc SiSVidCopyInit(ScreenPtr pScreen) 
{
    return SiS_libc_memcpy;
}

#endif

#else  /* no benchmark: use libc memcpy() */
 
vidCopyFunc SiSVidCopyInit(ScreenPtr pScreen) 
{
    return SiS_libc_memcpy;
}

#endif

