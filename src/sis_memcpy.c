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
#include "compiler.h"
#include "sis.h"

#if 0			/* Debug */
#define SISDGBMC	
#endif

extern FBLinearPtr SISAllocateOverlayMemory(ScrnInfoPtr pScrn, FBLinearPtr linear, int size);

#define FL_LIBC  0x001
#define FL_BI    0x002
#define FL_SSE   0x004
#define FL_MMX   0x008
#define FL_3DNOW 0x010
#define FL_MMX2  0x020
#define FL_BI2   0x040

#define CPUBUFSIZE 2048      /* Size of /proc/cpuinfo buffer */
#define BUFSIZ (576 * 1152)  /* Matches 720x576 YUV420 */


/************************************************************************/
/*                   arch specific memcpy() routines                    */
/************************************************************************/

/* i386, AMD64 */

#define FENCE 			\
     __asm__ __volatile__( 	\
		  " sfence\n" 	\
		  :		\
		  :		\
		  : "memory");	

#define FENCEMMS 		\
     __asm__ __volatile__ (	\
     	          " sfence\n"	\
	          " emms\n"	\
	          :		\
		  :		\
		  : "memory");
				       
#define FEMMS 			\
     __asm__ __volatile__(	\
      	 	  " femms\n"	\
		  :		\
		  :		\
		  : "memory");

#define EMMS 			\
     __asm__ __volatile__(	\
     		  " emms\n"	\
		  :		\
		  :		\
		  : "memory");

#define SSE_PREFETCH " prefetchnta "
#define NOW_PREFETCH " prefetch "

#define PREFETCH1(arch_prefetch,from)		\
    __asm__ __volatile__ (			\
		  arch_prefetch "(%0)\n"	\
		  arch_prefetch "32(%0)\n"	\
		  arch_prefetch "64(%0)\n"	\
		  arch_prefetch "96(%0)\n"	\
		  arch_prefetch "128(%0)\n"	\
		  arch_prefetch "160(%0)\n"	\
		  arch_prefetch "192(%0)\n"	\
		  arch_prefetch "256(%0)\n"	\
		  arch_prefetch "288(%0)\n"	\
		  : 				\
		  : "r" (from) );

#define PREFETCH2(arch_prefetch,from)		\
    __asm__ __volatile__ (			\
		  arch_prefetch "320(%0)\n"	\
		  : 				\
		  : "r" (from) );
		  
#define PREFETCH3(arch_prefetch,from)		\
    __asm__ __volatile__ (			\
		  arch_prefetch "288(%0)\n"	\
		  : 				\
		  : "r" (from) );

#define small_memcpy_i386(to,from,n)					\
    {									\
	__asm__ __volatile__(						\
		  " cld\n"						\
		  " shrl $1, %%ecx\n"					\
		  " jnc 1f\n"						\
		  " movsb\n"						\
	        "1: shrl $1, %%ecx\n"					\
	          " jnc 2f\n"						\
		  " movsw\n"						\
	        "2: rep ; movsl"					\
		  : "=&D" (to), "=&S" (from)				\
		  : "c" (n), "0" ((long) to), "1" ((long) from) 	\
		  : "memory", "cc");					\
    }

#define small_memcpy_amd64(to,from,n)					\
    {									\
	__asm__ __volatile__(						\
		  " cld\n"						\
		  " shrq $1, %%rcx\n"					\
		  " jnc 1f\n"						\
		  " movsb\n"						\
	        "1: shrq $1, %%rcx\n"					\
	          " jnc 2f\n"						\
		  " movsw\n"						\
	        "2: shrq $1, %%rcx\n"					\
		  " jnc 3f\n"						\
		  " movsl\n"						\
		"3: rep ; movsq"					\
		  : "=&D" (to), "=&S" (from)				\
		  : "c" (n), "0" ((long) to), "1" ((long) from) 	\
		  : "memory", "cc");					\
    }    
    
#define MMX_CPY(prefetch,from,to,dummy,lcnt)				\
    __asm__ __volatile__ (						\
	        "1:\n"							\
		    prefetch "320(%1)\n"				\
	          " movq (%1), %%mm0\n"					\
		  " movq 8(%1), %%mm1\n"				\
		  " movq 16(%1), %%mm2\n"				\
		  " movq 24(%1), %%mm3\n"				\
		  " movq %%mm0, (%0)\n"					\
		  " movq %%mm1, 8(%0)\n"				\
		  " movq %%mm2, 16(%0)\n"				\
		  " movq %%mm3, 24(%0)\n"				\
		    prefetch "352(%1)\n"				\
		  " movq 32(%1), %%mm0\n"				\
		  " movq 40(%1), %%mm1\n"				\
		  " movq 48(%1), %%mm2\n"				\
		  " movq 56(%1), %%mm3\n"				\
		  " leal 64(%1),%1\n"					\
		  " movq %%mm0, 32(%0)\n"				\
		  " movq %%mm1, 40(%0)\n"				\
		  " movq %%mm2, 48(%0)\n"				\
		  " movq %%mm3, 56(%0)\n"				\
		  " decl %2\n"						\
		  " leal 64(%0),%0\n"					\
		  " jne 1b\n"						\
		  : "=&D"(to), "=&S"(from), "=&r"(dummy)		\
		  : "0" (to), "1" (from), "2" (lcnt) 			\
		  : "memory", "cc");     
    
#define SSE_CPY(prefetch,from,to,dummy,lcnt)				\
    if((unsigned long) from & 15) {					\
	__asm__ __volatile__ (						\
		"1:\n"							\
                    prefetch "320(%1)\n"				\
		  " movups (%1), %%xmm0\n"				\
		  " movups 16(%1), %%xmm1\n"				\
		  " movntps %%xmm0, (%0)\n"				\
		  " movntps %%xmm1, 16(%0)\n"				\
                    prefetch "352(%1)\n"				\
		  " movups 32(%1), %%xmm2\n"				\
		  " movups 48(%1), %%xmm3\n"				\
		  " leal 64(%1),%1\n"					\
		  " movntps %%xmm2, 32(%0)\n"				\
		  " movntps %%xmm3, 48(%0)\n"				\
		  " decl %2\n"						\
		  " leal 64(%0),%0\n"					\
		  " jne 1b\n"						\
		  : "=&D"(to), "=&S"(from), "=&r"(dummy)		\
		  : "0" (to), "1" (from), "2" (lcnt)			\
		  : "memory", "cc"); 					\
    } else {								\
	__asm__ __volatile__ (						\
	        "2:\n"							\
		    prefetch "320(%1)\n"				\
		  " movaps (%1), %%xmm0\n"				\
		  " movaps 16(%1), %%xmm1\n"				\
		  " movntps %%xmm0, (%0)\n"				\
		  " movntps %%xmm1, 16(%0)\n"				\
                    prefetch "352(%1)\n"				\
		  " movaps 32(%1), %%xmm2\n"				\
		  " movaps 48(%1), %%xmm3\n"				\
		  " leal 64(%1),%1\n"					\
		  " movntps %%xmm2, 32(%0)\n"				\
		  " movntps %%xmm3, 48(%0)\n"				\
		  " decl %2\n"						\
		  " leal 64(%0),%0\n"					\
		  " jne 2b\n"						\
		  : "=&D"(to), "=&S"(from), "=&r"(dummy)		\
		  : "0" (to), "1" (from), "2" (lcnt)			\
		  : "memory", "cc");					\
    }

#define MMXEXT_CPY(prefetch,from,to,dummy,lcnt)				\
    __asm__ __volatile__ (						\
		  ".p2align 4,,7\n"					\
		 "1:\n"							\
		    prefetch "320(%1)\n"				\
		  " movq (%1), %%mm0\n"					\
		  " movq 8(%1), %%mm1\n"				\
		  " movq 16(%1), %%mm2\n"				\
		  " movq 24(%1), %%mm3\n"				\
		  " movntq %%mm0, (%0)\n"				\
		  " movntq %%mm1, 8(%0)\n"				\
		  " movntq %%mm2, 16(%0)\n"				\
		  " movntq %%mm3, 24(%0)\n"				\
		    prefetch "352(%1)\n"				\
		  " movq 32(%1), %%mm0\n"				\
		  " movq 40(%1), %%mm1\n"				\
		  " movq 48(%1), %%mm2\n"				\
		  " movq 56(%1), %%mm3\n"				\
		  " leal 64(%1),%1\n"					\
		  " movntq %%mm0, 32(%0)\n"				\
		  " movntq %%mm1, 40(%0)\n"				\
		  " movntq %%mm2, 48(%0)\n"				\
		  " movntq %%mm3, 56(%0)\n"				\
		  " decl %2\n"						\
		  " leal 64(%0),%0\n"					\
		  " jne 1b\n"						\
		  : "=&D"(to), "=&S"(from), "=&r"(dummy)		\
		  : "0" (to), "1" (from), "2" (lcnt) 			\
		  : "memory", "cc"); 
		  

#define PREFETCH_FUNC(prefix,itype,ptype,begin,fence,small)		\
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
	   small(to, from, rest);					\
	   PREFETCH3(ptype##_PREFETCH,from);				\
	}								\
	fence;								\
    }

#define NOPREFETCH_FUNC(prefix,itype,begin,fence,small)			\
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
	   small(to, from, rest);					\
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
/* We only do all that stuff under gcc; no idea what other compilers 	*/
/* would do with our asm code.  					*/
/************************************************************************/

#ifndef __GNUC__

vidCopyFunc SiSVidCopyInit(ScreenPtr pScreen) 
{
    return SiS_libc_memcpy;
}

#else /* Everything below is gcc specific */

/************************************************************************/
/*                   Built-in memcpy() - arch specific                  */
/************************************************************************/

#ifdef __i386__   

/* Built-in memcpy for i386 */
static __inline void * __memcpy(void * to, const void * from, size_t n)
{
    int d1,d2,d3;

    __asm__ __volatile__(
    	   	 " cld\n"		 
    		 " shrl $1, %%ecx\n"
		 " jnc 1f\n"
		 " movsb\n"
	       "1: shrl $1, %%ecx\n"
		 " jnc 2f\n"
		 " movsw\n"
	       "2: rep ; movsl\n"
		 : "=&c" (d1), "=&D" (d2), "=&S" (d3)	
		 : "0" (n), "1" ((long) to), "2" ((long) from)
		 : "memory", "cc");

    return(to);
}

/* Alternative for 586: Unroll loop, copy 32 bytes at a time */
static void SiS_builtin_memcp2(unsigned char *to, const unsigned char *from, int n)
{
    int d1,d2,d3;

    __asm__ __volatile__(
		 " movl %%edi, %%eax\n"
		 " cmpl $32, %%ecx\n"
		 " cld\n"
		 " jbe 3f\n"
		 " negl %%eax\n"		/* Align dest */
		 " andl $3, %%eax\n"
		 " subl %%eax, %%ecx\n"
		 " xchgl %%eax, %%ecx\n"
		 " rep ; movsb\n"
		 " movl %%eax, %%ecx\n"
		 " subl $32, %%ecx\n"
		 " js 2f\n"
		 " movl (%%edi), %%eax\n"
	       "1: movl 28(%%edi), %%edx\n"   	/* Trick: Read-ahead */
	 	 " subl $32, %%ecx\n"
		 " movl (%%esi), %%eax\n"
		 " movl 4(%%esi), %%edx\n"
		 " movl %%eax, (%%edi)\n"
		 " movl %%edx, 4(%%edi)\n"
		 " movl 8(%%esi), %%eax\n"
		 " movl 12(%%esi), %%edx\n"
		 " movl %%eax, 8(%%edi)\n"
		 " movl %%edx, 12(%%edi)\n"
		 " movl 16(%%esi), %%eax\n"
		 " movl 20(%%esi), %%edx\n"
		 " movl %%eax, 16(%%edi)\n"
		 " movl %%edx, 20(%%edi)\n"
		 " movl 24(%%esi), %%eax\n"
		 " movl 28(%%esi), %%edx\n"
		 " movl %%eax, 24(%%edi)\n"
		 " movl %%edx, 28(%%edi)\n"
		 " leal 32(%%esi), %%esi\n"
		 " leal 32(%%edi), %%edi\n"
		 " jns 1b\n"
	       "2: addl $32, %%ecx\n"
	       "3: rep ; movsb"
		 : "=&c" (d1), "=&D" (d2), "=&S" (d3)	
		 : "0" (n), "1" ((long) to), "2" ((long) from)
		 : "eax", "edx", "memory", "cc");
		 
}

#elif defined(__AMD64__)

/* Built-in memcpy for AMD64 */
static __inline void * __memcpy(void * to, const void * from, int n)
{
    long d1, d2, d3;
    
    __asm__ __volatile__ (
    		" cld\n"
    		" rep ; movsq\n"
		" movq %4, %%rcx\n"
		" rep ; movsb"
		: "=%c" (d1), "=&D" (d2), "=&S" (d3)
		: "0" ((unsigned long)(n >> 3)), "q" ((unsigned long)(n & 7)), 
		  "1" ((long) to), "2" ((long) from)
		: "memory");

    return(to);
}

/* Alternative: Unroll loop, copy 32 bytes at a time */
static void SiS_builtin_memcp2(unsigned char *to, const unsigned char *from, int n)
{
    long d1,d2,d3;
    
    __asm__ __volatile__(
		 " movq %%rdi, %%rax\n"
		 " cmpq $32, %%rcx\n"
		 " cld\n"			/* Pipeline; no other flags but DF */
		 " jbe 1f\n"
		 " negq %%rax\n"		/* Align dest */
		 " andq $7, %%rax\n"
		 " subq %%rax, %%rcx\n"
		 " xchgq %%rax, %%rcx\n"
		 " rep ; movsb\n"
		 " movq %%rax, %%rcx\n"
		 " subq $32, %%rcx\n"
		 " js 2f\n"
		 ".p2align 4\n"
	       "3: subq $32, %%rcx\n"
		 " movq (%%rsi), %%rax\n"
		 " movq 8(%%rsi), %%rdx\n"
		 " movq 16(%%rsi), %%r8\n"
		 " movq 24(%%rsi), %%r9\n"
		 " movq %%rax, (%%rdi)\n"
		 " movq %%rdx, 8(%%rdi)\n"
		 " movq %%r8, 16(%%rdi)\n"
		 " movq %%r9, 24(%%rdi)\n"
		 " leaq 32(%%rsi), %%rsi\n"
		 " leaq 32(%%rdi), %%rdi\n"
		 " jns 3b\n"
	       "2: addq $32, %%rcx\n"
	       "1: rep ; movsb"
		 : "=&c" (d1), "=&D" (d2), "=&S" (d3)	
		 :"0" ((unsigned long) n), "1" ((long) to), "2" ((long) from)
		 : "rax", "rdx", "r8", "r9", "memory", "cc");
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

#if defined(__i386__) /* ***************************************** i386 */

#define SiS_canBenchmark	/* Can we perform a benchmark? */
#ifdef linux
#define SiS_haveProc		/* Do we have /proc/cpuinfo or similar? */
#endif

static unsigned int taketime(void)	/* get current time (for benchmarking) */
{
    unsigned int eax;
    
    __asm__ volatile (
		" cpuid\n"
		" .byte 0x0f, 0x31\n" 
		: "=a" (eax)
		: "0"(0)
		: "ebx", "ecx", "edx", "cc");     
		
    return(eax); 
}

#elif defined(__AMD64__) /*************************************** AMD64 */

#define SiS_canBenchmark	/* Can we perform a benchmark? */
#ifdef linux
#define SiS_haveProc		/* Do we have /proc/cpuinfo or similar? */
#endif

static unsigned int taketime(void)	/* get current time (for benchmarking) */
{
    unsigned int eax;
    
    __asm__ volatile (
		" cpuid\n"
		" rdtsc\n" 
		: "=a" (eax)
		: "0" (0)
		: "rbx", "rcx", "rdx", "cc");     
		
    return(eax); 
}

#else		/* **************************************** Other archs */

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
	       "Benchmarking system RAM to video RAM memory transfer methods:\n");
	       
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
       if((*cpuFreq) < 10.0) frqBuf = NULL; /* sanity check */
       if(frqBuf) {
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
   
   if(!(cpuinfo = strstr(cpuinfo, "processor\t:"))) return 0;
   if(!(flagLoc = strstr(cpuinfo, "flags\t\t:"))) return 0;
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

/**********************************************************************/
/*                      Arch-specific routines                        */
/**********************************************************************/

#ifdef __i386__   /* i386 *************************************/

PREFETCH_FUNC(SiS_sse,SSE,SSE,,FENCE,small_memcpy_i386) 
PREFETCH_FUNC(SiS_mmxext,MMXEXT,SSE,EMMS,FENCEMMS,small_memcpy_i386)
PREFETCH_FUNC(SiS_now,MMX,NOW,FEMMS,FEMMS,small_memcpy_i386)
NOPREFETCH_FUNC(SiS_mmx,MMX,EMMS,EMMS,small_memcpy_i386)

char *sse_cpuflags[]    = {" sse ", 0};
char *mmx_cpuflags[]    = {" mmx ", 0};
char *now_cpuflags[]    = {" 3dnow ", 0};
char *mmx2_cpuflags[]   = {" mmxext ", " sse ", 0};

static SISMCFuncData MCFunctions_i386[] = {
    {SiS_libc_memcpy,   "libc",      NULL,         FL_LIBC},
    {SiS_builtin_memcpy,"built-in-1",NULL,         FL_BI},    
    {SiS_builtin_memcp2,"built-in-2",NULL, 	   FL_BI2},
    {SiS_mmx_memcpy,    "MMX",       mmx_cpuflags, FL_MMX},
    {SiS_sse_memcpy,    "SSE",       sse_cpuflags, FL_SSE}, 
    {SiS_now_memcpy,    "3DNow!",    now_cpuflags, FL_3DNOW}, 
    {SiS_mmxext_memcpy, "MMX2",      mmx2_cpuflags,FL_MMX2},
    {NULL,              "",          NULL,         0}
}; 

#define Def_FL  (FL_LIBC | FL_BI | FL_BI2)  /* Default methods */

#define cpuid(op, eax, ebx, ecx, edx) 		\
    __asm__ __volatile__ (			\
    		" pushl %%ebx\n"		\
		" cpuid\n"			\
		" movl %%ebx, %1\n"		\
		" popl %%ebx\n"			\
		: "=a" (eax), "=r" (ebx), 	\
		  "=c" (ecx), "=d" (edx)	\
		: "a" (op)			\
		: "cc")			

static Bool cpuIDSupported(ScrnInfoPtr pScrn)
{
    int eax, ebx, ecx, edx;
       
    /* Check for cpuid instruction */
    __asm__ __volatile__ (
    		" pushf\n"
		" popl %0\n"
		" movl %0, %1\n"
		" xorl $0x200000, %0\n"
		" push %0\n"
		" popf\n"
		" pushf\n"
		" popl %0\n"
		: "=a" (eax), "=c" (ecx)
		:
		: "cc");
		
    if(eax == ecx) {
       xf86DrvMsg(pScrn->scrnIndex, X_PROBED, "CPU does not support CPUID instruction\n");
       return FALSE;
    }
    
    /* Check for cpuid level */
    cpuid(0x00000000, eax, ebx, ecx, edx);	
    if(!eax) {
       return FALSE;
    }
    
    /* Check for RDTSC */
    cpuid(0x00000001, eax, ebx, ecx, edx);

    if(!(edx & 0x10)) {
       xf86DrvMsg(pScrn->scrnIndex, X_PROBED, "CPU does not support RDTSC instruction\n");
       return FALSE;
    }
    
    return TRUE;
}

static int SiS_GetCpuFeatures(void)
{
    unsigned int flags = 0, eax, ebx, ecx, edx;
    Bool IsAMD;
    
    cpuid(0x00000000, eax, ebx, ecx, edx);
    
    IsAMD = (ebx == 0x68747541) && (edx == 0x69746e65) && (ecx == 0x444d4163);
    
    cpuid(0x00000001, eax, ebx, ecx, edx);
    /* MMX */
    if(edx & 0x00800000) flags |= FL_MMX;
    /* SSE, MMXEXT */
    if(edx & 0x02000000) flags |= (FL_SSE | FL_MMX2);
    /* SSE2 - don't need this one directly, set SSE instead */
    if(edx & 0x04000000) flags |= FL_SSE; 
    
    cpuid(0x80000000, eax, ebx, ecx, edx);
    if(eax >= 0x80000001) {
       cpuid(0x80000001, eax, ebx, ecx, edx);
       /* 3DNow! */
       if(edx & 0x80000000) flags |= FL_3DNOW;
       /* AMD MMXEXT */
       if(IsAMD && (edx & 0x00400000)) flags |= FL_MMX2;
    }

    return flags;
}

static Bool CheckOSforSSE(ScrnInfoPtr pScrn)
{
    int signo;
    
#ifdef SISDGBMC
    xf86DrvMsg(pScrn->scrnIndex, X_INFO, "Checking OS SSE support\n");
#endif                  
    
    /* The following does not work yet: If sig 4 is catched, that
     * idiotic signal handler just "return"s, and causes an immediate
     * second sig 4 since the "return" returns to the beginning of the
     * illegal instruction. Need to find a way to install my own signal
     * handler here, and use a setjmp/longjmp method...
     */
    xf86InterceptSignals(&signo);
    
    __asm__ __volatile__ (" xorps %xmm0, %xmm0\n");
		
    xf86InterceptSignals(NULL);
    
#ifdef SISDGBMC
    xf86DrvMsg(pScrn->scrnIndex, X_INFO, "OS SSE support signal %d\n", signo);
#endif                      
    
    if(signo == 4) {
       xf86DrvMsg(pScrn->scrnIndex, X_PROBED, 
       		"OS does not support SSE instructions\n");
    } else if(signo >= 0) {
       xf86DrvMsg(pScrn->scrnIndex, X_PROBED,
       		"Checking SSE instructions caused signal %d - this should not happen!\n",
		signo);
    }
    
    return (signo >= 0) ? FALSE : TRUE;
}

#elif defined(__AMD64__) /* AMD64 ************************* */

PREFETCH_FUNC(SiS_sse,SSE,SSE,,FENCE,small_memcpy_amd64) 

static SISMCFuncData MCFunctions_AMD64[] = {
    {SiS_libc_memcpy,   "libc",      NULL, FL_LIBC},
    {SiS_builtin_memcpy,"built-in-1",NULL, FL_BI}, 
    {SiS_builtin_memcp2,"built-in-2",NULL, FL_BI2}, 
    {SiS_sse_memcpy,    "SSE",       NULL, FL_SSE}, 
    {NULL,              "",          NULL, 0}
}; 

#define Def_FL  (FL_LIBC | FL_BI | FL_BI2 | FL_SSE)

static int SiS_GetCpuFeatures(void)
{
    return((int)(FL_SSE));
}

static Bool CheckOSforSSE(ScrnInfoPtr pScrn)
{
    int signo;
    
    xf86InterceptSignals(&signo);
    
    __asm__ __volatile__ (" xorps %xmm0, %xmm0\n");
		
    xf86InterceptSignals(NULL);
    
    if(signo == 4) {
       xf86DrvMsg(pScrn->scrnIndex, X_PROBED, 
       		"OS does not support SSE/MMXEXT instructions\n");
    } else if(signo >= 0) {
       xf86DrvMsg(pScrn->scrnIndex, X_PROBED,
       		"Checking SSE/MMXEXT instructions caused signal %d - this should not happen!\n",
		signo);
    }
    
    return (signo >= 0) ? FALSE : TRUE;
}

#else  /* Other archs ************************************* */

/* Fill in here */

#endif

/**********************************************************************/
/*     Benchmark the video copy routines and choose the fastest       */
/**********************************************************************/

#ifdef SiS_canBenchmark
static vidCopyFunc SiSVidCopyInitGen(ScreenPtr pScreen, SISMCFuncData *MCFunctions) 
{
    ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];  
    SISPtr pSiS = SISPTR(pScrn);
    FBLinearPtr tmpFbBuffer = NULL;
    char *frqBuf = NULL;
    unsigned char *buf1, *buf2, *buf3;
    double cpuFreq = 0.0;
    unsigned int myCPUflags = 0;
    int best;
#ifdef SiS_haveProc   
    char buf[CPUBUFSIZE];
#endif    
    
    /* Bail out if user disabled benchmarking or acceleration in general */
    if((!pSiS->BenchMemCpy) && (pSiS->NoAccel)) {
       return SiS_libc_memcpy;
    }

#ifdef SiS_haveProc   	
    /* Read /proc/cpuinfo into buf */
    if(SiS_ReadProc(buf, "/proc/cpuinfo")) {
       
       /* Extract CPU frequency */ 
       frqBuf = SiS_GetCPUFreq(pScrn, buf, &cpuFreq);
          
       /* Parse the CPU flags and convert them to our internal value */
       myCPUflags = SiS_ParseCPUFlags(buf, MCFunctions);
#ifdef SISDGBMC
       xf86DrvMsg(pScrn->scrnIndex, X_PROBED, "CPU flags from /proc: 0x%x\n", myCPUflags);
#endif       
       
    }   
#endif       
      
    if(!myCPUflags) {
     
       /* If no /proc or parsing failed, get features from cpuid */
       myCPUflags = SiS_GetCpuFeatures() | Def_FL;
#ifdef SISDGBMC
       xf86DrvMsg(pScrn->scrnIndex, X_PROBED, "CPU flags from cpuid: 0x%x\n", myCPUflags);
#endif              
    
    }
    
    if(myCPUflags & FL_SSE) {
    
       /* Check if OS supports usage of SSE instructions */
       if(!(CheckOSforSSE(pScrn))) {
          myCPUflags &= ~(FL_SSE);
       }
       
    }
	
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

/**********************************************************************/
/*                       main(): SiSVidCopyInit()                     */
/**********************************************************************/

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

#elif defined(__AMD64__)

vidCopyFunc SiSVidCopyInit(ScreenPtr pScreen)
{   
    return(SiSVidCopyInitGen(pScreen, MCFunctions_AMD64));
}

#else /* Other archs: For now, use libc memcpy() */

vidCopyFunc SiSVidCopyInit(ScreenPtr pScreen) 
{
    return SiS_libc_memcpy;
}

#endif

/**********************************************************************/
/*                   No benchmark: Return libc memcpy()               */
/**********************************************************************/

#else   /* cenBenchmark */
 
vidCopyFunc SiSVidCopyInit(ScreenPtr pScreen) 
{
    return SiS_libc_memcpy;
}

#endif  /* cenBenchmark */

#endif /* GNU C */
