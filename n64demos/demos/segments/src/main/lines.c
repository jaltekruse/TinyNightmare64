#define static
/**************************************************************************
 *                                                                        *
 *               Copyright (C) 1995, Silicon Graphics, Inc.               *
 *                                                                        *
 *  These coded instructions, statements, and computer programs  contain  *
 *  unpublished  proprietary  information of Silicon Graphics, Inc., and  *
 *  are protected by Federal copyright  law.  They  may not be disclosed  *
 *  to  third  parties  or copied or duplicated in any form, in whole or  *
 *  in part, without the prior written consent of Silicon Graphics, Inc.  *
 *                                                                        *
 *************************************************************************/

/*---------------------------------------------------------------------*
        Copyright (C) 1997,1998 Nintendo. (Originated by SGI)
        
        $RCSfile: lines.c,v $
        $Revision: 1.13 $
        $Date: 1999/04/16 09:25:25 $
 *---------------------------------------------------------------------*/

/*
 * File:	lines.c
 * Create Date:	Thu Jun 22 09:28:01 PDT 1995
 *
 *
 */

#include <ultra64.h>
#include <PR/ramrom.h>	/* needed for argument passing into the app */
#include <assert.h>

#include "lines.h"
#include "static.h"

#define F3DEX2 0
#define L3DEX2 1

/*
 * Symbol genererated by "makerom" to indicate the end of the code segment
 * in virtual (and physical) memory
 */
extern char _codeSegmentBssEnd[];

/*
 * Symbols generated by "makerom" to tell us where the static segment is
 * in ROM.
 */

/*
 * Stacks for the threads as well as message queues for synchronization
 * This stack is ridiculously large, and could also be reclaimed once
 * the main thread is started.
 */
u64	bootStack[STACKSIZE/sizeof(u64)];

static void	idle(void *);
static void	mainproc(void *);

static OSThread	idleThread;
static u64	idleThreadStack[STACKSIZE/sizeof(u64)];

static OSThread	mainThread;
static u64	mainThreadStack[STACKSIZE/sizeof(u64)];

/* this number (the depth of the message queue) needs to be equal
 * to the maximum number of possible overlapping PI requests.
 * For this app, 1 or 2 is probably plenty, other apps might
 * require a lot more.
 */
#define NUM_PI_MSGS     8

static OSMesg PiMessages[NUM_PI_MSGS];
static OSMesgQueue PiMessageQ;

OSMesgQueue     dmaMessageQ, rdpMessageQ, retraceMessageQ;
OSMesg          dmaMessageBuf, rdpMessageBuf, retraceMessageBuf;
OSMesg          dummyMessage;
OSIoMesg        dmaIOMessageBuf;

OSMesg		dummyMesg;
OSTask		*tlistp;
Dynamic		*dynamicp;


/*
 * Dynamic data.
 */
Dynamic dynamic;

/*
 * Task descriptor.
 */
OSTask	tlist =
{
    M_GFXTASK,			/* task type */
    OS_TASK_DP_WAIT,		/* task flags */
    NULL,			/* boot ucode pointer (fill in later) */
    0,				/* boot ucode size (fill in later) */
    NULL,			/* task ucode pointer (fill in later) */
    SP_UCODE_SIZE,		/* task ucode size */
    NULL,			/* task ucode data pointer (fill in later) */
    SP_UCODE_DATA_SIZE,		/* task ucode data size */
    &dram_stack[0],		/* task dram stack pointer */
    SP_DRAM_STACK_SIZE8,	/* task dram stack size */
    NULL,                       /* task fifo buffer start ptr */
    NULL,                       /* task fifo buffer end ptr */
    NULL,			/* task data pointer (fill in later) */
    0,				/* task data size (fill in later) */
    NULL,			/* task yield buffer ptr (not used here) */
    0				/* task yield buffer size (not used here) */
};

Gfx		*glistp;	/* global for test case procs */
    
/*
 * global variables
 */
static int      draw_buffer = 0;
static int      FrameRate   = 1;

float  theta                = 0;
float  TranslateHorizontal  = 0;
float  TranslateVertical    = 0;

void    *cfb_ptrs[2];

OSPiHandle	*handler;

void
boot(void)
{

    osInitialize();

    handler = osCartRomInit();

    osCreateThread(&idleThread, 1, idle, (void *)0,
		   idleThreadStack+STACKSIZE/sizeof(u64), 10);
    osStartThread(&idleThread);

    /* never reached */
}

static void
idle(void *arg)
{
    /* Initialize video */
    osCreateViManager(OS_PRIORITY_VIMGR);
    osViSetMode(&osViModeTable[OS_VI_NTSC_LAN1]);
    
    /*
     * Start PI Mgr for access to cartridge
     */
    osCreatePiManager((OSPri)OS_PRIORITY_PIMGR, &PiMessageQ, PiMessages, 
		      NUM_PI_MSGS);
    
    /*
     * Create main thread
     */
    osCreateThread(&mainThread, 3, mainproc, (void *)0,
		   mainThreadStack+STACKSIZE/sizeof(u64), 10);
    osStartThread(&mainThread);

    /*
     * Become the idle thread
     */
    osSetThreadPri(0, 0);

    for (;;);
}

void CreateMessageQueues(void)
{
  osCreateMesgQueue(&dmaMessageQ, &dmaMessageBuf, 1);
  
  osCreateMesgQueue(&rdpMessageQ, &rdpMessageBuf, 1);
  osSetEventMesg(OS_EVENT_DP, &rdpMessageQ, dummyMessage);
  
  osCreateMesgQueue(&retraceMessageQ, &retraceMessageBuf, 1);
  osViSetEvent(&retraceMessageQ, dummyMessage, FrameRate);  
}

void SetupSegments(void)
{
  /* Tell RCP where each segment is */

  gSPSegment(glistp++, 0, 0x0);   
}

void CreateTaskStructure(int type)
{
  /* Build graphics task */

  tlistp->t.ucode_boot = (u64 *) rspbootTextStart;
  tlistp->t.ucode_boot_size = (u32)rspbootTextEnd - (u32)rspbootTextStart;

  /* choose which ucode to run */
  /* RSP output over XBUS to RDP: */
  if (type == F3DEX2)
    {
      tlistp->t.ucode      = (u64 *) gspF3DEX2_xbusTextStart;
      tlistp->t.ucode_data = (u64 *) gspF3DEX2_xbusDataStart;
    }
  else
    {
      tlistp->t.ucode      = (u64 *) gspL3DEX2_xbusTextStart;
      tlistp->t.ucode_data = (u64 *) gspL3DEX2_xbusDataStart;
    }
  
  /* initial display list: */
  tlistp->t.data_ptr = (u64 *) dynamicp->glist;
}

static void SwapViBuffer(void)
{
  /* setup to swap buffers */
  osViSwapBuffer(cfb_ptrs[draw_buffer]); 
  
  /* Make sure there isn't an old retrace in queue */
  if (MQ_IS_FULL(&retraceMessageQ))
    (void)osRecvMesg(&retraceMessageQ, NULL, OS_MESG_BLOCK);
  
  /* Wait for Vertical retrace to finish swap buffers */
  (void)osRecvMesg(&retraceMessageQ, NULL, OS_MESG_BLOCK);
  draw_buffer ^= 1;
}

static void IdentMatrices(void)
{
  guRotate(&dynamicp->modeling1, 0, 0.0F, 0.0F, 1.0F); 

  gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&(dynamic.modeling1)),
	    G_MTX_MODELVIEW|G_MTX_LOAD|G_MTX_NOPUSH);

}

static void SetupViewing(void)
{
  u16 perspNorm;

#if 0
  guOrtho(&dynamicp->projection,
	  0.00, SCREEN_WD-1,
	  0.00, SCREEN_HT-1, 
	  -200, 200, 
	  1.0);

  gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&(dynamic.projection)),
	    G_MTX_PROJECTION|G_MTX_LOAD|G_MTX_NOPUSH);        
#else

  guPerspective(&dynamicp->projection,&perspNorm,
		33, 320.0/240.0, 1, 1000, 1.0);

  guLookAt(&dynamicp->viewing, 
	   160, 120, 400,
	   160, 120,   0,
	   0, 1, 0);

  gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&(dynamicp->projection)),
	    G_MTX_PROJECTION|G_MTX_LOAD|G_MTX_NOPUSH);

  gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&(dynamicp->viewing)),
	    G_MTX_PROJECTION|G_MTX_MUL|G_MTX_NOPUSH);  

  gSPPerspNormalize(glistp++, perspNorm);
  
#endif

}

static void SetupMatrices(void)
{
  guTranslate(&dynamicp->modeling2,  160.0, 120.0, 0.0);
  gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&(dynamic.modeling2)),
	    G_MTX_MODELVIEW|G_MTX_LOAD|G_MTX_NOPUSH);

  guRotate(&dynamicp->modeling3, theta, 1.0, 1.0, 0.0);  
  gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&(dynamic.modeling3)),
	    G_MTX_MODELVIEW|G_MTX_MUL|G_MTX_NOPUSH);

  guTranslate(&dynamicp->modeling4, 
	      -160.0 + TranslateHorizontal, 
	      -120.0 + TranslateVertical, 
	      0.0); 

  gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&(dynamic.modeling4)),
	    G_MTX_MODELVIEW|G_MTX_MUL|G_MTX_NOPUSH);

  theta += 1.0F;
}


static void doPolyFuncs(void)
{
  /* Setup display modes antialiased in 1 cycle */

  gDPSetRenderMode(glistp++, 
		   G_RM_ZB_OPA_SURF,
		   G_RM_ZB_OPA_SURF2);
  
  SetupViewing();
  IdentMatrices(); 
  gSPDisplayList(glistp++, background_dl);
}

static void doLineFuncs(void)
{
  /* Setup display modes antialiased in 1 cycle */

  gDPSetRenderMode(glistp++, 
		   G_RM_AA_ZB_XLU_LINE  /* | Z_UPD /* */, 
		   G_RM_AA_ZB_XLU_LINE2 /* | Z_UPD /* */);

  SetupViewing();
  IdentMatrices();
  gSPDisplayList(glistp++, border1); 
  SetupMatrices();
  gSPDisplayList(glistp++, sphere);
}

static void InitDisplayLists(void)
{
  /* pointers to build the display list */
  glistp = dynamicp->glist;
  
  SetupSegments();
  
  /* Initialize RDP state */
  gSPDisplayList(glistp++, rdpinit_dl);
  
  /* Initialize RSP state */
  gSPDisplayList(glistp++, rspinit_dl);  
}

static void ClearFrameBuffer(void)
{
  /* Clear color framebuffer */

  gDPSetCycleType(glistp++, G_CYC_FILL);
  gDPSetColorImage(glistp++, G_IM_FMT_RGBA, G_IM_SIZ_16b, SCREEN_WD, 
		    OS_K0_TO_PHYSICAL(cfb_ptrs[draw_buffer]));

  gDPSetFillColor(glistp++, GPACK_RGBA5551(64,64,64,1) << 16 | 
		   GPACK_RGBA5551(64,64,64,1));

  gDPFillRectangle(glistp++, 0, 0, SCREEN_WD-1, SCREEN_HT-1);
  gDPPipeSync(glistp++);

  gDPSetFillColor(glistp++, GPACK_RGBA5551(64,64,255,1) << 16 | 
		   GPACK_RGBA5551(64,64,255,1));

  gDPFillRectangle(glistp++, 20, 20, SCREEN_WD-20, SCREEN_HT-20);
  gDPPipeSync(glistp++);

  gDPSetCycleType(glistp++, G_CYC_1CYCLE);  
}

static void ClearZBuffer (void)
{
  gDPSetColorImage(glistp++, G_IM_FMT_RGBA, G_IM_SIZ_16b, 
		   SCREEN_WD, OS_K0_TO_PHYSICAL(zbuffer));

  gDPPipeSync(glistp++);
  gDPSetCycleType(glistp++, G_CYC_FILL);

  gDPSetFillColor(glistp++, 
		  GPACK_ZDZ(G_MAXFBZ, 0) << 16 | GPACK_ZDZ(G_MAXFBZ, 0));

  gDPFillRectangle(glistp++, 0, 0, SCREEN_WD-1, SCREEN_HT-1);

  gDPPipeSync(glistp++);
}

/* stupid procedure name... */
static void CleanupAndSendDisplayList(int MicrocodeType)
{
  gDPFullSync(glistp++);
  gSPEndDisplayList(glistp++);

#ifdef DEBUG	
#ifndef __MWERKS__
  assert((glistp-dynamicp->glist) < GLIST_LEN);
#endif
#endif

  tlistp->t.data_size = (u32)((glistp - dynamicp->glist) * sizeof(Gfx));

  /* Write back dirty cache lines that need to be read by the RCP */
  osWritebackDCache(&dynamic, sizeof(dynamic));
	
  /* start up the RSP task */
  CreateTaskStructure(MicrocodeType);
  osSpTaskStart(tlistp);
	
  /* wait for RDP completion */
  (void)osRecvMesg(&rdpMessageQ, NULL, OS_MESG_BLOCK);
}


static void doLinePoly(void)
{
  /* Main game loop */
  while (1) 
    {
      /* Do poly stuff */
      InitDisplayLists();
      ClearZBuffer(); 
      ClearFrameBuffer();

      doPolyFuncs(); 
      CleanupAndSendDisplayList(F3DEX2);	

      /* Do line stuff */
      InitDisplayLists();
      doLineFuncs(); 
      CleanupAndSendDisplayList(L3DEX2);		

      SwapViBuffer();
    }
}


static void mainproc(void *arg)
{
  tlistp = &tlist;
  dynamicp = &dynamic;
  cfb_ptrs[0] = cfb_16_a; 
  cfb_ptrs[1] = cfb_16_b; 

  CreateMessageQueues();
  doLinePoly();
}
