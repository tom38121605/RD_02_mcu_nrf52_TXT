/*********************************************************************
*                   (c) SEGGER Microcontroller GmbH                  *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*       (c) 2003 - 2024     SEGGER Microcontroller GmbH              *
*                                                                    *
*       www.segger.com     Support: www.segger.com/ticket            *
*                                                                    *
**********************************************************************
*                                                                    *
*       emUSB-Device * USB Device stack for embedded applications    *
*                                                                    *
*       Please note: Knowledge of this file may under no             *
*       circumstances be used to write a similar product.            *
*       Thank you for your fairness !                                *
*                                                                    *
**********************************************************************
*                                                                    *
*       emUSB-Device version: V3.64.0                                *
*                                                                    *
**********************************************************************
----------------------------------------------------------------------
Licensing information
Licensor:                 SEGGER Microcontroller Systems LLC
Licensed to:              Ommo Technologies Inc., 3330 Keller Springs Rd. Ste. 205, Carrollton, TX 75006, USA
Licensed SEGGER software: emUSB-Device
License number:           USBD-00543
License model:            Time Limited Evaluation Agreement, V200225, signed 06.03.2024 by Minjie Zheng
Evaluation Period:        11.03.2024 - 11.05.2024
----------------------------------------------------------------------
Purpose : Target USB driver for Nordic Semi devices
-------------------------- END-OF-HEADER -----------------------------
*/

#define USB_HW_NRF_C_  //--add test
#ifdef USB_HW_NRF_C_
#include "USB_Private.h"
   
/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

#define EVENTCAUSE_SUSPEND          (1uL << 8)
#define EVENTCAUSE_RESUME           (1uL << 9)
#define EVENTCAUSE_READY            (1uL << 11)

#define INT_USBRESET                (1uL << 0)
#define INT_STARTED                 (1uL << 1)
#define INT_EP0DATADONE             (1uL << 10)
#define INT_SOF                     (1uL << 21)
#define INT_USBEVENT                (1uL << 22)
#define INT_EP0SETUP                (1uL << 23)
#define INT_EPDATA                  (1uL << 24)

#define VBUSDETECT                  (1uL << 0)
#define OUTPUTRDY                   (1uL << 1)

/****** Number of physical endpoints ********************************/
#define NUM_PHY_EP             8u

#define EP0_MAX_PACKET_SIZE   64u

/*********************************************************************
*
*       Types
*
**********************************************************************
*/
//lint -esym(754, USB_REGS::*, _EasyDMAIN::*, _EasyDMAOUT::*)
typedef struct {
           U32   Reserved0a;
  volatile U32   TASKS_STARTEPIN[9];
  volatile U32   TASKS_STARTEPOUT[9];
  volatile U32   TASKS_EP0RCVOUT;
  volatile U32   TASKS_EP0STATUS;
  volatile U32   TASKS_EP0STALL;
           U32   Reserved000[42];
  // Offset 0x100
  volatile U32   EVENTS_USBRESET;
  volatile U32   EVENTS_STARTED;
  volatile U32   EVENTS_ENDEPIN[8];
  volatile U32   EVENTS_EP0DATADONE;
  volatile U32   EVENTS_ENDISOIN;
  volatile U32   EVENTS_ENDEPOUT[8];
  volatile U32   EVENTS_ENDISOOUT;
  volatile U32   EVENTS_SOF;
  volatile U32   EVENTS_USBEVENT;
  volatile U32   EVENTS_EP0SETUP;
  volatile U32   EVENTS_EPDATA;
           U32   Reserved100[39];
  // Offset 0x200
  volatile U32   SHORTS;
           U32   Reserved200[63];
  // Offset 0x300
  volatile U32   INTEN;
  volatile U32   INTENSET;
  volatile U32   INTENCLR;
           U32   Reserved300[61];
  // Offset 0x400
  volatile U32   EVENTCAUSE;
           U32   Reserved4a[7];
  volatile U32   HALTED_EPIN[9];
  volatile U32   HALTED_EPOUT[9];
  volatile U32   EPSTATUS;
  volatile U32   EPDATASTATUS;
  volatile U32   USBADDR;
           U32   Reserved4b[3];
  volatile U32   BMREQUESTTYPE;
  volatile U32   BMREQUEST;
  volatile U32   WVALUEL;
  volatile U32   WVALUEH;
  volatile U32   WINDEXL;
  volatile U32   WINDEXH;
  volatile U32   WLENGTHL;
  volatile U32   WLENGTHH;
  volatile U32   SIZE_EPOUT[9];
           U32   Reserved400[15];
  // Offset 0x500
  volatile U32   ENABLE;
  volatile U32   USBPULLUP;
  volatile U32   DPDMVALUE;
  volatile U32   DTOGGLE;
  volatile U32   EPINEN;
  volatile U32   EPOUTEN;
  volatile U32   EPSTALL;
  volatile U32   ISOSPLIT;
  volatile U32   FRAMECNTR;
           U32   Reserved5a[2];
  volatile U32   LOWPOWER;
  volatile U32   ISOINCONFIG;
           U32   Reserved500[51];
  // Offset 0x600
  struct _EasyDMAIN {
    volatile U32 PTR;
    volatile U32 MAXCNT;
    volatile U32 AMOUNT;
             U32 Reserved[2];
   } EPIN[9];
           U32   Reserved600[19];
  // Offset 0x700
  struct _EasyDMAOUT {
    volatile U32 PTR;
    volatile U32 MAXCNT;
    volatile U32 AMOUNT;
             U32 Reserved[2];
   } EPOUT[9];
           U32   Reserved700[19];
  // Offset 0x800
  volatile U32   SecretMagicRegs[2];
} USB_REGS;

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
static          U8 _InUseEPIN;
static          U8 _InUseEPOUT;
static volatile I8 _SetupActive;              // 1: Setup read, -1: Setup write
static volatile I8 _EPIN_Busy[NUM_PHY_EP];
static volatile I8 _EPIN_Enabled[NUM_PHY_EP];
static volatile I8 _EPOUT_Enabled[NUM_PHY_EP];
static volatile I8 _EPOUT_DataPresent[NUM_PHY_EP];
static          U8 _ExtraPacket_NumBytes;
static          U8 _ExtraPacket_EPIndex;

static         U32 _PacketBuff[64 / 4];

#ifdef USB_BASE_ADDR
static USB_REGS  * _pUSBRegs = (USB_REGS *)USB_BASE_ADDR;       //lint !e923 !e9078  D:103[a]
#else
static USB_REGS  * _pUSBRegs;
#endif

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

/*********************************************************************
*
*       _IsInRAM
*
*  Function description
*    Check if given pointer contain address of internal SRAM
*/
static int _IsInRAM(const void *p) {
  if ((SEGGER_PTR2ADDR(p) & 0xE0000000u) == 0x20000000u) {         // lint D:103[c]
    return 1;
  }
  return 0;
}

/*********************************************************************
*
*       _WaitEvent
*
*/
static void _WaitEvent(volatile const U32 *pEvent) {
  int i;

  //
  // Only one DMA channel can be active at the same time.
  // While DMA is in progress, USB registers can not be accessed.
  // A maximum of 16 words are transferred by DMA.
  // For these reasons, we perform a busy wait loop until DMA finishes.
  //
  for (i = 0; ++i < 500;) {
    if (*pEvent != 0u) {
      return;
    }
  }
  USB_WARN((USB_MTYPE_DRIVER, "Timeout waiting for EasyDMA"));
}

/*********************************************************************
*
*       _ReadPacket
*
*/
static void _ReadPacket(unsigned PhyEP, unsigned EPIndex) {
  unsigned    NumBytesReceived;
  USB_REGS  * pUSBRegs = _pUSBRegs;
  void      * pData;

  NumBytesReceived = pUSBRegs->SIZE_EPOUT[PhyEP] & 0x7Fu;
  pUSBRegs->EVENTS_ENDEPOUT[PhyEP]  = 0;
  pData = USB__GetpDest(EPIndex, NumBytesReceived);
  if (pData != NULL) {
    pUSBRegs->EPOUT[PhyEP].PTR      = SEGGER_PTR2ADDR(pData);         // lint D:103[a]
  } else {
    pUSBRegs->EPOUT[PhyEP].PTR      = SEGGER_PTR2ADDR(_PacketBuff);   // lint D:103[a]
  }
  pUSBRegs->EPOUT[PhyEP].MAXCNT     = NumBytesReceived;
  pUSBRegs->TASKS_STARTEPOUT[PhyEP] = 1;
  _WaitEvent(&pUSBRegs->EVENTS_ENDEPOUT[PhyEP]);
  if (pData != NULL) {
    USB__OnRxZeroCopy(EPIndex, NumBytesReceived);
  } else {
    USB__OnRx(EPIndex, (U8 *)_PacketBuff, NumBytesReceived);
  }
}

/*********************************************************************
*
*       _ReadPacketEP0
*
*/
static void _ReadPacketEP0(void) {
  unsigned    NumBytesReceived;
  USB_REGS  * pUSBRegs = _pUSBRegs;

  NumBytesReceived = pUSBRegs->SIZE_EPOUT[0] & 0x7Fu;
  pUSBRegs->EVENTS_ENDEPOUT[0]  = 0;
  pUSBRegs->EPOUT[0].PTR        = SEGGER_PTR2ADDR(_PacketBuff);       // lint D:103[a]
  pUSBRegs->EPOUT[0].MAXCNT     = NumBytesReceived;
  pUSBRegs->TASKS_STARTEPOUT[0] = 1;
  _WaitEvent(&pUSBRegs->EVENTS_ENDEPOUT[0]);
  USB__OnRx(0, (U8 *)_PacketBuff, NumBytesReceived);
}

/*********************************************************************
*
*       _DisableRxInterruptEP
*
*/
static void _DisableRxInterruptEP(unsigned EPIndex) {
  unsigned PhyEP;

  PhyEP = USB__EPIndex2Addr(EPIndex) & 0x7FuL;
  _EPOUT_Enabled[PhyEP] = 0;
  if (EPIndex == 0u && _SetupActive < 0) {
    //
    // Issue status stage
    //
    _pUSBRegs->TASKS_EP0STATUS = 1;
    _SetupActive = 0;
  }
}

/*********************************************************************
*
*       _EnableRxInterruptEP
*
*/
static void _EnableRxInterruptEP(unsigned EPIndex, U8 *pData, U32 NumBytesRequested) {
  unsigned    PhyEP;
  unsigned    NumBytesReceived;
  USB_REGS  * pUSBRegs = _pUSBRegs;

  USB_USE_PARA(NumBytesRequested);
  USB_USE_PARA(pData);
  PhyEP = USB__EPIndex2Addr(EPIndex) & 0x7FuL;
  _EPOUT_Enabled[PhyEP] = 1;
  if (_EPOUT_DataPresent[PhyEP] != 0) {
    //
    // USB controller already contains a received packet.
    //
    NumBytesReceived = pUSBRegs->SIZE_EPOUT[PhyEP] & 0x7Fu;
    pUSBRegs->EVENTS_ENDEPOUT[PhyEP]  = 0;
    pUSBRegs->EPOUT[PhyEP].PTR        = SEGGER_PTR2ADDR(_PacketBuff);     // lint D:103[a]
    pUSBRegs->EPOUT[PhyEP].MAXCNT     = NumBytesReceived;
    //
    // Force trigger of interrupt to hande extra packet in IRQHandler
    //
    pUSBRegs->EVENTS_STARTED          = 0;
    pUSBRegs->INTENSET                = INT_STARTED;
    pUSBRegs->TASKS_STARTEPOUT[PhyEP] = 1;
    //
    // Store as extra packet.
    //
    _ExtraPacket_NumBytes             = NumBytesReceived;
    _ExtraPacket_EPIndex              = EPIndex;
    _WaitEvent(&pUSBRegs->EVENTS_ENDEPOUT[PhyEP]);
    _EPOUT_DataPresent[PhyEP] = 0;
  }
}

/*********************************************************************
*
*       _IRQHandler
*
*  Function description
*    USB interrupt handler.
*/
static void _IRQHandler(void) {
  U32        Status;
  unsigned   i;
  unsigned   EPIndex;
  USB_REGS * pUSBRegs = _pUSBRegs;
  USB_SETUP_PACKET SetupPacket;

  USBD_ENTER_ISR();
  if ((USBREGSTATUS & OUTPUTRDY) == 0u) {
    //
    // PHY is powered down. Ignore all interrupts.
    //
    pUSBRegs->EVENTS_USBEVENT = 0;
    pUSBRegs->EVENTS_USBRESET = 0;
    pUSBRegs->EVENTCAUSE = pUSBRegs->EVENTCAUSE;
    USB__OnStatusChange(USB_STAT_ATTACHED);
    USBD_LEAVE_ISR();
    return;
  }
  if (pUSBRegs->EVENTS_USBRESET != 0u) {
    pUSBRegs->EVENTS_USBRESET = 0;
    pUSBRegs->EPDATASTATUS    = pUSBRegs->EPDATASTATUS;
    pUSBRegs->EVENTS_EPDATA   = 0;
    pUSBRegs->EPINEN          = 0;
    pUSBRegs->EPOUTEN         = 0;
    USB__OnBusReset();
    _SetupActive = 0;
  }
  if (pUSBRegs->EVENTS_USBEVENT != 0u) {
    pUSBRegs->EVENTS_USBEVENT = 0;
    Status = pUSBRegs->EVENTCAUSE;
    pUSBRegs->EVENTCAUSE = Status;              // Clear all bits
    if ((Status & EVENTCAUSE_SUSPEND) != 0u) {
      USB__OnSuspend();
    }
    if ((Status & EVENTCAUSE_RESUME) != 0u) {
      USB__OnResume();
    }
  }
  if (pUSBRegs->EVENTS_EP0SETUP != 0u) {
    pUSBRegs->EVENTS_EP0SETUP = 0;
    SetupPacket.bmRequestType = pUSBRegs->BMREQUESTTYPE;
    SetupPacket.bRequest      = pUSBRegs->BMREQUEST;
    SetupPacket.wValueLow     = pUSBRegs->WVALUEL;
    SetupPacket.wValueHigh    = pUSBRegs->WVALUEH;
    SetupPacket.wIndexLow     = pUSBRegs->WINDEXL;
    SetupPacket.wIndexHigh    = pUSBRegs->WINDEXH;
    SetupPacket.wLengthLow    = pUSBRegs->WLENGTHL;
    SetupPacket.wLengthHigh   = pUSBRegs->WLENGTHH;
    pUSBRegs->EVENTS_EP0DATADONE = 0;
    pUSBRegs->INTENSET           = INT_EP0DATADONE;
    if ((SetupPacket.bmRequestType & 0x80u) == 0u) {
      _SetupActive = -1;
    } else {
      _SetupActive = 1;
    }
    _EPIN_Busy[0] = 0;
    USB__HandleSetup(&SetupPacket);
    if (_SetupActive < 0) {
      //
      // Issue EP0 read
      //
      pUSBRegs->TASKS_EP0RCVOUT = 1;
    }
  }
  //
  // Handle extra packet
  //
  EPIndex = _ExtraPacket_EPIndex;
  if (EPIndex != 0u) {
    _ExtraPacket_EPIndex = 0;
    USB__OnRx(EPIndex, (U8 *)_PacketBuff, _ExtraPacket_NumBytes);
  }
  pUSBRegs->EVENTS_STARTED = 0;
  pUSBRegs->INTENCLR       = INT_STARTED;
  //
  // EP0-related interrupt occurred
  //
  if (pUSBRegs->EVENTS_EP0DATADONE != 0u) {
    pUSBRegs->EVENTS_EP0DATADONE = 0;
    if (_SetupActive > 0) {
      _EPIN_Busy[0] = 0;
      USB__OnTx(0);
    }
    if (_SetupActive < 0) {
      _ReadPacketEP0();
      if (_SetupActive < 0) {
        //
        // Issue next EP0 read
        //
        pUSBRegs->TASKS_EP0RCVOUT = 1;
      }
    }
  }
  //
  // Any other EPx-related interrupt occurred
  //
  pUSBRegs->EVENTS_EPDATA = 0;
  Status = pUSBRegs->EPDATASTATUS;
  if (Status != 0u) {
    pUSBRegs->EPDATASTATUS = Status;              // Clear all bits
    for (i = 1; i < NUM_PHY_EP; i++) {
      Status >>= 1;
      if ((Status & 1u) != 0u) {
        _EPIN_Busy[i] = 0;
        if (_EPIN_Enabled[i] != 0) {
          EPIndex = USB__EPAddr2Index(i | 0x80u) ;
          USB__OnTx(EPIndex);
        }
      }
    }
    Status >>= (18u - NUM_PHY_EP);                // Shift to EPOUT bits
    for (i = 1; i < NUM_PHY_EP; i++) {
      if ((Status & 1u) != 0u) {
        if (_EPOUT_Enabled[i] != 0) {
          EPIndex = USB__EPAddr2Index(i) ;
          _ReadPacket(i, EPIndex);
        } else {
          _EPOUT_DataPresent[i] = 1;
        }
      }
      Status >>= 1;
    }
  }
  USBD_LEAVE_ISR();
}

/*********************************************************************
*
*       _SendEPEx
*
*  Function description
*    Send data through the dedicated EPIndex.
*
*  Parameters
*    EPIndex            - EP index number.
*    NumDataParts       - Number of data parts given via pParts. Must always >= 1.
*    pParts             - Pointer to array of USB_DATA_PART.
*    pNumOfFullPackets  - Number of continuous full packets send.
*/
static int _SendEPEx(unsigned EPIndex, unsigned NumDataParts, const USB_DATA_PART *pParts, unsigned *pNumOfFullPackets) {
  unsigned     PhyEP;
  unsigned     MaxPacketSize;
  U32          NumBytes;
  const void * pData;
  USB_REGS   * pUSBRegs = _pUSBRegs;

  USB_USE_PARA(NumDataParts);
  PhyEP = USB__EPIndex2Addr(EPIndex) & 0x7FuL;
  if (_EPIN_Busy[PhyEP] != 0) {
    return -1;
  }
  NumBytes = pParts->NumBytesRem;
  MaxPacketSize = USB_aEPStat[EPIndex].MaxPacketSize;
  if (NumBytes >= MaxPacketSize) {
    NumBytes = MaxPacketSize;
    (*pNumOfFullPackets)++;
  } else {
    *pNumOfFullPackets = 0;
  }
  if (PhyEP == 0u && _SetupActive < 0) {
    //
    // Issue status stage
    //
    _pUSBRegs->TASKS_EP0STATUS = 1;
    _SetupActive = 0;
    return 0;
  }
  _EPIN_Busy[PhyEP] = 1;
  if (NumBytes == 0u) {
    pData = _PacketBuff;
  } else {
    pData = pParts->pData;
    if (_IsInRAM(pData) == 0) {
      USB_MEMCPY(_PacketBuff, pData, NumBytes);
      pData = _PacketBuff;
    }
  }
  pUSBRegs->EVENTS_ENDEPIN[PhyEP]  = 0;
  pUSBRegs->EPIN[PhyEP].PTR        = SEGGER_PTR2ADDR(pData);      // lint D:103[a]
  pUSBRegs->EPIN[PhyEP].MAXCNT     = NumBytes;
  pUSBRegs->TASKS_STARTEPIN[PhyEP] = 1;
  _WaitEvent(&pUSBRegs->EVENTS_ENDEPIN[PhyEP]);
  return NumBytes;
}

/*********************************************************************
*
*       _StallEP0
*/
static void _StallEP0(void) {
  _pUSBRegs->TASKS_EP0STALL = 1;
  _SetupActive = 0;
}

/*********************************************************************
*
*       _SetClrStallEP
*/
static void _SetClrStallEP(unsigned EPIndex, int OnOff) {
  unsigned Addr;

  Addr  = USB__EPIndex2Addr(EPIndex);
  if (OnOff != 0) {
    Addr |= 0x100u;
  }
  _pUSBRegs->EPSTALL = Addr;
  if ((Addr & 0x80u) == 0u) {
    _pUSBRegs->SIZE_EPOUT[Addr] = 0;
    _ExtraPacket_EPIndex        = 0;
  }
}

/*********************************************************************
*
*       _SetAddress
*/
static void _SetAddress(unsigned char Addr) {
  USB_USE_PARA(Addr);
}

/*********************************************************************
*
*       _Attach
*/
static void _Attach(void) {
  USB_Global.pfAttach();
  _pUSBRegs->USBPULLUP = 1;
}

/*********************************************************************
*
*       _Detach
*
*  Function description
*    Detach the device from USB. The device will then be
*    removed from the USB host.
*
*/
static int _Detach(void) {
  _pUSBRegs->USBPULLUP = 0;
  return 0;
}

/*********************************************************************
*
*       _AllocEP
*/
static U8 _AllocEP(U8 InDir, U8 TransferType) {
  unsigned   i;
  unsigned   EPMask;

  (void)TransferType;
  if (InDir != 0u) {
    EPMask = _InUseEPIN;
  } else {
    EPMask = _InUseEPOUT;
  }
  for (i = 1; i < NUM_PHY_EP; i++) {
    if ((EPMask & (1uL << i)) == 0u) {
      //
      // Found an EP
      //
      EPMask |= (1uL << i);
      if (InDir != 0u) {
        _InUseEPIN  = EPMask;
      } else {
        _InUseEPOUT = EPMask;
      }
      return i;
    }
  }
  return 0xFFu;
}

/*********************************************************************
*
*       _UpdateEP
*/
static void _UpdateEP(EP_STAT * pEPStat) USBD_API_USE {
  unsigned PhyEP;

  PhyEP = pEPStat->EPAddr & 0x7FuL;
  if ((pEPStat->EPAddr & 0x80u) != 0u) {
    _pUSBRegs->EPINEN   |= (1uL << PhyEP);
    _EPIN_Busy[PhyEP]    = 0;
    _EPIN_Enabled[PhyEP] = 0;
  } else {
    _pUSBRegs->EPOUTEN |= (1uL << PhyEP);
    _pUSBRegs->SIZE_EPOUT[PhyEP] = 0;
    _EPOUT_Enabled[PhyEP] = 0;
    _ExtraPacket_EPIndex  = 0;
  }
}

/*********************************************************************
*
*       _Enable
*/
static void _Enable(void) {
}

/*********************************************************************
*
*       _GetMaxPacketSize
*
*  Function description
*    Returns the MaxPacketSize for endpoint zero.
*    This function is not called for other endpoints.
*
*  Parameters
*    EPIndex    - Deprecated.
*
*  Return value
*    MaxPacketSize  -  The max packet size of the endpoint.
*/
static unsigned _GetMaxPacketSize(unsigned EPIndex) {
  USB_USE_PARA(EPIndex);
  return EP0_MAX_PACKET_SIZE;
}

/*********************************************************************
*
*       _StartTx
*
*  Function description
*    Sends a packet on the given EP.
*
*  Parameters
*    EPIndex -  Any IN EP incl. 0.
*/
static void _StartTx(unsigned EPIndex) {
  unsigned PhyEP;

  PhyEP = USB__EPIndex2Addr(EPIndex) & 0x7FuL;
  _EPIN_Enabled[PhyEP] = 1;
  USB__Send(EPIndex);
}

/*********************************************************************
*
*       _DisableTx
*
*  Function description
*    Disables Tx interrupts for the given EP.
*
*  Parameters
*    EPIndex   Any IN EP != 0.
*/
static void _DisableTx(unsigned EPIndex) {
  unsigned PhyEP;

  PhyEP = USB__EPIndex2Addr(EPIndex) & 0x7FuL;
  _EPIN_Enabled[PhyEP] = 0;
  if (PhyEP == 0u && _SetupActive > 0) {
    //
    // Issue status stage
    //
    _pUSBRegs->TASKS_EP0STATUS = 1;
    _SetupActive = 0;
  }
}

/*********************************************************************
*
*       _IsInHighSpeedMode
*/
static int _IsInHighSpeedMode(void) {
  return 0;
}

/*********************************************************************
*
*       _ResetEP
*
*   Function description
*     Resets the data toggle to DATA0.
*     It is useful after removing a HALT condition on a BULK endpoint.
*     Refer to Chapter 5.8.5 in the USB Serial Bus Specification, Rev.2.0.
*/
static void _ResetEP(unsigned EPIndex) {
  U32 Addr;

  Addr = USB__EPIndex2Addr(EPIndex);
  _pUSBRegs->DTOGGLE = Addr;
  _pUSBRegs->DTOGGLE = Addr | 0x100u;
}

__attribute__((always_inline)) static inline void ISB(void)
{
  __asm volatile ("isb 0xF":::"memory");
}


/**
  \brief   Data Synchronization Barrier
  \details Acts as a special kind of Data Memory Barrier.
           It completes when all explicit memory accesses before this instruction complete.
 */
__attribute__((always_inline)) static inline void DSB(void)
{
  __asm volatile ("dsb 0xF":::"memory");
}

/*********************************************************************
*
*       _Start
*
*  Function description
*    Sets default values for EP0 and enables the various interrupts
*    needed for USB operations.
*
*/
static void _Start(void) {
  USB_REGS   * pUSBRegs = _pUSBRegs;

  USB_OS_IncDI();

  //Updated USB enable code from Nordic SDK
  //static inline void usbd_errata_187_211_begin(void)
  if (*((volatile U32 *)(0x4006EC00)) == 0x00000000)
  {
      *((volatile U32 *)(0x4006EC00)) = 0x00009375;
      *((volatile U32 *)(0x4006ED14)) = 0x00000003;
      *((volatile U32 *)(0x4006EC00)) = 0x00009375;
  }
  else
  {
      *((volatile U32 *)(0x4006ED14)) = 0x00000003;
  }
      
      //static inline void usbd_errata_171_begin(void)
  if (*((volatile U32 *)(0x4006EC00)) == 0x00000000)
  {
      *((volatile U32 *)(0x4006EC00)) = 0x00009375;
      *((volatile U32 *)(0x4006EC14)) = 0x000000C0;
      *((volatile U32 *)(0x4006EC00)) = 0x00009375;
  }
  else
  {
      *((volatile U32 *)(0x4006EC14)) = 0x000000C0;
  }

  /* Enable the peripheral */
  pUSBRegs->ENABLE = 1;
  ISB();
  DSB();

  /* Waiting for peripheral to enable, this should take a few us */
  while ((pUSBRegs->EVENTCAUSE & EVENTCAUSE_READY) == 0u) {
  }
  pUSBRegs->EVENTCAUSE = EVENTCAUSE_READY;  
  ISB();
  DSB();

      //static inline void usbd_errata_171_end(void)
  if (*((volatile U32 *)(0x4006EC00)) == 0x00000000)
  {
      *((volatile U32 *)(0x4006EC00)) = 0x00009375;
      *((volatile U32 *)(0x4006EC14)) = 0x00000000;
      *((volatile U32 *)(0x4006EC00)) = 0x00009375;
  }
  else
  {
      *((volatile U32 *)(0x4006EC14)) = 0x00000000;
  }

      //static inline void usbd_errata_187_211_end(void)
  if (*((volatile U32 *)(0x4006EC00)) == 0x00000000)
  {
      *((volatile U32 *)(0x4006EC00)) = 0x00009375;
      *((volatile U32 *)(0x4006ED14)) = 0x00000000;
      *((volatile U32 *)(0x4006EC00)) = 0x00009375;
  }
  else
  {
      *((volatile U32 *)(0x4006ED14)) = 0x00000000;
  }

  //Original USB enable code
  /*
  pUSBRegs->EVENTCAUSE = EVENTCAUSE_READY;
  //
  // Enable USB controller
  //
  USB_OS_IncDI();
  if (pUSBRegs->ENABLE == 0u) {
#ifdef USE_WORKAROUND_187
    int WorkAround187 = 0;                                    //lint --e{923, 9078}  D:103[a]

    if ((((*(U32 *)0xF0000FE0u) & 0xFFu) == 0x08u) &&         // nRF52840 Rev 1.1 Errata [187]
        (((*(U32 *)0xF0000FE4u) & 0x0Fu) == 0x00u) &&
        (((*(U32 *)0xF0000FECu) & 0xF0u) == 0x00u)) {
      WorkAround187 = 1;
      if (*((volatile U32 *)(0x4006EC00u)) == 0x00000000u) {
        *((volatile U32 *)(0x4006EC00u)) = 0x00009375u;
        *((volatile U32 *)(0x4006ED14u)) = 0x00000003u;
        *((volatile U32 *)(0x4006EC00u)) = 0x00009375u;
      }
      else {
        *((volatile U32 *)(0x4006ED14u)) = 0x00000003u;
      }
    }
#endif
    pUSBRegs->ENABLE = 1;
    //
    // Wait for USB controller to become ready
    //
    while ((pUSBRegs->EVENTCAUSE & EVENTCAUSE_READY) == 0u) {
    }
    pUSBRegs->EVENTCAUSE = EVENTCAUSE_READY;
#ifdef USE_WORKAROUND_187
    if (WorkAround187 != 0) {
      if (*((volatile U32 *)(0x4006EC00u)) == 0x00000000u) {
        *((volatile U32 *)(0x4006EC00u)) = 0x00009375u;
        *((volatile U32 *)(0x4006ED14u)) = 0x00000000u;
        *((volatile U32 *)(0x4006EC00u)) = 0x00009375u;
      }
      else {
        *((volatile U32 *)(0x4006ED14u)) = 0x00000000u;
      }
    }
#endif
  }
  */

  USB_Global.pfEnableISR(_IRQHandler);
  //
  // Disable all EPs
  //
  pUSBRegs->EPINEN          = 0;
  pUSBRegs->EPOUTEN         = 0;
  //
  // Clear events
  //
  pUSBRegs->EVENTCAUSE   = pUSBRegs->EVENTCAUSE;
  pUSBRegs->EPDATASTATUS = pUSBRegs->EPDATASTATUS;
  pUSBRegs->EVENTS_EPDATA   = 0;
  pUSBRegs->EVENTS_USBEVENT = 0;
  pUSBRegs->EVENTS_USBRESET = 0;
  //
  // Enable interrupts
  //
  pUSBRegs->INTEN = INT_USBRESET
                  | INT_USBEVENT
                  | INT_EP0SETUP
                  | INT_EPDATA
                  ;
  USB_OS_DecRI();
}

/*********************************************************************
*
*       _Init
*
*  Function description
*    Initialize the USB device module in order to have it
*    in the same state after a reset.
*
*/
static void _Init(void) {
  _InUseEPIN  = 1;
  _InUseEPOUT = 1;
}

/*********************************************************************
*
*       _DeInit
*
*/
static int _DeInit(void) {
  _pUSBRegs->ENABLE = 0;
  return 0;
}

#else

/*********************************************************************
*
*       USB_HW_NRF_c
*
*  Function description
*    Dummy function to avoid problems with certain compilers which
*    can not handle empty object files.
*/
void USB_HW_NRF_c(void);
void USB_HW_NRF_c(void) {
}

#endif /* USB_HW_NRF_C_ */

/*************************** End of file ****************************/
