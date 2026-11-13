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
Purpose : General purpose routines
-------------------------- END-OF-HEADER -----------------------------
*/
#include "USB_Private.h"

/*********************************************************************
*
*       Defines fixed
*
**********************************************************************
*/
#define VERSION2STRING_HELPER(x) #x  //lint !e9024 N:999
#define VERSION2STRING(x) VERSION2STRING_HELPER(x)

/*********************************************************************
*
*       Defines configurable
*
**********************************************************************
*/
//
// This is for experimentation only, do not set to zero.
//
#ifndef   USB_CHECK_MAX_PACKET_SIZE
  #define USB_CHECK_MAX_PACKET_SIZE   (1)
#endif

/*********************************************************************
*
*       Types
*
**********************************************************************
*/
typedef struct {
  USB_ASYNC_IO_CONTEXT * pIOContext;
  EP_STAT              * pEPStat;
  unsigned               NumBytesToTransfer;
  USB_EVENT_CALLBACK     EventCallback;
} ASYNC_HOOK;

/*********************************************************************
*
*       Global data
*
**********************************************************************
*/
 EP_STAT                  USB_aEPStat[USB_NUM_EPS];
 USBD_GLOBAL              USB_Global;
 USB_CHECK_ADDRESS_FUNC * USBD__pfCheckValidDMAAddress;

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

static ASYNC_HOOK _AsyncHook[USB_NUM_EPS];

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

/*********************************************************************
*
*       _OnRxCheckDone
*
*  Function description
*    Check if a RX transfer is completed.
*
*  Return value
*    1 - Transfer completed, 0 - Transfer still in progress.
*/
static int _OnRxCheckDone(unsigned EPIndex, EP_STAT * pEPStat) {
  if (pEPStat->Dir.RxInfo.NumBytesRem == 0u || pEPStat->Dir.RxInfo.AllowShortPacket != 0u) {
#if USBD_OS_LAYER_EX > 0
    USB_OS_Signal(EPIndex, pEPStat->TransactionCnt);
#else
    USB_OS_Signal(EPIndex);
#endif
    pEPStat->Dir.RxInfo.pData = NULL;
    if ((pEPStat->Dir.RxInfo.ReadMode & USB_READ_MODE_CONTINUOUS) == 0u) {
      USB_Global.pDriver->pfDisableRxInterruptEP(EPIndex);
      pEPStat->Dir.RxInfo.RxInterruptEnabled = 0;
    }
    return 1;
  }
  return 0;
}

/*********************************************************************
*
*       USB__Read
*
*  Function description
*   Read data from USB EP.
*   This function needs to be called from a task, if one of the blocking modes are selected.
*
*  Parameters
*    EPIndex       - Handle to the endpoint.
*    pData         - Pointer to the buffer to store the read data.
*    NumBytesReq   - Number of bytes to be read.
*    ms            - Timeout given in ms. 0 -> special handling: no timeout.
*    Mode          - see USB_RD_MODE_.. definitions.
*
*  Return value
*    Number of bytes read; or a negative value in case of an error
*/
int USB__Read(unsigned EPIndex, void* pData, unsigned NumBytesReq, unsigned ms, unsigned Mode) {
  EP_STAT * pEPStat;
  int       NumBytesTransfered;
  unsigned  NumBytesRem;
  U8        AllowShortPacket;
  U8        RxPacketByPacket;
  U8        ReadMode;
  static U8 Dummy;

  pEPStat = &USB_aEPStat[EPIndex];
  USB_OS_IncDI();
  if (pEPStat->IsHalted == 1) {
    NumBytesTransfered = USB_STATUS_EP_HALTED;
    goto Done;
  }
  if ((USB_Global.State & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)) != USB_STAT_CONFIGURED) {    //lint !e9032 N:100
    NumBytesTransfered = USB_STATUS_ERROR;
    goto Done;
  }
  if (pEPStat->Dir.RxInfo.pData != NULL) {
    //
    // Read overlapped is in progress, need to sync before next read can be executed
    //
    NumBytesTransfered = USB_STATUS_EP_BUSY;
    goto Done;
  }
  if (pEPStat->Dir.RxInfo.Buffer.Size == 0u) {
    //
    // if we don't have an endpoint buffer: Always full packets must be read with short reads allowed.
    // This is used for ISO EPs only.
    // The caller is responsible to provide a correct value for NumBytesReq according to the actual MaxPacketSize (AltSetting).
    //
    if ((Mode & ~USB_RD_MODE_NO_WAIT) != USB_RD_MODE_BLOCK_SHORT) {
      NumBytesTransfered = USB_STATUS_BAD_READ_MODE;
      goto Done;
    }
    //
    // Short cut to speed up raw reads (like ISO)
    //
    NumBytesRem                          = NumBytesReq;
    pEPStat->Dir.RxInfo.AllowShortPacket = 1;
    RxPacketByPacket                     = 0;
    NumBytesTransfered                   = 0;
  } else {
    ReadMode = pEPStat->Dir.RxInfo.ReadMode & USB_READ_MODE_CONTINUOUS;
    if (pData == NULL && NumBytesReq == 0u && (Mode & USB_RD_MODE_BLOCK) != 0u) {
      //
      // Special case: Read NULL packet.
      // Allowed only, if buffer is empty
      //
      if (pEPStat->Dir.RxInfo.Buffer.NumBytesIn != 0u) {
        NumBytesTransfered = USB_STATUS_EP_BUSY;
        goto Done;
      }
      NumBytesTransfered = 0;
      //
      // pData must be != 0 to indicate, that Read function is waiting for USB transfer to finish.
      //
      pData = &Dummy;
    } else {
      if (NumBytesReq == 0u) {
        ReadMode = USB_READ_MODE_CONTINUOUS;
      }
      //
      // Read from buffer as far as possible
      //
      NumBytesTransfered = USBD_BUFFER_Read(&pEPStat->Dir.RxInfo.Buffer, (U8 *)pData, NumBytesReq);
      //
      // Check if the internal buffer has fully serviced the read request,
      // or if short packets are allowed and data have been transmitted from the buffer.
      //
      if ((NumBytesReq - (unsigned)NumBytesTransfered == 0u) ||
          ((Mode & USB_RD_MODE_BLOCK_SHORT) != 0u && NumBytesTransfered != 0)) {
        //
        // If ContinuousRead is configured, we trigger a new read transfer as soon as there is enough space in the buffer.
        //
        if (ReadMode == 0u ||
            pEPStat->Dir.RxInfo.Buffer.Size - pEPStat->Dir.RxInfo.Buffer.NumBytesIn < pEPStat->MaxPacketSize) {
          //
          // We are finished here.
          //
          goto Done;
        }
        //
        // Force a start read transfer.
        //
        Mode = USB_RD_MODE_NO_BLOCK | USB_RD_MODE_NO_WAIT;
      }
    }
    AllowShortPacket = 0;
    if ((Mode & USB_RD_MODE_BLOCK_SHORT) != 0u) {
      AllowShortPacket = 1;
      //
      // Since we only want to receive one packet we pass MaxPacketSize of user buffer.
      // This makes sure that the drivers that use RX zero copy can handle the operation
      // properly.
      //
      if (NumBytesReq > pEPStat->MaxPacketSize) {
        NumBytesReq = pEPStat->MaxPacketSize;
      }
    }
    if ((Mode & USB_RD_MODE_NO_BLOCK) != 0u) {
      if (ReadMode == 0u) {
        goto Done;
      }
      //
      // Don't service user request, just trigger receiving more data.
      //
      NumBytesRem = 0;
      pData       = NULL;
    } else {
      pData       = (U8*)pData + NumBytesTransfered;
      NumBytesRem = NumBytesReq - (unsigned)NumBytesTransfered;
    }
    RxPacketByPacket = 0;
#if USB_EXACT_DATASIZE_ON_CANCEL
    if (USB_Global.Flags.MultiPacketRxBehavior != 0) {
      //
      // If we have a read with timeout and the driver will report only complete transfers for multiple packets,
      // then we only can request one packet at a time to achieve correct timeout behavior.
      // The same is necessary on async. reads to make GetNumBytesRemToRead() to work correctly.
      //
      if (ms != 0u || (Mode & USB_RD_MODE_BLOCK) == 0u) {
        RxPacketByPacket = 1;
      }
    }
#endif
    //
    // Store relevant information in the internal structure of the endpoint
    //
    pEPStat->Dir.RxInfo.AllowShortPacket = AllowShortPacket;
  }
  pEPStat->Dir.RxInfo.NumBytesRem        = NumBytesRem;
  pEPStat->Dir.RxInfo.pData              = (U8*)pData;
  pEPStat->Dir.RxInfo.RxPacketByPacket   = RxPacketByPacket;
  pEPStat->TransactionCnt                = ++USB_Global.TransactionCnt;
  //
  // Allow to receive Data and its interrupts.
  //
  if (pEPStat->Dir.RxInfo.RxInterruptEnabled == 0) {
#if USB_EXACT_DATASIZE_ON_CANCEL
    if (NumBytesRem > pEPStat->MaxPacketSize && RxPacketByPacket != 0u) {
      NumBytesRem = pEPStat->MaxPacketSize;
    }
#endif
    if ((Mode & USB_RD_MODE_NO_ABORT) != 0u) {
      NumBytesRem = 0; // Avoid zero-copy
    }
    pEPStat->Dir.RxInfo.RxInterruptEnabled = 1;
    USB_Global.pDriver->pfEnableRxInterruptEP(EPIndex, (U8*)pData, NumBytesRem);
  }
  if ((Mode & USB_RD_MODE_NO_WAIT) != 0u) {
    goto Done;
  }
#if USBD_OS_LAYER_EX > 0
  USB_OS_DecRI();
  if (ms != 0u) {
    if (USB_OS_WaitTimed(EPIndex, ms, pEPStat->TransactionCnt) != 0) {
      if ((Mode & USB_RD_MODE_NO_ABORT) == 0u) {
        USB__RemovePendingOperation(EPIndex, 0);
      } else {
        pEPStat->Dir.RxInfo.pData = NULL;
      }
    }
  } else {
    USB_OS_Wait(EPIndex, pEPStat->TransactionCnt);
  }
#else
  if (ms != 0u) {
    if (USB_OS_WaitTimed(EPIndex, ms)) {   // Timeout occurred while waiting for data? --> USB_OS_WaitTimed() temporarily enables interrupts.
      if ((Mode & USB_RD_MODE_NO_ABORT) == 0u) {
        USB__RemovePendingOperation(EPIndex, 0);
      } else {
        pEPStat->Dir.RxInfo.pData = NULL;
      }
    }
  } else {
    USB_OS_Wait(EPIndex);
  }
#endif
  NumBytesTransfered = NumBytesReq - pEPStat->Dir.RxInfo.NumBytesRem;
  //
  // If there was a detach or USBD_Stop while reading, we return an error instead of the data size
  // because the EP are invalidated and the buffer was flushed.
  //
  if ((USB_Global.State & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)) != USB_STAT_CONFIGURED) {   //lint !e9032 N:100
    NumBytesTransfered = USB_STATUS_ERROR;
  }
#if USBD_OS_LAYER_EX > 0
  return NumBytesTransfered;
#endif
Done:
  USB_OS_DecRI();
  return NumBytesTransfered;
}

/*********************************************************************
*
*       _WaitForEndOfRXTransfer
*
*  Function description
*    Wait until the current transfer on a particular EP has completed
*    This function needs to be called from a task.
*
*  Return value
*    0 - Transfer completed, 1 - Timeout occurred.
*/
static int _WaitForEndOfRXTransfer(unsigned EPIndex, unsigned Timeout, int AbortOnTimeout) {
  int       r;
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  r = 0;
#if USBD_OS_LAYER_EX == 0
  USB_OS_IncDI();
#endif
  if (pEPStat->Dir.RxInfo.pData != NULL) {
    if (Timeout != 0u) {
#if USBD_OS_LAYER_EX > 0
      if (USB_OS_WaitTimed(EPIndex, Timeout, pEPStat->TransactionCnt) != 0)
#else
      if (USB_OS_WaitTimed(EPIndex, Timeout) != 0)
#endif
      {
        if (AbortOnTimeout != 0) {
          USB__RemovePendingOperation(EPIndex, 0);
        }
        r = 1;
      }
    } else {
#if USBD_OS_LAYER_EX > 0
      USB_OS_Wait(EPIndex, pEPStat->TransactionCnt);
#else
      USB_OS_Wait(EPIndex);
#endif
    }
    //
    // If there was a detach or USBD_Stop while reading, we return an error
    // because the EP are invalidated and the buffer was flushed.
    //
    if ((USB_Global.State & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)) != USB_STAT_CONFIGURED) {    //lint !e9032 N:100
      r = USB_STATUS_ERROR;
    }
  }
#if USBD_OS_LAYER_EX == 0
  USB_OS_DecRI();
#endif
  return r;
}

/*********************************************************************
*
*       _GetNumTXBytesRem
*
*/
static unsigned _GetNumTXBytesRem(EP_STAT *pEPStat) {
  unsigned r;
  int i;
  USB_DATA_PART *pPart;

  if (pEPStat->Dir.TxInfo.TxNumBytesPending > 1) {
    r = (unsigned)pEPStat->Dir.TxInfo.TxNumBytesPending - 1u;
  } else {
    r = 0;
  }
  pPart = &pEPStat->Dir.TxInfo.DataParts[0];
  for (i = pEPStat->Dir.TxInfo.NumDataParts; i > 0; i--) {
    r += pPart->NumBytesRem;
    pPart++;
  }
  return r;
}

/*********************************************************************
*
*       _WaitForEndOfTXTransfer
*
*  Function description
*    Wait until the current transfer on a particular EP has completed
*    This function needs to be called from a task.
*/
static unsigned _WaitForEndOfTXTransfer(unsigned EPIndex, unsigned Timeout, int AbortOnTimeout) {
  unsigned NumBytesRem = 0;
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
#if USBD_OS_LAYER_EX == 0
  USB_OS_IncDI();
#endif
  if (pEPStat->Dir.TxInfo.TxNumBytesPending != 0 || pEPStat->Dir.TxInfo.NumDataParts != 0u) {
    pEPStat->Dir.TxInfo.SignalOnTXReady = 0;
    if (Timeout != 0u) {
#if USBD_OS_LAYER_EX > 0
      if (USB_OS_WaitTimed(EPIndex, Timeout, pEPStat->TransactionCnt) != 0)
#else
      if (USB_OS_WaitTimed(EPIndex, Timeout) != 0)
#endif
      {
        NumBytesRem = _GetNumTXBytesRem(pEPStat);
        if (AbortOnTimeout != 0) {
          USB__RemovePendingOperation(EPIndex, 0);
        }
      }
    } else {
#if USBD_OS_LAYER_EX > 0
      USB_OS_Wait(EPIndex, pEPStat->TransactionCnt);
#else
      USB_OS_Wait(EPIndex);
#endif
    }
  }
#if USBD_OS_LAYER_EX == 0
  USB_OS_DecRI();
#endif
  return NumBytesRem;
}

/*********************************************************************
*
*       _PopDataPart
*
*  Function description
*    Remove first data part from the queue.
*    Must not be called, if NumDataParts < 2 !
*/
static void _PopDataPart(EP_STAT *pEPStat) {
  int            NumDataParts;
  USB_DATA_PART *pPart;

  NumDataParts = pEPStat->Dir.TxInfo.NumDataParts;
#if USB_DEBUG_LEVEL > 0
  if (NumDataParts <= 1) {
    USB_PANIC("General stack error");
  }
#endif
  pPart = &pEPStat->Dir.TxInfo.DataParts[0];
  while (--NumDataParts > 0) {
    *pPart = *(pPart + 1);
    pPart++;
  }
  pEPStat->Dir.TxInfo.NumDataParts--;
}

/*********************************************************************
*
*       _EventCallbacks
*
*  Function description
*    Call all registered callback functions.
*/
static void _EventCallbacks(const EP_STAT *pEPStat, unsigned Events) {
  USB_EVENT_CALLBACK *pCallback;

  pCallback = pEPStat->pEventCallbacks;
  while (pCallback != NULL) {
    pCallback->pfEventCb(Events, pCallback->pContext);
    pCallback = pCallback->pNext;
  }
}

/*********************************************************************
*
*       _HandleMagicHack
*
*  Function description
*    Handle special case for SEGGER Windows bulk driver
*/
static void _HandleMagicHack(EP_STAT *pEPStat) {
  if (pEPStat->Dir.TxInfo.NumBytesWithout0Packet != 0u) {
    //
    // Check special case of the host bulk driver: No 0-packet after 2048 bytes of data.
    //
    if (pEPStat->Dir.TxInfo.NumFullPacketsSend != 0u) {
      unsigned BytesSend;
      BytesSend = (unsigned)pEPStat->Dir.TxInfo.NumFullPacketsSend * pEPStat->MaxPacketSize;
      if ((BytesSend & (pEPStat->Dir.TxInfo.NumBytesWithout0Packet - 1u)) == 0u) {
        //
        // We encountered the special case here and must suppress the 0-packet.
        //
        pEPStat->Dir.TxInfo.NumFullPacketsSend = 0;
      }
    }
  }
}

/*********************************************************************
*
*       _AddEP
*/
static unsigned _AddEP(const USB_ADD_EP_INFO * pInfo, U8 * pBuffer, unsigned BufferSize) {
  unsigned        r;
  unsigned        MaxPacketSize;
  U8              EPDevice;
  EP_STAT       * pEPStat;
  const USB_HW_DRIVER * pDriver;

#if USB_SUPPORT_TRANSFER_ISO == 0
  if (pInfo->TransferType == USB_TRANSFER_TYPE_ISO) {
    USB_PANIC("ISO EP added but USB_SUPPORT_TRANSFER_ISO is 0");
  }
#endif
  pDriver = USB_Global.pDriver;
  r = 0;
  if (USB_Global.NumEPs < USB_NUM_EPS) {
    pEPStat  = &USB_aEPStat[USB_Global.NumEPs];
    MaxPacketSize = USB__CalcMaxPacketSize(pInfo->MaxPacketSize, pInfo->TransferType, USB__GetSpeedCapability());
    if (MaxPacketSize == 0u) {
      USB_PANIC("No valid packet size or transfer type");
    }
    if (pDriver->pfAllocEPEx != NULL) {
      EPDevice = pDriver->pfAllocEPEx(pInfo->InDir, pInfo->TransferType, MaxPacketSize);
    } else {
#if USB_SUPPORT_TRANSFER_ISO
      if (pInfo->TransferType == USB_TRANSFER_TYPE_ISO) {
        USB_PANIC("This driver does not support ISO transfers");
      }
#endif
      if (pInfo->TransferType == USB_TRANSFER_TYPE_INT && MaxPacketSize > 64u) {
        USB_PANIC("This driver does not support INT EPs > 64 bytes");
      }
      EPDevice = pDriver->pfAllocEP(pInfo->InDir, pInfo->TransferType);
    }
    if (EPDevice != 0u && EPDevice != 0xFFu) {
      USB_MEMSET(pEPStat, 0, sizeof(EP_STAT));
      pEPStat->EPAddr           = (EPDevice) | (pInfo->InDir << 7);
      pEPStat->EPType           = pInfo->TransferType;
#if USB_SUPPORT_TRANSFER_ISO
      if ((pInfo->Flags & USB_ADD_EP_FLAG_USE_ISO_SYNC_TYPES) == USB_ADD_EP_FLAG_USE_ISO_SYNC_TYPES) {
        if (pEPStat->EPType == USB_TRANSFER_TYPE_ISO) {
          pEPStat->Flags |= (pInfo->ISO_Type << 2) & USB_EP_STAT_ISO_TYPES_MASK;
        }
      }
#endif
      if (pInfo->InDir == 0u) {
#if USB_SUPPORT_TRANSFER_ISO
        if (pInfo->TransferType == USB_TRANSFER_TYPE_ISO) {
          if (BufferSize != 0u) {
            USB_WARN((USB_MTYPE_CORE, "Unused ISO endpoint buffer"));
          }
          BufferSize = 0;
        } else
#endif
        {
          //
          // The buffer used for OUT needs to be at least MaxPacketSize large or a multiple of it.
          //
          if (BufferSize < MaxPacketSize) {
            USB_PANIC("Invalid endpoint buffer size");
          }
        }
        pEPStat->Dir.RxInfo.Buffer.pData = pBuffer;
        pEPStat->Dir.RxInfo.Buffer.Size  = BufferSize;
      }
      pEPStat->MaxPacketSize      = MaxPacketSize;
      pEPStat->MaxPacketSizeUser  = MaxPacketSize;
      pEPStat->Interval           = pInfo->Interval;
      r = USB_Global.NumEPs++;
      if (EPDevice < (USB_MAX_ALLOWED_EPADDR + 1u) / 2u) {
        USB_Global.aPhyAddr2EPIndex[(EPDevice << 1) + pInfo->InDir] = r;
      } else {
        //
        // Endpoint table full, endpoint can't be handled.
        //
        USB_PANIC("Exceeding USB_MAX_ALLOWED_EPADDR");
      }
    } else {
      //
      // The application is trying to allocate more endpoints than the hardware can support.
      //
      USB_PANIC("Could not allocate endpoint (hardware does not support more EPs)");
    }
  } else {
    //
    // The application is trying to add more endpoints than configured (USB_NUM_EPS).
    //
    USB_PANIC("Could not allocate endpoint (you can increase USB_NUM_EPS)");
  }
  return r;
}

/*********************************************************************
*
*       _DummyAttach
*
*  Function description
*    Empty function, used if no attach function was set.
*
*/
static void _DummyAttach(void) {
}

/*********************************************************************
*
*       _AsyncCb
*
*  Function description
*    Internal callback function used for asynchonous reads and writes.
*/
static void _AsyncCb(unsigned Events, void *pContext) {
  ASYNC_HOOK           * pHook;
  USB_ASYNC_IO_CONTEXT * pIOContext;

  pHook = (ASYNC_HOOK *)pContext;     //lint !e9087 D:100[a]
  pIOContext = pHook->pIOContext;
  if (pIOContext == NULL) {
    return;
  }
  if ((Events & (USB_EVENT_DATA_ACKED | USB_EVENT_WRITE_COMPLETE)) != 0u) {
    pIOContext->NumBytesTransferred = pHook->NumBytesToTransfer - _GetNumTXBytesRem(pHook->pEPStat);
  }
  if ((Events & USB_EVENT_DATA_READ) != 0u) {
    pIOContext->NumBytesTransferred = pHook->NumBytesToTransfer - pHook->pEPStat->Dir.RxInfo.NumBytesRem;
  }
  if ((Events & (USB_EVENT_READ_COMPLETE | USB_EVENT_WRITE_COMPLETE | USB_EVENT_READ_ABORT | USB_EVENT_WRITE_ABORT)) == 0u) {
    //
    // Transfer not complete yet.
    //
    return;
  }
  pHook->pIOContext = NULL;
  if ((Events & (USB_EVENT_READ_COMPLETE | USB_EVENT_WRITE_COMPLETE)) != 0u) {
    pIOContext->Status = 0;
  } else {
    pIOContext->Status = USB_STATUS_ERROR;
  }
  pIOContext->pfOnComplete(pIOContext);
}

/*********************************************************************
*
*       _InstallAsyncHook
*
*  Function description
*    Installs internal callback function used for asynchronous reads and writes.
*/
static ASYNC_HOOK * _InstallAsyncHook(unsigned EPIndex) {
  EP_STAT    * pEPStat;
  ASYNC_HOOK * pHook;

  pEPStat = &USB_aEPStat[EPIndex];
  pHook   = &_AsyncHook[EPIndex];
  if ((pEPStat->Flags & USB_EP_STAT_ASYNC_CALLBACK_USED) == 0u) {
    pEPStat->Flags |= USB_EP_STAT_ASYNC_CALLBACK_USED;
    pHook->pEPStat = pEPStat;
    USBD_SetOnEvent(EPIndex, &pHook->EventCallback, _AsyncCb, pHook);
  }
  return pHook;
}

/*********************************************************************
*
*       _GetBCDVersion
*
*  Function description
*    Returns the default BCD Version of the USB device.
*/
static U16 _GetBCDVersion(void) {
  return 0x0100;
}

/*********************************************************************
*
*       _HandleSOFs
*
*  Function description
*    Called from the driver on every SOFs. Calls all user callback functions.
*/
static void _HandleSOFs(void) {
  USB_SOF_CALLBACK_HOOK *pHook;

  pHook = USB_Global.pSOFCbHook;
  while (pHook != NULL) {
    if (--pHook->Count == 0u) {
      pHook->Count = pHook->Interval;
      pHook->cb(pHook->pContext);
    }
    pHook = pHook->pNext;
  }
}

/*********************************************************************
*
*       _RegisterDeInitHook
*/
static void _RegisterDeInitHook(USB_DEINIT_HOOK *pHook) {
  USB_DEINIT_HOOK *pList;
  USB_DEINIT_HOOK *p;

  pList = USB_Global.pDeInitHandler;
  //
  // Check if hook is already in list
  //
  for (p = pList; p != NULL; p = p->pNext) {
    if (p == pHook) {
      return;
    }
  }
  //
  // Add to list
  //
  pHook->pNext = pList;
  USB_Global.pDeInitHandler = pHook;
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/

/*********************************************************************
*
*       USBD_AddEPEx
*
*  Function description
*    Returns an endpoint "handle" that can be used for the desired USB interface.
*
*  Parameters
*    pInfo:          Pointer to a structure of type USB_ADD_EP_INFO.
*    pBuffer:        Pointer to an endpoint buffer that is used for OUT-transactions.
*                    For IN-endpoints or ISO endpoints this parameter should be NULL.
*    BufferSize:     Size of the endpoint buffer (OUT endpoints only).
*                    Must be >= the maximum packet size of the endpoint.
*                    For IN-endpoints or ISO endpoints this parameter should be 0.
*
*  Return value
*     > 0:    A valid endpoint handle is returned.
*    == 0:    Error.
*
*  Additional information
*    This function must be called after USBD_Init() and before USBD_Start().
*/
unsigned USBD_AddEPEx(const USB_ADD_EP_INFO * pInfo, U8 * pBuffer, unsigned BufferSize) {
  return _AddEP(pInfo, pBuffer, BufferSize);
}

/*********************************************************************
*
*       USBD_AddEP
*
*  Function description
*    Returns an endpoint "handle" that can be used for the desired USB interface.
*
*  Parameters
*    InDir:          Specifies the direction of the desired endpoint.
*                    * USB_DIR_IN
*                    * USB_DIR_OUT
*    TransferType:   Specifies the transfer type of the endpoint.
*                    The following values are allowed:
*                    * USB_TRANSFER_TYPE_BULK
*                    * USB_TRANSFER_TYPE_INT
*                    ISO endpoints must be created using USBD_AddEPEx().
*    Interval:       Specifies the interval measured in units of 125us (micro frames).
*                    This value should be zero for a bulk endpoint.
*    pBuffer:        Pointer to a buffer that is used for OUT-transactions.
*                    For IN-endpoints this parameter must be NULL.
*    BufferSize:     Size of the buffer (OUT endpoints only). Must be a multiple of the maximum packet size.
*
*  Return value
*     > 0:    A valid endpoint handle is returned.
*    == 0:    Error.
*
*  Additional information
*    The Interval parameter specifies the frequency in which the endpoint should be
*    polled for information by the host. It must be specified in units of 125 us.
*
*    Depending on the actual speed of the device during enumeration, the USB stack
*    converts the interval to the correct value required for the endpoint descriptor
*    according to the USB specification (into milliseconds for low/full-speed, into 125 us
*    for high-speed).
*
*    For endpoints of type USB_TRANSFER_TYPE_BULK the value is ignored and should be set to 0.
*
*    This function must be called after USBD_Init() and before USBD_Start().
*/
unsigned USBD_AddEP(U8 InDir, U8 TransferType, U16 Interval, U8 * pBuffer, unsigned BufferSize) {
  unsigned MaxPacketSize;
  USB_ADD_EP_INFO EP_Info;
  static const U16 BulkSizes[3] = { 64, 512, 1024 };

  switch (TransferType) {
  case USB_TRANSFER_TYPE_INT:
    MaxPacketSize = 64;
    break;
  case USB_TRANSFER_TYPE_BULK:
    MaxPacketSize = BulkSizes[USB__GetSpeedCapability()];
    break;
  default:
    MaxPacketSize = 0;
    USB_PANIC("Invalid transfer type.");
    break;
  }
  EP_Info.Flags         = 0;
  EP_Info.InDir         = InDir;
  EP_Info.TransferType  = TransferType;
  EP_Info.Interval      = Interval;
  EP_Info.MaxPacketSize = MaxPacketSize;
  return _AddEP(&EP_Info, pBuffer, BufferSize);
}

/*********************************************************************
*
*       USB__EPAddr2Index
*/
U8 USB__EPAddr2Index(unsigned EPAddr) {
  unsigned  i;

  if ((EPAddr & 0x7Fu) == 0u) {
    return 0;
  }
  i = ((EPAddr & 0x0Fu) << 1);
  if ((EPAddr & 0x80u) != 0u) {
    i++;
  }
  return USB_Global.aPhyAddr2EPIndex[i];
}

/*********************************************************************
*
*       USB__EPIndex2Addr
*/
U8 USB__EPIndex2Addr(unsigned EPIndex) {
  return USB_aEPStat[EPIndex].EPAddr;
}

/*********************************************************************
*
*       USBD_IsConfigured
*
*  Function description
*    Checks if the USB device is initialized and ready.
*
*  Return value
*    ==  0: USB device is not configured.
*    ==  1: USB device is configured.
*/
char USBD_IsConfigured(void) {
  return (USB_Global.State & USB_STAT_CONFIGURED) != 0u ? 1 : 0;
}

/*********************************************************************
*
*       USB__UpdateEPHW
*/
void USB__UpdateEPHW(void) {
  EP_STAT * pEPStat;
  unsigned  i;
  const USB_HW_DRIVER * pDriver;

  pDriver = USB_Global.pDriver;
  pEPStat = USB_aEPStat;
  for (i = USB_Global.NumEPs; --i != 0u; ) {
    pDriver->pfUpdateEP(++pEPStat);
  }
  pDriver->pfEnable();
}

/*********************************************************************
*
*       USB__EmptyBuffer
*
*  Function description
*     Discards all data in RX buffer of an Out EP
*
*  Parameters
*    EPIndex:  Endpoint index.
*/
void USB__EmptyBuffer(unsigned EPIndex) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  USB_OS_IncDI();
  pEPStat->Dir.RxInfo.Buffer.NumBytesIn  = 0;
  pEPStat->Dir.RxInfo.Buffer.RdPos       = 0;
  USB_OS_DecRI();
}

/*********************************************************************
*
*       USBD_Init
*
*  Function description
*     Initializes the USB device with its settings.
*/
void USBD_Init(void) {
  const USB_HW_DRIVER * pDriver;

  //
  // Setup global data
  //
  USB_MEMSET(&USB_Global, 0, sizeof(USB_Global));
  USB_Global.sCopyright = "SEGGER emUSBD V" VERSION2STRING(USB_VERSION);
  USB_Global.NumEPs     = 1;
  USB_Global.USBSpecRev = 0x0200;
  USB_Global.MaxPower   = USB_MAX_POWER;
  USB_Global.MaxPowerSS = USB_MAX_POWER / 4u;
  USB_Global.pfOnBCDVersion = _GetBCDVersion;
  USB_MEMSET(USB_aEPStat, 0, sizeof(USB_aEPStat));
  //
  // Initialize OS Layer module
  //
  USB_OS_Init();
  //
  // Configure driver
  //
#if USB_V2_V3_MIGRATION_CONFIG > 0
  USB_X_AddDriver();
  USB_Global.pfAttach    = USB_X_HWAttach;
  USB_Global.pfEnableISR = USB_X_EnableISR;
#else
  USBD_X_Config();
#endif
#if USBD_SUPPORT_PROFILE
  //
  // Grant SystemViewer some time to connect after the target has started.
  // A delay of 30ms seems to be enough. To be on the safe side we use 50ms
  // as it does not hurt during initialization.
  //
  USB_OS_Delay(50);
#endif
  //
  // SystemView profiling for emUSB-Device is normally initialized inside USBD_X_Config.
  // Therefore we only start profiling after that.
  //
  USBD_PROFILE_CALL_VOID(USBD_EVTID_INIT);
  pDriver = USB_Global.pDriver;
#if USB_DEBUG_LEVEL > 0
  if (sizeof(USB_Global.sCopyright) > sizeof(PTR_ADDR)) {
    USB_PANIC(("Bad PTR_ADDR definition!"));
  }
  if (pDriver == NULL) {
    USB_WARN((USB_MTYPE_CORE, "No driver was added through the USBD_X_Config function!!!"));
    USB_PANIC("No driver added");
  }
  if (USB_Global.pfEnableISR == NULL) {
    USB_PANIC("Configuration incomplete");
  }
#endif
  if (USB_Global.pfAttach == NULL) {
    USB_Global.pfAttach = _DummyAttach;
  }
  if (pDriver->pfInit != NULL) {
    pDriver->pfInit();
  }
  USB_aEPStat[0].MaxPacketSize = pDriver->pfGetMaxPacketSize(0);
  USB_Global.IsInited = 1;
  USBD_PROFILE_END_CALL(USBD_EVTID_INIT);
}

/*********************************************************************
*
*       USBD_Start
*
*  Function description
*     Starts the emUSB-Device Core.
*
*  Additional information
*    This function should be called after configuring USB Core. It initiates a hardware
*    attach and updates the endpoint configuration. When the USB cable is connected to
*    the device, the host will start enumeration of the device.
*/
void USBD_Start(void) {
  const USB_HW_DRIVER * pDriver;

  USBD_PROFILE_CALL_VOID(USBD_EVTID_START);
#if USB_V2_V3_MIGRATION_DEVINFO > 0
  static USB_DEVICE_INFO DevInfo;

  DevInfo.VendorId       = USB_GetVendorId();
  DevInfo.ProductId      = USB_GetProductId();
  DevInfo.sVendorName    = (char *)USB_GetVendorName();
  DevInfo.sProductName   = (char *)USB_GetProductName();
  DevInfo.sSerialNumber  = (char *)USB_GetSerialNumber();
  USB_Global.pDeviceInfo = &DevInfo;
#endif
  USB_LOG((USB_MTYPE_INIT, "USBD_Start"));
#if USB_DEBUG_LEVEL > 0
  if (USB_Global.pDeviceInfo == NULL) {
    USB_PANIC("Device info missing");
  }
#endif
  if (USB_Global.EventsIdx != 0u && USB_Global.EventsIdx < USB_Global.NumEPs) {
    USB_PANIC("Can't allocate event (increase USB_EXTRA_EVENTS)");
  }
  pDriver = USB_Global.pDriver;
  //
  // Fire up the hardware
  //
  pDriver->pfStart();
  if (pDriver->pfControl != NULL) {
    //
    // Get driver flags
    //
    (void)pDriver->pfControl(USB_DRIVER_CMD_GET_FLAGS,
                             SEGGER_PTR2PTR(USB_IOCTL_PARA, &USB_Global.Flags));     // lint D:100[f]
  }
  USB_Global.State = USB_STAT_ATTACHED;
  pDriver->pfAttach();
  USBD_PROFILE_END_CALL(USBD_EVTID_START);
}

/*********************************************************************
*
*       USBD_BUFFER_Read
*
*  Function description
*    Read data from USB EP
*
*  Return value
*    Number of bytes read
*/
unsigned USBD_BUFFER_Read(USB_BUFFER * pBuffer, U8 * pData, unsigned NumBytesReq) {
  unsigned EndPos;
  unsigned NumBytesAtOnce;
  unsigned NumBytesTransfered;

  NumBytesTransfered = 0;
  if (pData == NULL && NumBytesReq != 0u) {
    USB_PANIC("buffer read: invalid arguments");
  }
  while (NumBytesReq != 0u && pBuffer->NumBytesIn != 0u) {
    EndPos =  pBuffer->RdPos + pBuffer->NumBytesIn;
    if (EndPos > pBuffer->Size) {
      EndPos = pBuffer->Size;
    }
    NumBytesAtOnce = EndPos - pBuffer->RdPos;
    NumBytesAtOnce = SEGGER_MIN(NumBytesAtOnce, NumBytesReq);
    USB_MEMCPY(pData, pBuffer->pData + pBuffer->RdPos, NumBytesAtOnce);
    NumBytesReq         -= NumBytesAtOnce;
    pBuffer->NumBytesIn -= NumBytesAtOnce;
    NumBytesTransfered  += NumBytesAtOnce;
    pData               += NumBytesAtOnce;
    pBuffer->RdPos      += NumBytesAtOnce;
    if (pBuffer->RdPos == pBuffer->Size) {
      pBuffer->RdPos = 0;
    }
  }
  //
  // Optimization for speed: If buffer is empty, read position is reset.
  //
  if (pBuffer->NumBytesIn == 0u) {
    pBuffer->RdPos = 0;
  }
  return NumBytesTransfered;
}

/*********************************************************************
*
*       USBD_BUFFER_Write
*
*  Function description
*    Write data into ring buffer
*/
unsigned USBD_BUFFER_Write(USB_BUFFER * pBuffer, const U8 * pData, unsigned NumBytes) {
  unsigned WrPos;
  unsigned EndPos;
  int      NumBytesFree;
  unsigned NumBytesAtOnce;

  //
  //  Check if we have enough space to store the data.
  //
  NumBytesFree = pBuffer->Size - pBuffer->NumBytesIn;
  if (NumBytesFree < (int)NumBytes) {
    USB_PANIC("RX buffer Overflow");
  }
  while (NumBytes != 0u) {
    //
    // Compute number of bytes to copy at once
    //
    WrPos = pBuffer->RdPos + pBuffer->NumBytesIn;
    //
    // Got at the end of the buffer, wrap-around.
    //
    if (WrPos >= pBuffer->Size) {
      WrPos -= pBuffer->Size;
    }
    EndPos =  WrPos + NumBytes;
    if (EndPos > pBuffer->Size) {
      EndPos = pBuffer->Size;
    }
    NumBytesAtOnce = EndPos - WrPos;
    NumBytesAtOnce = SEGGER_MIN(NumBytesAtOnce, NumBytes);
    //
    // Copy
    //
    USB_MEMCPY(pBuffer->pData + WrPos, pData, NumBytesAtOnce);
    //
    // Update variables
    //
    NumBytes            -= NumBytesAtOnce;
    pBuffer->NumBytesIn += NumBytesAtOnce;
    pData               += NumBytesAtOnce;
    NumBytesFree        -= (int)NumBytesAtOnce;
  }
  return (unsigned)NumBytesFree;
}

/*********************************************************************
*
*       USBD_Read
*
*  Function description
*    Reads data from the host.
*
*  Parameters
*    EPOut:        Handle to an OUT endpoint returned by USBD_AddEP().
*    pData:        Pointer to a buffer where the received data will be stored.
*    NumBytesReq:  Number of bytes to read.
*    Timeout:      Timeout given in milliseconds. A zero value results in an infinite timeout.
*
*  Return value
*    == NumBytes:        Requested data was successfully read within the given timeout.
*    >= 0 && < NumBytes: Timeout has occurred (Number of bytes read before timeout).
*    <  0:               An error occurred.
*
*  Additional information
*    This function blocks the task until all data has been read or a timeout occurs. In case of
*    a reset or a disconnect USB_STATUS_ERROR is returned.
*
*    If the USB stack receives a data packet from the host containing more bytes than requested,
*    the remaining bytes are stored into the internal buffer of the endpoint,
*    that was provided via the USBD_AddEP() function. This data can be retrieved by a later call to
*    USBD_Receive() or USBD_Read(). See also USBD_GetNumBytesInBuffer().
*
*    In case of a timeout, the read transfer is aborted (see \ref{Timeout handling}).
*/
int USBD_Read(unsigned EPOut, void* pData, unsigned NumBytesReq, unsigned Timeout) {
  int r;

  USBD_PROFILE_CALL_U32x4(USBD_EVTID_READ, EPOut, pData, NumBytesReq, Timeout);
  r = USB__Read(EPOut, pData, NumBytesReq, Timeout, USB_RD_MODE_BLOCK);
  USBD_PROFILE_END_CALL_U32(USBD_EVTID_READ, r);
  return r;
}

/*********************************************************************
*
*       USBD_ReadOverlapped
*
*  Function description
*    Reads data from the host asynchronously.
*
*  Parameters
*    EPOut:        Handle to an OUT endpoint returned by USBD_AddEP().
*    pData:        Pointer to a buffer where the received data will be stored.
*    NumBytesReq:  Number of bytes to read.
*
*  Return value
*    >= 0:     Number of bytes that have been read from the internal buffer (success).
*    == 0:     No data was found in the internal buffer, read transfer started (success).
*    < 0:      An error occurred.
*
*  Additional information
*    This function will not block the calling task. The read transfer will be initiated and the
*    function returns immediately. In order to synchronize, USBD_WaitForEndOfTransfer() needs
*    to be called.
*
*    Another synchronization method would be to periodically call
*    USBD_GetNumBytesRemToRead() in order to see how many bytes still need to be
*    received (this method is preferred when a non-blocking solution is necessary).
*
*    The read operation can be canceled using USBD_CancelIO().
*
*    The buffer pointed to by pData must be valid until the read operation is terminated.
*/
int USBD_ReadOverlapped(unsigned EPOut, void* pData, unsigned NumBytesReq) {
  int r;

  USBD_PROFILE_CALL_U32x3(USBD_EVTID_READOVERLAPPED, EPOut, pData, NumBytesReq);
  r = USB__Read(EPOut, pData, NumBytesReq, 0, USB_RD_MODE_NO_WAIT);
  USBD_PROFILE_END_CALL_U32(USBD_EVTID_READOVERLAPPED, r);
  return r;
}

/*********************************************************************
*
*       USBD_Receive
*
*  Function description
*    Reads data from host. The function blocks until any data have been received. In contrast
*    to USBD_Read() this function does not wait for all of NumBytes to be
*    received, but returns after the first packet has been received or after the timeout occurs.
*    In case of a timeout, the read transfer is aborted (see \ref{Timeout handling}).
*
*  Parameters
*    EPOut:        Handle to an OUT endpoint returned by USBD_AddEP().
*    pData:        Pointer to a buffer where the received data will be stored.
*    NumBytesReq:  Number of bytes to read.
*    Timeout:      Timeout given in milliseconds. A zero value results in an infinite timeout.
*                  If Timeout is -1, the function never blocks and only reads data from the internal
*                  endpoint buffer.
*
*  Return value
*    >  0:      Number of bytes that have been read within the given timeout.
*    == 0:      A timeout occurred (if Timeout > 0), zero packet received (not every controller supports this!),
*               no data in buffer (if Timeout < 0)
*               or the target was disconnected during the function call and no data was read so far.
*    <  0:      An error occurred.
*
*  Additional information
*    If no error occurs, this function returns the number of bytes received.
*    Calling USBD_Receive() will return as much data as is currently available up to
*    the size of the buffer specified within the specified timeout. This function also returns
*    when the target is disconnected from the host or when a USB reset occurred during the function
*    call, it will then return the number of bytes read so far. If the target was disconnected
*    before this function was called, it returns USB_STATUS_ERROR.
*
*    If the USB stack receives a data packet from the host containing more bytes than requested,
*    the remaining bytes are stored into the internal buffer of the endpoint,
*    that was provided via USBD_AddEP(). This data can be retrieved by a later call to
*    USBD_Receive() / USBD_Read(). See also USBD_GetNumBytesInBuffer().
*
*    A call of USBD_Receive(EPOut, NULL, 0, -1) can be used to trigger an asynchronous read that
*    stores the data into the internal buffer.
*/
int USBD_Receive(unsigned EPOut, void* pData, unsigned NumBytesReq, int Timeout) {
  unsigned Mode;
  int      r;

  USBD_PROFILE_CALL_U32x4(USBD_EVTID_RECEIVE, EPOut, pData, NumBytesReq, Timeout);
  if (Timeout < 0) {
    Mode = USB_RD_MODE_NO_BLOCK | USB_RD_MODE_NO_WAIT;
    Timeout = 0;
  } else {
    Mode = USB_RD_MODE_BLOCK_SHORT;
  }
  r = USB__Read(EPOut, pData, NumBytesReq, (unsigned)Timeout, Mode);
  USBD_PROFILE_END_CALL_U32(USBD_EVTID_RECEIVE, r);
  return r;
}

/*********************************************************************
*
*       USBD_ReceivePoll
*
*  Function description
*    Reads data from host. The function blocks until any data have been received. In contrast
*    to USBD_Read() this function does not wait for all of NumBytes to be
*    received, but returns after the first packet has been received or after the timeout occurs.
*    In contrast to USBD_Receive() this function will continue the read transfer asynchronously in case of a timeout.
*
*  Parameters
*    EPOut:        Handle to an OUT endpoint returned by USBD_AddEP().
*    pData:        Pointer to a buffer where the received data will be stored.
*    NumBytesReq:  Number of bytes to read.
*    Timeout:      Timeout given in milliseconds. A zero value results in an infinite timeout.
*
*  Return value
*    >  0:      Number of bytes that have been read within the given timeout.
*    == 0:      A timeout occurred (if Timeout > 0) or a zero packet received (not every controller supports this!),
*               or the target was disconnected during the function call and no data was read so far.
*    <  0:      An error occurred.
*
*  Additional information
*    If no error occurs, this function returns the number of bytes received.
*    Calling USBD_ReceivePoll() will return as much data as is currently available up to
*    the size of the buffer specified within the specified timeout. This function also returns
*    when the target is disconnected from the host or when a USB reset occurred during the function
*    call, it will then return the number of bytes read so far. If the target was disconnected
*    before this function was called, it returns USB_STATUS_ERROR.
*
*    If the USB stack receives a data packet from the host containing more bytes than requested,
*    the remaining bytes are stored into the internal buffer of the endpoint,
*    that was provided via USBD_AddEP(). This data can be retrieved by a later call to
*    USBD_Receive() / USBD_Read(). See also USBD_GetNumBytesInBuffer().
*
*    If a timeout occurs, the read transfer is not affected. Data send from the host after the timeout
*    is stored into the internal buffer of the endpoint and can be read by later calls to USBD_ReceivePoll().
*
*    If Timeout == 0, the function behaves like USBD_Receive().
*/
int USBD_ReceivePoll(unsigned EPOut, void* pData, unsigned NumBytesReq, unsigned Timeout) {
  int      r;

  USBD_PROFILE_CALL_U32x4(USBD_EVTID_RECEIVE, EPOut, pData, NumBytesReq, Timeout);
  r = USB__Read(EPOut, pData, NumBytesReq, Timeout, USB_RD_MODE_BLOCK_SHORT | USB_RD_MODE_NO_ABORT);
  USBD_PROFILE_END_CALL_U32(USBD_EVTID_RECEIVE, r);
  return r;
}

/*********************************************************************
*
*       USBD_WriteEP0FromISR
*
*  Function description
*   Write data to EP0 (control endpoint).
*   This function may be called in an interrupt context.
*
*  Parameters
*    pData:    Data that should be written.
*    NumBytes: Number of bytes to write.
*    Send0PacketIfRequired: Specifies that a zero-length packet should be sent when the
*                           last data packet to the host is a multiple of MaxPacketSize.
*                           Normally MaxPacketSize for control mode transfer is 64 byte.
*/
void USBD_WriteEP0FromISR(const void* pData, unsigned NumBytes, char Send0PacketIfRequired) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[0];
  //
  // Enqueue it
  //
  pEPStat->Dir.TxInfo.DataParts[0].pData       = (const U8 *)pData;
  pEPStat->Dir.TxInfo.DataParts[0].NumBytesRem = NumBytes;
  pEPStat->Dir.TxInfo.NumDataParts             = 1;
  pEPStat->Dir.TxInfo.Send0PacketIfRequired    = (U8)Send0PacketIfRequired;
  pEPStat->Dir.TxInfo.NumFullPacketsSend       = 0;
  //
  // Start transfer
  //
  USB_Global.pDriver->pfStartTx(0);
}

/*********************************************************************
*
*       USBD_WriteOverlapped
*
*  Function description
*    Write data to USB EP
*
*  Return value
*     0 on success; or a negative value in case of an error.
*/
int USBD_WriteOverlapped(unsigned EPIndex, const void* pData, unsigned NumBytes, char Send0PacketIfRequired) {
  EP_STAT * pEPStat;
  int       r;
  USB_DATA_PART *pPart;

  USBD_PROFILE_CALL_U32x4(USBD_EVTID_WRITEOVERLAPPED, EPIndex, pData, NumBytes, Send0PacketIfRequired);
  if ((USB_Global.State & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)) != USB_STAT_CONFIGURED) {    //lint !e9032 N:100
    return USB_STATUS_ERROR;
  }
#if USB_DEBUG_LEVEL > 0
  if (EPIndex == 0) {
    USB_PANIC("Illegal/non-existant endpoint accessed");
  }
#endif
  pEPStat = &USB_aEPStat[EPIndex];
  if (pEPStat->IsHalted != 0) {
    return USB_STATUS_EP_HALTED;
  }
  USB_OS_IncDI();
  //
  // Check if a transfer is in progress. In this case,
  // we either extend the running transfer or queue another transfer.
  // If the queue is full, we return USB_STATUS_EP_BUSY.
  //
  pPart = &pEPStat->Dir.TxInfo.DataParts[0];
  if (pEPStat->Dir.TxInfo.NumDataParts != 0u) {
    pPart += (pEPStat->Dir.TxInfo.NumDataParts - 1u);
    if ((pEPStat->Dir.TxInfo.WriteMode & USB_WRITE_MODE_EXTEND) != 0u &&
        pPart->pData + pPart->NumBytesRem == (const U8*)pData) {
      //
      // Extend current transfer
      //
      pPart->NumBytesRem += NumBytes;
      pEPStat->Dir.TxInfo.Send0PacketIfRequired = (U8)Send0PacketIfRequired;
      r = 0;
      goto Done;
    }
    //
    // Check for a free slot.
    //
    if (pEPStat->Dir.TxInfo.NumDataParts == USB_NUM_TX_DATA_PARTS) {
      r = USB_STATUS_EP_BUSY;
      goto Done;
    }
    //
    // Move to next free slot
    //
    pPart++;
  }
  //
  // Put new request into queue.
  //
  pPart->pData       = (const U8 *)pData;
  pPart->NumBytesRem = NumBytes;
  pEPStat->Dir.TxInfo.NumDataParts++;
  pEPStat->Dir.TxInfo.Send0PacketIfRequired = (U8)Send0PacketIfRequired;
  pEPStat->TransactionCnt = ++USB_Global.TransactionCnt;
  //
  // If no transfer in progress, start one
  //
  if (pEPStat->Dir.TxInfo.TxNumBytesPending == 0) {
    USB_Global.pDriver->pfStartTx(EPIndex);
  }
  r = 0;
Done:
  USB_OS_DecRI();
  USBD_PROFILE_END_CALL_U32(USBD_EVTID_WRITEOVERLAPPED, r);
  return r;
}

/*********************************************************************
*
*       USBD_WaitForEndOfTransferEx
*
*  Function description
*    Wait until the current transfer on a particular EP has completed.
*    This function must be called from a task.
*
*  Parameters
*    EPIndex:        Handle to the endpoint returned by USBD_AddEP().
*    Timeout:        Timeout in milliseconds, 0 means infinite wait.
*    AbortOnTimeout: If a timeout occurs, then the current transfer is terminated if AbortOnTimeout != 0.
*                    The current transfer is not affected in case of a timeout if AbortOnTimeout == 0.
*                    See also \ref{Timeout handling}.
*
*  Return value
*    ==  0 - Transfer completed.
*    ==  1 - Timeout occurred.
*/
int USBD_WaitForEndOfTransferEx(unsigned EPIndex, unsigned Timeout, int AbortOnTimeout) {
  EP_STAT * pEPStat;
  int       r;

  pEPStat = &USB_aEPStat[EPIndex];
  if ((pEPStat->EPAddr & 0x80u) != 0u) {
    r = 0;
    if (_WaitForEndOfTXTransfer(EPIndex, Timeout, AbortOnTimeout) != 0u) {
      r = 1;
    }
  } else {
    r = _WaitForEndOfRXTransfer(EPIndex, Timeout, AbortOnTimeout);
  }
  return r;
}

/*********************************************************************
*
*       USBD_WaitForEndOfTransfer
*
*  Function description
*    Wait until the current transfer on a particular EP has completed.
*    This function must be called from a task.
*
*  Parameters
*    EPIndex: Handle to the endpoint returned by USBD_AddEP().
*    Timeout: Timeout in milliseconds, 0 means infinite wait.
*
*  Return value
*    ==  0 - Transfer completed.
*    ==  1 - Timeout occurred.
*
*  Additional information
*    In case of a timeout, a current transfer is canceled (see \ref{Timeout handling}).
*/
int USBD_WaitForEndOfTransfer(unsigned EPIndex, unsigned Timeout) {
  return USBD_WaitForEndOfTransferEx(EPIndex, Timeout, 1);
}

/*********************************************************************
*
*       USBD_WaitForTXReady
*
*  Function description
*    Waits (blocking) until the TX queue can accept another data packet.
*    This function is used in combination with a non-blocking call to USBD_Write(), it
*    waits until a new asynchronous write data transfer will be accepted by the USB stack.
*
*  Parameters
*    EPIndex:  Handle to an IN endpoint returned by USBD_AddEP().
*    Timeout:  Timeout in milliseconds. 0 means infinite.
*              If Timeout is negative, the function will return immediately.
*
*  Return value
*    == 0: A new asynchronous write data transfer will be accepted.
*    == 1: The write queue is full,
*          a call to  USBD_Write() would return USB_STATUS_EP_BUSY.
*    < 0:  Error occurred.
*
*  Additional information
*    If Timeout is 0, the function never returns 1.
*    If Timeout is -1, the function will not wait, but immediately return the current state.
*/
int USBD_WaitForTXReady(unsigned EPIndex, int Timeout) {
  int Result;
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  Result  = 0;
  USB_OS_IncDI();
  if (pEPStat->Dir.TxInfo.NumDataParts >= USB_NUM_TX_DATA_PARTS) {
    if (Timeout < 0) {
      Result = 1;
    } else {
      pEPStat->Dir.TxInfo.SignalOnTXReady = 1;
#if USBD_OS_LAYER_EX > 0
      USB_OS_DecRI();
      if (Timeout != 0) {
        if (USB_OS_WaitTimed(EPIndex, (unsigned)Timeout, pEPStat->TransactionCnt) != 0) {
          Result = 1;
        }
      } else {
        USB_OS_Wait(EPIndex, pEPStat->TransactionCnt);
      }
      pEPStat->Dir.TxInfo.SignalOnTXReady = 0;
      return Result;
#else
      if (Timeout) {
        if (USB_OS_WaitTimed(EPIndex, Timeout)) {
          Result = 1;
        }
      } else {
        USB_OS_Wait(EPIndex);
      }
      pEPStat->Dir.TxInfo.SignalOnTXReady = 0;
#endif
    }
  }
  USB_OS_DecRI();
  return Result;
}

/*********************************************************************
*
*       USBD_Write
*
*  Function description
*    Writes data to the host. Depending on the Timeout parameter, the function may block
*    until NumBytes have been written or a timeout occurs.
*
*  Parameters
*    EPIndex:   Handle to an IN endpoint returned by USBD_AddEP().
*    pData:     Pointer to data that should be sent to the host.
*    NumBytes:  Number of bytes to be written.
*    Send0PacketIfRequired: Specifies that a zero-length packet should be sent when the
*                           last data packet to the host is a multiple of MaxPacketSize.
*    ms:        Timeout in milliseconds. 0 means infinite. If Timeout is -1,
*               the function returns immediately and the transfer is processed asynchronously.
*
*  Return value
*    == 0:                 Successful started an asynchronous write transfer
*                          or a timeout has occurred and no data was written.
*    > 0 && < NumBytes:    Number of bytes that have been written before a timeout occurred.
*    == NumBytes:          Write transfer successful completed.
*    <  0:                 An error occurred.
*
*  Additional information
*    This function also returns when the target is disconnected from host or when a USB reset
*    occurred.
*
*    The USB stack is able to queue a small number of asynchronous write
*    transfers (Timeout == -1). If a write transfer is still in progress when this
*    function is called and the USB stack can not accept another write transfer request,
*    the functions returns USB_STATUS_EP_BUSY. A synchronous write transfer
*    (Timeout >= 0) will always block until the transfer (including all
*    pending transfers) are finished.
*
*    In order to synchronize, USBD_WaitForEndOfTransfer() needs to be called. Another synchronization
*    method would be to periodically call USBD_GetNumBytesRemToWrite() in
*    order to see how many bytes still need to be written (this method is preferred when
*    a non-blocking solution is necessary).
*
*    In case of a timeout, the write transfer is aborted (see \ref{Timeout handling}).
*
*    The write operation can be canceled using USBD_CancelIO().
*
*    If pData == NULL and NumBytes == 0, a zero-length packet is sent to the host.
*
*    The content of the buffer pointed to by pData must not be changed until the transfer
*    has been completed.
*/
int USBD_Write(unsigned EPIndex, const void* pData, unsigned NumBytes, char Send0PacketIfRequired, int ms) {
  int r;
  unsigned NumBytesRem;

  USBD_PROFILE_CALL_U32x5(USBD_EVTID_WRITE, EPIndex, pData, NumBytes, Send0PacketIfRequired, ms);
  if (ms < 0) {
    return USBD_WriteOverlapped(EPIndex, pData, NumBytes, Send0PacketIfRequired);
  }
  for (;;) {
    r = USBD_WriteOverlapped(EPIndex, pData, NumBytes, Send0PacketIfRequired);
    if (r != USB_STATUS_EP_BUSY) {
      break;
    }
    if (USBD_WaitForTXReady(EPIndex, ms) != 0) {
      return 0;                                                         // timeout
    }
  }
  //
  // USBD_WriteOverlapped() never returns values > 0.
  //
  if (r == 0) {
    NumBytesRem = _WaitForEndOfTXTransfer(EPIndex, (unsigned)ms, 1);
    if (NumBytes > NumBytesRem) {
      r = NumBytes - NumBytesRem;
    }
  }
  USBD_PROFILE_END_CALL_U32(USBD_EVTID_WRITE, r);
  return r;
}

/*********************************************************************
*
*       USBD_SetClassRequestHook
*
*  Function description
*    Sets a callback function that is called when a setup class request
*    is sent from the host to the specified interface index.
*
*  Parameters
*    InterfaceNum:      Interface index that for setting the class
*                       request callback.
*    pfOnClassRequest:  Pointer to the callback.
*
*  Additional information
*    Note that the callback will be called within an ISR, therefore it should never block.
*    If it is necessary to send data from the callback function through endpoint 0, use the
*    function USBD_WriteEP0FromISR().
*/
void USBD_SetClassRequestHook(unsigned InterfaceNum, USB_ON_CLASS_REQUEST * pfOnClassRequest) {
  USB_Global.aIF[InterfaceNum].pfOnClassRequest = pfOnClassRequest;
}

/*********************************************************************
*
*       USBD_SetVendorRequestHook
*
*  Function description
*    Sets a callback function that is called when a setup vendor request
*    is sent from the host to the specified interface index.
*
*  Parameters
*    InterfaceNum:       Interface index that for setting the class
*                        request callback.
*    pfOnVendorRequest:  Pointer to the callback.
*
*  Additional information
*    Note that the callback will be called within an ISR, therefore it should never block.
*    If it is necessary to send data from the callback function through endpoint 0, use the
*    function USBD_WriteEP0FromISR().
*/
void USBD_SetVendorRequestHook(unsigned InterfaceNum, USB_ON_CLASS_REQUEST * pfOnVendorRequest) {
  USB_Global.aIF[InterfaceNum].pfOnVendorRequest = pfOnVendorRequest;
}

/*********************************************************************
*
*       USBD_SetOnSetupHook
*
*  Function description
*    Obsolete, use USBD_SetOnSetup().
*    Sets a callback function that is called when any setup request
*    is sent from the host.
*
*  Parameters
*    InterfaceNum:  Interface index that for setting the setup
*                   request callback.
*    pfOnSetup:     Pointer to the callback function.
*
*  Additional information
*    Note that the callback will be called within an ISR, therefore it should never block.
*    If it is necessary to send data from the callback function through endpoint 0, use the
*    function USBD_WriteEP0FromISR().
*/
void USBD_SetOnSetupHook(unsigned InterfaceNum, USB_ON_SETUP * pfOnSetup) {
  static USB_SETUP_HOOK Hook;

  USB_USE_PARA(InterfaceNum);
  USBD_SetOnSetup(&Hook, pfOnSetup);
}

/*********************************************************************
*
*       USBD_SetOnSetup
*
*  Function description
*    Sets a callback function that is called when any setup request
*    is sent from the host.
*
*  Parameters
*    pHook:      Pointer to a USB_SETUP_HOOK structure (will be initialized by this function).
*    pfOnSetup:  Pointer to the callback function.
*
*  Additional information
*    The USB_SETUP_HOOK structure is private to the USB stack. It will be initialized by USBD_SetOnSetup().
*    The USB stack keeps track of all setup callback functions using a linked list. The
*    USB_SETUP_HOOK structure will be included into this linked list and must reside in static memory.
*
*    Note that the callback will be called within an ISR, therefore it should never block.
*    If it is necessary to send data from the callback function through endpoint 0, use the
*    function USBD_WriteEP0FromISR().
*/
void USBD_SetOnSetup(USB_SETUP_HOOK * pHook, USB_ON_SETUP * pfOnSetup) {
  pHook->cb = pfOnSetup;
  //
  // Make new hook first in list.
  //
  pHook->pNext = USB_Global.pFirstSetupHook;
  USB_Global.pFirstSetupHook = pHook;
}

/*********************************************************************
*
*       USBD_SetOnRxEP0
*
*  Function description
*    Sets a callback when data are received in the data stage of
*    the setup request.
*
*  Parameters
*    pfOnRx:      Pointer to a function that should be called when receiving data
*                 other than setup packets on EP0.
*
*  Additional information
*    Please note that this function can be called multiple times
*    from different classes in order to check the data.
*
*    Note that the callback will be called within an ISR, therefore it should never block.
*    If it is necessary to send data from the callback function through endpoint 0, use the
*    function USBD_WriteEP0FromISR().
*/
void USBD_SetOnRxEP0(USB_ON_RX_FUNC * pfOnRx) {
  if (USB_Global.NumOnRxEP0Callbacks >= SEGGER_COUNTOF(USB_Global.apfOnRxEP0)) {
    USB_PANIC("Too many callbacks. Increase USB_MAX_NUM_COMPONENTS");
    return;
  }
  USB_Global.apfOnRxEP0[USB_Global.NumOnRxEP0Callbacks] = pfOnRx;
  ++USB_Global.NumOnRxEP0Callbacks;
}

/*********************************************************************
*
*       USB__GetpDest
*
*/
void * USB__GetpDest(unsigned EPIndex, unsigned NumBytes) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  //
  // Check if a task is waiting for >= NumBytes
  //
  if (pEPStat->Dir.RxInfo.NumBytesRem >= NumBytes) {
    return pEPStat->Dir.RxInfo.pData;
  }
  //
  // No destination pointer for Zero copy available
  //
  return NULL;
}

/*********************************************************************
*
*       USB__GetNextRX
*
*  Function description
*    If there are data to read requested by the application then
*    return number of bytes to read and pointer to the destination buffer.
*/
U32 USB__GetNextRX(unsigned EPIndex, U8 **p) {
  EP_STAT * pEPStat;
  U32       NumBytes;

  pEPStat = &USB_aEPStat[EPIndex];
  if (pEPStat->Dir.RxInfo.pData != NULL) {
    NumBytes = pEPStat->Dir.RxInfo.NumBytesRem;
    //
    // Because the SEGGER USB BULK driver for Windows does not send a zero packet after a multiple of 2048 byte of data,
    // to avoid a hang up on a read transfer we always request only a single packet.
    //
    if (NumBytes > pEPStat->MaxPacketSize && pEPStat->Dir.RxInfo.RxPacketByPacket != 0u) {
      NumBytes = pEPStat->MaxPacketSize;
    }
    *p = pEPStat->Dir.RxInfo.pData;
    return NumBytes;
  }
  return 0;
}

/*********************************************************************
*
*       USB__OnRxZeroCopyEx
*
*/
void USB__OnRxZeroCopyEx(unsigned EPIndex, unsigned NumBytes, int Done) {
  EP_STAT * pEPStat;
  unsigned  Events;

  pEPStat = &USB_aEPStat[EPIndex];
  if (pEPStat->Dir.RxInfo.pfOnRx != NULL) {
    pEPStat->Dir.RxInfo.pfOnRx(pEPStat->Dir.RxInfo.pData, NumBytes);
  }
  pEPStat->Dir.RxInfo.pData       += NumBytes;
  pEPStat->Dir.RxInfo.NumBytesRem -= NumBytes;
  Events  = USB_EVENT_DATA_READ;
  if (Done != 0) {
    if (_OnRxCheckDone(EPIndex, pEPStat) != 0) {
      Events |= USB_EVENT_READ_COMPLETE;
    }
  }
  //
  // Call USB event callback function.
  //
  _EventCallbacks(pEPStat, Events);
}

/*********************************************************************
*
*       USB__OnRxZeroCopy
*/
void USB__OnRxZeroCopy(unsigned EPIndex, unsigned NumBytes) {
  USB__OnRxZeroCopyEx(EPIndex, NumBytes, 1);
}

/*********************************************************************
*
*       USB__OnRx
*
*  Function description
*    Store the received data
*    This function is called from an ISR,  USB-IRQ Level
*/
void USB__OnRx(unsigned EPIndex, const U8 * pData, unsigned NumBytes) {
  unsigned NumBytesAtOnce;
  unsigned Events;
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  //
  // Handle EP0 data separately: Only callback for data is allowed.
  //
  if (EPIndex == 0u) {
    unsigned i;

    for (i = 0; i < USB_Global.NumOnRxEP0Callbacks; ++i) {
      if (USB_Global.apfOnRxEP0[i] != NULL) {
        USB_Global.apfOnRxEP0[i](pData, NumBytes);
      }
    }
    return;
  }
  if (pEPStat->Dir.RxInfo.pfOnRx != NULL) {
    pEPStat->Dir.RxInfo.pfOnRx(pData, NumBytes);
  }
  Events  = USB_EVENT_DATA_READ;
  //
  // Handle requested data.
  //
  if (pEPStat->Dir.RxInfo.pData != NULL) {
    NumBytesAtOnce = SEGGER_MIN(NumBytes, pEPStat->Dir.RxInfo.NumBytesRem);
    if (NumBytesAtOnce != 0u) {
      USB_MEMCPY(pEPStat->Dir.RxInfo.pData, pData, NumBytesAtOnce);
      NumBytes             -= NumBytesAtOnce;
      pData                += NumBytesAtOnce;
      pEPStat->Dir.RxInfo.pData       += NumBytesAtOnce;
      pEPStat->Dir.RxInfo.NumBytesRem -= NumBytesAtOnce;
    }
    if (_OnRxCheckDone(EPIndex, pEPStat) != 0) {
      Events |= USB_EVENT_READ_COMPLETE;
    }
  }
  if (NumBytes != 0u
#if USB_SUPPORT_TRANSFER_ISO
                     && pEPStat->EPType != USB_TRANSFER_TYPE_ISO
#endif
                                                                ) {
    unsigned NumBytesFreeInBuffer;
    //
    // Put the remainder of data in the buffer.
    // Returns the number of free bytes in the buffer.
    //
    NumBytesFreeInBuffer = USBD_BUFFER_Write(&pEPStat->Dir.RxInfo.Buffer, pData, NumBytes);
    //
    //  Check the free buffer size. If we can hold of max packet size,
    //  re-trigger a new transfer. Otherwise disable read transfer.
    //
    //  In order to use this. We recommend to use a buffer which
    //  is multiple of default max packet size.
    //  FS BULK EPs have a default max packet size of 64  bytes.
    //  HS BULK EPs have a default max packet size of 512 bytes.
    //  The size of the buffer can be set with the USBD_AddEP() function.
    //
    if ((pEPStat->Dir.RxInfo.ReadMode & USB_READ_MODE_CONTINUOUS) == 0u || NumBytesFreeInBuffer < pEPStat->MaxPacketSize) {
      if (pEPStat->Dir.RxInfo.RxInterruptEnabled != 0) {
        USB_Global.pDriver->pfDisableRxInterruptEP(EPIndex);
        pEPStat->Dir.RxInfo.RxInterruptEnabled = 0;
      }
    }
  }
  //
  // Call USB event callback function.
  //
  _EventCallbacks(pEPStat, Events);
}

/*********************************************************************
*
*       USB__EventDataSend
*
*  Function description
*    Trigger USB_EVENT_DATA_SEND. Called by the driver
*    if data was transferred to the controller and buffers may be reused.
*/
void USB__EventDataSend(unsigned EPIndex) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  _EventCallbacks(pEPStat, USB_EVENT_DATA_SEND);
  pEPStat->Dir.TxInfo.SendEventOccurred = 1;
}

/*********************************************************************
*
*       USB__Send
*/
void USB__Send(unsigned EPIndex) {
  unsigned NumBytes;
  EP_STAT * pEPStat;
  USB_DATA_PART *pPart;

  pEPStat = &USB_aEPStat[EPIndex];
#if USB_DEBUG_LEVEL > 0
  if (EPIndex) {
    if ((pEPStat->EPAddr & 0x80u) == 0u) {
      USB_PANIC("Illegal/non-existant endpoint accessed");             // Should only be called for IN (=Tx) EPs.
    }
  }
#endif
  //
  // If there are no data parts, then we should not really be here and there is no need to send a packet.
  //
  if (pEPStat->Dir.TxInfo.NumDataParts == 0u) {
    return;
  }
  if (EPIndex != 0u && pEPStat->Dir.TxInfo.TxNumBytesPending != 0) {
    return;
  }
  pEPStat->Dir.TxInfo.SendEventOccurred = 0;
  pPart = &pEPStat->Dir.TxInfo.DataParts[0];
  //
  // Suppress 0-packet if followed by more data
  //
  if (pPart->pData != NULL && pPart->NumBytesRem == 0u && pEPStat->Dir.TxInfo.NumDataParts > 1u) {
    _PopDataPart(pEPStat);
  }
  if (USB_Global.pDriver->pfSendEPEx != NULL) {
    unsigned NumBytesProcessed;
    unsigned NumFullPacketsSend;
    int      r;
    //
    // Optimized driver function SendEPEx available.
    //
    NumFullPacketsSend = pEPStat->Dir.TxInfo.NumFullPacketsSend;
    r = USB_Global.pDriver->pfSendEPEx(EPIndex, pEPStat->Dir.TxInfo.NumDataParts, pPart, &NumFullPacketsSend);
    pEPStat->Dir.TxInfo.NumFullPacketsSend = NumFullPacketsSend;
    if (r < 0) {
      //
      // The driver has not consumed any data because a transfer is still in progress.
      // The driver will later call USB__OnTx(), triggered by a USB interrupt,
      // which causes USB__Send() to be called again and send the data.
      //
      pEPStat->Dir.TxInfo.TxNumBytesPending = 1;
      return;
    }
    NumBytes = (unsigned)r;
    pEPStat->Dir.TxInfo.TxNumBytesPending = NumBytes + 1u;
    NumBytesProcessed = NumBytes;
    while (NumBytesProcessed > pPart->NumBytesRem) {
      //
      // first packet completely handled: remove it
      //
      NumBytesProcessed -= pPart->NumBytesRem;
      _PopDataPart(pEPStat);
    }
    pPart->NumBytesRem -= NumBytesProcessed;
    pPart->pData       += NumBytesProcessed;
  } else {
    //
    // Use standard driver function SendEP.
    //
    NumBytes = SEGGER_MIN(pPart->NumBytesRem, pEPStat->MaxPacketSize);
    pEPStat->Dir.TxInfo.TxNumBytesPending = NumBytes + 1u;
    USB_Global.pDriver->pfSendEP(EPIndex, pPart->pData, NumBytes);
    pPart->NumBytesRem -= NumBytes;
    pPart->pData       += NumBytes;
    if (NumBytes == 0u || (NumBytes & ((unsigned)pEPStat->MaxPacketSize - 1u)) != 0u) {
      pEPStat->Dir.TxInfo.NumFullPacketsSend = 0;
    } else {
      pEPStat->Dir.TxInfo.NumFullPacketsSend += NumBytes / pEPStat->MaxPacketSize;
    }
  }
  _HandleMagicHack(pEPStat);
  //
  // Do we need to send an other packet ?
  //
  if (pPart->NumBytesRem != 0u) {    // More data ?
    return;                          // Send an other packet with data on next interrupt.
  }
  if (pEPStat->Dir.TxInfo.NumDataParts > 1u) {
    //
    // Remove data part from queue.
    //
    _PopDataPart(pEPStat);
    return;                    // Send next data part on next interrupt.
  }
  //
  // When we reached this line, pPart->NumBytesRem == 0
  // Check if we need to send a 0-packet.
  // This is only the case if we send a packet - or in case of zero copy - multiple packets that are a multiple of
  // of MaxPacketSize, in this case we need to send a 0-packet in order to signal end of transfer.
  //
  if (NumBytes != 0u && pEPStat->Dir.TxInfo.NumFullPacketsSend != 0u && pEPStat->Dir.TxInfo.Send0PacketIfRequired != 0u) {
    pEPStat->Dir.TxInfo.Send0PacketIfRequired = 0;    // Make sure we send no more than a single 0-packet.
    return;                                           // Send an other packet without data on next interrupt.
  }
  pEPStat->Dir.TxInfo.NumDataParts = 0;               // Indicate that all transfers are completed.
}

/*********************************************************************
*
*       USB__OnTx
*/
void USB__OnTx(unsigned EPIndex) {
  EP_STAT * pEPStat;
  unsigned  Events;

  pEPStat = &USB_aEPStat[EPIndex];
  //
  // Call USB event callback function.
  //
  if (EPIndex != 0u && pEPStat->Dir.TxInfo.TxNumBytesPending > 1) {       // no callback on 0 packet (TxNumBytesPending == 1).
    //
    // TxNumBytesPending must be reset before the callback are executed in to order to return a correct result from USB_GetNumBytesToWrite()
    // and USBD_TxIsPending() if called inside the callback function.
    // TxNumBytesPending must not be 0 here to avoid a recursion if any USBD_Write function is called from the callback function.
    // So we use temporarily the value -1.
    //
    pEPStat->Dir.TxInfo.TxNumBytesPending = -1;
    Events = USB_EVENT_DATA_ACKED;
    if (pEPStat->Dir.TxInfo.SendEventOccurred == 0u) {
      Events |= USB_EVENT_DATA_SEND;
    }
    _EventCallbacks(pEPStat, Events);
  }
  pEPStat->Dir.TxInfo.TxNumBytesPending = 0;
  //
  // Send more data if possible.
  //
  if (pEPStat->Dir.TxInfo.NumDataParts != 0u) {
    USB__Send(EPIndex);
    if (pEPStat->Dir.TxInfo.NumDataParts < USB_NUM_TX_DATA_PARTS && pEPStat->Dir.TxInfo.SignalOnTXReady != 0u) {
#if USBD_OS_LAYER_EX > 0
      USB_OS_Signal(EPIndex, pEPStat->TransactionCnt);
#else
      USB_OS_Signal(EPIndex);
#endif
    }
    return;
  }
  //
  // Disable Tx interrupts.
  //
  USB_Global.pDriver->pfDisableTx(EPIndex);
  //
  // Wake task / Notify setup.
  //
  if (EPIndex != 0u) {
#if USBD_OS_LAYER_EX > 0
    USB_OS_Signal(EPIndex, pEPStat->TransactionCnt);
#else
    USB_OS_Signal(EPIndex);
#endif
    _EventCallbacks(pEPStat, USB_EVENT_WRITE_COMPLETE);
  } else {
    USB__OnTx0Done();
  }
}

/*********************************************************************
*
*       USB__OnLPMChange
*/
void USB__OnLPMChange(int State, unsigned BESL) {
  if (USB_Global.pfOnLPMChange != NULL) {
    USB_Global.pfOnLPMChange(State, BESL);
  }
}

/*********************************************************************
*
*       USBD_SetClrStallEP
*/
void  USBD_SetClrStallEP(unsigned EPIndex, int OnOff) {
  USB_aEPStat[EPIndex].IsHalted = OnOff;
  USB_Global.pDriver->pfSetClrStallEP(EPIndex, OnOff);
}

/*********************************************************************
*
*       USBD_StallEP
*
*  Function description
*    Stalls an endpoint.
*
*  Parameters
*    EPIndex: Handle to the endpoint handle returned by USBD_AddEP().
*/
void  USBD_StallEP(unsigned EPIndex) {
  USBD_SetClrStallEP(EPIndex, 1);
}

/*********************************************************************
*
*       USBD_AddDriver
*
*  Function description
*    Adds a USB device driver to the USB stack. This function should be called from within
*    USBD_X_Config() which is implemented in \file{USB_Config_*.c}.
*
*  Parameters
*    pDriver:    Pointer to the driver API structure.
*/
void USBD_AddDriver(const USB_HW_DRIVER * pDriver) {
  USB_LOG((USB_MTYPE_CONFIG, "Config: Adding driver %p to stack", pDriver));
  USB_Global.pDriver = pDriver;
}

/*********************************************************************
*
*       USBD_RegisterSCHook
*
*  Function description
*    Sets a callback function that will be called on every state change of the USB device.
*
*  Parameters
*    pHook:      Pointer to a USB_HOOK structure (will be initialized by this function).
*    pfStateCb:  Pointer to the callback routine that will be called on every state change.
*    pContext:   A pointer which is used as parameter for the callback function.
*
*  Additional information
*    The USB_HOOK structure is private to the USB stack. It will be initialized by USBD_RegisterSCHook().
*    The USB stack keeps track of all state change callback functions using a linked list. The
*    USB_HOOK structure will be included into this linked list and must reside
*    in static memory.
*
*    Note that the callback function will be called within an ISR, therefore it should never block.
*
*  Return value
*    == 0: OK.
*    == 1: Error, specified hook already exists.
*/
int USBD_RegisterSCHook(USB_HOOK * pHook, USB_STATE_CALLBACK_FUNC *pfStateCb, void * pContext) {
  USB_HOOK *  p;

  pHook->cb            = pfStateCb;
  pHook->pContext      = pContext;
  //
  // Check if this hook is already in list. If so, return error.
  //
  p = USB_Global.pFirstSCHook;
  while (p != NULL) {
    if (p == pHook) {
      return 1;     // Error, hook already in list.
    }
    p = p->pNext;
  }
  //
  // Make new hook first in list.
  //
  pHook->pNext = USB_Global.pFirstSCHook;
  USB_Global.pFirstSCHook = pHook;
  return 0;
}

/*********************************************************************
*
*       USBD_UnregisterSCHook
*
*  Function description
*    Removes hook function from the hook list for state change
*
*  Return value
*    0    - OK, removed
*    1    - Error, not in list
*/
int USBD_UnregisterSCHook(USB_HOOK * pHook) {
  // TBD
  USB_USE_PARA(pHook);
  return 1;
}

/*********************************************************************
*
*       USBD_SetOnEvent
*
*  Function description
*    Sets a callback function for an endpoint that will be called on every RX or TX event
*    for that endpoint.
*
*  Parameters
*    EPIndex:    Endpoint index returned by USBD_AddEP().
*    pEventCb:   Pointer to a USB_EVENT_CALLBACK structure (will be initialized by this function).
*    pfEventCb:  Pointer to the callback routine that will be called on every event on
*                the USB endpoint.
*    pContext:   A pointer which is used as parameter for the callback function.
*
*  Additional information
*    The USB_EVENT_CALLBACK structure is private to the USB stack. It will be initialized by USBD_SetOnEvent().
*    The USB stack keeps track of all event callback functions using a linked list. The
*    USB_EVENT_CALLBACK structure will be included into this linked list and must reside
*    in static memory.
*
*    The callback function is called only, if a read or write operation was started for the endpoint using one
*    of the USBD_Read...() or USBD_Write...() functions.
*/
void USBD_SetOnEvent(unsigned EPIndex, USB_EVENT_CALLBACK *pEventCb, USB_EVENT_CALLBACK_FUNC *pfEventCb, void *pContext) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  pEventCb->pfEventCb = pfEventCb;
  pEventCb->pContext  = pContext;
  pEventCb->pNext     = pEPStat->pEventCallbacks;
  pEPStat->pEventCallbacks = pEventCb;
}

/*********************************************************************
*
*       USBD_RemoveOnEvent
*
*  Function description
*    Removes a callback function which was added via USBD_SetOnEvent from the callback list.
*
*  Parameters
*    EPIndex:    Endpoint index returned by USBD_AddEP().
*    pEventCb:   Pointer to a USB_EVENT_CALLBACK structure which was used with USBD_SetOnEvent.
*/
void USBD_RemoveOnEvent(unsigned EPIndex, const USB_EVENT_CALLBACK *pEventCb) {
  USB_EVENT_CALLBACK * pPrev;
  USB_EVENT_CALLBACK * pCurrent;
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  if (pEventCb == pEPStat->pEventCallbacks) {
    pEPStat->pEventCallbacks = pEPStat->pEventCallbacks->pNext;
  } else {
    pPrev = pEPStat->pEventCallbacks;
    pCurrent = pPrev->pNext;
    while (pCurrent != NULL) {
      if (pEventCb == pCurrent) {
        pPrev->pNext = pCurrent->pNext;
        break;
      }
      pPrev = pCurrent;
      pCurrent = pCurrent->pNext;
    }
  }
}

/*********************************************************************
*
*       USB__OnBusReset
*
*  Function description
*    Called from the driver when a USB bus reset is detected.
*    This function is called from an ISR.
*/
void USB__OnBusReset(void) {
  unsigned i;

  if ((USB_Global.State & USB_STAT_ATTACHED) == 0u) {
    USB_LOG((USB_MTYPE_CORE, "Bus RESET in unattached state!"));
  }
  USB_Global.Addr = 0;
  for (i = 1; i < USB_Global.NumEPs; i++) {
    EP_STAT * pEPStat;

    pEPStat = &USB_aEPStat[i];
    pEPStat->IsHalted = 0;
  }
  USB_Global.DeviceState &= 1u;
  USB__OnStatusChange((USB_Global.State & USB_STAT_ATTACHED) | USB_STAT_READY);
}

/*********************************************************************
*
*       USBD_GetState
*
*  Function description
*    Returns the state of the USB device.
*
*  Return value
*    A bitwise combination of the USB state flags:
*
*    ==  USB_STAT_ATTACHED   - Device is attached. (Note 1)
*    ==  USB_STAT_READY      - Device is ready. (Note 2)
*    ==  USB_STAT_ADDRESSED  - Device is addressed. (Note 3)
*    ==  USB_STAT_CONFIGURED - Device is configured. (Note 4)
*    ==  USB_STAT_SUSPENDED  - Device is suspended. (Note 5)
*
*  Additional information
*    A USB device has several possible states. Some of these states are visible to the USB
*    and the host, while others are internal to the USB device. Refer to Universal Serial
*    Bus Specification, Revision 2.0, Chapter 9 for detailed information.
*
*  Notes
*    (1) Attached in a USB-specification sense of the word does not mean that the device is physically
*        connected to the host via a USB cable, it only means that the pull-up resistor on the device
*        side is connected. The status can be "attached" regardless of whether the device is
*        connected to a host or not. This state can normally be ignored.
*
*    (2) Ready denotes the USB controller state, the controller is "ready" after a bus reset.
*        This state can normally be ignored.
*
*    (3) A device is in an addressed state after it receives a valid (non-zero) USB address from the USB host.
*        This state can normally be ignored.
*
*    (4) When a device is "configured" the enumeration of the device has been successfully
*        completed and the host can communicate with the device.
*
*    (5) Suspend is set when the device is physically disconnected from the host or when the USB host
*        suspends the connected device.
*/
unsigned USBD_GetState(void) {
  return USB_Global.State;
}

/*********************************************************************
*
*       USBD_GetDeviceState
*
*  Function description
*    Returns the state of the USB device, set by the host
*    (except USB_DEVSTAT_SELF_POWERED, which is configured by the device, see USBD_SetMaxPower()).
*
*  Return value
*    A bitwise combination of the USB device state flags:
*
*    ==  USB_DEVSTAT_SELF_POWERED           - Device is self-powered.
*    ==  USB_DEVSTAT_REMOTE_WAKEUP_ALLOWED  - Remote Wakeup is allowed.
*    ==  USB_DEVSTAT_U1_ENABLE              - Link power state U1 is enabled (SuperSpeed only).
*    ==  USB_DEVSTAT_U2_ENABLE              - Link power state U2 is enabled (SuperSpeed only).
*    ==  USB_DEVSTAT_LPM_ENABLE             - Link power management is enabled (SuperSpeed only).
*/
unsigned USBD_GetDeviceState(void) {
  return USB_Global.DeviceState;
}

/*********************************************************************
*
*       USBD_GetSpeed
*
*  Function description
*    Returns the current connection speed.
*
*  Return value
*    == USB_SPEED_NONE - Unknown speed.
*    == USB_SPEED_FS   - Full-speed.
*    == USB_SPEED_HS   - High-speed.
*    == USB_SPEED_SS   - SuperSpeed.
*/
int USBD_GetSpeed(void) {
  if ((USB_Global.State & USB_STAT_ADDRESSED) == 0u) {
    return 0;
  }
  return USB__GetSpeedMode() + 1;
}

/*********************************************************************
*
*       USB__OnSuspend
*
*  Function description
*    Called by the driver when a suspend condition is recognized.
*/
void USB__OnSuspend(void) {
  USB__OnStatusChange(USB_Global.State | USB_STAT_SUSPENDED);
}

/*********************************************************************
*
*       USB__OnResume
*
*  Function description
*    Called by the driver when a resume condition is recognized.
*/
void USB__OnResume(void) {
  USB__OnStatusChange(USB_Global.State & ~USB_STAT_SUSPENDED);
}

/*********************************************************************
*
*       USB__OnStatusChange
*
*  Function description
*    Called on every potential status change.
*    Signals all EPs and calls all registered hook functions on a real change.
*/
void USB__OnStatusChange(U8 NewState) {
  USB_HOOK            * pHook;
  const USB_HW_DRIVER * pDriver;
  U8                    OldState;
  USB_IOCTL_PARA        Param;
  unsigned              i;

  OldState = USB_Global.State;
  pDriver  = USB_Global.pDriver;

  if (OldState != NewState) {
    //
    // Check if we got into configured state
    //
    if ((OldState & USB_STAT_CONFIGURED) == 0u && (NewState & USB_STAT_CONFIGURED) != 0u) {
      //
      // Update endpoint configuration
      //
      USB__UpdateEPHW();
      //
      //  Set Endpoint into default state
      //
      for (i = 1; i < USB_Global.NumEPs; i++) {
        USBD_SetClrStallEP(i, 0);
        USB__ResetDataToggleEP(i);
      }
      //
      // Notify the driver that the configuration was set - if necessary.
      //
      if (pDriver->pfControl != NULL) {
        Param.Configuration = 1;
        (void)pDriver->pfControl(USB_DRIVER_CMD_SET_CONFIGURATION, &Param);
      }
    }
    //
    // From configured state back into non configured state
    //
    if ((OldState & USB_STAT_CONFIGURED) != 0u && (NewState & USB_STAT_CONFIGURED) == 0u) {
      //
      // Notify the driver that the configuration was cleared - if necessary.
      //
      if (pDriver->pfControl != NULL) {
        Param.Configuration = 0;
        (void)pDriver->pfControl(USB_DRIVER_CMD_SET_CONFIGURATION, &Param);
      }
    }
    USB_Global.State = NewState;
    if (((OldState | NewState) & (USB_STAT_CONFIGURED | USB_STAT_ADDRESSED)) != 0u) {     //lint !e9032 N:100
      for (i = 1; i < USB_Global.NumEPs; i++) {
        USB__RemovePendingOperation(i, -1);
      }
    }
    //
    // Send notifications by calling all hook functions.
    //
    pHook = USB_Global.pFirstSCHook;
    while(pHook != NULL) {
      if (pHook->cb != NULL) {
        (pHook->cb)(pHook->pContext, NewState);
      }
      pHook = pHook->pNext;
    }
  }
}

/*********************************************************************
*
*       USBD_GetNumBytesInBuffer
*
*  Function description
*    Returns the number of bytes that are available in the internal BULK-OUT endpoint buffer.
*    This function does not start a read transfer.
*
*  Parameters
*    EPIndex:     Handle to an OUT endpoint returned by USBD_AddEP().
*
*  Return value
*    Number of bytes which have been stored in the internal buffer.
*
*  Additional information
*    The number of bytes returned by this function can be read using USBD_Read() or USBD_Receive()
*    without blocking.
*/
unsigned USBD_GetNumBytesInBuffer(unsigned EPIndex) {
  unsigned  r;
  EP_STAT * pEPStat;

  r = 0;
  if ((USB_Global.State & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)) == USB_STAT_CONFIGURED) {   //lint !e9032 N:100
    pEPStat = &USB_aEPStat[EPIndex];
    r  = pEPStat->Dir.RxInfo.Buffer.NumBytesIn;
  }
  return r;
}

/*********************************************************************
*
*       USBD_GetNumBytesRemToWrite
*
*  Function description
*    This function is to be used in combination with a non-blocking call to
*    USBD_Write(). It returns the number of bytes which still have to be written during
*    the transaction.
*
*  Parameters
*    EPIndex:     Handle to an IN endpoint returned by USBD_AddEP().
*
*  Return value
*    Number of bytes which still have to be written.
*
*  Additional information
*    Note that this function does not return the number of bytes that have been written,
*    but the number of bytes which still have to be written.
*    This function does not block.
*/
unsigned USBD_GetNumBytesRemToWrite(unsigned EPIndex) {
  EP_STAT * pEPStat;
  unsigned r;

  pEPStat = &USB_aEPStat[EPIndex];
  USB_OS_IncDI();
  r = _GetNumTXBytesRem(pEPStat);
  USB_OS_DecRI();
  return r;
}

/*********************************************************************
*
*       USBD_GetNumBytesRemToRead
*
*  Function description
*    This function is to be used in combination with USBD_ReadOverlapped(). It
*    returns the number of bytes which still have to be read during the transaction.
*
*  Parameters
*    EPIndex:     Handle to an OUT endpoint returned by USBD_AddEP().
*
*  Return value
*    Number of bytes which still have to be read.
*
*  Additional information
*    Note that this function does not return the number of bytes that have been read, but
*    the number of bytes which still have to be read.
*    This function does not block.
*/
unsigned USBD_GetNumBytesRemToRead(unsigned EPIndex) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  if (pEPStat->Dir.RxInfo.pData != NULL) {
    return pEPStat->Dir.RxInfo.NumBytesRem;
  }
  return 0;
}

/*********************************************************************
*
*       USBD_IsNULLPacketRequired
*
*  Return value
*    0 - No NULL packet required.
*    1 - A NULL packet is required
*   -1 - Unknown because write transfers are still in progress.
*/
int USBD_IsNULLPacketRequired(unsigned EPIndex) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  if (pEPStat->Dir.TxInfo.NumDataParts != 0u) {
    return -1;
  }
  _HandleMagicHack(pEPStat);
  return (pEPStat->Dir.TxInfo.NumFullPacketsSend == 0u) ? 0 : 1;    //lint !e9031  N:105
}

/*********************************************************************
*
*       USB__CalcMaxPacketSize
*
*  Function description
*    Check for valid value of MaxPacketSize for a given SpeedMode.
*
*  Parameters
*    MaxPacketSize:  MaxPacketSize to be checked.
*    TransferType:   USB_TRANSFER_TYPE_ISO/BULK/INT.
*    SpeedMode:      0=FS, 1=HS, 2=SS.
*
*  Return value
*    MaxPacketSize, rounded down for the given SpeedMode.
*    0 on error (invalid MaxPacketSize).
*/
unsigned USB__CalcMaxPacketSize(unsigned MaxPacketSize, U8 TransferType, int SpeedMode) {
#if USB_CHECK_MAX_PACKET_SIZE
  const U16 *pMaxMaxPacketSize;
  static const U16 _MaxMaxPacketSize[3][3] = {
    //  FS     HS      SS
    { 1023,  3072,  48u * 1024u },      // USB_TRANSFER_TYPE_ISO
    {   64,   512,  16u * 1024u },      // USB_TRANSFER_TYPE_BULK
    {   64,  1024,        1024u }       // USB_TRANSFER_TYPE_INT
  };

  pMaxMaxPacketSize = _MaxMaxPacketSize[TransferType - 1u];
  if (MaxPacketSize > pMaxMaxPacketSize[2]) {                 // [1] Check absolute maximum
    return 0;
  }
  switch (TransferType) {
  case USB_TRANSFER_TYPE_BULK:
    //
    // Bulk transfers: full-speed 8, 16, 32, 64, high-speed 512 bytes, SuperSpeed N * 1024 bytes
    // (1 < N <= 16 means burst transfers)
    //
    switch(MaxPacketSize) {
    case 8:
    case 16:
    case 32:
    case 64:
      if (SpeedMode > 0) {
        return 0;
      }
      break;
#if USB_SUPPORT_HIGH_SPEED
    case 512:
      if (SpeedMode > 1) {
        return 0;
      }
      break;
    default:
      //
      // Maximum already checked in [1]
      //
      if (MaxPacketSize < 1024u) {
        return 0;
      }
      //
      // Check for correct SuperSpeed burst size (multiple of 1024)
      //
      if ((MaxPacketSize & (1024u - 1u)) != 0u) {
        return 0;
      }
      break;
#else
    default:
      return 0;
      //lint -e{9077}  D:102[a]
#endif
    }
    break;
  case USB_TRANSFER_TYPE_INT:
#if USB_SUPPORT_TRANSFER_ISO
  case USB_TRANSFER_TYPE_ISO:
#endif
    //
    // Interrupt transfers: full-speed 1-64 bytes, high-speed and SuperSpeed 1-1024 bytes.
    // ISO transfers: full-speed 1-1023 bytes, high-speed 1-3072 bytes and SuperSpeed 1-48K.
    // Maximum already checked in [1]
    //
    break;
  default:
    return 0;
    //lint -e{9077}  D:102[a]
  }
  //
  // Reduce MaxPacketSize to meet actual speed
  //
  return SEGGER_MIN(MaxPacketSize, pMaxMaxPacketSize[SpeedMode]);
#else
  (void)TransferType;
  if (SpeedMode == 0u && MaxPacketSize > 64u) {
    MaxPacketSize = 64;
  }
  return MaxPacketSize;
#endif
}

/*********************************************************************
*
*       USBD_StoreU32BE
*/
void USBD_StoreU32BE(U8 * p, U32 v) {
  *p       = (U8)((v >> 24) & 255u);
  *(p + 1) = (U8)((v >> 16) & 255u);
  *(p + 2) = (U8)((v >> 8) & 255u);
  *(p + 3) = (U8)( v       & 255u);
}

/*********************************************************************
*
*       USBD_StoreU32LE
*/
void USBD_StoreU32LE(U8 * p, U32 v) {
  *p       = (U8)((v      ) & 255u);
  *(p + 1) = (U8)((v >>  8) & 255u);
  *(p + 2) = (U8)((v >> 16) & 255u);
  *(p + 3) = (U8)((v >> 24) & 255u);
}

/*********************************************************************
*
*       USBD_StoreU24BE
*/
void USBD_StoreU24BE(U8 * p, U32 v) {
  *p       = (U8)((v >> 16) & 255u);
  *(p + 1) = (U8)((v >>  8) & 255u);
  *(p + 2) = (U8)( v        & 255u);
}

/*********************************************************************
*
*       USBD_StoreU24LE
*/
void USBD_StoreU24LE(U8 * p, U32 v) {
  *p       = (U8)((v      ) & 255u);
  *(p + 1) = (U8)((v >>  8) & 255u);
  *(p + 2) = (U8)((v >> 16) & 255u);
}

/*********************************************************************
*
*       USBD_StoreU16BE
*/
void USBD_StoreU16BE(U8 * p, unsigned v) {
  *(p)     = (U8)((v >> 8) & 255u);
  *(p + 1) = (U8)( v       & 255u);
}

/*********************************************************************
*
*       USBD_StoreU16LE
*/
void USBD_StoreU16LE(U8 * p, unsigned v) {
  *p       = (U8)((v      ) & 255u);
  *(p + 1) = (U8)((v >>  8) & 255u);
}

/*********************************************************************
*
*       USBD_GetU32BE
*/
U32 USBD_GetU32BE(const U8 * p) {
  U32 v;

  v  = (U32)p[0] << 24;
  v |= (U32)p[1] << 16;
  v |= (U32)p[2] << 8;
  v |= p[3];
  return v;
}

/*********************************************************************
*
*       USBD_GetU32LE
*/
U32 USBD_GetU32LE(const U8 * p) {
  U32 v;

  v  = *p;
  v |= (U32)p[1] << 8;
  v |= (U32)p[2] << 16;
  v |= (U32)p[3] << 24;
  return v;
}

/*********************************************************************
*
*       USBD_GetU24BE
*/
U32 USBD_GetU24BE(const U8 * p) {
  U32 v;

  v  = (U32)p[0] << 16;
  v |= (U32)p[1] << 8;
  v |= p[2];
  return v;
}

/*********************************************************************
*
*       USBD_GetU24LE
*/
U32 USBD_GetU24LE(const U8 * p) {
  U32 v;

  v  = *p;
  v |= (U32)p[1] << 8;
  v |= (U32)p[2] << 16;
  return v;
}

/*********************************************************************
*
*       USBD_GetU16BE
*/
U16 USBD_GetU16BE(const U8 * p) {
  U16 v;

  v  = (U16)*p << 8;
  v |= (U16)p[1];
  return v;
}

/*********************************************************************
*
*       USBD_GetU16LE
*/
U16 USBD_GetU16LE(const U8 * p) {
  U16 v;

  v  = *p;
  v |= (U16)p[1] << 8;
  return v;
}

/*********************************************************************
*
*       USBD_SwapU32
*/
U32 USBD_SwapU32(U32 v) {
  v = (v << 24) | ((v & 0xFF00u) << 8) | ((v >> 8) & 0xFF00u) | (v >> 24);
  return v;
}

/*********************************************************************
*
*       USBD_CalcBitsSet
*
*  Function description
*    Calculates the number ones set in a U32 value.
*
*  Parameters
*    x     : Value to examine.
*
*  Return value
*    Number of bit set.
*/
unsigned USBD_CalcBitsSet(unsigned int x) {
  x -= ((x >> 1) & 0x55555555u);
  x = (((x >> 2) & 0x33333333u) + (x & 0x33333333u));
  x = (((x >> 4) + x) & 0x0F0F0F0Fu);
  x += (x >> 8);
  x += (x >> 16);
  return (x & 0x0000003Fu);
}

/*********************************************************************
*
*       USB__GetSpeedCapability
*
*  Function description
*    This function returns the speed capability of the USB controller.
*
*  Return value
*    * 0 - set  - USB device is not high-speed nor SuperSpeed capable.
*    * 1 - set  - USB device is high-speed capable.
*    * 2 - set  - USB device is SuperSpeed capable.
*/
int USB__GetSpeedCapability(void) {
  unsigned v;

  v = (unsigned)(USB_Global.pDriver->pfGetSpeedInfo)();
  if ((v & (1uL << 3)) != 0u) {
    return 2;
  }
  if ((v & (1uL << 1)) != 0u) {
    return 1;
  }
  return 0;
}

/*********************************************************************
*
*       USB__GetSpeedMode
*
*  Function description
*    This function returns the current speed of the USB device.
*
*  Return value
*    * 0 - set  - USB device is not in high-speed nor in SuperSpeed mode.
*    * 1 - set  - USB device is in high-speed mode.
*    * 2 - set  - USB device is in SuperSpeed mode.
*/
int USB__GetSpeedMode(void) {
  unsigned v;

  v = (unsigned)(USB_Global.pDriver->pfGetSpeedInfo)();
  if ((v & (1uL << 2)) != 0u) {
    return 2;
  }
  if ((v & (1uL << 0)) != 0u) {
    return 1;
  }
  return 0;
}

/*********************************************************************
*
*       USB__AllocIF
*/
U8 USB__AllocIF(void) {
  unsigned InterFaceNo;

  InterFaceNo = USB_Global.NumIFs++;
  if (USB_Global.NumIFs > USB_MAX_NUM_IF) {
    USB_PANIC("Too many interfaces (increase USB_MAX_NUM_IF)");
  }
  return InterFaceNo;
}

/*********************************************************************
*
*       USB__AllocAltIF
*/
#if USB_MAX_NUM_ALT_IF > 0u
U8 USB__AllocAltIF(U8 InterFaceNo) {
  unsigned AltInterFaceNo;

  AltInterFaceNo = USB_Global.NumAltIFs++;
  if (USB_Global.NumAltIFs > USB_MAX_NUM_ALT_IF) {
    USB_PANIC("Too many alternate interfaces (increase USB_MAX_NUM_ALT_IF)");
  }
  //
  // Map the alternative interface to the real interface number.
  //
  USB_Global.aAltIF[AltInterFaceNo].IFNo = InterFaceNo;
  return AltInterFaceNo;
}
#endif

/*********************************************************************
*
*       USB__AllocEvent
*
*  Function description
*    This function returns an index to an event that is not used by any endpoint
*    and may be used as argument for USB_OS_Wait/Signal() by an interface implementation.
*    Endpoints use event index 1 to (USB_Global.NumEPs - 1).
*    This function can allocate event index 0 (not used by the USB stack)
*    and indexes from (USB_NUM_EPS + USB_EXTRA_EVENTS - 1) down to the index of the last used EP plus 1.
*/
U8 USB__AllocEvent(void) {
  if (USB_Global.EventsIdx == 0u) {
    USB_Global.EventsIdx = USB_NUM_EPS + USB_EXTRA_EVENTS;
    return 0;
  }
  if (USB_Global.EventsIdx <= USB_Global.NumEPs) {
    USB_PANIC("Can't allocate event (increase USB_EXTRA_EVENTS)");
  }
  return --USB_Global.EventsIdx;
}

/*********************************************************************
*
*       USB__StallEP0
*
*  Function description
*    Set the stall flag on the control endpoint (EP0).
*    The flag will be reset after the next setup packet is received.
*/
void USB__StallEP0(void) {
  USB_Global.pDriver->pfStallEP0();
}

/*********************************************************************
*
*       USBD_SetOnRXHookEP
*
*  Function description
*    Sets a callback whenever data are received from a given
*    endpoint handle.
*    The callback function is called within the interrupt context and must not block.
*
*  Parameters
*    EPIndex    - Any valid endpoint handle > 0.
*    pfOnRx     - Pointer to the callback.
*/
void USBD_SetOnRXHookEP(unsigned EPIndex, USB_ON_RX_FUNC * pfOnRx) {
  EP_STAT * pEpStat;

  pEpStat = &USB_aEPStat[EPIndex];
  pEpStat->Dir.RxInfo.pfOnRx = pfOnRx;
}

/*********************************************************************
*
*       USBD_CancelIO
*
*  Function description
*     Cancel any read or write operation.
*
*  Parameters
*    EPIndex:     Handle to an endpoint returned by USBD_AddEP().
*/
void USBD_CancelIO(unsigned EPIndex) {
  USBD_PROFILE_CALL_U32(USBD_EVTID_CANCELIO, EPIndex);
  USB__RemovePendingOperation(EPIndex, 1);
  USBD_PROFILE_END_CALL(USBD_EVTID_CANCELIO);
}

/*********************************************************************
*
*       USBD_DoRemoteWakeup
*
*  Function description
*    Performs a remote wakeup in order to wake up the host from the
*    standby/suspend state.
*    This will only work, when the USB device driver supports this.
*    The function must be called only, if either:
*
*    A) The USB device is in suspend state: (USBD_GetState() & USB_STAT_SUSPENDED) != 0
*    and remote wakeup is allowed by the host: (USBD_GetDeviceState() & USB_DEVSTAT_REMOTE_WAKEUP_ALLOWED) != 0.
*
*    OR
*
*    B) The USB bus is in L1 state and remote wakeup is allowed by the host, see USBD_SetOnLPMChange().
*
*  Additional information
*    This function cannot be called from an ISR context.
*/
void USBD_DoRemoteWakeup(void) {
  const USB_HW_DRIVER * pDriver;

  pDriver = USB_Global.pDriver;
  if (pDriver->pfControl != NULL) {
    (void)pDriver->pfControl(USB_DRIVER_CMD_REMOTE_WAKEUP, NULL);
  }
}

/*********************************************************************
*
*       USBD_Stop
*
*  Function description
*    Stops the USB communication. This also makes sure that the device
*    is detached from the HOST.
*/
void USBD_Stop(void) {
  const USB_HW_DRIVER * pDriver;
  U8                    EPIndex;

  USBD_PROFILE_CALL_VOID(USBD_EVTID_STOP);
  USB_LOG((USB_MTYPE_INIT, "USBD_Stop"));
  USB_OS_IncDI();
  pDriver = USB_Global.pDriver;
  if (pDriver->pfDetach != NULL) {
    (void)pDriver->pfDetach();
  }
  if (USB_Global.pfDetach != NULL) {
    USB_Global.pfDetach();
  }
  USB__OnStatusChange(USB_Global.State & ~(USB_STAT_ATTACHED | USB_STAT_ADDRESSED | USB_STAT_CONFIGURED));
  for(EPIndex = 0; EPIndex < USB_Global.NumEPs; EPIndex++) {
    USB__RemovePendingOperation(EPIndex, 1);
  }
  USB_OS_DecRI();
  USBD_PROFILE_END_CALL(USBD_EVTID_STOP);
}

/*********************************************************************
*
*       USBD_DeInit
*
*  Function description
*     De-initialize the complete USB stack.
*
*  Additional information
*    This function also calls USBD_Stop() internally.
*/
void USBD_DeInit(void) {
  const USB_HW_DRIVER * pDriver;
  USB_DEINIT_HOOK     * pHook;

  USBD_PROFILE_CALL_VOID(USBD_EVTID_DEINIT);
  if (USB_Global.IsInited != 0u) {
    USBD_Stop();
    for (pHook = USB_Global.pDeInitHandler; pHook != NULL; pHook = pHook->pNext) {
      if (pHook->pFlag != NULL) {
        *pHook->pFlag = 0;
      }
      if (pHook->pfCallback != NULL) {
        pHook->pfCallback();
      }
    }
    pDriver = USB_Global.pDriver;
    if (pDriver->pfDeInit != NULL) {
      (void)pDriver->pfDeInit();
    }
#if USBD_OS_LAYER_EX > 0
    USB_OS_DeInit();
#endif
    USB_Global.IsInited = 0;
  }
  USBD_PROFILE_END_CALL(USBD_EVTID_DEINIT);
}

/*********************************************************************
*
*       USB__RegisterDeInitHandler
*
*  Function description
*    Adds a DeInit handler that shall be called within the USBD_DeInit()
*    function.
*
*  Parameters
*    pHook     - Pointer to a static hook structure.
*                Will be initialized by this function.
*    pfHandler - Pointer to the DeInit handler of the component.
*/
void USB__RegisterDeInitHandler(USB_DEINIT_HOOK *pHook, USB_DEINIT_FUNC * pfHandler) {
  pHook->pfCallback = pfHandler;
  pHook->pFlag      = NULL;
  _RegisterDeInitHook(pHook);
}

/*********************************************************************
*
*       USB__RegisterDeInitFlag
*
*  Function description
*    Adds a static flag that shall be set to zero within the USBD_DeInit()
*    function.
*
*  Parameters
*    pHook     - Pointer to a static hook structure.
*                Will be initialized by this function.
*    pFlag     - Pointer to the flag to be zeroed.
*/
void USB__RegisterDeInitFlag(USB_DEINIT_HOOK *pHook, U8 * pFlag) {
  pHook->pFlag      = pFlag;
  pHook->pfCallback = NULL;
  _RegisterDeInitHook(pHook);
}

/*********************************************************************
*
*       USBD_SetDetachFunc
*
*  Function description
*    Sets a detach callback in order to detach when attach is set by
*    USB_Global.pfAttach.
*    Detach is done by calling USBD_Stop or USBD_DeInit().
*
*  Parameters
*    pfDetach     -  Pointer to the detach function.
*/
void USBD_SetDetachFunc(USB_DETACH_FUNC * pfDetach) {
  USB_Global.pfDetach = pfDetach;
}

/*********************************************************************
*
*       USBD_TxIsPending
*
*  Function description
*    Checks whether the TX (IN endpoint) is currently pending.
*    This means whether we have queued a package to be sent.
*    Can be called in any context.
*/
int USBD_TxIsPending(unsigned EPIndex) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  if (pEPStat->Dir.TxInfo.TxNumBytesPending > 0 || pEPStat->Dir.TxInfo.NumDataParts != 0u) {
    return 1;
  }
  return 0;
}

/*********************************************************************
*
*       USBD_GetMaxPacketSize
*
*  Function description
*    Returns the max packet size for a given endpoint handle
*
*  Parameters
*    EPIndex     - Handle of the endpoint.
*
*  Return value
*    MaxPacketSize (8, 16, 64 ,512 etc)
*/
unsigned USBD_GetMaxPacketSize(unsigned EPIndex) {
  return USB_aEPStat[EPIndex].MaxPacketSize;
}

/*********************************************************************
*
*       USBD_GetInternalBufferSize
*
*  Function description
*    Returns the size of the internal buffer.
*    This internal buffer is only used for OUT-EPs.
*    It is used to store either a whole package or the remaining data of a
*    packet not served by the application.
*
*  Parameters
*    EPIndex     - Handle of the OUT endpoint.
*
*  Return value
*    Size of the internal buffer
*    In case of an wrong EP, the size
*/
unsigned USBD_GetInternalBufferSize(unsigned EPIndex) {
  EP_STAT * pEPStat;
  unsigned  r = 0;

  if (EPIndex < USB_NUM_EPS) {
    pEPStat = &USB_aEPStat[EPIndex];
    if ((pEPStat->EPAddr & 0x80u) == 0u) {
      r = pEPStat->Dir.RxInfo.Buffer.Size;
    }
  }
  return r;
}

/*********************************************************************
*
*       USBD_GetReadMode
*
*  Function description
*    Get Read Mode flags
*
*  Parameters
*    EPIndex    -  Any valid OUT endpoint handle.
*/
unsigned USBD_GetReadMode(unsigned EPIndex) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  if ((pEPStat->EPAddr & 0x80u) != 0u) {
    USB_PANIC("Get read mode for bad EP");
  }
  return pEPStat->Dir.RxInfo.ReadMode;
}

/*********************************************************************
*
*       USBD_SetReadMode
*
*  Function description
*    Set Read Mode flags
*
*  Parameters
*    EPIndex    -  Any valid IN endpoint handle.
*    Mode       -  New Read Mode flags, see USB_READ_MODE_... defines.
*/
void USBD_SetReadMode(unsigned EPIndex, unsigned Mode) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  if ((pEPStat->EPAddr & 0x80u) != 0u ||
      (pEPStat->Dir.RxInfo.Buffer.Size == 0u && Mode != 0u)) {
      USB_PANIC("Set read mode for bad EP");
  }
  pEPStat->Dir.RxInfo.ReadMode = Mode;
}

/*********************************************************************
*
*       USBD_GetWriteMode
*
*  Function description
*    Get Write Mode flags
*
*  Parameters
*    EPIndex    -  Any valid OUT endpoint handle.
*/
unsigned USBD_GetWriteMode(unsigned EPIndex) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  if ((pEPStat->EPAddr & 0x80u) == 0u) {
    USB_PANIC("Get write mode for bad EP");
  }
  return pEPStat->Dir.TxInfo.WriteMode;
}

/*********************************************************************
*
*       USBD_SetWriteMode
*
*  Function description
*    Set Write Mode flags
*
*  Parameters
*    EPIndex    -  Any valid IN endpoint handle.
*    Mode       -  New Read Mode flags, see USB_WRITE_MODE_... defines.
*/
void USBD_SetWriteMode(unsigned EPIndex, unsigned Mode) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  if ((pEPStat->EPAddr & 0x80u) == 0u || pEPStat->EPType == USB_TRANSFER_TYPE_ISO) {
    USB_PANIC("Set write mode for bad EP");
  }
  pEPStat->Dir.TxInfo.WriteMode = Mode;
}

/*********************************************************************
*
*       USB__RemovePendingOperation
*
*  Function description
*    Removes any pending operation that is assigned to an endpoint.
*
*  Parameters
*    EPIndex     -  Handle to the endpoint.
*    Flag        -  If != 0, tasks shall be signaled -> wake them up.
*                   If < 0, EP should be invalidated.
*/
void USB__RemovePendingOperation(unsigned EPIndex, int Flag) {
  EP_STAT * pEPStat;
  unsigned  Event;
  const USB_HW_DRIVER * pDriver;

  pDriver = USB_Global.pDriver;
  pEPStat = &USB_aEPStat[EPIndex];
  if (Flag < 0) {
    pEPStat->IsHalted = 0;
  }
  if (EPIndex != 0u) {
    USB_OS_IncDI();
    if ((pEPStat->EPAddr & 0x80u) != 0u) {
      pDriver->pfDisableTx(EPIndex);
      pEPStat->Dir.TxInfo.TxNumBytesPending = 0;
      pEPStat->Dir.TxInfo.NumDataParts = 0;
      pEPStat->Dir.TxInfo.NumFullPacketsSend = 0;
      Event = USB_EVENT_WRITE_ABORT;
    } else {
      if (pEPStat->Dir.RxInfo.RxInterruptEnabled != 0 || USB_Global.Flags.AlwaysDisableRX != 0) {
        pDriver->pfDisableRxInterruptEP(EPIndex);
      }
      pEPStat->Dir.RxInfo.RxInterruptEnabled = 0;
      pEPStat->Dir.RxInfo.pData = NULL;
      Event = USB_EVENT_READ_ABORT;
      if (Flag < 0) {
        pEPStat->Dir.RxInfo.Buffer.NumBytesIn  = 0;
        pEPStat->Dir.RxInfo.Buffer.RdPos       = 0;
      }
    }
    USB_OS_DecRI();
    _EventCallbacks(pEPStat, Event);
    if (Flag != 0) {
#if USBD_OS_LAYER_EX > 0
      USB_OS_Signal(EPIndex, pEPStat->TransactionCnt);
#else
      USB_OS_Signal(EPIndex);
#endif
    }
  }
}

/*********************************************************************
*
*       USBD_SetAttachFunc
*
*  Function description
*    Sets a function to perform hardware-specific actions to attach USB.
*
*  Parameters
*    pfAttach:       Pointer to the attach function.
*/
void USBD_SetAttachFunc(USB_ATTACH_FUNC *pfAttach) {
  USB_Global.pfAttach = pfAttach;
}

/*********************************************************************
*
*       USBD_SetISREnableFunc
*
*  Function description
*    Register function to enable USB interrupts.
*
*  Parameters
*    pfEnableISR:      Pointer to the function to install the interrupt handler and enable the USB interrupt.
*/
void USBD_SetISREnableFunc(USB_ENABLE_ISR_FUNC *pfEnableISR) {
  USB_Global.pfEnableISR = pfEnableISR;
}

/*********************************************************************
*
*       USBD_SetDeviceInfo
*
*  Function description
*    Sets a all information used during device enumeration.
*
*  Parameters
*    pDeviceInfo:   Pointer to a structure containing the device information. Must
*                   point to static data that is not changed while the stack is running.
*/
void USBD_SetDeviceInfo(const USB_DEVICE_INFO *pDeviceInfo) {
  USB_Global.pDeviceInfo = pDeviceInfo;
}

/*********************************************************************
*
*       USBD_SetOnSetInterfaceHook
*
*  Function description
*    Sets a callback for the Set Interface command.
*
*  Parameters
*    InterfaceNum:      Interface index that for setting
*                       the SetInterface request callback.
*    pfOnSetInterface:  Pointer to the callback.
*    pContext:          Pointer to a context which is
*                       passed to the callback function.
*/
void USBD_SetOnSetInterfaceHook(unsigned InterfaceNum, USB_ON_SET_IF_FUNC * pfOnSetInterface, void * pContext) {
  USB_Global.aIF[InterfaceNum].OnSetInterfaceHook.cb.Set = pfOnSetInterface;
  USB_Global.aIF[InterfaceNum].OnSetInterfaceHook.pContext = pContext;
}

/*********************************************************************
*
*       USBD_SetOnGetInterfaceHook
*
*  Function description
*    Sets a callback for the Get Interface command.
*
*  Parameters
*    InterfaceNum:      Interface index that for setting
*                       the SetInterface request callback.
*    pfOnSetInterface:  Pointer to the callback.
*    pContext:          Pointer to a context which is
*                       passed to the callback function.
*/
#if USB_SUPPORT_GET_INTERFACE_CB
void USBD_SetOnGetInterfaceHook(unsigned InterfaceNum, USB_ON_GET_IF_FUNC * pfOnSetInterface, void * pContext) {
  USB_Global.aIF[InterfaceNum].OnGetInterfaceHook.cb.Get = pfOnSetInterface;
  USB_Global.aIF[InterfaceNum].OnGetInterfaceHook.pContext = pContext;
}
#endif

/*********************************************************************
*
*       USBD_GetUSBAddr
*
*  Function description
*    Returns the current USB address.
*
*  Return value
*    The current USB address assigned by the host.
*/
unsigned USBD_GetUSBAddr(void) {
  return USB_Global.Addr;
}

/*********************************************************************
*
*       USBD_SetCacheConfig()
*
*  Function description
*    Configures cache related functionality that might be required by
*    the stack for cache handling in drivers.
*
*  Parameters
*    pConfig : Pointer to an element of SEGGER_CACHE_CONFIG .
*    ConfSize: Size of the passed structure in case library and
*              header size of the structure differs.
*
*  Additional information
*    This function has to called in USBD_X_Config().
*    This function replaces the legacy cache functions BSP_CACHE_CleanRange
*    and BSP_CACHE_InvalidateRange.
*    If you still want to use these routines please set USBD_USE_LEGACY_CACHE_ROUTINES
*    to 1 in your USB_Conf.h file.
*/
void USBD_SetCacheConfig(const SEGGER_CACHE_CONFIG *pConfig, unsigned ConfSize) {
#if USB_DEBUG_LEVEL > 0
  if (USB_Global.IsInited != 0) {
    USB_PANIC("Config function called too late");
    return;
  }
#endif
  if (ConfSize > sizeof(USB_Global.CacheConfig)) {
    ConfSize = sizeof(USB_Global.CacheConfig);
  }
  USB_MEMSET(&USB_Global.CacheConfig, 0, sizeof(USB_Global.CacheConfig));
  USB_MEMCPY(&USB_Global.CacheConfig, pConfig, ConfSize);
}

/*********************************************************************
*
*       USBD_AssignMemory
*
*  Function description
*    Assigns an area of RAM to be used for the endpoint buffers and
*    transfer descriptors by the USB driver. This function should be called
*    from within the USBD_X_Config() function. Not all drivers support this function.
*
*    If the driver uses DMA, the USB controller must have DMA access to this area.
*    For some drivers, the memory should be aligned to a given boundary.
*    If not aligned, the driver will increase the start address and reduce the size of
*    the area to achieve proper alignment. This results in wasting of RAM
*    and may cause the driver to run out of memory.
*
*  Parameters
*    pMem:     Pointer to the start of the RAM area to be used by the
*              driver.
*    MemSize:  Size of the RAM area in bytes.
*
*  Additional information
*    If the memory is not sufficient for the class and endpoint configuration,
*    the USB driver will run into the USB_OS_Panic() function during initialization,
*    if compiled for DEBUG mode (USB_DEBUG_LEVEL > 0).
*    After successful initialization, the driver will usually issue a
*    USB_LOG() message to report, how many bytes of the assigned memory are not
*    used. The size of the memory area may then be adjusted.
*/
void USBD_AssignMemory(void *pMem, U32 MemSize) {
  int              r;
  USB_IOCTL_PARA   Param;
  U8             * pMem8;
  U32              BytesToAlign;

  if (USB_Global.pDriver->pfControl == NULL) {
    r = -1;
    goto End;
  }
  //
  // Get alignment requirement from the driver.
  //
  pMem8 = (U8 *)pMem;
  r = USB_Global.pDriver->pfControl(USB_DRIVER_CMD_GET_MEM_ALIGNMENT, &Param);
  if (r == 0) {
    //
    // Align memory pointer
    //
    BytesToAlign = SEGGER_PTR2ADDR(pMem) & (Param.MemAlignment - 1u);   // lint D:103[a]
    if (BytesToAlign != 0u) {
      BytesToAlign = Param.MemAlignment - BytesToAlign;
      if (MemSize < BytesToAlign) {
        r = 1;
        goto End;
      }
      pMem8   += BytesToAlign;
      MemSize -= BytesToAlign;
    }
  }
  Param.MemAssign.pMem    = pMem8;
  Param.MemAssign.MemSize = MemSize;
  r = USB_Global.pDriver->pfControl(USB_DRIVER_CMD_ASSIGN_MEMORY, &Param);
End:
  if (r < 0) {
    USB_PANIC("Driver does not support assigning USB memory");
  }
  if (r > 0) {
    USB_PANIC("Assigning USB memory area failed");
  }
}

/*********************************************************************
*
*       USBD_ReadAsync
*
*  Function description
*    Reads data from the host asynchronously. The function does not wait for
*    the data to be received. A callback function is called after the transfer
*    has completed successfully, an error occurred or the transfer was canceled.
*
*  Parameters
*    EPIndex:   Handle to an OUT endpoint returned by USBD_AddEP().
*    pContext:  Pointer to an I/O context containing parameters and pointer to the callback function.
*    ShortRead: * 0: The transfer is completed successfully after all bytes have been read.
*               * 1: The transfer is completed successfully after one packet has been read.
*/
void USBD_ReadAsync(unsigned EPIndex, USB_ASYNC_IO_CONTEXT * pContext, int ShortRead) {
  int          r;
  ASYNC_HOOK * pHook;
  unsigned     Mode;
  U32          MinBytesToTransfer;

  pHook = _InstallAsyncHook(EPIndex);
  pContext->NumBytesTransferred = 0;
  if (ShortRead != 0) {
    pHook->NumBytesToTransfer = USB_aEPStat[EPIndex].MaxPacketSize;
    if (pHook->NumBytesToTransfer > pContext->NumBytesToTransfer) {
      pHook->NumBytesToTransfer = pContext->NumBytesToTransfer;
    }
    MinBytesToTransfer        = 1;
    Mode = USB_RD_MODE_BLOCK_SHORT | USB_RD_MODE_NO_WAIT;
  } else {
    pHook->NumBytesToTransfer = pContext->NumBytesToTransfer;
    MinBytesToTransfer        = pContext->NumBytesToTransfer;
    Mode = USB_RD_MODE_NO_WAIT;
  }
  USB_OS_IncDI();
  r = USB__Read(EPIndex, pContext->pData, pHook->NumBytesToTransfer, 0, Mode);
  if (r < 0) {
    pContext->Status = r;
  } else {
    pContext->NumBytesTransferred = (unsigned)r;
    if ((unsigned)r < MinBytesToTransfer) {
      //
      // Wait for callback to execute
      //
      pHook->pIOContext = pContext;
      USB_OS_DecRI();
      return;
    }
    pContext->Status = 0;
  }
  USB_OS_DecRI();
  pContext->pfOnComplete(pContext);
}

/*********************************************************************
*
*       USBD_WriteAsync
*
*  Function description
*    Sends data to the host asynchronously. The function does not wait for
*    the data to be sent. A callback function is called after the transfer
*    has completed successfully, an error occurred or the transfer was canceled.
*
*  Parameters
*    EPIndex:               Handle to an IN endpoint returned by USBD_AddEP().
*    pContext:              Pointer to an I/O context containing parameters and pointer to the callback function.
*    Send0PacketIfRequired: Specifies that a zero-length packet shall be sent when
*                           the last data packet is a multiple of MaxPacketSize.
*/
void USBD_WriteAsync(unsigned EPIndex, USB_ASYNC_IO_CONTEXT * pContext, char Send0PacketIfRequired) {
  int          r;
  ASYNC_HOOK * pHook;

  pContext->NumBytesTransferred = 0;
  USB_OS_IncDI();
  if (USB_aEPStat[EPIndex].Dir.TxInfo.NumDataParts != 0u) {
    r = USB_STATUS_EP_BUSY;
  } else {
    pHook = _InstallAsyncHook(EPIndex);
    r = USBD_WriteOverlapped(EPIndex, pContext->pData, pContext->NumBytesToTransfer, Send0PacketIfRequired);
    if (r >= 0) {
      pHook->NumBytesToTransfer = pContext->NumBytesToTransfer;
      pHook->pIOContext = pContext;
    }
  }
  USB_OS_DecRI();
  if (r < 0) {
    pContext->Status = r;
    pContext->pfOnComplete(pContext);
  }
}

/*********************************************************************
*
*       Deprecated functions
*
**********************************************************************
*/

#if USB_V2_V3_MIGRATION_API > 0
/*********************************************************************
*
*       USB_ReadEP
*
*  Function description
*    Read data from USB EP
*    This function must be called from a task.
*
*  Return value
*    Number of bytes read; or a negative value in case of an error
*/
int USB_ReadEP(unsigned EPIndex, void* pData, unsigned NumBytesReq) {
  return USB__Read(EPIndex, pData, NumBytesReq, 0, USB_RD_MODE_BLOCK);
}

/*********************************************************************
*
*       USB_ReadEPTimed
*
*  Function description
*    Read data from USB EP with time out
*    This function must be called from a task.
*
*  Return value
*    Number of bytes read; or a negative value in case of an error.
*/
int USB_ReadEPTimed(unsigned EPIndex, void * pData, unsigned NumBytesReq, unsigned ms) {
  return USB__Read(EPIndex, pData, NumBytesReq, ms, USB_RD_MODE_BLOCK);
}

/*********************************************************************
*
*       USB_ReceiveEP
*
*  Function description
*    Read data from USB EP
*    This function must be called from a task.
*
*  Return value
*    Number of bytes read; or a negative value in case of an error
*/
int USB_ReceiveEP(unsigned EPIndex, void* pData, unsigned NumBytesReq) {
  return USB__Read(EPIndex, pData, NumBytesReq, 0, USB_RD_MODE_BLOCK_SHORT);
}

/*********************************************************************
*
*       USB_ReceiveEPTimed
*
*  Function description
*    Read data from USB EP
*    This function must be called from a task.
*
*  Return value
*    Number of bytes read; or a negative value in case of an error
*/
int USB_ReceiveEPTimed(unsigned EPIndex, void* pData, unsigned NumBytesReq, unsigned ms) {
  return USB__Read(EPIndex, pData, NumBytesReq, ms, USB_RD_MODE_BLOCK_SHORT);
}

/*********************************************************************
*
*       USB_ReadEPOverlapped
*
*  Function description
*    Read data from USB EP
*    This function must be called from a task.
*
*  Parameters
*    EPIndex:     Any valid endpoint handle > 0.
*    pData:       Pointer to a buffer to store the data.
*    NumBytesReq: Size of the buffer in bytes.
*
*  Return value
*    Number of bytes read; or a negative value in case of an error
*/
int USB_ReadEPOverlapped(unsigned EPIndex, void * pData, unsigned NumBytesReq) {
  return USB__Read(EPIndex, pData, NumBytesReq, 0, USB_RD_MODE_NO_WAIT);
}

/*********************************************************************
*
*       USB_StartReadTransfer
*
*  Function description
*    Wrapper for backward compatibility
*
*  Parameters
*    EPIndex    -  Any valid endpoint handle > 0.
*/
void USB_StartReadTransfer(unsigned EPIndex) {
  EP_STAT * pEPStat;

  pEPStat = &USB_aEPStat[EPIndex];
  if ((pEPStat->EPAddr & 0x80u) == 0u) {
    pEPStat->Dir.RxInfo.ReadMode |= USB_READ_MODE_CONTINUOUS;
    (void)USB__Read(EPIndex, NULL, 0, 0, USB_RD_MODE_NO_BLOCK | USB_RD_MODE_NO_WAIT);
  }
}

/*********************************************************************
*
*       USB_IsStartReadTransferActive
*
*  Function description
*    Wrapper for backward compatibility
*
*  Parameters
*    EPIndex     - Handle of the OUT endpoint.
*
*  Return value
*    1 - StartReadTransfer is active.
*    0 - StartReadTransfer is not active.
*   -1 - Invalid EP handle.
*/
int USB_IsStartReadTransferActive(unsigned EPIndex) {
  EP_STAT * pEPStat;
  I8  r = -1;

  if (EPIndex < USB_NUM_EPS) {
    pEPStat = &USB_aEPStat[EPIndex];
    if ((pEPStat->EPAddr & 0x80u) == 0u) {
      r = pEPStat->Dir.RxInfo.ReadMode & USB_READ_MODE_CONTINUOUS;
    }
  }
  return r;
}
#endif

/*********************************************************************
*
*       USBD_SetCheckAddress
*
*  Function description
*    Installs a function that checks if an address can be used for DMA transfers.
*    Installed function must return 0, if DMA access is allowed for the given address,
*    any value != 0 otherwise.
*
*  Parameters
*    pfCheckValidDMAAddress:    Pointer to the function.
*
*  Additional information
*    If the function reports a memory region not valid for DMA, the driver uses a temporary
*    transfer buffer to copy data to and from this area.
*/
void USBD_SetCheckAddress(USB_CHECK_ADDRESS_FUNC * pfCheckValidDMAAddress) {
  USBD__pfCheckValidDMAAddress = pfCheckValidDMAAddress;
}

/*********************************************************************
*
*       USBD_GetVersion
*
*  Function description
*    Returns the version of the stack.
*
*  Return value
*    Format: Mmmrr; e.g: 32401 is 3.24.1
*/
U32 USBD_GetVersion(void) {
  return USB_VERSION;
}

/*********************************************************************
*
*       USBD_SetOnSOF
*
*  Function description
*    Installs a function that will be called, when a SOF was received from the host.
*    The callback function is called within the interrupt context and must not block.
*
*  Parameters
*    pfSOFCallback:    Pointer to the callback function.
*    Interval:         Function will be called every time a number of 'Interval' SOFs were received.
*    pContext:         A pointer which is used as parameter for the callback function.
*    pHook:            Pointer to a USB_SOF_CALLBACK_HOOK structure (will be initialized by this function).
*
*  Return value
*    == 0: Callback function successfully installed.
*    != 0: SOF callback not supported by the driver.
*
*  Additional information
*    The USB_EVENT_CALLBACK structure is private to the USB stack. It will be initialized by USBD_SetOnEvent().
*    The USB stack keeps track of all event callback functions using a linked list. The
*    USB_EVENT_CALLBACK structure will be included into this linked list and must reside
*    in static memory.
*/
int USBD_SetOnSOF(void (*pfSOFCallback)(void *pContext), U16 Interval, void *pContext, USB_SOF_CALLBACK_HOOK *pHook) {
  USB_IOCTL_PARA Param;
  USBD_GLOBAL * pG = &USB_Global;

  Param.pfSOFCallback = _HandleSOFs;
  if (pG->pDriver->pfControl != NULL) {
    if (pG->pDriver->pfControl(USB_DRIVER_CMD_SET_SOF_CALLBACK, &Param) == 0) {
      pHook->cb       = pfSOFCallback;
      pHook->Interval = Interval;
      pHook->Count    = Interval;
      pHook->pContext = pContext;
      pHook->pNext    = pG->pSOFCbHook;
      pG->pSOFCbHook  = pHook;
      return 0;
    }
  }
  return 1;
}

/*********************************************************************
*
*       USBD_RemoveOnSOF
*
*  Function description
*    Removes a callback function which was added via USBD_SetOnSOF() from the callback list.
*
*  Parameters
*    pHook:   Pointer to a USB_SOF_CALLBACK_HOOK structure which was was installed using USBD_SetOnSOF().
*/
void USBD_RemoveOnSOF(const USB_SOF_CALLBACK_HOOK *pHook) {
  USBD_GLOBAL * pG = &USB_Global;
  USB_IOCTL_PARA Param;
  USB_SOF_CALLBACK_HOOK * pPrev;
  USB_SOF_CALLBACK_HOOK * pCurrent;

  pPrev = pG->pSOFCbHook;
  if (pHook == pPrev) {
    pG->pSOFCbHook = pPrev->pNext;
    if (pG->pSOFCbHook == NULL) {
      Param.pfSOFCallback = NULL;
      (void)pG->pDriver->pfControl(USB_DRIVER_CMD_SET_SOF_CALLBACK, &Param);
    }
  } else {
    pCurrent = pPrev->pNext;
    while (pCurrent != NULL) {
      if (pHook == pCurrent) {
        pPrev->pNext = pCurrent->pNext;
        break;
      }
      pPrev = pCurrent;
      pCurrent = pCurrent->pNext;
    }
  }
}

/*************************** End of file ****************************/
