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
Purpose : Handling of enumeration and setup
-------------------------- END-OF-HEADER -----------------------------
*/


#include "USB_Private.h"

/*********************************************************************
*
*       Defines non-configurable
*
**********************************************************************
*/
//lint -esym(750, USB_REQ_*)  D:109
#define USB_REQ_GET_STATUS                        0
#define USB_REQ_CLEAR_FEATURE                     1
#define USB_REQ_SET_FEATURE                       3
#define USB_REQ_SET_ADDRESS                       5
#define USB_REQ_GET_DESCRIPTOR                    6
#define USB_REQ_SET_DESCRIPTOR                    7
#define USB_REQ_GET_CONFIGURATION                 8
#define USB_REQ_SET_CONFIGURATION                 9
#define USB_REQ_GET_INTERFACE                    10
#define USB_REQ_SET_INTERFACE                    11
#define USB_REQ_SYNCH_FRAME                      12
#define USB_REQ_SET_SEL                          48
#define USB_REQ_SET_ISOCH_DELAY                  49

/*********************************************************************
*
*       Static const data
*
**********************************************************************
*/
//
// Binary device object store descriptor
//
static const U8 _USBBOSDscrHead[] = {
    0x05,                              // Descriptor size
    USB_DESC_TYPE_BOS,                 // Device descriptor type
    0x16,0x00,                         // Length of this descriptor and all sub descriptors (updated in function)
    0x01,                              // Number of device capability descriptors (updated in function)
    //
    // USB 2.0 extension
    //
    0x07,                              // Descriptor size
    USB_DESC_TYPE_CAPABILITY,          // Device capability type descriptor
    USB_BOS_CAP_TYPE_USB20_EXTENSION   // USB 2.0 extension capability type
};

//
// WebUSB Platform Capability Descriptor
//
static const U8 _WebUSBDesc[] = {
    24,                                // Descriptor size
    USB_DESC_TYPE_CAPABILITY,          // Device capability type descriptor
    USB_BOS_CAP_TYPE_PLATFORM,         // PLATFORM capability type
    0x00,                              // Reserved
    0x38, 0xB6, 0x08, 0x34,
    0xA9, 0x09, 0xA0, 0x47,            // PlatformCapabilityUUID
    0x8B, 0xFD, 0xA0, 0x76,
    0x88, 0x15, 0xB6, 0x65,
    0x00, 0x01                         // bcdVersion
    //
    // bVendorCode and iLandingPage is added by the function.
    //
};

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

static U8                      _aDescBuffer[USB_DESC_BUFFER_SIZE];
static const USB_WEBUSB_INFO * _pWebUSBInfo;
static I8                      _SetSEL;
#if USBD_OS_USE_ISR_FLAG
  volatile I8                  USBD_IsInInterrupt;
#endif

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

/*********************************************************************
*
*       _CountBits
*/
static unsigned _CountBits(unsigned Mask) {
  unsigned NumBits;

  NumBits = 0;
  while (Mask != 0u) {
    if ((Mask & 1u) != 0u) {
      NumBits++;
    }
    Mask >>= 1;
  }
  return NumBits;
}

/*********************************************************************
*
*       _CalcInterval
*
*  Function description
*    Computes the interval parameter for the end point descriptor
*    according to USB Spec, 9.6.6
*/
static U8 _CalcInterval(const EP_STAT * pEP) {
  U16 Interval;
  U8  Ret;
  int Speed;

  Interval = pEP->Interval;
  Speed    = USB__GetSpeedMode();
  switch (pEP->EPType) {
  default:
    return (Speed >= 2) ? 0 : 1;
    //lint -e{9090} D:102[b]
  case USB_TRANSFER_TYPE_INT:
    if (Speed == 0) {                       // full-speed
      Interval = (Interval + 7u) >> 3;      // Interval is measured in ms for full/low-speed.
      if (Interval == 0u) {
        Interval = 4;                       // Default interval for full-speed INT EPs
      }
      if (Interval > 255u) {
        Interval = 255;
      }
      return Interval;
    }
    if (Interval == 0u) {
      Interval = 8;                         // Default interval for high-speed INT EPs
    }
    break;
  case USB_TRANSFER_TYPE_ISO:
    //
    // Check if Interval is a power of 2
    //
    if (Interval == 0u || ((Interval - 1u) & Interval) != 0u) {
      USB_WARN((USB_MTYPE_ENUMERATION, "Illegal ISO endpoint interval %u", Interval));
    }
    if (Speed == 0) {                       // full-speed
      Interval = (Interval + 7u) >> 3;      // Interval is measured in ms for full/low-speed.
    }
    break;
  }
  //
  // Interval must be given as exponent of a power of 2.
  // Calculate exponent.
  //
  Ret = 0;
  do {
    Ret++;
    Interval >>= 1;
  } while (Interval > 0u);
  return Ret;
}


/*********************************************************************
*
*       USB_IB_Init
*/
void USB_IB_Init(USB_INFO_BUFFER * pInfoBuffer, U8 * pBuffer, unsigned SizeofBuffer) {
  pInfoBuffer->Cnt = 0;
  pInfoBuffer->pBuffer  = pBuffer;
  pInfoBuffer->Sizeof   = SizeofBuffer;
}

/*********************************************************************
*
*       USB_IB_AddU8
*/
void USB_IB_AddU8(USB_INFO_BUFFER * pInfoBuffer, U8 Data) {
  if (pInfoBuffer->Cnt < pInfoBuffer->Sizeof) {
    *(pInfoBuffer->pBuffer + pInfoBuffer->Cnt) = Data;
    pInfoBuffer->Cnt++;
  } else {
    USB_PANIC("Info buffer size too small, increase USB_DESC_BUFFER_SIZE");
  }
}

/*********************************************************************
*
*       USB_IB_AddU16
*/
void USB_IB_AddU16(USB_INFO_BUFFER * pInfoBuffer, U16 Data) {
  USB_IB_AddU8(pInfoBuffer, (U8)(Data & 0xFFu));
  USB_IB_AddU8(pInfoBuffer, (U8)(Data >> 8));
}

/*********************************************************************
*
*       USB_IB_AddU32
*/
void USB_IB_AddU32(USB_INFO_BUFFER * pInfoBuffer, U32 Data) {
  USB_IB_AddU8(pInfoBuffer, (U8)(Data &  0xFFu));
  USB_IB_AddU8(pInfoBuffer, (U8)(Data >>   8));
  USB_IB_AddU8(pInfoBuffer, (U8)(Data >>  16));
  USB_IB_AddU8(pInfoBuffer, (U8)(Data >>  24));
}

/*********************************************************************
*
*       USB_IB_AddBytes
*/
void USB_IB_AddBytes(USB_INFO_BUFFER * pInfoBuffer, const U8 * pData, U32 NumBytes) {
  if ((pInfoBuffer->Cnt + NumBytes) > pInfoBuffer->Sizeof) {
    USB_PANIC("Info buffer size too small, increase USB_DESC_BUFFER_SIZE");
  } else {
    USB_MEMCPY(pInfoBuffer->pBuffer + pInfoBuffer->Cnt, pData, NumBytes);
    pInfoBuffer->Cnt += NumBytes;
  }
}

/*********************************************************************
*
*       USB_IB_GetP
*/
U8 * USB_IB_GetP(const USB_INFO_BUFFER * pInfoBuffer) {
  return pInfoBuffer->pBuffer + pInfoBuffer->Cnt;
}

/*********************************************************************
*
*       _BuildDeviceQualDesc
*
*/
#if USB_OTHER_SPEED_DESC!= 0 && USB_SUPPORT_HIGH_SPEED != 0
static const U8 * _BuildDeviceQualDesc(void) {
  USB_INFO_BUFFER InfoBuffer;

  USB_IB_Init  (&InfoBuffer, &_aDescBuffer[0], sizeof(_aDescBuffer));
  USB_IB_AddU8 (&InfoBuffer, 10);                             // desc length
  USB_IB_AddU8 (&InfoBuffer, USB_DESC_TYPE_QUALIFIER);        // desc type
  USB_IB_AddU16(&InfoBuffer, 0x200);                          // USB Spec Rev (Low Order)
  USB_IB_AddU8 (&InfoBuffer, USB_Global.Class);               // Class
  USB_IB_AddU8 (&InfoBuffer, USB_Global.SubClass);            // Device Subclass
  USB_IB_AddU8 (&InfoBuffer, USB_Global.Protocol);            // Device Protocol
  USB_IB_AddU8 (&InfoBuffer, (U8)USB_aEPStat[0].MaxPacketSize);   // Packet Size         8/16/32/64 are valid
  USB_IB_AddU8 (&InfoBuffer, 0x01);                           // bNumConfigurations: Number of Configurations
  USB_IB_AddU8 (&InfoBuffer, 0x00);                           // Reserved
  return _aDescBuffer;
}
#endif

/*********************************************************************
*
*       _GetString
*
*  Function description
*    Retrieves a string from a given StringIndex.
*
*/
static const char * _GetString(unsigned StringIndex) {
  USB_GET_STRING_DESC_HOOK * pGetStringHook;
  const char               * pString;

  switch (StringIndex) {    //lint --e{9090,9085} D:102[a]
  case STRING_INDEX_MANUFACTURER:   return USB_Global.pDeviceInfo->sVendorName;
  case STRING_INDEX_PRODUCT:        return USB_Global.pDeviceInfo->sProductName;
  case STRING_INDEX_SN:             return USB_Global.pDeviceInfo->sSerialNumber;
  case STRING_INDEX_CONFIG:         return "Configuration";
  default: break;
  }
  if (StringIndex >= STRING_INDEX_OTHER &&
      StringIndex - STRING_INDEX_OTHER < USB_Global.NumStringDesc) {
    return USB_Global.aStringDesc[StringIndex - STRING_INDEX_OTHER];
  }
  //
  // Check if we have callback functions in order to check for other strings.
  //
  for (pGetStringHook = USB_Global.pfGetString; pGetStringHook != NULL; pGetStringHook = pGetStringHook->pNext) {
    pString = pGetStringHook->pfGetString(StringIndex);
    if (pString != NULL) {
      return pString;
    }
  }
  return NULL;
}

/*********************************************************************
*
*       _BuildStringDesc
*
*/
static const U8 * _BuildStringDesc(unsigned StringIndex) {
  USB_INFO_BUFFER InfoBuffer;
  unsigned i;
  unsigned NumChars;
  const char * s;

  //
  // String Index == 0 means Language identifier.
  //
  if (StringIndex == 0u) {
    USB_LOG((USB_MTYPE_ENUMERATION, "ENUM Build Language Descriptor"));
    USB_IB_Init(&InfoBuffer, &_aDescBuffer[0], sizeof(_aDescBuffer));
    USB_IB_AddU8(&InfoBuffer, 4);
    USB_IB_AddU8(&InfoBuffer, USB_DESC_TYPE_STRING);
    USB_IB_AddU16(&InfoBuffer, 0x409);
  } else {
    //
    // Get string from application
    //
    s = _GetString(StringIndex);
    if (s == NULL) {
      return NULL;
    }
    //
    // Build the descriptor from string
    //
    NumChars = USB_STRLEN(s);
    if (NumChars > 126u) {
      USB_PANIC("String descriptor too long");
    }
    USB_IB_Init(&InfoBuffer, &_aDescBuffer[0], sizeof(_aDescBuffer));
    USB_IB_AddU8(&InfoBuffer, (U8)(2u + 2u * NumChars));
    USB_IB_AddU8(&InfoBuffer, USB_DESC_TYPE_STRING);
    for (i = 0; i < NumChars; i++) {
      U16 Char16;
      Char16 = (U8)s[i];
      USB_IB_AddU16(&InfoBuffer, Char16);
    }
  }
  return _aDescBuffer;
}


/*********************************************************************
*
*       _CheckSetIsoPacketSize
*
*   Function description
*     Ask driver to check / set high bandwidth ISO packet size
*     and a packet count (return value)
*/
#if USB_SUPPORT_TRANSFER_ISO
static unsigned _CheckSetIsoPacketSize(int Cmd, unsigned EPIndex, unsigned MaxPacketSize) {
  USB_IOCTL_PARA Para;

  Para.PacketSize.EPIndex = EPIndex;
  Para.PacketSize.MaxPacketSize = MaxPacketSize;
  if (USB_Global.pDriver->pfControl != NULL) {
    if (USB_Global.pDriver->pfControl(Cmd, &Para) == 0) {
      return Para.PacketSize.MaxPacketSize;
    }
  }
  return 0;
}
#endif

/*********************************************************************
*
*       _WriteEP0
*/
static void _WriteEP0(const U8 * pData, unsigned NumBytes, char Send0PacketIfRequired) {
  USBD_WriteEP0FromISR(pData, NumBytes, Send0PacketIfRequired);
}

/*********************************************************************
*
*       _WriteEP0_NULL
*/
static void _WriteEP0_NULL(void) {
  _WriteEP0(NULL, 0, 0);
}

/*********************************************************************
*
*       _CmdSetAddress
*
*  Function description
*    Takes address assigned by the HOST and enters it into the USBA
*    register. This request is different depending on whether the
*    device already has an assigned address or not. Device enters the
*    ADDRESSED state at this point.
*
*/
static void _CmdSetAddress(U8 Addr) {
  if (USB_Global.Flags.SetAddressBehavior != 0u) {
    USB_Global.pDriver->pfSetAddress(Addr);
    if (Addr != 0u) {
      USB__OnStatusChange(USB_Global.State | USB_STAT_ADDRESSED);
    } else {
      USB__OnStatusChange(USB_Global.State & ~USB_STAT_ADDRESSED);
    }
    USB_Global.Addr = Addr;
    if (USB_Global.Flags.SetAddressBehavior == 1u) {
      _WriteEP0_NULL();
    }
  } else {
    _WriteEP0_NULL();
    USB_Global.Addr = Addr | 0x80u;
  }
}

/*********************************************************************
*
*       _BuildInterfaceDesc
*
*/
static void _BuildInterfaceDesc(USB_INFO_BUFFER * pInfoBuffer, int iIF, const INTERFACE * pInterface, int OtherSpeed) {
  unsigned          i;
  unsigned          NumEPs;
  int               SpeedCapability;
  int               SpeedMode;
  unsigned          MaxPacketSize;
  EP_STAT         * pEP;
  NumEPs = _CountBits((unsigned)pInterface->EPs >> 1);     // Do not count EP0
  //
  // Add interface descriptor.  (9 bytes)
  //
  USB_IB_AddU8 (pInfoBuffer, 9);                        // Descriptor len
  USB_IB_AddU8 (pInfoBuffer, USB_DESC_TYPE_INTERFACE);  // Descriptor type: 4
  USB_IB_AddU8 (pInfoBuffer, (U8)iIF);                  // Interface Number
  USB_IB_AddU8 (pInfoBuffer, pInterface->IFAlternateSetting);   // Alternate settings
  USB_IB_AddU8 (pInfoBuffer, NumEPs);                   // No of used EPs
  USB_IB_AddU8 (pInfoBuffer, pInterface->IFClass);      // Interface Class
  USB_IB_AddU8 (pInfoBuffer, pInterface->IFSubClass);   // Interface Subclass
  USB_IB_AddU8 (pInfoBuffer, pInterface->IFProtocol);   // Interface Protocol
  USB_IB_AddU8 (pInfoBuffer, pInterface->iName);   // Interface Descriptor String
  //
  // Add Functional descriptor via optional callback
  //
  if (pInterface->pfAddFuncDesc != NULL) {
    pInterface->pfAddFuncDesc(iIF, pInterface->IFAlternateSetting, pInfoBuffer);
  }
  //
  // Add Endpoint descriptors.  (9 bytes)
  //
  SpeedCapability = USB__GetSpeedCapability();
  SpeedMode       = USB__GetSpeedMode();
  pEP             = &USB_aEPStat[1];
  for (i = 1; i < USB_Global.NumEPs; i++) {
    if ((pInterface->EPs & (1uL << i)) != 0u) {
      USB_IB_AddU8 (pInfoBuffer, 7);                             // Descriptor len
      USB_IB_AddU8 (pInfoBuffer, USB_DESC_TYPE_ENDPOINT);        // Descriptor type: 5
      USB_IB_AddU8 (pInfoBuffer, pEP->EPAddr);
      USB_IB_AddU8 (pInfoBuffer, pEP->EPType | (pEP->Flags & USB_EP_STAT_ISO_TYPES_MASK));         // Endpoint Attributes
      if (OtherSpeed != 0) {
        //
        // The function is called with 'OtherSpeed' != 0 only,
        // if the device is at least HS capable and is enumerated in full- or high-speed.
        // SpeedMode = 0 or 1. Get MaxPacketSize of the other speed.
        //
        MaxPacketSize = USB__CalcMaxPacketSize(pEP->MaxPacketSizeUser, pEP->EPType, 1 - SpeedMode);
      } else {
        if (SpeedCapability > SpeedMode) {
          //
          // Reduce MaxPacketsize to meet requirements for lower speed
          //
          MaxPacketSize = USB__CalcMaxPacketSize(pEP->MaxPacketSizeUser, pEP->EPType, SpeedMode);
        } else {
          //
          // Device enumerated in highest possible speed, take original MaxPacketSize
          //
          MaxPacketSize = pEP->MaxPacketSizeUser;
        }
        //
        // Update current MaxPacketSize of this EP
        //
        pEP->MaxPacketSize = MaxPacketSize;
      }
#if USB_SUPPORT_TRANSFER_ISO
      //
      // Get custom MaxPacketSize for the iso endpoint. (if available)
      //
      if (pEP->EPType == USB_TRANSFER_TYPE_ISO) {
        if (pInterface->pfGetCustomMaxPacketSize != NULL) {
          MaxPacketSize = pInterface->pfGetCustomMaxPacketSize((U8)iIF, pInterface->IFAlternateSetting, pEP->EPAddr, MaxPacketSize);
        }
      }
#endif
      if (MaxPacketSize > 1024u) {
#if USB_SUPPORT_TRANSFER_ISO
        if (pEP->EPType == USB_TRANSFER_TYPE_ISO) {
          if (SpeedMode == 1) {
            //
            // HS ISO high bandwidth
            //
            MaxPacketSize = _CheckSetIsoPacketSize(USB_DRIVER_CMD_CHECK_MAX_PACKET_SIZE, i, MaxPacketSize);
            if (MaxPacketSize == 0u) {
              USB_PANIC("Illegal ISO packet size! Must be rounded up!");
              MaxPacketSize = 1;
            }
          } else {
            unsigned Mult;
            //
            // SuperSpeed ISO high bandwidth with bursts ?
            //
            Mult = (MaxPacketSize - 1u) / (1024u * 16u);
            MaxPacketSize = (Mult << 11) | 1024u;
          }
        } else
#endif
        {
          //
          // BULK SuperSpeed burst EPs
          //
          MaxPacketSize = 1024;
        }
      }
      USB_IB_AddU16(pInfoBuffer, MaxPacketSize);     // Max Packet Size
      USB_IB_AddU8 (pInfoBuffer, _CalcInterval(pEP));  // Polling Interval
      //
      // Add SuperSpeed endpoint companion descriptor
      //
      if (USB_Global.pfAddSSEndpointCompanion != NULL) {
        USB_Global.pfAddSSEndpointCompanion(pEP, pInfoBuffer);
      }
      //
      // Add Functional descriptor via optional callback
      //
      if (pInterface->pfAddEndpointFuncDesc != NULL) {
        pInterface->pfAddEndpointFuncDesc(iIF, pInterface->IFAlternateSetting, pEP->EPAddr, pInfoBuffer);
      }
    }
    pEP++;
  }
}

/*********************************************************************
*
*       _BuildConfigDesc
*
*  Notes
*    (1) The config descriptor (9.6.3 of the spec) includes interface and
*        endpoint descriptors.
*/
static const U8 * _BuildConfigDesc(int OtherSpeed) {
  USB_INFO_BUFFER   InfoBuffer;
  unsigned          iIF;
#if USB_MAX_NUM_ALT_IF > 0u
  unsigned          iAltIF;
  ALT_INTERFACE   * pAltInterface;
#endif

  USB_IB_Init(&InfoBuffer, &_aDescBuffer[0], sizeof(_aDescBuffer));
  //
  // Add the real config descriptor, which includes interface and endpoint descriptors.  (9 bytes)
  //
  USB_IB_AddU8 (&InfoBuffer, 9);                               // Descriptor len of first part of config descriptor
  USB_IB_AddU8 (&InfoBuffer, OtherSpeed != 0 ? USB_DESC_TYPE_SPEED_CONFIG : USB_DESC_TYPE_CONFIG);   // Descriptor type
  USB_IB_AddU16(&InfoBuffer, 0);                                // Data length - to be filled in later
  USB_IB_AddU8 (&InfoBuffer, USB_Global.NumIFs);               // Number of Interfaces Supported
  USB_IB_AddU8 (&InfoBuffer, 1);                               // Config value
  USB_IB_AddU8 (&InfoBuffer, STRING_INDEX_CONFIG);             // iConfig: index of config string descriptor
  USB_IB_AddU8 (&InfoBuffer, USB_Global.ConfigAttr | 0x80u);   // Config Attributes (Self Powered)
  //
  // Max Power in units of 2mA (high-speed) or 8mA (SuperSpeed)
  //
  USB_IB_AddU8 (&InfoBuffer, USB__GetSpeedMode() >= 2 ? USB_Global.MaxPowerSS : USB_Global.MaxPower);
#if USB_MAX_NUM_IF > 1u
  for (iIF = 0; iIF < USB_Global.NumIFs; iIF++)
#else
  iIF = 0;
#endif
  {
    if (USB_Global.pIadAPI != NULL) {
      USB_Global.pIadAPI->pfAddIadDesc(iIF, &InfoBuffer);
    }
    _BuildInterfaceDesc(&InfoBuffer, iIF, &USB_Global.aIF[iIF], OtherSpeed);
#if USB_MAX_NUM_ALT_IF > 0u
    pAltInterface = USB_Global.aAltIF;
    for (iAltIF = 0; iAltIF < USB_Global.NumAltIFs; iAltIF++) {
      if (pAltInterface->IFNo == iIF) {
       if (USB__GetSpeedMode() >= pAltInterface->MinSpeed) {
          _BuildInterfaceDesc(&InfoBuffer, iIF, &pAltInterface->AltIF, OtherSpeed);
        } else {
          USB_WARN((USB_MTYPE_ENUMERATION, "Iterface %u alternate %u not compatible with enumerated speed: Dropped", iIF, iAltIF));
        }
      }
      ++pAltInterface;
    }
#endif
  }
  //
  // Fill in length of config descriptor
  //
  USBD_StoreU16LE(&_aDescBuffer[2], InfoBuffer.Cnt);
  return _aDescBuffer;
}

/*********************************************************************
*
*       _BuildDeviceDesc
*
*/
static const U8 * _BuildDeviceDesc(void) {
  int             SpeedMode;
  U16             USBSpecRev;
  USB_INFO_BUFFER InfoBuffer;

  USB_IB_Init  (&InfoBuffer, &_aDescBuffer[0], sizeof(_aDescBuffer));
  SpeedMode = USB__GetSpeedMode();
  USB_IB_AddU8 (&InfoBuffer, 18);                                                // desc length
  USB_IB_AddU8 (&InfoBuffer, USB_DESC_TYPE_DEVICE);                              // desc type
  USBSpecRev = USB_Global.USBSpecRev;
  if (SpeedMode < 2 && USBSpecRev > 0x0210u) {
    USBSpecRev = 0x0210;
  }
  USB_IB_AddU16(&InfoBuffer, USBSpecRev);                                        // USB Spec Rev (Low Order)
  USB_IB_AddU8 (&InfoBuffer, USB_Global.Class);
  USB_IB_AddU8 (&InfoBuffer, USB_Global.SubClass);
  USB_IB_AddU8 (&InfoBuffer, USB_Global.Protocol);
  USB_IB_AddU8 (&InfoBuffer, SpeedMode >= 2 ? 9u : USB_aEPStat[0].MaxPacketSize);   // Packet Size 8/16/32/64 or 09 == 512
  USB_IB_AddU16(&InfoBuffer, USB_Global.pDeviceInfo->VendorId);
  USB_IB_AddU16(&InfoBuffer, USB_Global.pDeviceInfo->ProductId);
  USB_IB_AddU16(&InfoBuffer, USB_Global.pfOnBCDVersion());                       // Device Release Number (BCD)
  USB_IB_AddU8 (&InfoBuffer, STRING_INDEX_MANUFACTURER);                         // Index of String Desc (Manuf)
  USB_IB_AddU8 (&InfoBuffer, STRING_INDEX_PRODUCT);                              // Index of String Desc (Product)
  if (USB_Global.pDeviceInfo->sSerialNumber != NULL) {
    USB_IB_AddU8(&InfoBuffer, STRING_INDEX_SN);                                  // Index of String Desc (Serial #)
  } else {
    USB_IB_AddU8(&InfoBuffer, 0x00u);                                            //  0 - do not send serial number
  }
  USB_IB_AddU8 (&InfoBuffer, 0x01);                                              // Number of Configurations
  return _aDescBuffer;
}

/*********************************************************************
*
*       _CmdGetDescriptor
*
*  Function description
*    Determines what descriptor string is to be returned to the HOST.
*    Calls SendEP0Data to transfer data to HOST.
*/
static void _CmdGetDescriptor(const USB_SETUP_PACKET * pSetupPacket) {
  int       LenType;
  unsigned  Len;
  const U8* pData = NULL;
  unsigned  MaxLen;
  char      Send0PacketIfRequired;

  LenType = 0;
  switch (pSetupPacket->wValueHigh) {    // Mask to determine what descriptor is being asked.
  case USB_DESC_TYPE_DEVICE:             // Send Device Descriptor.
    USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: GetDeviceDescriptor"));
    pData = _BuildDeviceDesc();
    break;
  case USB_DESC_TYPE_CONFIG:             // Send Config Descriptor.
    USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: GetConfigDescriptor"));
    pData = _BuildConfigDesc(0);
    LenType = 1;
    break;
  case USB_DESC_TYPE_STRING:
    USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: GetStringDescriptor(%d)", pSetupPacket->wValueLow));
    pData = _BuildStringDesc(pSetupPacket->wValueLow);
    break;
#if USB_SUPPORT_HIGH_SPEED
#if USB_OTHER_SPEED_DESC
  case USB_DESC_TYPE_QUALIFIER:
    USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: GetDeviceQualifierDescriptor"));
    if (USB__GetSpeedCapability() != 0 && USB__GetSpeedMode() < 2) {
      pData = _BuildDeviceQualDesc();
    }
    break;
  case USB_DESC_TYPE_SPEED_CONFIG:
    USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: GetOtherSpeedSettingsDescriptor"));
    if (USB__GetSpeedCapability() != 0 && USB__GetSpeedMode() < 2) {
      pData = _BuildConfigDesc(1);
      LenType = 1;
    }
    break;
#endif
  case USB_DESC_TYPE_BOS:
    USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: Get BOS Descriptor"));
    if (USB_Global.pfBuildBOSDesc != NULL) {
      pData = USB_Global.pfBuildBOSDesc();
      LenType = 1;
    }
    break;
#endif
  default:
    // MISRA comment
    break;
  }
  if (pData == NULL) {     // Unhandled descriptor type.
    USB_Global.pDriver->pfStallEP0();
    return;
  }
  if (LenType != 0) {
    Len = USBD_GetU16LE(pData + 2);
  } else {
    Len = pData[0];
  }
  //
  // Make sure we send no more data than what the host asks for.
  //
  Send0PacketIfRequired = 1;
  MaxLen = ((unsigned)(pSetupPacket->wLengthHigh) << 8) | (pSetupPacket->wLengthLow);
  if (Len >= MaxLen) {
    Len = MaxLen;
    //
    // Make sure we do not send a ZLP when the descriptor matches the EP0 max size.
    //
    Send0PacketIfRequired = 0;
  }
  //
  // Send the data.
  //
  _WriteEP0(pData, Len, Send0PacketIfRequired);
}

/*********************************************************************
*
*       _FindEP
*/
static int _FindEP(U8 EPAddr) {
  unsigned i;

  if ((EPAddr & 0x7Fu) == 0u) {
    return 0;
  }
  for (i = 0; i < USB_Global.NumEPs; i++) {
    if (USB_aEPStat[i].EPAddr == EPAddr) {
      return i;
    }
  }
  return -1;
}

/*********************************************************************
*
*       _CmdGetStatus
*
*  Function description
*    Returns various status information to the HOST about the state of the device
*    only valid in the Default and Configured State. If a request for status is
*    made that is invalid, then the device STALLs the request per the USB spec.
*    Note that there is no valid information for INTERFACE status, but the
*    device just returns 0x00 0x00 per the spec. Valid endpoint status is
*    also returned with this command. Linked to SET/CLEAR FEATURE requests.
*/
static void _CmdGetStatus(const USB_SETUP_PACKET * pSetupPacket) {
#if USB_SUPPORT_STATUS == 0
  (void)pSetupPacket;                          // Cmd not supported, low ROM memory configuration
#else
  int i;

  if (pSetupPacket->wValueLow != 0u || pSetupPacket->wValueHigh != 0u ||
      pSetupPacket->wLengthLow != 2u || pSetupPacket->wLengthHigh != 0u) {
    goto RequestError;      // send STALL and leave function
  }
  switch (pSetupPacket->bmRequestType & 0x0Fu) {         //lint --e{9042}  D:102[a]
  case 0:
    //
    // Device Status
    //
    _aDescBuffer[0] = USB_Global.DeviceState;
    break;
  case 1:
    //
    // Interface status
    //
    if ((USB_Global.State & USB_STAT_CONFIGURED) == 0u || pSetupPacket->wIndexLow >= USB_Global.NumIFs) {
      goto RequestError;
    }
    _aDescBuffer[0] = 0;
    break;
  case 2:
    //
    // Endpoint status
    //
    if ((USB_Global.State & USB_STAT_CONFIGURED) == 0u && pSetupPacket->wIndexLow != 0u) {
      goto RequestError;
    }
    i = _FindEP(pSetupPacket->wIndexLow);
    if (i < 0) {
      goto RequestError;
    }
    _aDescBuffer[0] = (U8)USB_aEPStat[i].IsHalted;
    break;
  default:
    goto RequestError;
    //lint -e{9077}  D:102[a]
  }
  _aDescBuffer[1] = 0;
  _WriteEP0(_aDescBuffer, 2, 0);
  return;
RequestError:
#endif
  USB_Global.pDriver->pfStallEP0();
}

/*********************************************************************
*
*       _SetClrFeature3
*
*  Function description
*    Implement Set and Clear Feature commands for USB 3.0
*/
static int _SetClrFeature3(U8 Recipient, U16 wValue, int SetClr) {
  unsigned       Mask;
  int            Cmd;
  USB_IOCTL_PARA Para;

  if (Recipient == 1u && wValue == 0u) {
    //
    // FUNCTION_SUSPEND (USB 3.0)
    //
    _WriteEP0_NULL();
    return 0;
  }
  if (Recipient == 0u && (USB_Global.State & USB_STAT_CONFIGURED) != 0u) {
    Cmd = -1;
    switch(wValue) {
    //
    // Feature selectors for SuperSpeed devices
    //
    case 48:
      Mask = USB_DEVSTAT_U1_ENABLE;
      Cmd  = USB_DRIVER_CMD_U1_CONTROL;
      break;
    case 49:
      Mask = USB_DEVSTAT_U2_ENABLE;
      Cmd  = USB_DRIVER_CMD_U2_CONTROL;
      break;
    case 50:
      Mask = USB_DEVSTAT_LPM_ENABLE;
      break;
    default:
      return 1;
      //lint -e{9077}  D:102[a]
    }
    if (Cmd >= 0 && USB_Global.pDriver->pfControl != NULL) {
      Para.LPM_Enable = SetClr;
      (void)USB_Global.pDriver->pfControl(Cmd, &Para);
    }
    if (SetClr != 0) {
      USB_Global.DeviceState |=  Mask;
    } else {
      USB_Global.DeviceState &= ~Mask;
    }
    _WriteEP0_NULL();
    return 0;
  }
  return 1;
}

/*********************************************************************
*
*       _SetClrFeature
*
*  Function description
*    Implement Set and Clear Feature commands
*/
static void _SetClrFeature(const USB_SETUP_PACKET * pSetupPacket, int SetFeature) {
  U16             wValue;
  U8              Recipient;
  int             i;

  wValue    = pSetupPacket->wValueLow  | ((U16)pSetupPacket->wValueHigh << 8);
  Recipient = pSetupPacket->bmRequestType & 0x0Fu;
#if USB_SUPPORT_TEST_MODE
  //
  //  Check if we need to enter TEST_MODE
  //
  if (wValue == 2u) {
    if (pSetupPacket->bmRequestType == 0u && SetFeature != 0) {
      U8  TestMode;

      TestMode = pSetupPacket->wIndexHigh;
      if (TestMode != 0u && TestMode < USB_CMD_TESTMODE_TEST_FORCE_ENABLE &&
          USB__GetSpeedMode() != 0 && USB_Global.pDriver->pfControl != NULL) {
        USB_Global.TestMode = TestMode;
        _WriteEP0_NULL();
      } else {
        USB_Global.pDriver->pfStallEP0();
      }
      return;
    }
  }
#endif
  if (pSetupPacket->wLengthLow != 0u || pSetupPacket->wLengthHigh != 0u) {
    goto RequestError;      // send STALL and leave function
  }
  if (Recipient == 2u && wValue == 0u) {
    //
    // Endpoint Halt
    //
    i = _FindEP(pSetupPacket->wIndexLow);
    if (i < 0) {
      goto RequestError;
    }
    if (i == 0) {
      //
      // EP0
      //
      USB_aEPStat[0].IsHalted = SetFeature;
    } else {
      //
      // Other EPs
      //
      if ((USB_Global.State & USB_STAT_CONFIGURED) == 0u) {
        goto RequestError;
      }
      //
      // Remove all pending operations on that endpoint.
      // Invalidate all internal saved data
      //
      USB__RemovePendingOperation((unsigned)i, -1);
      USBD_SetClrStallEP((unsigned)i, (U8)SetFeature);
      if (SetFeature == 0) {
        //
        // Set the data toggles bit back to 0.
        //
        USB__ResetDataToggleEP((unsigned)i);
      }
    }
    _WriteEP0_NULL();
    return;
  }
  if (Recipient == 0u && wValue == 1u) {
    //
    // Allow remote wakeup (USB 2.0)
    //
    if (SetFeature != 0) {
      USB_Global.DeviceState |=  USB_DEVSTAT_REMOTE_WAKEUP_ALLOWED;
    } else {
      USB_Global.DeviceState &= ~USB_DEVSTAT_REMOTE_WAKEUP_ALLOWED;
    }
    _WriteEP0_NULL();
    return;
  }
  if (USB_Global.pfSetClrFeature != NULL) {
    if (USB_Global.pfSetClrFeature(Recipient, wValue, SetFeature) == 0) {
      return;
    }
  }
RequestError:
  USB_Global.pDriver->pfStallEP0();
}

/*********************************************************************
*
*       _CmdGetConfiguration
*
*  Function description
*    Returns the current configuration value to the HOST. If the device
*    is un-configured then this request should return 0x00. Associated
*    with the SET_CONFIGURATION request.
*
*/
static void _CmdGetConfiguration(void) {
  U8 *pData = _aDescBuffer;
  *pData = (USB_Global.State & USB_STAT_CONFIGURED) != 0u ? 1 : 0;
  _WriteEP0(pData, 1, 0);
}

/*********************************************************************
*
*       _CmdGetInterface
*
*  Function description
*    Returns the correct ALTERNATE_INTERFACE setting of the current
*    configuration.
*/
static void _CmdGetInterface(const USB_SETUP_PACKET * pSetupPacket){
  U8 wLength;
  U8 Data;
  U8 wIndex;

  wLength = pSetupPacket->wLengthLow;
  wIndex  = pSetupPacket->wIndexLow; // Interface number.
  if (pSetupPacket->wValueLow != 0x00u || wLength != 0x01u || wIndex >= USB_Global.NumIFs) {
    USB_Global.pDriver->pfStallEP0();
    return;               // Incorrect values so STALL here
  }
  if (((USB_Global.State & (USB_STAT_ADDRESSED | USB_STAT_CONFIGURED)) != (USB_STAT_ADDRESSED | USB_STAT_CONFIGURED))) {   // If in DEFAULT or ADDRESSED
    USB_Global.pDriver->pfStallEP0();
    return;
  }
#if USB_SUPPORT_GET_INTERFACE_CB
  if (USB_Global.aIF[wIndex].OnGetInterfaceHook.cb.Get != NULL) {
    USB_Global.aIF[wIndex].OnGetInterfaceHook.cb.Get(wIndex, USB_Global.aIF[wIndex].OnGetInterfaceHook.pContext);
  }
#endif
  //
  // Send answer (1 byte)
  //
#if USB_MAX_NUM_ALT_IF > 0u
  Data = USB_Global.aIF[wIndex].CurrentAltSetting;
#else
  Data = 0;
#endif
  _WriteEP0(&Data, 1, 0);
}

/*********************************************************************
*
*       _CmdSetInterface
*
*  Function description
*    The HOST uses this command to set the current interface to a different
*    (alternate) setting. This interface is a subset of its configuration
*    and allows the HOST to adjust bandwidth or endpoints according to
*    bus loading or need. This is necessary for ISO endpoints so that they can
*    be shut off if bus loading is too great. Alternate interfaces for
*    simple devices (only a few EP's, bulk & interrupts in nature) might
*    only have a singular interface and would not have to include an
*    alt. interface. In that case, the EP descriptors would simply follow
*    the interface descriptor and both bCurInt and bAltSet would = 0x00.
*    Alternate interfaces should be described in order of endpoint usage.
*    For example, Alt. Int. 0 should describe no endpoints (only EP0 active).
*    Alt. Int. 1 can describe the endpoints used and their sizes in increasing size,
*    and so on. Endpoint MAXP adjustments and enabling should be made
*    here, not with the SET_CONFIGURATION command, as it just specifies
*    a configuration to use. This is more advanced that this example, but
*    should be noted here for clarity.
*
*  Notes
*   (1)  - in the USB specification it is said the a set interface can only be set
*          when the device is in an addressed and configured state, otherwise this
*          needs to be ignored.
*
*/
static void _CmdSetInterface(const USB_SETUP_PACKET * pSetupPacket){
  const USB_HW_DRIVER * pDriver;
  U8        wIndex;
  unsigned  wValue;
  unsigned  i;
  U32       EPs;
  INTERFACE *pInterface;

  wIndex  = pSetupPacket->wIndexLow; // Interface number.
  pDriver = USB_Global.pDriver;
  //
  // If device is neither addressed nor configured, EP0 is stalled, s. note 1
  //
  if (((USB_Global.State & (USB_STAT_ADDRESSED | USB_STAT_CONFIGURED)) != (USB_STAT_ADDRESSED | USB_STAT_CONFIGURED))) {   // If in DEFAULT or ADDRESSED
    pDriver->pfStallEP0();
    return;
  }
  if (wIndex > USB_Global.NumIFs) {
    pDriver->pfStallEP0();
    return;
  }
  //
  // wValue contains the alternate setting number
  // 0      - No alternate setting
  // 1~255  - Alternate setting 1 to 255
  //
  if (pSetupPacket->wValueHigh == 0u) {
    _WriteEP0_NULL();
    wValue = pSetupPacket->wValueLow;
    pInterface = &USB_Global.aIF[wIndex];
    if (wValue == 0u) {
      //
      // Regular interfaces.
      //
#if USB_MAX_NUM_ALT_IF > 0u
      pInterface->CurrentAltSetting = 0;
#endif
      EPs = pInterface->EPs;
      for (i = 1; i < USB_Global.NumEPs; i++) {
        if (((1uL << i) & EPs) != 0u) {
          USB__RemovePendingOperation(i, -1);
          USBD_SetClrStallEP(i, 0);
          USB__ResetDataToggleEP(i);
#if USB_SUPPORT_TRANSFER_ISO
          //
          // Reconfigure endpoint
          //
          if (USB_aEPStat[i].EPType == USB_TRANSFER_TYPE_ISO) {
            (void)_CheckSetIsoPacketSize(USB_DRIVER_CMD_SET_ALT_PACKET_SIZE, i, USB_aEPStat[i].MaxPacketSize);
          }
#endif
        }
      }
    }
#if USB_MAX_NUM_ALT_IF > 0u
    else {
      unsigned j;
      ALT_INTERFACE *pAltInterface;

      pInterface->CurrentAltSetting = wValue;
      //
      // Alternate interfaces.
      //
      pAltInterface = USB_Global.aAltIF;
      for (j = 0; j < USB_MAX_NUM_ALT_IF; j++) {
        if (wIndex == pAltInterface->IFNo && wValue == pAltInterface->AltIF.IFAlternateSetting) {
          EPs = pAltInterface->AltIF.EPs;
          for (i = 1; i < USB_Global.NumEPs; i++) {
            if (((1uL << i) & EPs) != 0u) {
              USB__RemovePendingOperation(i, -1);
              USBD_SetClrStallEP(i, 0);
              USB__ResetDataToggleEP(i);
#if USB_SUPPORT_TRANSFER_ISO
              //
              // Get custom MaxPacketSize for the ISO endpoint (if available)
              // and reconfigure endpoint
              //
              if (pAltInterface->AltIF.pfGetCustomMaxPacketSize != NULL) {
                EP_STAT * pEP = &USB_aEPStat[i];

                if (pEP->EPType == USB_TRANSFER_TYPE_ISO) {
                  (void)_CheckSetIsoPacketSize(USB_DRIVER_CMD_SET_ALT_PACKET_SIZE, i,
                                               pAltInterface->AltIF.pfGetCustomMaxPacketSize(j, wValue, pEP->EPAddr, pEP->MaxPacketSize));
                }
              }
#endif
            }
          }
        }
        pAltInterface++;
      }
    }
#endif
    //
    // Execute the callback if available.
    //
    if (USB_Global.aIF[wIndex].OnSetInterfaceHook.cb.Set != NULL) {
      USB_Global.aIF[wIndex].OnSetInterfaceHook.cb.Set(wIndex, wValue, USB_Global.aIF[wIndex].OnSetInterfaceHook.pContext);
    }
    return;
  }
  pDriver->pfStallEP0();
}

/*********************************************************************
*
*       _CmdSetConfiguration
*
*  Function description
*    Handles "Set Configuration" requests acc. to [USB] 9.4.7
*
*  Notes
*    (1)  USB Spec
*         The lower byte of the wValue field specifies the desired configuration. This configuration value must be
*         zero or match a configuration value from a configuration descriptor. If the configuration value is zero, the
*         device is placed in its Address state. The upper byte of the wValue field is reserved.
*         If wIndex, wLength, or the upper byte of wValue is non-zero, then the behavior of this request is not
*         specified.
*         Default state: Device behavior when this request is received while the device is in the Default state
*                        is not specified.
*         Address state: If the specified configuration value is zero, then the device remains in the Address
*                        state. If the specified configuration value matches the configuration value from a
*                        configuration descriptor, then that configuration is selected and the device enters the
*                        Configured state. Otherwise, the device responds with a Request Error.
*         Configured state: If the specified configuration value is zero, then the device enters the Address state.
*                        If the specified configuration value matches the configuration value from a
*                        configuration descriptor, then that configuration is selected and the device remains in
*                        the Configured state. Otherwise, the device responds with a Request Error.
*/
static void _CmdSetConfiguration(const USB_SETUP_PACKET * pSetupPacket) {
  U8 v;
  U8 State;

  v = pSetupPacket->wValueLow;
  if (v > 1u) {
    USB_Global.pDriver->pfStallEP0();
    return;
  }
  _WriteEP0_NULL();    // Send 0-byte packet as confirmation (status stage)
  State = USB_Global.State;
  if (v == 0u) {
    State &= ~USB_STAT_CONFIGURED;
  } else {
    State |= USB_STAT_CONFIGURED;
  }
  USB__OnStatusChange(State);
}

/*********************************************************************
*
*       _AddSSEndpointCompanion
*
*  Function description
*    Add SuperSpeed endpoint companion descriptor
*/
static void _AddSSEndpointCompanion(const EP_STAT *pEPStat, USB_INFO_BUFFER *pInfoBuffer) {
  unsigned Bursts;
  unsigned Mult;

  if (USB__GetSpeedMode() != 2) {
    return;
  }
  USB_IB_AddU8(pInfoBuffer, 6);                            // Descriptor size
  USB_IB_AddU8(pInfoBuffer, USB_DESC_TYPE_SS_EP_COMPN);    // SS endpoint companion descriptor type
  Bursts = ((unsigned)pEPStat->MaxPacketSize - 1u) / 1024u;
#if USB_SUPPORT_TRANSFER_ISO
  Mult = Bursts / 16u;
  //
  // Mult > 0 can happen for ISO EPs only
  //
  Bursts /= (Mult + 1u);
#else
  Mult = 0;
#endif
  USB_IB_AddU8(pInfoBuffer, Bursts);                       // Max no. of packets in a burst - 1
  USB_IB_AddU8(pInfoBuffer, Mult);                         // Mult value for ISO
  USB_IB_AddU16(pInfoBuffer, pEPStat->MaxPacketSize);      // Number of bytes per interval
}

/*********************************************************************
*
*       _AddSSDeviceCap
*
*  Function description
*    Add SuperSpeed device capability descriptor
*/
static unsigned _AddSSDeviceCap(USB_INFO_BUFFER *pInfoBuffer) {
  U16      SpeedSupport;
  USB_IOCTL_PARA Param;

  if (USB__GetSpeedCapability() < 2) {
    return 0;
  }
  //
  // SuperSpeed device capability
  //
  USB_IB_AddU8(pInfoBuffer, 0x0A);                                   // Descriptor size
  USB_IB_AddU8(pInfoBuffer, USB_DESC_TYPE_CAPABILITY);               // Device capability type descriptor
  USB_IB_AddU8(pInfoBuffer, USB_BOS_CAP_TYPE_SUPER_SPEED);           // SuperSpeed device capability type
  USB_IB_AddU8(pInfoBuffer, 0x00);                                   // Supported device level features
  SpeedSupport = (0xFFuL >> (5u + USB_Global.MinimumSpeed)) << (USB_Global.MinimumSpeed + 1u);
  USB_IB_AddU16(pInfoBuffer, SpeedSupport);                          // Speeds supported by the device : SS, HS and FS
  USB_IB_AddU8(pInfoBuffer, USB_Global.MinimumSpeed + 1u);           // Functionality support
  (void)USB_Global.pDriver->pfControl(USB_DRIVER_CMD_GET_EXIT_LATENCY, &Param);
  USB_IB_AddU8(pInfoBuffer, Param.ExitLatency.U1);                   // U1 Device Exit latency
  USB_IB_AddU16(pInfoBuffer, Param.ExitLatency.U2);                  // U2 Device Exit latency
  return 1;
}

/*********************************************************************
*
*       _BuildBOSDesc
*
*/
static const U8 * _BuildBOSDesc(void) {
  USB_INFO_BUFFER InfoBuffer;
  USB_IOCTL_PARA  Param;
  U32             LPM_Config;

  USB_IB_Init(&InfoBuffer, &_aDescBuffer[0], sizeof(_aDescBuffer));
  USB_IB_AddBytes(&InfoBuffer, _USBBOSDscrHead, sizeof(_USBBOSDscrHead));
  Param.LPM_Config = 0;
  if (USB_Global.pDriver->pfControl != NULL) {
    (void)USB_Global.pDriver->pfControl(USB_DRIVER_CMD_GET_LPM_CONFIG, &Param);
  }
  LPM_Config = Param.LPM_Config;
  if ((LPM_Config & ((1uL << 4) | (1uL << 3) | (1uL << 1))) == (1uL << 1)) {
    //
    // if LPM enabled and BESL values not set by the driver
    // --> Add BESL values configured by the application.
    //
    LPM_Config |= (U32)USB_Global.RecommendedBESLValue;
  }
  USB_IB_AddU32(&InfoBuffer, LPM_Config);
  if (USB_Global.pfAddSSDeviceCap != NULL) {
    //
    // Add SuperSpeed device capability descriptor
    //
    _aDescBuffer[4] += USB_Global.pfAddSSDeviceCap(&InfoBuffer);
  }
  _aDescBuffer[4] += USB__AddPlatformDesc(&InfoBuffer);
  //
  // Add WebUSB Platform Capability Descriptor
  //
  if (USB_Global.pfAddWebUSBDesc != NULL) {
    _aDescBuffer[4] += USB_Global.pfAddWebUSBDesc(&InfoBuffer);
  }
  //
  // Fill in total length of descriptor
  //
  USBD_StoreU16LE(&_aDescBuffer[2], InfoBuffer.Cnt);
  return _aDescBuffer;
}

/*********************************************************************
*
*       _AddWebUSBDesc
*
*  Function description
*    Add WebUSB Platform Capability Descriptor
*/
static unsigned _AddWebUSBDesc(USB_INFO_BUFFER *pInfoBuffer) {
  USB_IB_AddBytes(pInfoBuffer, _WebUSBDesc, sizeof(_WebUSBDesc));
  USB_IB_AddU8(pInfoBuffer, _pWebUSBInfo->VendorCode);                // bVendorCode
  USB_IB_AddU8(pInfoBuffer, _pWebUSBInfo->DescIndex);                 // iLandingPage
  return 1;
}

/*********************************************************************
*
*       _OnSetupWebUSB
*
*  Function description
*    This function is called by the USB stack when a setup request is received.
*    It is used for the processing of WebUSB descriptors.
*/
static int _OnSetupWebUSB(const USB_SETUP_PACKET * pSetupPacket) {
  unsigned ReqLen;
  unsigned Len;

  if (pSetupPacket->bmRequestType != 0xC0u ||                 // Device to Host, vendor specific
      pSetupPacket->bRequest != _pWebUSBInfo->VendorCode ||
      pSetupPacket->wValueLow != _pWebUSBInfo->DescIndex ||
      pSetupPacket->wIndexLow != 2u) {                        // GET_URL
    return 1;
  }
  Len = USB_STRLEN(_pWebUSBInfo->sURL) + 3u;
  if (Len > 255u) {
    Len = 255u;
  }
  _aDescBuffer[0] = Len;
  _aDescBuffer[1] = 3;                                // WEBUSB_URL
  _aDescBuffer[2] = _pWebUSBInfo->URLPrefix;
  USB_MEMCPY(_aDescBuffer + 3, _pWebUSBInfo->sURL, Len - 3u);
  ReqLen = ((unsigned)pSetupPacket->wLengthHigh << 8) | pSetupPacket->wLengthLow;
  USBD_WriteEP0FromISR(_aDescBuffer, SEGGER_MIN(Len, ReqLen), 1);
  return 0;
}

/*********************************************************************
*
*       _OnRxEP0
*
*  Function description
*    Called when data is received on EP0.
*    Used for SetSEL request.
*/
static void _OnRxEP0(const U8 * pData, unsigned NumBytes) {
  if (_SetSEL != 0) {
    if (NumBytes == 6u) {
      USB_USE_PARA(pData);
      USBD_WriteEP0FromISR(NULL, 0, 0);
    } else {
      USBD_StallEP(0);
    }
  }
  _SetSEL = 0;
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/

/*********************************************************************
*
*       USB__OnTx0Done
*
*  Function description
*    Called when a Tx transfer on EP0 is completed
*/
void USB__OnTx0Done(void) {
  U8 Addr;

  if ((USB_Global.Addr & 0x80u) != 0u) {
    Addr = USB_Global.Addr & 0x7Fu;
    USB_Global.Addr = Addr;
    USB_Global.pDriver->pfSetAddress(Addr);
    if (Addr != 0u) {
      USB__OnStatusChange(USB_Global.State | USB_STAT_ADDRESSED);
    } else {
      USB__OnStatusChange(USB_Global.State & ~USB_STAT_ADDRESSED);
    }
  }
#if USB_SUPPORT_TEST_MODE
  //
  //  Check if we need to enter TEST_MODE
  //
  if (USB_Global.TestMode != 0u) {
    USB_IOCTL_PARA  Param;

    Param.TestMode = USB_Global.TestMode;
    (void)USB_Global.pDriver->pfControl(USB_DRIVER_CMD_TESTMODE, &Param);   // pDriver->pfControl != NULL was checked before
    USB_PANIC("Test mode needs power cycle");
  }
#endif
}

/*********************************************************************
*
*       USB__HandleSetup
*
*  Function description
*    Determines what request type, (standard, class, or vendor) is being
*    asked for.
*
*/
void USB__HandleSetup(const USB_SETUP_PACKET * pSetupPacket) {
  const USB_HW_DRIVER * pDriver;
  USB_SETUP_HOOK      * pHook;
#if USB_SUPPORT_CLASS_REQUESTS > 0 || USB_SUPPORT_VENDOR_REQUESTS > 0
  unsigned              InterfaceIndex;
  INTERFACE           * pInterface;
#if USB_SUPPORT_CLASS_REQUESTS
  int                   Endpoint;
  unsigned              i;
#endif
#endif

  USB_LOG((USB_MTYPE_ENUMERATION, "Enter HandleSetup: Setup %02x %02x %02x %02x %02x %02x %02x %02x", pSetupPacket->bmRequestType, pSetupPacket->bRequest,
                                                                                                      pSetupPacket->wValueLow,     pSetupPacket->wValueHigh,
                                                                                                      pSetupPacket->wIndexLow,     pSetupPacket->wIndexHigh,
                                                                                                      pSetupPacket->wLengthLow,    pSetupPacket->wLengthHigh));
  for (pHook = USB_Global.pFirstSetupHook; pHook != NULL; pHook = pHook->pNext) {
    if (pHook->cb(pSetupPacket) == 0) {
      return;
    }
  }
  pDriver = USB_Global.pDriver;
  switch(pSetupPacket->bmRequestType & 0x60u) {  // Mask off unnecessary bits
  case 0x00:
    switch (pSetupPacket->bRequest) {
    case USB_REQ_GET_STATUS:
      USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: GetStatus"));
      _CmdGetStatus(pSetupPacket);
      break;
    case USB_REQ_CLEAR_FEATURE:
      USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: ClearFeature"));
      _SetClrFeature(pSetupPacket, 0);
      break;
    case USB_REQ_SET_FEATURE:
      USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: SetFeature"));
      _SetClrFeature(pSetupPacket, 1);
      break;
    case USB_REQ_SET_ADDRESS:
      USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: SetAddress"));
      _CmdSetAddress(pSetupPacket->wValueLow);
      break;
    case USB_REQ_GET_DESCRIPTOR:
      USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: GetDescriptor"));
      _CmdGetDescriptor(pSetupPacket);
      break;
    case USB_REQ_GET_CONFIGURATION:
      USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: GetConfiguration"));
      _CmdGetConfiguration();
      break;
    case USB_REQ_SET_CONFIGURATION:
      USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: SetConfiguration"));
      _CmdSetConfiguration(pSetupPacket);
      break;
    case USB_REQ_GET_INTERFACE:
      USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: GetInterface"));
      _CmdGetInterface(pSetupPacket);
      break;
    case USB_REQ_SET_INTERFACE:
      USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: SetInterface"));
      _CmdSetInterface(pSetupPacket);
      break;
    case USB_REQ_SET_SEL:
      _SetSEL = 1;
      break;
    case USB_REQ_SET_ISOCH_DELAY:
      _WriteEP0_NULL();   // Ack and ignore.
      break;
    default:
      pDriver->pfStallEP0();
      break;
    }
    break;
#if USB_SUPPORT_CLASS_REQUESTS
  case 0x20:              // Class specific requests
    USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: OnClassRequest"));
    pInterface = NULL;
    if ((pSetupPacket->bmRequestType & 0x1Fu) == 2u) { // Recipient is Endpoint
      Endpoint = _FindEP(pSetupPacket->wIndexLow);
      if (Endpoint >= 0) {
        //
        // Find the Interface to which the endpoint belongs.
        // Check regular interfaces.
        //
        for (i = 0; i < USB_Global.NumIFs; i++) {
          if ((USB_Global.aIF[i].EPs & (1uL << (unsigned)Endpoint)) != 0u) {
            pInterface = &USB_Global.aIF[i];
            break;
          }
        }
        //
        // Check alternate interfaces.
        //
#if USB_MAX_NUM_ALT_IF > 0u
        for (i = 0; i < USB_Global.NumAltIFs; i++) {
          if ((USB_Global.aAltIF[i].AltIF.EPs & (1uL << (unsigned)Endpoint)) != 0u) {
            pInterface = &USB_Global.aAltIF[i].AltIF;
            break;
          }
        }
#endif
      }
    } else { // Recipient is Device/Interface/Other/Reserved
      InterfaceIndex = pSetupPacket->wIndexLow;
      if (InterfaceIndex < USB_Global.NumIFs) {
        pInterface = &USB_Global.aIF[InterfaceIndex];
      }
    }
    if (pInterface != NULL) {
      if (pInterface->pfOnClassRequest != NULL) {
        if (pInterface->pfOnClassRequest(pSetupPacket) == 0) {
          return;
        }
      }
    }
    pDriver->pfStallEP0();
    break;
#endif
#if USB_SUPPORT_VENDOR_REQUESTS
  case 0x40:              // USB vendor request
    USB_LOG((USB_MTYPE_ENUMERATION, "ENUM: OnVendorRequest"));
    for (InterfaceIndex = 0; InterfaceIndex < USB_Global.NumIFs; InterfaceIndex++) {
      pInterface     = &USB_Global.aIF[InterfaceIndex];
      if (pInterface->pfOnVendorRequest != NULL) {
        if (pInterface->pfOnVendorRequest(pSetupPacket) == 0) {
          return;
        }
      }
    }
    pDriver->pfStallEP0();
    break;
#endif
  default:
    pDriver->pfStallEP0();
    break;
  }
}

/*********************************************************************
*
*       USBD_SetMaxPower
*
*  Function description
*    Sets the maximum power consumption reported to the host during enumeration.
*    This function also sets whether the device is self-powered (MaxPower == 0) or not.
*
*  Parameters
*    MaxPower:    Maximum power consumption of the device given in mA.
*                 MaxPower shall be in range between 0mA - 500mA,
*                 for SuperSpeed devices up to 900mA.
*
*  Additional information
*    This function shall be called before USBD_Start(), as it will specify how much power
*    the device will consume from the host.
*    If this function is not called, a default value of 100 mA will be used.
*/
void USBD_SetMaxPower(unsigned MaxPower) {
  USB_Global.MaxPower   = (SEGGER_MIN(MaxPower, 500u) + 1u) >> 1;
  USB_Global.MaxPowerSS = (MaxPower + 7u) >> 3;
  if (MaxPower == 0u) {
    USB_Global.DeviceState |= 1u;
    USB_Global.ConfigAttr  |= (1u << 6);
  } else {
    USB_Global.DeviceState &= ~1u;
    USB_Global.ConfigAttr  &= ~(1u << 6);
  }
}

/*********************************************************************
*
*       USB__ResetDataToggleEP
*
*  Function description
*    Resets data toggle of specified EP to DATA0.
*/
void USB__ResetDataToggleEP(unsigned EPIndex) {
  const USB_HW_DRIVER * pDriver;

  pDriver = USB_Global.pDriver;
  if (pDriver->pfResetEP != NULL) {
    pDriver->pfResetEP(EPIndex);
  }
}

/*********************************************************************
*
*       USB__AllocStringDesc
*
*  Function description
*    Add string descriptor to list and return index.
*/
U8 USB__AllocStringDesc(const char *sString) {
  unsigned i;

  if (USB_Global.NumStringDesc >= USB_MAX_STRING_DESC) {
    USB_PANIC("Too many string descriptors");
    return 0;
  }
  i = USB_Global.NumStringDesc++;
  USB_Global.aStringDesc[i] = sString;
  return i + STRING_INDEX_OTHER;
}

/*********************************************************************
*
*       USB__GetDescBuffer
*
*  Function description
*    Returns the descriptor buffer and its buffer size.
*/
U8 * USB__GetDescBuffer(unsigned * pDescBufferSize) {
  *pDescBufferSize = sizeof (_aDescBuffer);
  return &_aDescBuffer[0];
}

/*********************************************************************
*
*       USBD_EnableSuperSpeed
*
*  Function description
*    Enable SuperSpeed in the USB stack. Must be called in USBD_X_Config().
*    If the USB driver or USB controller does not support SuperSpeed,
*    calling this function has no effect.
*/
void USBD_EnableSuperSpeed(void) {
  if (USB_Global.pDriver->pfControl != NULL) {
    if(USB_Global.pDriver->pfControl(USB_DRIVER_CMD_ENABLE_SUPER_SPEED, NULL) == 0) {
      USB_Global.pfAddSSEndpointCompanion = _AddSSEndpointCompanion;
      USB_Global.pfAddSSDeviceCap         = _AddSSDeviceCap;
      USB_Global.USBSpecRev               = 0x0320;
      USB_Global.pfBuildBOSDesc           = _BuildBOSDesc;      // USBD_UseV210()
      USB_Global.pfSetClrFeature          = _SetClrFeature3;
      USBD_SetOnRxEP0(_OnRxEP0);
    }
  }
}

/*********************************************************************
*
*       USBD_UseV210
*
*  Function description
*    Enable use of USB V2.10 specification revision.  Must be called in USBD_X_Config().
*    It enables providing a BOS descriptor to the host and also enables
*    link power management (LPM), if supported by the driver and the USB controller.
*/
void USBD_UseV210(void) {
  USB_Global.pfBuildBOSDesc = _BuildBOSDesc;
  if (USB_Global.USBSpecRev < 0x0210u) {
    USB_Global.USBSpecRev = 0x0210;
  }
}

/*********************************************************************
*
*       USBD_SetIsSelfPowered
*
*  Function description
*    Sets whether the device is self-powered or not.
*    Obsolete function, please use USBD_SetMaxPower().
*
*  Parameters
*    IsSelfPowered:    * 0 - Device is not self-powered.
*                      * 1 - Device is self-powered.
*
*  Additional information
*    This function has to be called before USBD_Start(), as it will specify if the device is
*    self-powered or not. The default value is 0 (not self-powered).
*/
void USBD_SetIsSelfPowered(U8 IsSelfPowered) {
  if (IsSelfPowered != 0u) {
    USB_Global.DeviceState |= 1u;
    USB_Global.ConfigAttr  |= (1u << 6);
  } else {
    USB_Global.DeviceState &= ~1u;
    USB_Global.ConfigAttr  &= ~(1u << 6);
  }
}

/*********************************************************************
*
*       USBD_SetAllowRemoteWakeUp
*
*  Function description
*    Allows the device to publish that remote wake is available.
*
*  Parameters
*    AllowRemoteWakeup:    * 1 - Allows and publishes that remote wakeup is available.
*                          * 0 - Publish that remote wakeup is not available.
*
*  Additional information
*    This function must be called before the function USBD_Start() is called. This ensures
*    that the Host is informed that USB remote wake up is available.
*/
void USBD_SetAllowRemoteWakeUp(U8 AllowRemoteWakeup) {
  if (AllowRemoteWakeup != 0u) {
    USB_Global.ConfigAttr |= (1u << 5);
  } else {
    USB_Global.ConfigAttr &= ~(1u << 5);
  }
}

/*********************************************************************
*
*       USBD_SetGetStringHook
*
*  Function description
*    Sets a call to determine the string of a specified string index.
*
*  Parameters
*    pHook             - Pointer to static USB_GET_STRING_DESC_HOOK structure.
*    pfOnGetString     - Pointer to GetString callback.
*
*  Additional information
*    The USB_GET_STRING_DESC_HOOK structure is private to the USB stack. It will be initialized by USBD_SetGetStringHook().
*    The USB stack keeps track of all 'GetString' callback functions using a linked list. The
*    USB_GET_STRING_DESC_HOOK structure will be included into this linked list and must reside in static memory.
*/
void USBD_SetGetStringHook(USB_GET_STRING_DESC_HOOK * pHook, USB_GET_STRING_FUNC * pfOnGetString) {
  pHook->pfGetString = pfOnGetString;
  pHook->pNext = USB_Global.pfGetString;
  USB_Global.pfGetString = pHook;
}

/*********************************************************************
*
*       USBD_SetGetStringFunc
*
*  Function description
*    Deprecated function! Sets a call to determine the string of a specified string index.
*
*  Parameters
*    pfOnGetString     - Pointer to GetString callback.
*/
void USBD_SetGetStringFunc(USB_GET_STRING_FUNC * pfOnGetString) {
  static USB_GET_STRING_DESC_HOOK Hook;

  USBD_SetGetStringHook(&Hook, pfOnGetString);
}

/*********************************************************************
*
*       USBD_SetOnBCDVersionFunc
*
*  Function description
*    Sets a call back to determine the BCD version of the device.
*
*  Parameters
*    pfOnBCDVersion    - Pointer to GetBCDVersion callback.
*/
void USBD_SetOnBCDVersionFunc(USB_ON_BCD_VERSION_FUNC * pfOnBCDVersion) {
  USB_Global.pfOnBCDVersion = pfOnBCDVersion;
}

/*********************************************************************
*
*       USBD_SetWebUSBInfo
*
*  Function description
*    For WebUSB capable USB devices this function may be called before USBD_Start()
*    to enable WebUSB specific descriptors. This function can be used only, if the
*    USB controller supports USB 2.1 compatibility, especially link power management (LPM).
*
*  Parameters
*    pWebUSBInfo:   Pointer to a structure containing the device information. Must
*                   point to static data that is not changed while the stack is running.
*/
void USBD_SetWebUSBInfo(const USB_WEBUSB_INFO * pWebUSBInfo) {
  static USB_SETUP_HOOK Hook;

  _pWebUSBInfo = pWebUSBInfo;
  USB_Global.pfAddWebUSBDesc = _AddWebUSBDesc;
  USBD_SetOnSetup(&Hook, _OnSetupWebUSB);
  USBD_UseV210();
}

/*********************************************************************
*
*       USBD_SetBESLValues
*
*  Function description
*    Set recommended BESL (Best Effort Service Latency) values to be used in the BOS descriptor
*    when using LPM  (Link Power Management).
*    See "Errata for USB 2.0 ECN: Link Power Management (LPM) - 7/2007" from usb.org for
*    an explanation of these values.
*    Calling this function has no effect, if LPM is not enabled (see USBD_UseV210()) or
*    not supported by the driver or USB controller.
*
*  Parameters
*    BaselineBESL:  Recommended Baseline BESL value. Must be in range -1 to 15.
*                   A value of -1 means, no BESL value is stored in the BOS descriptor (the default).
*                   Values of 0,1,...,14,15 specify a BESL of 125us,150us,...,9000us,10000us
*                   respectively (see LPM document from usb.org).
*    DeepBESL:      Recommended Deep BESL value. Must be in range -1 to 15 (see above).
*/
void USBD_SetBESLValues(int BaselineBESL, int DeepBESL) {
  U16 LPM_Config = 0;

  if (BaselineBESL >= 0) {
    LPM_Config |= (1uL << 3) | (((unsigned)BaselineBESL & 0xFuL) << 8);
  }
  if (DeepBESL >= 0) {
    LPM_Config |= (1uL << 4) | (((unsigned)DeepBESL & 0xFuL) << 12);
  }
  USB_Global.RecommendedBESLValue = LPM_Config;
}

/*********************************************************************
*
*       USBD_SetLPMResponse
*
*  Function description
*    Defines the behavior of the device on LPM requests from the host.
*    Calling this function has no effect, if LPM is not enabled (see USBD_UseV210()) or
*    not supported by the driver or USB controller.
*
*  Parameters
*    Response:  * 0 - LPM requests are rejected (NYET).
*               * 1 - LPM requests are acknowledged.
*/
void USBD_SetLPMResponse(U8 Response) {
  USB_IOCTL_PARA Param;
  USBD_GLOBAL * pG = &USB_Global;

  Param.LPM_Response = Response;
  if (pG->pDriver->pfControl != NULL) {
    (void)pG->pDriver->pfControl(USB_DRIVER_CMD_SET_LPM_RESPONSE, &Param);
  }
}

/*********************************************************************
*
*       USBD_SetOnLPMChange
*
*  Function description
*    Sets a call back to report LPM transition on the USB lines (L0 <-> L1).
*
*  Parameters
*    pfOnLPMChange    - Pointer to callback.
*/
void USBD_SetOnLPMChange(USB_ON_LPM_CHANGE * pfOnLPMChange) {
  USB_Global.pfOnLPMChange = pfOnLPMChange;
}

/*************************** End of file ****************************/
