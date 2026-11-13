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
Purpose : Target USB driver for Nordic Semi nRF52xxx devices
-------------------------- END-OF-HEADER -----------------------------
*/
#include "USB_Private.h"


/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

#define USB_BASE_ADDR   0x40027000u

#define USBREGSTATUS         (*(/*lint -e(923) -e(9078)  D:103[a]*/(volatile U32 *)0x40000438u))

#define USE_WORKAROUND_187


/*********************************************************************
*
*       Main driver code
*
**********************************************************************
*/
#define USB_HW_NRF_C_
//lint -e9019 D:113
#include "USB_HW_Nordic_nRF.c"

/*********************************************************************
*
*       Public const
*
**********************************************************************
*/
const USB_HW_DRIVER USB_Driver_Nordic_nRF52xxx = {
  _Start,
  _AllocEP,
  _UpdateEP,
  _Enable,
  _Attach,
  _GetMaxPacketSize,
  _IsInHighSpeedMode,
  _SetAddress,
  _SetClrStallEP,
  _StallEP0,
  _DisableRxInterruptEP,
  _EnableRxInterruptEP,
  _StartTx,
  NULL,
  _DisableTx,
  _ResetEP,
  NULL,
  _DeInit,
  _Detach,
  NULL,
  _SendEPEx,
  _Init
};

/*************************** End of file ****************************/
