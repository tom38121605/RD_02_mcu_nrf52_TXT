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
File    : USB_CDC.c
Purpose : Implementation of USB CDC function.
-------------------------- END-OF-HEADER -----------------------------
*/

#include "USB_Private.h"
#include "USB_CDC_Private.h"

/*********************************************************************
*
*       defines, configurable
*
**********************************************************************
*/
#ifndef    USB_CDC_DEVICES
  #define  USB_CDC_DEVICES      2u
#endif

/*********************************************************************
*
*       defines, fix
*
**********************************************************************
*/
#define CS_INTERFACE                    0x24

#define SET_USB_CDC_LINE_CODING         0x20
#define GET_USB_CDC_LINE_CODING         0x21
#define SET_CONTROL_LINE_STATE          0x22
#define SEND_BREAK                      0x23

#if USB_DEBUG_LEVEL >= 1
  #define CDC_CHECK_IF_HINST_VALID(hInst)                  if ((hInst) >= _NumInstances) { USB_PANIC("Invalid handle"); return;}
  #define CDC_CHECK_IF_HINST_VALID_R(hInst)                if ((hInst) >= _NumInstances) { USB_PANIC("Invalid handle"); return -1;}
#else
  #define CDC_CHECK_IF_HINST_VALID(hInst)
  #define CDC_CHECK_IF_HINST_VALID_R(hInst)
#endif

/*********************************************************************
*
*       Types
*
**********************************************************************
*/
typedef struct {
  USB_CDC_ON_SET_LINE_CODING        * pfOnSetLineCoding;
  USB_CDC_ON_SET_CONTROL_LINE_STATE * pfOnSetControlLineState;
  void                              * pfOnSetControlLineStateContext;
  USB_CDC_ON_SET_BREAK              * pfOnBreak;
  USB_CDC_LINE_CODING                 LineCoding;
  U16                                 ControlInterfaceNum;
  U16                                 DataInterfaceNum;
  U8                                  EPIn;
  U8                                  EPOut;
  U8                                  EPInt;
  U8                                  aLineCodingBuffer[7];
  U8                                  aStatus[10];
} USB_CDC_INST;

/*********************************************************************
*
*       static const
*
**********************************************************************
*/
static const USB_CDC_LINE_CODING _DefaultLineCoding = {115200, 0, 0, 8};

/*********************************************************************
*
*       static data
*
**********************************************************************
*/
static USB_CDC_INST _aInst[USB_CDC_DEVICES];
static int          _CurrentLineCodingPendingInst;
static U8           _NumInstances;
static U8           _IsInited;

/*********************************************************************
*
*       static code
*
**********************************************************************
*/

/*********************************************************************
*
*       _InterfaceNum2Inst
*
*  Function description
*    Converts the interface number of the CDC control interface
*    to a CDC instance.
*/
static int _InterfaceNum2Inst(unsigned InterfaceNum) {
  unsigned i;

  for (i = 0; i < _NumInstances; i++) {
    if (_aInst[i].ControlInterfaceNum == InterfaceNum) {
      return i;
    }
  }
  return -1;
}

/*********************************************************************
*
*       _OnRxEP0
*
*  Function description
*    Called when data is received on EP0.
*    This is typically a "SetLineCoding" Packet.
*/
static void _OnRxEP0(const U8 * pData, unsigned NumBytes) {
  USB_CDC_INST * pInst;

  if (_CurrentLineCodingPendingInst >= 0) {
    pInst = &_aInst[_CurrentLineCodingPendingInst];
    if (NumBytes != 7u) {
      USB_WARN((USB_MTYPE_CDC, "CDC: Warning: Received an unrecognized packet on EP0."));
      USBD_StallEP(0);
    } else {
      pInst->LineCoding.DTERate    = USBD_GetU32LE(pData);
      pInst->LineCoding.CharFormat = *(pData + 4);
      pInst->LineCoding.ParityType = *(pData + 5);
      pInst->LineCoding.DataBits   = *(pData + 6);
      USB_LOG((USB_MTYPE_CDC, " CDC: DTERate=%u, CharFormat=%u, ParityType=%u, DataBits=%u",
                    pInst->LineCoding.DTERate,
                    pInst->LineCoding.CharFormat,
                    pInst->LineCoding.ParityType,
                    pInst->LineCoding.DataBits));
      if (pInst->pfOnSetLineCoding != NULL) {
        pInst->pfOnSetLineCoding(&pInst->LineCoding);
      }
      USBD_WriteEP0FromISR(NULL, 0, 0);
    }
    _CurrentLineCodingPendingInst = -1;
  }
}

/*********************************************************************
*
*       _OnClassRequest
*
*  Function description
*    Handles all CDC ACM required SETUP class request.
*
*  Parameters
*    pSetupPacket     - Pointer to the SETUP class request.
*
*  Return value
*    0 - Request was successfully handled by the function.
*    1 - Request cannot be handled.
*/
static int _OnClassRequest(const USB_SETUP_PACKET * pSetupPacket) {
  USB_CDC_INST * pInst;
  unsigned       InterfaceNum;
  int            Inst;
  unsigned       BreakDuration;

  InterfaceNum = (unsigned)pSetupPacket->wIndexLow | ((unsigned)pSetupPacket->wIndexHigh << 8);
  Inst         = _InterfaceNum2Inst(InterfaceNum);
  if (Inst < 0) {
    return 1;
  }
  pInst        = &_aInst[Inst];
  switch (pSetupPacket->bRequest) {
  case SET_USB_CDC_LINE_CODING:
    USB_LOG((USB_MTYPE_CDC, "CDC: SET_USB_CDC_LINE_CODING"));
    _CurrentLineCodingPendingInst = Inst;
    break;
  case GET_USB_CDC_LINE_CODING:
    USB_LOG((USB_MTYPE_CDC, "CDC: GET_USB_CDC_LINE_CODING"));
    USBD_StoreU32LE(&pInst->aLineCodingBuffer[0], pInst->LineCoding.DTERate);
    pInst->aLineCodingBuffer[4] = pInst->LineCoding.CharFormat;
    pInst->aLineCodingBuffer[5] = pInst->LineCoding.ParityType;
    pInst->aLineCodingBuffer[6] = pInst->LineCoding.DataBits;
    USBD_WriteEP0FromISR(&pInst->aLineCodingBuffer[0], sizeof(pInst->aLineCodingBuffer), 0);
    break;
  case SET_CONTROL_LINE_STATE:
    //
    // bit0 = DTR, bit1 = RTS
    //
    USB_LOG((USB_MTYPE_CDC,  "CDC: SET_CONTROL_LINE_STATE %X", pSetupPacket->wValueLow));
    if (pInst->pfOnSetControlLineState != NULL) {
      USB_CDC_CONTROL_LINE_STATE CLState;

      CLState.DTR = pSetupPacket->wValueLow & 1u;
      CLState.RTS = (pSetupPacket->wValueLow >> 1) & 1u;
      pInst->pfOnSetControlLineState(&CLState, pInst->pfOnSetControlLineStateContext);
    }
    USBD_WriteEP0FromISR(NULL, 0, 1);
    break;
  case SEND_BREAK:
    BreakDuration = ((unsigned)(pSetupPacket->wValueHigh) << 8) | pSetupPacket->wValueLow;
    USB_LOG((USB_MTYPE_CDC, "CDC: SEND_BREAK %X", BreakDuration));
    if (pInst->pfOnBreak != NULL) {
      pInst->pfOnBreak(BreakDuration);
    }
    USBD_WriteEP0FromISR(NULL, 0, 1);
    break;
  default:
    //
    // Nothing to do.
    //
    return 1;
  }                            //lint !e9077 D:102[a]
  return 0;
}

/*********************************************************************
*
*       _AddFuncDesc
*
*  Function description
*    Adds the required CDC function header into the interface descriptor.
*    This function was previously added in USBD_CDC_Add().
*
*  Parameters
*    InterfaceNo        -  Interface No for which the additional description is added.
*    IFAlternateSetting -  Alternate Interface number.
*    pInfoBuffer        -  Pointer to the Infobuffer in which the descriptor is stored.
*/
static void _AddFuncDesc(int InterfaceNo, U8 IFAlternateSetting, USB_INFO_BUFFER * pInfoBuffer) {
  USB_CDC_INST * pInst;
  int            Inst;

  USB_USE_PARA(IFAlternateSetting);
  Inst         = _InterfaceNum2Inst((unsigned)InterfaceNo);
  pInst        = &_aInst[Inst];
  //
  // Header functional desc
  //
  USB_IB_AddU8(pInfoBuffer,  0x05);
  USB_IB_AddU8(pInfoBuffer,  CS_INTERFACE);
  USB_IB_AddU8(pInfoBuffer,  0x00);
  USB_IB_AddU16(pInfoBuffer, 0x0110);
  //
  // Call management functional desc
  //
  USB_IB_AddU8(pInfoBuffer, 0x05);
  USB_IB_AddU8(pInfoBuffer, CS_INTERFACE);
  USB_IB_AddU8(pInfoBuffer, 0x01);
  USB_IB_AddU8(pInfoBuffer, 0x03);  // bmCapabilities = device handles call management
  USB_IB_AddU8(pInfoBuffer, (U8)pInst->DataInterfaceNum);  // bDataInterface
  //
  // ACM functional desc
  //
  USB_IB_AddU8(pInfoBuffer, 0x04);
  USB_IB_AddU8(pInfoBuffer, CS_INTERFACE);
  USB_IB_AddU8(pInfoBuffer, 0x02);
  USB_IB_AddU8(pInfoBuffer, (pInst->EPInt != 0u) ? 0x06u : 0x02u); // bmCapabilities: 0x06 with notification EP, 0x02 without
  //
  // Union functional desc
  //
  USB_IB_AddU8(pInfoBuffer, 0x05);
  USB_IB_AddU8(pInfoBuffer, CS_INTERFACE);
  USB_IB_AddU8(pInfoBuffer, 0x06);
  USB_IB_AddU8(pInfoBuffer, (U8)pInst->ControlInterfaceNum);  // bMasterInterface
  USB_IB_AddU8(pInfoBuffer, (U8)pInst->DataInterfaceNum);     // bSlaveInterface0
}

/*********************************************************************
*
*       _Init
*
*  Function description
*    Checks whether the CDC module is initialized if not
*    it will initialize it and adds the DeInit handler that
*    shall be called when USBD_DeInit() is called.
*/
static void _Init(void) {
  static USB_DEINIT_HOOK DeInitHook;

  if (_IsInited == 0u) {
    _IsInited = 1;
    USB_MEMSET(_aInst, 0, sizeof(_aInst));
    _NumInstances = 0;
    _CurrentLineCodingPendingInst = -1;
    USB__RegisterDeInitFlag(&DeInitHook, &_IsInited);
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/

/*********************************************************************
*
*       USBD_CDC_SetOnLineCoding
*
*  Function description
*    Sets a user callback that shall be called when a SET_LINE_CODING
*    request is sent to the device.
*
*  Parameters
*    hInst:     Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pf:        Pointer to the callback function.
*
*  Additional information
*    This function is used to register a user callback which notifies the application that the
*    host has changed the line coding.
*
*    The callback is called in an ISR context, therefore
*    it should should execute quickly.
*/
void USBD_CDC_SetOnLineCoding(USB_CDC_HANDLE hInst, USB_CDC_ON_SET_LINE_CODING * pf) {
  CDC_CHECK_IF_HINST_VALID(hInst);
  _aInst[hInst].pfOnSetLineCoding = pf;
}

/*********************************************************************
*
*       USBD_CDC_SetOnControlLineState
*
*  Function description
*    Sets a user callback that shall be called when a SET_LINE_STATE
*    request is sent to the device.
*
*  Parameters
*    hInst:  Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pf:        Pointer to the callback function.
*
*  Additional information
*    This function is used to register a user callback which notifies the application that the
*    host has changed the control line state.
*
*    The callback is called in an ISR context, therefore
*    it should execute quickly and must never block.
*/
void USBD_CDC_SetOnControlLineState(USB_CDC_HANDLE hInst, USB_CDC_ON_SET_CONTROL_LINE_STATE * pf, void * pContext) {
  CDC_CHECK_IF_HINST_VALID(hInst);
  _aInst[hInst].pfOnSetControlLineState = pf;
  _aInst[hInst].pfOnSetControlLineStateContext = pContext;
}

/*********************************************************************
*
*       USBD_CDC_Read
*
*  Function description
*    Reads data from the host.
*
*  Parameters
*    hInst:     Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pData:     Pointer to a buffer where the received data will be stored.
*    NumBytes:  Number of bytes to read.
*    Timeout:   Timeout given in milliseconds. A zero value results in an infinite timeout.
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
*    that was provided via USBD_AddEP(). This data can be retrieved by a later call to
*    USBD_CDC_Receive() / USBD_CDC_Read(). See also USBD_CDC_GetNumBytesInBuffer().
*
*    In case of a timeout, the read transfer is aborted (see \ref{Timeout handling}).
*/
int USBD_CDC_Read(USB_CDC_HANDLE hInst, void * pData, unsigned NumBytes, unsigned Timeout) {
  CDC_CHECK_IF_HINST_VALID_R(hInst);
  return USBD_Read(_aInst[hInst].EPOut, pData, NumBytes, Timeout);
}

/*********************************************************************
*
*       USBD_CDC_ReadOverlapped
*
*  Function description
*    Reads data from the host asynchronously.
*
*  Parameters
*    hInst:     Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pData:     Pointer to a buffer where the received data will be stored.
*    NumBytes:  Number of bytes to read.
*
*  Return value
*    >= 0:     Number of bytes that have been read from the internal buffer (success).
*    == 0:     No data was found in the internal buffer (success).
*    < 0:      An error occurred.
*
*  Additional information
*    This function will not block the calling task. The read transfer will be initiated and the
*    function returns immediately. In order to synchronize, USBD_CDC_WaitForRX() needs
*    to be called.
*
*    Another synchronization method would be to periodically call
*    USBD_CDC_GetNumBytesRemToRead() in order to see how many bytes still need to be
*    received (this method is preferred when a non-blocking solution is necessary).
*
*    The read operation can be canceled using USBD_CDC_CancelRead().
*
*    The buffer pointed to by pData must be valid until the read operation is terminated.
*/
int USBD_CDC_ReadOverlapped(USB_CDC_HANDLE hInst, void * pData, unsigned NumBytes) {
  CDC_CHECK_IF_HINST_VALID_R(hInst);
  return USBD_ReadOverlapped(_aInst[hInst].EPOut, pData, NumBytes);
}

/*********************************************************************
*
*       USBD_CDC_WaitForRX
*
*  Function description
*    This function is to be used in combination with USBD_CDC_ReadOverlapped() and
*    waits for the reading data transfer from the host to complete.
*
*  Parameters
*    hInst:    Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    Timeout:  Timeout in milliseconds. 0 means infinite.
*
*  Return value
*    == 0:      Transfer completed.
*    == 1:      Timeout occurred.
*
*  Additional information
*    This function shall be called in order to synchronize task with the read data transfer
*    previously initiated.
*    The function blocks until the number of bytes specified by
*    USBD_CDC_ReadOverlapped() has been read from the host.
*    In case of a timeout, the read transfer is aborted (see \ref{Timeout handling}).
*/
int USBD_CDC_WaitForRX(USB_CDC_HANDLE hInst, unsigned Timeout) {
  CDC_CHECK_IF_HINST_VALID_R(hInst);
  return USBD_WaitForEndOfTransferEx(_aInst[hInst].EPOut, Timeout, 1);
}

/*********************************************************************
*
*       USBD_CDC_PollForRX
*
*  Function description
*    This function is to be used in combination with USBD_CDC_ReadOverlapped() and
*    waits for the reading data transfer from the host to complete.
*
*  Parameters
*    hInst:    Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    Timeout:  Timeout in milliseconds. 0 means infinite.
*
*  Return value
*    == 0:      Transfer completed.
*    == 1:      Timeout occurred.
*
*  Additional information
*    This function shall be called in order to synchronize task with the read data transfer
*    previously initiated.
*    The function blocks until the number of bytes specified by
*    USBD_CDC_ReadOverlapped() has been read from the host.
*    In case of a timeout, the current transfer is not affected.
*    The function may be called repeatedly until it does not report
*    a timeout any more.
*/
int USBD_CDC_PollForRX(USB_CDC_HANDLE hInst, unsigned Timeout) {
  CDC_CHECK_IF_HINST_VALID_R(hInst);
  return USBD_WaitForEndOfTransferEx(_aInst[hInst].EPOut, Timeout, 0);
}

/*********************************************************************
*
*       USBD_CDC_Receive
*
*  Function description
*    Reads data from the host. The function blocks until any data have been received. In contrast
*    to USBD_CDC_Read() this function does not wait for all of NumBytes to be
*    received, but returns after the first packet has been received or after the timeout occurs.
*    In case of a timeout, the read transfer is aborted (see \ref{Timeout handling}).
*
*  Parameters
*    hInst:     Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pData:     Pointer to a buffer where the received data will be stored.
*    NumBytes:  Number of bytes to read.
*    Timeout:   Timeout given in milliseconds. A zero value results in an infinite timeout.
*               If Timeout is -1, the function never blocks and only reads data from the internal
*               endpoint buffer.
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
*    Calling USBD_CDC_Receive() will return as much data as is currently available up to
*    the size of the buffer specified within the specified timeout. This function also returns
*    when the target is disconnected from the host or when a USB reset occurred during the function
*    call, it will then return the number of bytes read so far. If the target was disconnected
*    before this function was called, it returns USB_STATUS_ERROR.
*
*    If the USB stack receives a data packet from the host containing more bytes than requested,
*    the remaining bytes are stored into the internal buffer of the endpoint,
*    that was provided via USBD_AddEP(). This data can be retrieved by a later call to
*    USBD_CDC_Receive() / USBD_CDC_Read(). See also USBD_CDC_GetNumBytesInBuffer().
*
*    A call of USBD_CDC_Receive(Inst, NULL, 0, -1) can be used to trigger an asynchronous read that
*    stores the data into the internal buffer.
*/
int USBD_CDC_Receive(USB_CDC_HANDLE hInst, void * pData, unsigned NumBytes, int Timeout) {
  CDC_CHECK_IF_HINST_VALID_R(hInst);
  return USBD_Receive(_aInst[hInst].EPOut, pData, NumBytes, Timeout);
}

/*********************************************************************
*
*       USBD_CDC_ReceivePoll
*
*  Function description
*    Reads data from the host. The function blocks until any data have been received. In contrast
*    to USBD_CDC_Read() this function does not wait for all of NumBytes to be
*    received, but returns after the first packet has been received or after the timeout occurs.
*    In contrast to USBD_CDC_Receive() this function will continue the read transfer
*    asynchronously in case of a timeout.
*
*  Parameters
*    hInst:     Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pData:     Pointer to a buffer where the received data will be stored.
*    NumBytes:  Number of bytes to read.
*    Timeout:   Timeout given in milliseconds. A zero value results in an infinite timeout.
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
*    Calling USBD_CDC_ReceivePoll() will return as much data as is currently available up to
*    the size of the buffer specified within the specified timeout. This function also returns
*    when the target is disconnected from the host or when a USB reset occurred during the function
*    call, it will then return the number of bytes read so far. If the target was disconnected
*    before this function was called, it returns USB_STATUS_ERROR.
*
*    If the USB stack receives a data packet from the host containing more bytes than requested,
*    the remaining bytes are stored into the internal buffer of the endpoint,
*    that was provided via USBD_AddEP(). This data can be retrieved by a later call to
*    USBD_CDC_ReceivePoll() / USBD_CDC_Read(). See also USBD_CDC_GetNumBytesInBuffer().
*
*    If a timeout occurs, the read transfer is not affected. Data send from the host after the timeout
*    is stored into the internal buffer of the endpoint and can be read by later calls to USBD_CDC_ReceivePoll().
*
*    If Timeout == 0, the function behaves like USBD_CDC_Receive().
*/
int USBD_CDC_ReceivePoll(USB_CDC_HANDLE hInst, void * pData, unsigned NumBytes, unsigned Timeout) {
  CDC_CHECK_IF_HINST_VALID_R(hInst);
  return USBD_ReceivePoll(_aInst[hInst].EPOut, pData, NumBytes, Timeout);
}

/*********************************************************************
*
*       USBD_CDC_CancelRead
*
*  Function description
*    Cancel a pending read operation.
*
*  Parameters
*    hInst:  Handle to a valid CDC instance, returned by USBD_CDC_Add().
*
*  Additional information
*    This function can be called when a pending asynchronous read operation (triggered
*    by USBD_CDC_ReadOverlapped()) should be canceled. The function can be called
*    from any task.
*
*    The function can also be used to cancel a call to one of the blocking read functions
*    (when called from a different task or interrupt function).
*/
void USBD_CDC_CancelRead(USB_CDC_HANDLE hInst) {
  CDC_CHECK_IF_HINST_VALID(hInst);
  USBD_CancelIO(_aInst[hInst].EPOut);
}

/*********************************************************************
*
*       USBD_CDC_Write
*
*  Function description
*    Writes data to the host. Depending on the Timeout parameter, the function may block
*    until NumBytes have been written or a timeout occurs.
*
*  Parameters
*    hInst:     Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pData:     Pointer to data that should be sent to the host.
*    NumBytes:  Number of bytes to be written.
*    Timeout:   Timeout in milliseconds. 0 means infinite. If  Timeout  is -1,
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
*    In order to synchronize, USBD_CDC_WaitForTX() needs to be called. Another synchronization
*    method would be to periodically call USBD_CDC_GetNumBytesRemToWrite() in
*    order to see how many bytes still need to be written (this method is preferred when
*    a non-blocking solution is necessary).
*
*    The write operation can be canceled using USBD_CDC_CancelWrite().
*
*    If pData == NULL and NumBytes == 0, a zero-length packet is sent to the host.
*
*    In case of a timeout, the write transfer is aborted (see \ref{Timeout handling}).
*
*    The content of the buffer pointed to by pData must not be changed until the transfer
*    has been completed.
*/
int USBD_CDC_Write(USB_CDC_HANDLE hInst, const void * pData, unsigned NumBytes, int Timeout) {
  CDC_CHECK_IF_HINST_VALID_R(hInst);
  return USBD_Write(_aInst[hInst].EPIn, pData, NumBytes, 1, Timeout);
}

/*********************************************************************
*
*       USBD_CDC_WriteEx
*
*  Function description
*    Send data to the USB host with NULL packet control.
*    This function behaves exactly like USBD_CDC_Write(). Additionally sending of a zero
*    length packet after sending the data can be suppressed by setting
*    Send0PacketIfRequired = 0.
*
*  Parameters
*    hInst:                 Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pData:                 Pointer to a buffer that contains the written data.
*    NumBytes:              Number of bytes to write.
*    Send0PacketIfRequired: Specifies that a zero-length packet shall be sent when
*                           the last data packet is a multiple of MaxPacketSize.
*                           Normally MaxPacketSize for full-speed devices is 64
*                           bytes. For high-speed devices the normal packet size is
*                           between 64 and 512 bytes.
*    Timeout:               Timeout in milliseconds. 0 means infinite. If  Timeout  is -1,
*                           the function returns immediately and the transfer is
*                           processed asynchronously.
*
*  Return value
*    == 0:               Successful started an asynchronous write transfer
*                        or a timeout has occurred and no data was written.
*    > 0 && < NumBytes:  Number of bytes that have been written before a timeout occurred.
*    == NumBytes:        Write transfer successful completed.
*    < 0 :               Error occurred.
*
*  Additional information
*    Normally USBD_CDC_Write() is called to let the stack send the data to the host and
*    send an optional zero-length packet to tell the host that this was the last packet. This
*    is the case when the last packet sent is MaxPacketSize bytes in size.
*    When using this function, the zero-length packet handling can be controlled. This
*    means the function can be called when sending data in multiple steps.
*/
int USBD_CDC_WriteEx(USB_CDC_HANDLE hInst, const void* pData, unsigned NumBytes, char Send0PacketIfRequired, int Timeout) {
  unsigned EPIndex;
  int      ReqNULL;

  CDC_CHECK_IF_HINST_VALID_R(hInst);
  EPIndex = _aInst[hInst].EPIn;
  if (NumBytes == 0u) {
    //
    // We only send a 0-packet if necessary
    //
    for(;;) {
      ReqNULL = USBD_IsNULLPacketRequired(EPIndex);
      if (ReqNULL > 0) {
        //
        // NULL packet required, goto USBD_Write()
        //
        break;
      }
      if (Timeout < 0) {
        //
        // No blocking allowed, return immediately.
        //
        return (ReqNULL == 0) ? 0 : USB_STATUS_EP_BUSY;   //lint !e9031  N:105
      }
      //
      // Timeout is >= 0: Sync all writes.
      //
      if (USBD_WaitForEndOfTransferEx(EPIndex, (unsigned)Timeout, 1) != 0 || ReqNULL == 0) {
        return 0;
      }
      //
      // ReqNULL is < 0 here, so we have to retry (call USBD_IsNULLPacketRequired() again).
      //
    }
  }
  return USBD_Write(EPIndex, pData, NumBytes, Send0PacketIfRequired, Timeout);
}


/*********************************************************************
*
*       USBD_CDC_WaitForTX
*
*  Function description
*    This function is to be used in combination with a non-blocking call to
*    USBD_CDC_Write(). This function waits for the writing data transfer to the host to
*    complete.
*
*  Parameters
*    hInst:    Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    Timeout:  Timeout in milliseconds. 0 means infinite.
*
*  Return value
*    == 0:      Transfer completed.
*    == 1:      Timeout occurred.
*
*  Additional information
*    This function shall be called in order to synchronize task with the write data transfer
*    previously initiated.
*    This function blocks until the number of bytes specified by USBD_CDC_Write() has
*    been written to the host.
*    In case of a timeout, the write transfer is aborted (see \ref{Timeout handling}).
*/
int USBD_CDC_WaitForTX(USB_CDC_HANDLE hInst, unsigned Timeout) {
  CDC_CHECK_IF_HINST_VALID_R(hInst);
  return USBD_WaitForEndOfTransferEx(_aInst[hInst].EPIn, Timeout, 1);
}

/*********************************************************************
*
*       USBD_CDC_PollForTX
*
*  Function description
*    This function is to be used in combination with a non-blocking call to
*    USBD_CDC_Write(). This function waits for the writing data transfer to the host to
*    complete.
*
*  Parameters
*    hInst:    Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    Timeout:  Timeout in milliseconds. 0 means infinite.
*
*  Return value
*    == 0:      Transfer completed.
*    == 1:      Timeout occurred.
*
*  Additional information
*    This function shall be called in order to synchronize task with the write data transfer
*    previously initiated.
*    This function blocks until the number of bytes specified by USBD_CDC_Write() has
*    been written to the host.
*    In case of a timeout, the current transfer is not affected.
*    The function may be called repeatedly until it does not report
*    a timeout any more.
*/
int USBD_CDC_PollForTX(USB_CDC_HANDLE hInst, unsigned Timeout) {
  CDC_CHECK_IF_HINST_VALID_R(hInst);
  return USBD_WaitForEndOfTransferEx(_aInst[hInst].EPIn, Timeout, 0);
}

/*********************************************************************
*
*       USBD_CDC_WaitForTXReady
*
*  Function description
*    Waits (blocking) until the TX queue can accept another data packet.
*    This function is used in combination with a non-blocking call to USBD_CDC_Write(), it
*    waits until a new asynchronous write data transfer will be accepted by the USB stack.
*
*  Parameters
*    hInst:    Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    Timeout:  Timeout in milliseconds. 0 means infinite.
*              If  Timeout  is negative, the function will return immediately.
*
*  Return value
*    == 0: A new asynchronous write data transfer will be accepted.
*    == 1: The write queue is full,
*          a call to  USBD_CDC_Write()  would return USB_STATUS_EP_BUSY.
*    < 0:  Error occurred.
*
*  Additional information
*    If Timeout is 0, the function never returns 1.
*
*    If Timeout is -1, the function will not wait, but immediately return the current state.
*/
int USBD_CDC_WaitForTXReady(USB_CDC_HANDLE hInst, int Timeout) {
  CDC_CHECK_IF_HINST_VALID_R(hInst);
  return USBD_WaitForTXReady(_aInst[hInst].EPIn, Timeout);
}

/*********************************************************************
*
*       USBD_CDC_CancelWrite
*
*  Function description
*    Cancel a pending write operation.
*
*  Parameters
*    hInst:  Handle to a valid CDC instance, returned by USBD_CDC_Add().
*
*  Additional information
*    This function shall be called when a pending asynchronous write operation
*    (triggered by non-blocking call to USBD_CDC_Write()) should be canceled.
*    It can be called from any task.
*
*    The function can also be used to cancel a call to a blocking write
*    functions (when called from a different task or interrupt function).
*/
void USBD_CDC_CancelWrite(USB_CDC_HANDLE hInst) {
  CDC_CHECK_IF_HINST_VALID(hInst);
  USBD_CancelIO(_aInst[hInst].EPIn);
}

/*********************************************************************
*
*       USBD_CDC_WriteSerialState
*
*  Function description
*    Sends the current control line serial state to the host.
*
*  Parameters
*    hInst:  Handle to a valid CDC instance, returned by USBD_CDC_Add().
*
*  Additional information
*    This function shall be called in order to inform the host about the control serial state
*    of the CDC instance.
*    The current control line serial state can be set using
*    USBD_CDC_UpdateSerialState().
*/
void USBD_CDC_WriteSerialState(USB_CDC_HANDLE hInst) {
  USB_CDC_INST * pInst;

  CDC_CHECK_IF_HINST_VALID(hInst);
  pInst = &_aInst[hInst];
  if (pInst->EPInt != 0u) {
    (void)USBD_Write(pInst->EPInt, pInst->aStatus, sizeof(pInst->aStatus), 0, -1);
  }
}

/*********************************************************************
*
*       USBD_CDC_UpdateSerialState
*
*  Function description
*    Sets the new serial state.
*
*  Parameters
*    hInst:         Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pSerialState:  Pointer to the USB_CDC_SERIAL_STATE structure.
*
*  Additional information
*    This function updates the control line state internally. In order to inform the host
*    about the serial state change, refer to the function USBD_CDC_WriteSerialState().
*/
void USBD_CDC_UpdateSerialState(USB_CDC_HANDLE hInst, const USB_CDC_SERIAL_STATE * pSerialState) {
  U8 State;

  CDC_CHECK_IF_HINST_VALID(hInst);
  State = 0;
  if ((pSerialState->DCD & 1u) != 0u) {
    State |= (1u << 0);
  }
  if ((pSerialState->DSR & 1u) != 0u) {
    State |= (1u << 1);
  }
  if ((pSerialState->Break & 1u) != 0u) {
    State |= (1u << 2);
  }
  if ((pSerialState->Ring & 1u) != 0u) {
    State |= (1u << 3);
  }
  if ((pSerialState->FramingError & 1u) != 0u) {
    State |= (1u << 4);
  }
  if ((pSerialState->ParityError & 1u) != 0u) {
    State |= (1u << 5);
  }
  if ((pSerialState->OverRunError & 1u) != 0u) {
    State |= (1u << 6);
  }
  _aInst[hInst].aStatus[8] = State;
}

/*********************************************************************
*
*       USBD_CDC_SetOnBreak
*
*  Function description
*    Sets a callback in order to inform the application about a
*    break condition sent by the host.
*
*  Parameters
*    hInst:     Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pfOnBreak: Pointer to the callback function.
*
*  Additional information
*    The callback is called in an ISR context, therefore
*    it should should execute quickly.
*/
void USBD_CDC_SetOnBreak(USB_CDC_HANDLE hInst, USB_CDC_ON_SET_BREAK * pfOnBreak) {
  CDC_CHECK_IF_HINST_VALID(hInst);
  _aInst[hInst].pfOnBreak = pfOnBreak;
}

/*********************************************************************
*
*       USBD_CDC_GetNumBytesInBuffer
*
*  Function description
*    Returns the number of bytes that are available in the internal BULK-OUT endpoint buffer.
*    This function does not start a read transfer.
*
*  Parameters
*    hInst:  Handle to a valid CDC instance, returned by USBD_CDC_Add().
*
*  Return value
*    Number of bytes which have been stored in the internal buffer.
*
*  Additional information
*    The number of bytes returned by this function can be read using USBD_CDC_Read()
*    without blocking.
*/
int USBD_CDC_GetNumBytesInBuffer(USB_CDC_HANDLE hInst) {
  USB_CDC_INST * pInst;

  CDC_CHECK_IF_HINST_VALID_R(hInst);
  pInst = &_aInst[hInst];
  return USBD_GetNumBytesInBuffer(pInst->EPOut);
}

/*********************************************************************
*
*       USBD_CDC_GetNumBytesRemToRead
*
*  Function description
*    This function is to be used in combination with USBD_CDC_ReadOverlapped(). It
*    returns the number of bytes which still have to be read during the transaction.
*
*  Parameters
*    hInst:  Handle to a valid CDC instance, returned by USBD_CDC_Add().
*
*  Return value
*    Number of bytes which still have to be read.
*
*  Additional information
*    Note that this function does not return the number of bytes that have been read, but
*    the number of bytes which still have to be read.
*    This function does not block.
*/
int USBD_CDC_GetNumBytesRemToRead(USB_CDC_HANDLE hInst) {
  USB_CDC_INST * pInst;

  CDC_CHECK_IF_HINST_VALID_R(hInst);
  pInst = &_aInst[hInst];
  return USBD_GetNumBytesRemToRead(pInst->EPOut);
}

/*********************************************************************
*
*       USBD_CDC_GetNumBytesRemToWrite
*
*  Function description
*    This function is to be used in combination with a non-blocking call to
*    USBD_CDC_Write(). It returns the number of bytes which still have to be written during
*    the transaction.
*
*  Parameters
*    hInst:  Handle to a valid CDC instance, returned by USBD_CDC_Add().
*
*  Return value
*    Number of bytes which still have to be written.
*
*  Additional information
*    Note that this function does not return the number of bytes that have been written,
*    but the number of bytes which still have to be written.
*    This function does not block.
*/
int USBD_CDC_GetNumBytesRemToWrite(USB_CDC_HANDLE hInst) {
  USB_CDC_INST * pInst;

  CDC_CHECK_IF_HINST_VALID_R(hInst);
  pInst = &_aInst[hInst];
  return USBD_GetNumBytesRemToWrite(pInst->EPIn);
}

/*********************************************************************
*
*       USBD_CDC_TxIsPending
*
*  Function description
*    Returns whether the TX (IN endpoint) is currently pending.
*    Can be called in any context.
*
*  Parameters
*    hInst:  Handle to a valid CDC instance, returned by USBD_CDC_Add().
*
*  Return value
*    0 - Transfer is not pending.
*    1 - Transfer is     pending.
*/
int USBD_CDC_TxIsPending(USB_CDC_HANDLE hInst) {
  USB_CDC_INST * pInst;

  CDC_CHECK_IF_HINST_VALID_R(hInst);
  pInst = &_aInst[hInst];
  return USBD_TxIsPending(pInst->EPIn);
}

/*********************************************************************
*
*       USBD_CDC_SetOnTXEvent
*
*  Function description
*    Sets a callback function for the IN endpoint that will be called on every TX event
*    for that endpoint.
*
*  Parameters
*    hInst:       Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pEventCb:    Pointer to a USB_EVENT_CALLBACK structure (will be initialized by this function).
*    pfEventCb:   Pointer to the callback routine that will be called on every event on
*                 the USB endpoint.
*    pContext:    A pointer which is used as parameter for the callback function.
*
*  Additional information
*    The USB_EVENT_CALLBACK structure is private to the USB stack. It will be initialized by USBD_CDC_SetOnTXEvent().
*    The USB stack keeps track of all event callback functions using a linked list. The
*    USB_EVENT_CALLBACK structure will be included into this linked list and must reside
*    in static memory.
*
*    The callback function is called only, if a write operation was started using one of the USBD_CDC_Write...() functions.
*/
void USBD_CDC_SetOnTXEvent(USB_CDC_HANDLE hInst, USB_EVENT_CALLBACK *pEventCb, USB_EVENT_CALLBACK_FUNC *pfEventCb, void *pContext) {
  CDC_CHECK_IF_HINST_VALID(hInst);
  USBD_SetOnEvent(_aInst[hInst].EPIn, pEventCb, pfEventCb, pContext);
}

/*********************************************************************
*
*       USBD_CDC_SetOnRXEvent
*
*  Function description
*    Sets a callback function for the OUT endpoint that will be called on every RX event
*    for that endpoint.
*
*  Parameters
*    hInst:       Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pEventCb:    Pointer to a USB_EVENT_CALLBACK structure (will be initialized by this function).
*    pfEventCb:   Pointer to the callback routine that will be called on every event on
*                 the USB endpoint.
*    pContext:    A pointer which is used as parameter for the callback function.
*
*  Additional information
*    The USB_EVENT_CALLBACK structure is private to the USB stack. It will be initialized by USBD_CDC_SetOnRXEvent().
*    The USB stack keeps track of all event callback functions using a linked list. The
*    USB_EVENT_CALLBACK structure will be included into this linked list and must reside
*    in static memory.
*
*    The callback function is called only, if a read operation was started using one of the USBD_CDC_Read...() or USBD_CDC_Receive() functions.
*/
void USBD_CDC_SetOnRXEvent(USB_CDC_HANDLE hInst, USB_EVENT_CALLBACK *pEventCb, USB_EVENT_CALLBACK_FUNC *pfEventCb, void *pContext) {
  CDC_CHECK_IF_HINST_VALID(hInst);
  USBD_SetOnEvent(_aInst[hInst].EPOut, pEventCb, pfEventCb, pContext);
}

/*********************************************************************
*
*       USBD_CDC_Add
*
*  Function description
*    Adds a CDC-ACM class to the stack.
*
*  Parameters
*    pInitData:    Pointer to a USB_CDC_INIT_DATA structure.
*
*  Return value
*    Handle to a valid CDC instance. The handle of the first CDC
*    instance is always 0.
*
*  Additional information
*    After the initialization of emUSB-Device, this is the first function
*    that needs to be called when the USB-CDC interface is used with emUSB-Device.
*    The returned value can be used with the CDC functions in order
*    to talk to the right CDC instance.
*
*    For creating more than one CDC instance please make sure the
*    USBD_EnableIAD() is called before, otherwise none but the first
*    CDC instance will work correctly. The same is true for composite
*    devices including CDC and another, different USB class.
*/
USB_CDC_HANDLE USBD_CDC_Add(const USB_CDC_INIT_DATA * pInitData) {
  unsigned          InterfaceNum;
  unsigned          FirstInterfaceNo;
  INTERFACE       * pInterface;
  USB_CDC_HANDLE    hInst;
  USB_CDC_INST    * pInst;
  static const U8   aDefaultStatus[] = {
    0xA1, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x02, 0x00,
    0x00, 0x00
  };

  _Init();
  if (_NumInstances >= USB_CDC_DEVICES) {
    USB_PANIC("Too many class interfaces");
  }
  USBD_SetOnRxEP0(_OnRxEP0);
  InterfaceNum     = USB__AllocIF();
  FirstInterfaceNo = InterfaceNum;
  pInterface                = &USB_Global.aIF[InterfaceNum];
  pInterface->pfAddFuncDesc = _AddFuncDesc;
  pInterface->IFClass       = 2;
  pInterface->IFSubClass    = 2;
  pInterface->IFProtocol    = 1;
  pInterface->EPs           = (pInitData->EPInt != 0u) ? (1uL << pInitData->EPInt) : 0uL;
  pInterface->iName         = USB__AllocStringDesc("CDC");
  pInterface->pfOnClassRequest    = _OnClassRequest;

  InterfaceNum              = USB__AllocIF();
  pInterface                = &USB_Global.aIF[InterfaceNum];
  pInterface->pfAddFuncDesc = NULL;
  pInterface->IFClass       = 0x0A;
  pInterface->IFSubClass    = 0;
  pInterface->IFProtocol    = 0;
  pInterface->EPs           = (1uL << pInitData->EPIn) | (1uL << pInitData->EPOut);
  pInterface->iName         = USB__AllocStringDesc("CDC DATA interface");

  if (USB_Global.pIadAPI != NULL) {
    USB_Global.pIadAPI->pfAdd((U8)FirstInterfaceNo, 2, 2, 2, 0);
  } else {
    USB_Global.Class    = 2;
    USB_Global.SubClass = 2;
    USB_Global.Protocol = 0;
  }
  pInst = &_aInst[_NumInstances];
  pInst->EPIn  = pInitData->EPIn;
  pInst->EPOut = pInitData->EPOut;
  pInst->EPInt = pInitData->EPInt;
  USBD_SetReadMode(pInst->EPOut, USB_READ_MODE_CONTINUOUS);
  USBD_SetWriteMode(pInst->EPIn, USB_WRITE_MODE_EXTEND);
  pInst->LineCoding = _DefaultLineCoding;
  pInst->ControlInterfaceNum = FirstInterfaceNo;
  pInst->DataInterfaceNum    = InterfaceNum;
  USB_LOG((USB_MTYPE_CDC, "CDC: Adding CDC interface with ControlInterface = %d, DataInterface = %d", FirstInterfaceNo, InterfaceNum));
  USB_MEMCPY(pInst->aStatus, aDefaultStatus, sizeof(aDefaultStatus));
  hInst = _NumInstances;
  _NumInstances++;
  return hInst;
}

/*********************************************************************
*
*       USBD_CDC_Init
*
*  Function description
*    Initialize the CDC component.
*/
void USBD_CDC_Init(void) {
  _IsInited = 0;
}

/*********************************************************************
*
*       USBD_CDC_Stall
*
*  Function description
*    Stall IN EP. Only used for testing.
*
*  Parameters
*    hInst  - Handle to CDC interface.
*/
void USBD_CDC_Stall(USB_CDC_HANDLE hInst) {
  CDC_CHECK_IF_HINST_VALID(hInst);
  USBD_StallEP(_aInst[hInst].EPIn);
}

/*********************************************************************
*
*       USBD_CDC_ReadAsync
*
*  Function description
*    Reads data from the host asynchronously. The function does not wait for
*    the data to be received. A callback function is called after the transfer
*    has completed successfully, an error occurred or the transfer was canceled.
*
*  Parameters
*    hInst:     Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pContext:  Pointer to an I/O context containing parameters and pointer to the callback function.
*    ShortRead: * 0: The transfer is completed successfully after all bytes have been read.
*               * 1: The transfer is completed successfully after one packet has been read.
*/
void USBD_CDC_ReadAsync(USB_CDC_HANDLE hInst, USB_ASYNC_IO_CONTEXT * pContext, int ShortRead) {
  CDC_CHECK_IF_HINST_VALID(hInst);
  USBD_ReadAsync(_aInst[hInst].EPOut, pContext, ShortRead);
}

/*********************************************************************
*
*       USBD_CDC_WriteAsync
*
*  Function description
*    Sends data to the host asynchronously. The function does not wait for
*    the data to be send. A callback function is called after the transfer
*    has completed successfully, an error occurred or the transfer was canceled.
*
*  Parameters
*    hInst:                 Handle to a valid CDC instance, returned by USBD_CDC_Add().
*    pContext:              Pointer to an I/O context containing parameters and pointer to the callback function.
*    Send0PacketIfRequired: Specifies that a zero-length packet shall be sent when
*                           the last data packet is a multiple of MaxPacketSize.
*/
void USBD_CDC_WriteAsync(USB_CDC_HANDLE hInst, USB_ASYNC_IO_CONTEXT * pContext, char Send0PacketIfRequired) {
  CDC_CHECK_IF_HINST_VALID(hInst);
  USBD_WriteAsync(_aInst[hInst].EPIn, pContext, Send0PacketIfRequired);
}

/*************************** End of file ****************************/
