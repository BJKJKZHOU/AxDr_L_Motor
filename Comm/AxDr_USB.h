#ifndef AXDR_USB_H
#define AXDR_USB_H

#include "tx_api.h"


UINT AxDr_USB_Tx_Init(TX_BYTE_POOL *Byte_Pool);
void AxDr_USB_Rx(void);
void AxDr_USB_Activate(void *Cdc);
void AxDr_USB_Deactivate(void);


#endif /* AXDR_USB_H */
