#ifndef USB_THREAD_H
#define USB_THREAD_H

#include "tx_api.h"


UINT USB_Tx_Thread_Init(TX_BYTE_POOL *Byte_Pool);
void USB_Rx_Thread(void);
void USB_Activate(void *Cdc);
void USB_Deactivate(void);


#endif /* USB_THREAD_H */
