#ifndef USB_THREAD_H
#define USB_THREAD_H

#include "tx_api.h"


#define USB_TX_RESP    (1UL << 0)
#define USB_TX_FAST    (1UL << 1)
#define USB_TX_NORMAL  (1UL << 2)
#define USB_TX_ALL     (USB_TX_RESP | USB_TX_FAST | USB_TX_NORMAL)


UINT USB_Tx_Thread_Init(TX_BYTE_POOL *Byte_Pool);
void USB_Rx_Thread(void);
void USB_Tx_Wake(ULONG Flag);
void USB_Tx_Poll(void);
void USB_Activate(void *Cdc);
void USB_Deactivate(void);


#endif /* USB_THREAD_H */
