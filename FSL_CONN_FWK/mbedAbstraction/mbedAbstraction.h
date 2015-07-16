#ifndef __MBED_ABSTRACTION_H__
#define __MBED_ABSTRACTION_H__


#ifdef __ICCARM__
#define OSA_EnterCritical(kCriticalDisableInt)  __disable_interrupt()
#define OSA_ExitCritical(kCriticalDisableInt)   __enable_interrupt()
#else
#define OSA_EnterCritical(kCriticalDisableInt)  __disable_irq()
#define OSA_ExitCritical(kCriticalDisableInt)   __enable_irq()
#endif



#endif
