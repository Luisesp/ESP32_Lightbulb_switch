#ifndef CONFIG_HARDWARE_IO_H
#define CONFIG_HARDWARE_IO_H


    #ifdef __cplusplus
    extern "C" {
    #endif
        typedef void (*callback_handler_t)(int);
        void HwConfigInput(uint32_t* pPortNums, int count);
        void HwConfigOutput(uint32_t* pPortNums, int count);
        void HwConfigInterrupts(uint32_t* pPortNums, int count, callback_handler_t interrupt_callbackhandler);

    #ifdef __cplusplus
    }
    #endif
#endif  //CONFIG_HARDWARE_IO_H