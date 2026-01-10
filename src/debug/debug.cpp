#include "debug.h"

// Use Atmel ICE and the debug interface, note which fault handler the code stops at, 
// then continue to the hard_fault_handler_c function to see the faulting address
// Enter bt into the debug console to see the backtrace - look for the function call 
// that caused the fault in the backtrace output

void enableFaultExceptions() {
    // Enable Fault Exceptions
  /*SCB->SHCSR |=
    SCB_SHCSR_MEMFAULTENA_Msk |
    SCB_SHCSR_BUSFAULTENA_Msk |
    SCB_SHCSR_USGFAULTENA_Msk;*/
}

bool wdtResetOccurred() {
    return (PM->RCAUSE.reg & PM_RCAUSE_WDT) != 0;
}

char *resetReason() {
    uint8_t reason = PM->RCAUSE.reg;
    switch (reason) {
        case PM_RCAUSE_POR: return (char *)"Power On Reset";
        case PM_RCAUSE_BOD12: return (char *)"Brown Out 12 Detector Reset";
        case PM_RCAUSE_BOD33: return (char *)"Brown Out 33 Detector Reset";
        case PM_RCAUSE_EXT: return (char *)"External Reset";
        case PM_RCAUSE_WDT: return (char *)"Watchdog Reset";
        case PM_RCAUSE_SYST: return (char *)"System Reset Request";
        default: return (char *)"Unknown Reset Reason";
    }
}

extern "C" void HardFault_Handler(void)
{
    __asm volatile
    (
        "movs r0, #4                       \n"
        "mov  r1, lr                       \n"
        "tst  r0, r1                       \n"
        "beq  1f                           \n"
        "mrs  r0, psp                      \n"
        "b    2f                           \n"
        "1:                                \n"
        "mrs  r0, msp                      \n"
        "2:                                \n"
        "b hard_fault_handler_c            \n"
    );
}

extern "C" void hard_fault_handler_c(uint32_t *stacked_regs)
{
    [[maybe_unused]] volatile uint32_t fault_pc = stacked_regs[6];
    [[maybe_unused]] volatile uint32_t fault_lr = stacked_regs[5];
    [[maybe_unused]] volatile uint32_t fault_psr = stacked_regs[7];

    __asm volatile("bkpt #0");  // breakpoint for debugger

    while (1);
}

extern "C" void BusFault_Handler(void)
{
    __asm volatile
    (
        "movs r0, #4                       \n"
        "mov  r1, lr                       \n"
        "tst  r0, r1                       \n"
        "beq  1f                           \n"
        "mrs  r0, psp                      \n"
        "b    2f                           \n"
        "1:                                \n"
        "mrs  r0, msp                      \n"
        "2:                                \n"
        "b hard_fault_handler_c            \n"
    );
}

extern "C" void MemManage_Handler(void)
{
    __asm volatile
    (
        "movs r0, #4                       \n"
        "mov  r1, lr                       \n"
        "tst  r0, r1                       \n"
        "beq  1f                           \n"
        "mrs  r0, psp                      \n"
        "b    2f                           \n"
        "1:                                \n"
        "mrs  r0, msp                      \n"
        "2:                                \n"
        "b hard_fault_handler_c            \n"
    );
}

extern "C" void UsageFault_Handler(void)
{
    __asm volatile
    (
        "movs r0, #4                       \n"
        "mov  r1, lr                       \n"
        "tst  r0, r1                       \n"
        "beq  1f                           \n"
        "mrs  r0, psp                      \n"
        "b    2f                           \n"
        "1:                                \n"
        "mrs  r0, msp                      \n"
        "2:                                \n"
        "b hard_fault_handler_c            \n"
    );
}
