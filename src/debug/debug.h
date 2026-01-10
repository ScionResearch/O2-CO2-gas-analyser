#pragma once

#include <Arduino.h>

// Fault handler declarations - definitions in debug.cpp
// Use Atmel ICE and the debug interface, note which fault handler the code stops at,
// then continue to the hard_fault_handler_c function to see the faulting address
// Enter bt into the debug console to see the backtrace

void enableFaultExceptions();
bool wdtResetOccurred();
char *resetReason();

extern "C" void HardFault_Handler(void);
extern "C" void BusFault_Handler(void);
extern "C" void MemManage_Handler(void);
extern "C" void UsageFault_Handler(void);
extern "C" void hard_fault_handler_c(uint32_t *stacked_regs);