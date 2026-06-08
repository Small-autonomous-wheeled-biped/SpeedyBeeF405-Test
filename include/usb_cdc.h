#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Minimal USB CDC ACM (virtual COM port) over OTG_FS (PA11/PA12, AF10).
 *
 * Presents the STM32 as VID=0x0483 / PID=0x5740 (ST CDC ACM).
 * Windows 10+ enumerates it without a driver.  Linux/macOS use cdc_acm.
 *
 * Usage:
 *   1. Call usb_cdc_init() once before the main loop.
 *   2. Call usb_cdc_poll() from the main loop (processes ISR-posted events).
 *   3. Use usb_cdc_write() to send bytes (non-blocking ring buffer).
 *   4. Ensure OTG_FS_IRQHandler() is in the vector table (startup_stm32f405xx.c
 *      has it as a weak alias — board.c overrides it).
 *
 * TX ring buffer: 512 bytes.  If full, bytes are silently dropped.
 * RX from PC: received but not stored (write-only interface for logging).
 */

/* Returns true once the host has opened the port (SET_CONFIGURATION done). */
bool usb_cdc_ready(void);

/* Queue bytes for CDC TX.  Returns number of bytes accepted.
   Non-blocking.  Safe to call from main loop only (not from IRQ). */
size_t usb_cdc_write(const uint8_t *data, size_t len);

/* Write a single character.  Used by board_log_char() integration. */
void usb_cdc_putchar(char c);

/* Initialise OTG_FS peripheral and USB descriptors.
   Call once after GPIO and clock init.                                      */
void usb_cdc_init(void);

/* Process deferred USB events.  Call from the main loop (not IRQ). */
void usb_cdc_poll(void);

/* Internal: called from OTG_FS_IRQHandler.  Do not call directly. */
void usb_cdc_irq(void);
