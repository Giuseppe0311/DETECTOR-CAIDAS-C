/* Automatically generated file; DO NOT EDIT */
/* Espressif IoT Development Framework Linker Script */
/* Generated from: D:\PROYECTOS-PERSONALES\CuidadoAbuelito\build\esp-idf\esp_system\ld\sections.ld.in */

/*

 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD

 *

 * SPDX-License-Identifier: Apache-2.0

 */
/*

 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD

 *

 * SPDX-License-Identifier: Apache-2.0

 */
/*

 * Automatically generated file. DO NOT EDIT.

 * Espressif IoT Development Framework (ESP-IDF) 5.4.1 Configuration Header

 */
       
/* List of deprecated options */
/* CPU instruction prefetch padding size for flash mmap scenario */
/*

 * PMP region granularity size

 * Software may determine the PMP granularity by writing zero to pmp0cfg, then writing all ones

 * to pmpaddr0, then reading back pmpaddr0. If G is the index of the least-significant bit set,

 * the PMP granularity is 2^G+2 bytes.

 */
/* CPU instruction prefetch padding size for memory protection scenario */
/* Memory alignment size for PMS */
    /* rtc timer data (s_rtc_timer_retain_mem, see esp_clk.c files). For rtc_timer_data_in_rtc_mem section. */
/* Default entry point */
ENTRY(call_start_cpu0);
SECTIONS
{
  /**

   * RTC fast memory holds RTC wake stub code,

   * including from any source file named rtc_wake_stub*.c

   */
  .rtc.text :
  {
    
 . = ALIGN(4); 
 _rtc_text_start = ABSOLUTE(.);
    *(.rtc.literal .rtc.text .rtc.text.*)
    *rtc_wake_stub*.*(.literal .text .literal.* .text.*)
    _rtc_text_end = ABSOLUTE(.);
  } > rtc_iram_seg
  /**

   * This section is required to skip rtc.text area because rtc_iram_seg and

   * rtc_data_seg are reflect the same address space on different buses.

   */
  .rtc.dummy :
  {
    _rtc_dummy_start = ABSOLUTE(.);
    _rtc_fast_start = ABSOLUTE(.);
    . = SIZEOF(.rtc.text);
    _rtc_dummy_end = ABSOLUTE(.);
  } > rtc_data_seg
  /**

   * This section located in RTC FAST Memory area.

   * It holds data marked with RTC_FAST_ATTR attribute.

   * See the file "esp_attr.h" for more information.

   */
  .rtc.force_fast :
  {
    
 . = ALIGN(4); 
 _rtc_force_fast_start = ABSOLUTE(.);
    _coredump_rtc_fast_start = ABSOLUTE(.);
    *(.rtc.fast.coredump .rtc.fast.coredump.*)
    _coredump_rtc_fast_end = ABSOLUTE(.);
    *(.rtc.force_fast .rtc.force_fast.*)
    
 . = ALIGN(4); 
 _rtc_force_fast_end = ABSOLUTE(.);
  } > rtc_data_seg
  /**

   * RTC data section holds RTC wake stub

   * data/rodata, including from any source file

   * named rtc_wake_stub*.c and the data marked with

   * RTC_DATA_ATTR, RTC_RODATA_ATTR attributes.

   * The memory location of the data is dependent on

   * CONFIG_ESP32_RTCDATA_IN_FAST_MEM option.

   */
  .rtc.data :
  {
    _rtc_data_start = ABSOLUTE(.);
    _coredump_rtc_start = ABSOLUTE(.);
    *(.rtc.coredump .rtc.coredump.*)
    _coredump_rtc_end = ABSOLUTE(.);
    *(.rtc.data .rtc.data.*)
    *(.rtc.rodata .rtc.rodata.*)
    *rtc_wake_stub*.*(.data .rodata .data.* .rodata.*)
    _rtc_data_end = ABSOLUTE(.);
  } > rtc_data_location
  /* RTC bss, from any source file named rtc_wake_stub*.c */
  .rtc.bss (NOLOAD) :
  {
    _rtc_bss_start = ABSOLUTE(.);
    *rtc_wake_stub*.*(.bss .bss.*)
    *rtc_wake_stub*.*(COMMON)
    *(.rtc.bss)
    _rtc_bss_end = ABSOLUTE(.);
  } > rtc_data_location
  /**

   * This section holds data that should not be initialized at power up

   * and will be retained during deep sleep.

   * User data marked with RTC_NOINIT_ATTR will be placed

   * into this section. See the file "esp_attr.h" for more information.

   * The memory location of the data is dependent on

   * CONFIG_ESP32_RTCDATA_IN_FAST_MEM option.

   */
  .rtc_noinit (NOLOAD):
  {
    
 . = ALIGN(4); 
 _rtc_noinit_start = ABSOLUTE(.);
    *(.rtc_noinit .rtc_noinit.*)
    
 . = ALIGN(4); 
 _rtc_noinit_end = ABSOLUTE(.);
  } > rtc_data_location
  /**

   * This section located in RTC SLOW Memory area.

   * It holds data marked with RTC_SLOW_ATTR attribute.

   * See the file "esp_attr.h" for more information.

   */
  .rtc.force_slow :
  {
    
 . = ALIGN(4); 
 _rtc_force_slow_start = ABSOLUTE(.);
    *(.rtc.force_slow .rtc.force_slow.*)
    
 . = ALIGN(4); 
 _rtc_force_slow_end = ABSOLUTE(.);
  } > rtc_slow_seg
  /**

   * This section holds RTC FAST data that should have fixed addresses.

   * The data are not initialized at power-up and are retained during deep

   * sleep.

   */
  .rtc_fast_reserved (NOLOAD):
  {
    
 . = ALIGN(4); 
 _rtc_fast_reserved_start = ABSOLUTE(.);
    /**

     * New data can only be added here to ensure existing data are not moved.

     * Because data have adhered to the end of the segment and code is relied

     * on it.

     * >> put new data here <<

     */
    KEEP(*(.bootloader_data_rtc_mem .bootloader_data_rtc_mem.*))
    _rtc_fast_reserved_end = ABSOLUTE(.);
  } > rtc_fast_reserved_seg
  _rtc_fast_reserved_length = _rtc_fast_reserved_end - _rtc_fast_reserved_start;
  ASSERT((_rtc_fast_reserved_length <= LENGTH(rtc_fast_reserved_seg)),
          "RTC FAST reserved segment data does not fit.")
  /**

   * This section holds RTC SLOW data that should have fixed addresses.

   * The data are not initialized at power-up and are retained during deep

   * sleep.

   */
  .rtc_slow_reserved (NOLOAD):
  {
    
 . = ALIGN(4); 
 _rtc_slow_reserved_start = ABSOLUTE(.);
    /**

     * New data can only be added here to ensure existing data are not moved.

     * Because data have adhered to the end of the segment and code is relied

     * on it.

     * >> put new data here <<

     */
    *(.rtc_timer_data_in_rtc_mem .rtc_timer_data_in_rtc_mem.*)
    _rtc_slow_reserved_end = ABSOLUTE(.);
  } > rtc_slow_reserved_seg
  _rtc_slow_reserved_length = _rtc_slow_reserved_end - _rtc_slow_reserved_start;
  _rtc_reserved_length = _rtc_slow_reserved_length;
  ASSERT((_rtc_slow_reserved_length <= LENGTH(rtc_slow_reserved_seg)),
          "RTC SLOW reserved segment data does not fit.")
  /* Get size of rtc slow data based on rtc_data_location alias */
  _rtc_slow_length = (ORIGIN(rtc_slow_seg) == ORIGIN(rtc_data_location))
                        ? (_rtc_force_slow_end - _rtc_data_start)
                        : (_rtc_force_slow_end - _rtc_force_slow_start);
  _rtc_fast_length = (ORIGIN(rtc_slow_seg) == ORIGIN(rtc_data_location))
                        ? (_rtc_force_fast_end - _rtc_fast_start)
                        : (_rtc_noinit_end - _rtc_fast_start);
  ASSERT((_rtc_slow_length <= LENGTH(rtc_slow_seg)),
          "RTC_SLOW segment data does not fit.")
  ASSERT((_rtc_fast_length <= LENGTH(rtc_data_seg)),
          "RTC_FAST segment data does not fit.")
  /* Send .iram0 code to iram */
  .iram0.vectors :
  {
    _iram_start = ABSOLUTE(.);
    /* Vectors go to IRAM */
    _vector_table = ABSOLUTE(.);
    . = 0x0;
    KEEP(*(.WindowVectors.text));
    . = 0x180;
    KEEP(*(.Level2InterruptVector.text));
    . = 0x1c0;
    KEEP(*(.Level3InterruptVector.text));
    . = 0x200;
    KEEP(*(.Level4InterruptVector.text));
    . = 0x240;
    KEEP(*(.Level5InterruptVector.text));
    . = 0x280;
    KEEP(*(.DebugExceptionVector.text));
    . = 0x2c0;
    KEEP(*(.NMIExceptionVector.text));
    . = 0x300;
    KEEP(*(.KernelExceptionVector.text));
    . = 0x340;
    KEEP(*(.UserExceptionVector.text));
    . = 0x3C0;
    KEEP(*(.DoubleExceptionVector.text));
    . = 0x400;
    _invalid_pc_placeholder = ABSOLUTE(.);
    *(.*Vector.literal)
    *(.UserEnter.literal);
    *(.UserEnter.text);
    . = ALIGN (16);
    *(.entry.literal)
    *(.entry.text)
    *(.init.literal)
    *(.init)
    _init_end = ABSOLUTE(.);
  } > iram0_0_seg
  .iram0.text :
  {
    /* Code marked as running out of IRAM */
    _iram_text_start = ABSOLUTE(.);
    *(.iram1 .iram1.*)
    *libapp_trace.a:app_trace.*(.literal .literal.* .text .text.*)
    *libapp_trace.a:app_trace_util.*(.literal .literal.* .text .text.*)
    *libapp_trace.a:port_uart.*(.literal .literal.* .text .text.*)
    *libesp_driver_gptimer.a:gptimer.*(.literal.gptimer_default_isr .text.gptimer_default_isr)
    *libesp_driver_spi.a:spi_dma.*(.literal.spi_dma_reset .text.spi_dma_reset)
    *libesp_driver_spi.a:spi_dma.*(.literal.spi_dma_start .text.spi_dma_start)
    *libesp_event.a:default_event_loop.*(.literal.esp_event_isr_post .text.esp_event_isr_post)
    *libesp_event.a:esp_event.*(.literal.esp_event_isr_post_to .text.esp_event_isr_post_to)
    *libesp_hw_support.a:clk_utils.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_compare_and_set .text.esp_cpu_compare_and_set)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_reset .text.esp_cpu_reset)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_stall .text.esp_cpu_stall)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_unstall .text.esp_cpu_unstall)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_wait_for_intr .text.esp_cpu_wait_for_intr)
    *libesp_hw_support.a:esp_memory_utils.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:mspi_timing_tuning.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:periph_ctrl.*(.literal.periph_module_reset .text.periph_module_reset)
    *libesp_hw_support.a:periph_ctrl.*(.literal.wifi_module_disable .text.wifi_module_disable)
    *libesp_hw_support.a:periph_ctrl.*(.literal.wifi_module_enable .text.wifi_module_enable)
    *libesp_hw_support.a:rtc_clk.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:rtc_init.*(.literal.rtc_vddsdio_get_config .text.rtc_vddsdio_get_config)
    *libesp_hw_support.a:rtc_init.*(.literal.rtc_vddsdio_set_config .text.rtc_vddsdio_set_config)
    *libesp_hw_support.a:rtc_sleep.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:rtc_time.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:rtc_wdt.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:sar_periph_ctrl.*(.literal.sar_periph_ctrl_power_enable .text.sar_periph_ctrl_power_enable)
    *libesp_mm.a:cache_esp32.*(.literal .literal.* .text .text.*)
    *libesp_mm.a:esp_cache.*(.literal .literal.* .text .text.*)
    *libesp_ringbuf.a:(.literal .literal.* .text .text.*)
    *libesp_rom.a:esp_rom_print.*(.literal .literal.* .text .text.*)
    *libesp_rom.a:esp_rom_spiflash.*(.literal .literal.* .text .text.*)
    *libesp_rom.a:esp_rom_sys.*(.literal .literal.* .text .text.*)
    *libesp_system.a:esp_err.*(.literal .literal.* .text .text.*)
    *libesp_system.a:esp_system_chip.*(.literal.esp_system_abort .text.esp_system_abort)
    *libesp_system.a:image_process.*(.literal .literal.* .text .text.*)
    *libesp_system.a:ubsan.*(.literal .literal.* .text .text.*)
    *libfreertos.a:(EXCLUDE_FILE(*libfreertos.a:app_startup.* *libfreertos.a:event_groups.* *libfreertos.a:freertos_compatibility.* *libfreertos.a:heap_idf.* *libfreertos.a:idf_additions.* *libfreertos.a:list.* *libfreertos.a:port.* *libfreertos.a:port_common.* *libfreertos.a:port_systick.* *libfreertos.a:queue.* *libfreertos.a:stream_buffer.* *libfreertos.a:tasks.* *libfreertos.a:timers.*) .literal EXCLUDE_FILE(*libfreertos.a:app_startup.* *libfreertos.a:event_groups.* *libfreertos.a:freertos_compatibility.* *libfreertos.a:heap_idf.* *libfreertos.a:idf_additions.* *libfreertos.a:list.* *libfreertos.a:port.* *libfreertos.a:port_common.* *libfreertos.a:port_systick.* *libfreertos.a:queue.* *libfreertos.a:stream_buffer.* *libfreertos.a:tasks.* *libfreertos.a:timers.*) .literal.* EXCLUDE_FILE(*libfreertos.a:app_startup.* *libfreertos.a:event_groups.* *libfreertos.a:freertos_compatibility.* *libfreertos.a:heap_idf.* *libfreertos.a:idf_additions.* *libfreertos.a:list.* *libfreertos.a:port.* *libfreertos.a:port_common.* *libfreertos.a:port_systick.* *libfreertos.a:queue.* *libfreertos.a:stream_buffer.* *libfreertos.a:tasks.* *libfreertos.a:timers.*) .text EXCLUDE_FILE(*libfreertos.a:app_startup.* *libfreertos.a:event_groups.* *libfreertos.a:freertos_compatibility.* *libfreertos.a:heap_idf.* *libfreertos.a:idf_additions.* *libfreertos.a:list.* *libfreertos.a:port.* *libfreertos.a:port_common.* *libfreertos.a:port_systick.* *libfreertos.a:queue.* *libfreertos.a:stream_buffer.* *libfreertos.a:tasks.* *libfreertos.a:timers.*) .text.*)
    *libfreertos.a:event_groups.*(.literal.xEventGroupGetBitsFromISR .literal.xEventGroupGetStaticBuffer .text .text.xEventGroupGetBitsFromISR .text.xEventGroupGetStaticBuffer)
    *libfreertos.a:heap_idf.*(.literal.xPortCheckValidListMem .text .text.xPortCheckValidListMem)
    *libfreertos.a:list.*(.text .text.uxListRemove .text.vListInsert .text.vListInsertEnd)
    *libfreertos.a:port.*(.literal.vApplicationStackOverflowHook .literal.vPortAssertIfInISR .literal.vPortExitCritical .literal.vPortExitCriticalCompliance .literal.vPortSetStackWatchpoint .literal.vPortTaskWrapper .literal.vPortYieldOtherCore .literal.xPortEnterCriticalTimeout .literal.xPortEnterCriticalTimeoutCompliance .literal.xPortInIsrContext .text .text.vApplicationStackOverflowHook .text.vPortAssertIfInISR .text.vPortExitCritical .text.vPortExitCriticalCompliance .text.vPortSetStackWatchpoint .text.vPortTaskWrapper .text.vPortYieldOtherCore .text.xPortEnterCriticalTimeout .text.xPortEnterCriticalTimeoutCompliance .text.xPortInIsrContext)
    *libfreertos.a:port_common.*(.text)
    *libfreertos.a:port_systick.*(.literal.xPortSysTickHandler .text .text.xPortSysTickHandler)
    *libfreertos.a:queue.*(.literal.prvCopyDataFromQueue .literal.prvCopyDataToQueue .literal.prvNotifyQueueSetContainer .literal.uxQueueMessagesWaitingFromISR .literal.xQueueGenericGetStaticBuffers .literal.xQueueGenericSendFromISR .literal.xQueueGetMutexHolderFromISR .literal.xQueueGiveFromISR .literal.xQueueIsQueueEmptyFromISR .literal.xQueueIsQueueFullFromISR .literal.xQueuePeekFromISR .literal.xQueueReceiveFromISR .literal.xQueueSelectFromSetFromISR .text .text.prvCopyDataFromQueue .text.prvCopyDataToQueue .text.prvNotifyQueueSetContainer .text.uxQueueMessagesWaitingFromISR .text.xQueueGenericGetStaticBuffers .text.xQueueGenericSendFromISR .text.xQueueGetMutexHolderFromISR .text.xQueueGiveFromISR .text.xQueueIsQueueEmptyFromISR .text.xQueueIsQueueFullFromISR .text.xQueuePeekFromISR .text.xQueueReceiveFromISR .text.xQueueSelectFromSetFromISR)
    *libfreertos.a:stream_buffer.*(.literal.prvReadMessageFromBuffer .literal.prvWriteMessageToBuffer .literal.xStreamBufferGetStaticBuffers .literal.xStreamBufferReceiveCompletedFromISR .literal.xStreamBufferReceiveFromISR .literal.xStreamBufferSendCompletedFromISR .literal.xStreamBufferSendFromISR .literal.xStreamBufferSpacesAvailable .text .text.prvBytesInBuffer .text.prvReadMessageFromBuffer .text.prvWriteMessageToBuffer .text.xStreamBufferGetStaticBuffers .text.xStreamBufferReceiveCompletedFromISR .text.xStreamBufferReceiveFromISR .text.xStreamBufferSendCompletedFromISR .text.xStreamBufferSendFromISR .text.xStreamBufferSpacesAvailable)
    *libfreertos.a:tasks.*(.literal.__getreent .literal.prvCheckTaskCanBeScheduledSMP .literal.prvCreateIdleTasks .literal.prvIsYieldRequiredSMP .literal.prvResetNextTaskUnblockTime .literal.prvSelectHighestPriorityTaskSMP .literal.prvTaskIsTaskSuspended .literal.pvTaskGetCurrentTCBForCore .literal.uxTaskPriorityGetFromISR .literal.vTaskGenericNotifyGiveFromISR .literal.vTaskGetSnapshot .literal.vTaskSwitchContext .literal.xTaskCatchUpTicks .literal.xTaskGenericNotifyFromISR .literal.xTaskGetSchedulerState .literal.xTaskGetStaticBuffers .literal.xTaskGetTickCount .literal.xTaskGetTickCountFromISR .literal.xTaskIncrementTick .literal.xTaskIncrementTickOtherCores .literal.xTaskRemoveFromEventList .literal.xTaskResumeFromISR .text .text.__getreent .text.prvCheckTaskCanBeScheduledSMP .text.prvCreateIdleTasks .text.prvIsYieldRequiredSMP .text.prvResetNextTaskUnblockTime .text.prvSelectHighestPriorityTaskSMP .text.prvTaskIsTaskSuspended .text.pvTaskGetCurrentTCBForCore .text.uxTaskPriorityGetFromISR .text.vTaskGenericNotifyGiveFromISR .text.vTaskGetSnapshot .text.vTaskSwitchContext .text.xTaskCatchUpTicks .text.xTaskGenericNotifyFromISR .text.xTaskGetSchedulerState .text.xTaskGetStaticBuffers .text.xTaskGetTickCount .text.xTaskGetTickCountFromISR .text.xTaskIncrementTick .text.xTaskIncrementTickOtherCores .text.xTaskRemoveFromEventList .text.xTaskResumeFromISR .text.xTimerCreateTimerTask)
    *libfreertos.a:timers.*(.literal.xTimerGetReloadMode .literal.xTimerGetStaticBuffer .literal.xTimerPendFunctionCallFromISR .text .text.xTimerGetReloadMode .text.xTimerGetStaticBuffer .text.xTimerPendFunctionCallFromISR)
    *libgcc.a:lib2funcs.*(.literal .literal.* .text .text.*)
    *libgcov.a:(.literal .literal.* .text .text.*)
    *libhal.a:cache_hal_esp32.*(.literal .literal.* .text .text.*)
    *libhal.a:gpio_hal.*(.literal.gpio_hal_isolate_in_sleep .text.gpio_hal_isolate_in_sleep)
    *libhal.a:i2c_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:ledc_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:mmu_hal.*(.literal .literal.* .text .text.*)
    *libhal.a:spi_flash_encrypt_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:spi_flash_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:spi_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:spi_slave_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:timer_hal.*(.literal.timer_hal_capture_and_get_counter_value .text.timer_hal_capture_and_get_counter_value)
    *libhal.a:wdt_hal_iram.*(.literal .literal.* .text .text.*)
    *libheap.a:multi_heap.*(.literal.assert_valid_block .text.assert_valid_block)
    *libheap.a:multi_heap.*(.literal.multi_heap_aligned_alloc_impl .text.multi_heap_aligned_alloc_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_aligned_alloc_impl_offs .text.multi_heap_aligned_alloc_impl_offs)
    *libheap.a:multi_heap.*(.literal.multi_heap_aligned_alloc_offs .text.multi_heap_aligned_alloc_offs)
    *libheap.a:multi_heap.*(.literal.multi_heap_free_impl .text.multi_heap_free_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_allocated_size_impl .text.multi_heap_get_allocated_size_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_block_address_impl .text.multi_heap_get_block_address_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_first_block .text.multi_heap_get_first_block)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_next_block .text.multi_heap_get_next_block)
    *libheap.a:multi_heap.*(.literal.multi_heap_internal_lock .text.multi_heap_internal_lock)
    *libheap.a:multi_heap.*(.literal.multi_heap_internal_unlock .text.multi_heap_internal_unlock)
    *libheap.a:multi_heap.*(.literal.multi_heap_is_free .text.multi_heap_is_free)
    *libheap.a:multi_heap.*(.literal.multi_heap_malloc_impl .text.multi_heap_malloc_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_realloc_impl .text.multi_heap_realloc_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_set_lock .text.multi_heap_set_lock)
    *libheap.a:tlsf.*(.literal.tlsf_alloc_overhead .text.tlsf_alloc_overhead)
    *libheap.a:tlsf.*(.literal.tlsf_block_size .text.tlsf_block_size)
    *libheap.a:tlsf.*(.literal.tlsf_free .text.tlsf_free)
    *libheap.a:tlsf.*(.literal.tlsf_get_pool .text.tlsf_get_pool)
    *libheap.a:tlsf.*(.literal.tlsf_malloc .text.tlsf_malloc)
    *libheap.a:tlsf.*(.literal.tlsf_memalign .text.tlsf_memalign)
    *libheap.a:tlsf.*(.literal.tlsf_memalign_offs .text.tlsf_memalign_offs)
    *libheap.a:tlsf.*(.literal.tlsf_realloc .text.tlsf_realloc)
    *libheap.a:tlsf.*(.literal.tlsf_size .text.tlsf_size)
    *liblog.a:log_lock.*(.literal .literal.* .text .text.*)
    *liblog.a:log_timestamp.*(.literal.esp_log_early_timestamp .text.esp_log_early_timestamp)
    *liblog.a:log_timestamp.*(.literal.esp_log_timestamp .text.esp_log_timestamp)
    *liblog.a:log_write.*(.literal.esp_log_write .text.esp_log_write)
    *libnewlib.a:abort.*(.literal .literal.* .text .text.*)
    *libnewlib.a:assert.*(.literal .literal.* .text .text.*)
    *libnewlib.a:heap.*(.literal .literal.* .text .text.*)
    *libnewlib.a:stdatomic.*(.literal .literal.* .text .text.*)
    *librtc.a:(.literal .literal.* .text .text.*)
    *libsoc.a:lldesc.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:flash_brownout_hook.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:memspi_host_driver.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_boya.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_gd.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_generic.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_issi.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_mxic.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_th.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_winbond.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_wrap.*(.literal .literal.* .text .text.*)
    *libxt_hal.a:(.literal .literal.* .text .text.*)
    *libxtensa.a:(EXCLUDE_FILE(*libxtensa.a:xt_trax.* *libxtensa.a:xtensa_intr.*) .literal EXCLUDE_FILE(*libxtensa.a:xt_trax.* *libxtensa.a:xtensa_intr.*) .literal.* EXCLUDE_FILE(*libxtensa.a:xt_trax.* *libxtensa.a:xtensa_intr.*) .text EXCLUDE_FILE(*libxtensa.a:xt_trax.* *libxtensa.a:xtensa_intr.*) .text.*)
  } > iram0_0_seg
  .dram0.data :
  {
    _data_start = ABSOLUTE(.);
    *(.gnu.linkonce.d.*)
    *(.data1)
    *(.sdata)
    *(.sdata.*)
    *(.gnu.linkonce.s.*)
    *(.gnu.linkonce.s2.*)
    *(.jcr)
    *(EXCLUDE_FILE(*libbt.a *libbtdm_app.a) .data EXCLUDE_FILE(*libbt.a *libbtdm_app.a) .data.*)
    *(.dram1 .dram1.*)
    _coredump_dram_start = ABSOLUTE(.);
    *(.dram2.coredump .dram2.coredump.*)
    _coredump_dram_end = ABSOLUTE(.);
    *libapp_trace.a:app_trace.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libapp_trace.a:app_trace_util.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libapp_trace.a:port_uart.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    . = ALIGN(4);
    _bt_data_start = ABSOLUTE(.);
    *libbt.a:(.data .data.*)
    . = ALIGN(4);
    _bt_data_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_controller_data_start = ABSOLUTE(.);
    *libbtdm_app.a:(.data .data.*)
    . = ALIGN(4);
    _bt_controller_data_end = ABSOLUTE(.);
    *libesp_hw_support.a:clk_utils.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_hw_support.a:esp_memory_utils.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_hw_support.a:mspi_timing_tuning.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_hw_support.a:rtc_clk.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_mm.a:cache_esp32.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_mm.a:esp_cache.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_rom.a:esp_rom_print.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_rom.a:esp_rom_spiflash.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_rom.a:esp_rom_sys.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_system.a:esp_err.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_system.a:image_process.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_system.a:ubsan.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libgcov.a:(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:cache_hal_esp32.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:i2c_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:ledc_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:mmu_hal.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:spi_flash_encrypt_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:spi_flash_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:spi_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:spi_slave_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:wdt_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *liblog.a:log_lock.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libnewlib.a:abort.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libnewlib.a:assert.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libnewlib.a:heap.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libnewlib.a:stdatomic.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libphy.a:(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libsoc.a:lldesc.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:flash_brownout_hook.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:memspi_host_driver.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_boya.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_gd.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_generic.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_issi.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_mxic.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_th.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_winbond.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_wrap.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    _data_end = ABSOLUTE(.);
  } > dram0_0_seg
  /**

   * This section holds data that won't be initialised when startup.

   * This section locates in External RAM region.

   */
  .ext_ram_noinit (NOLOAD) :
  {
    _ext_ram_noinit_start = ABSOLUTE(.);
    *(.ext_ram_noinit*)
    
 . = ALIGN(4); 
 _ext_ram_noinit_end = ABSOLUTE(.);
  } > extern_ram_seg
  /**

   * This section holds data that should not be initialized at power up.

   * The section located in Internal SRAM memory region. The macro _NOINIT

   * can be used as attribute to place data into this section.

   * See the "esp_attr.h" file for more information.

   */
  .noinit (NOLOAD):
  {
    
 . = ALIGN(4); 
 _noinit_start = ABSOLUTE(.);
    *(.noinit .noinit.*)
    
 . = ALIGN(4); 
 _noinit_end = ABSOLUTE(.);
  } > dram0_0_seg
  /* External Memory BSS. (Variables with EXT_RAM_BSS_ATTR attribute). */
  .ext_ram.bss (NOLOAD) :
  {
    
 . = ALIGN(4); 
 _ext_ram_bss_start = ABSOLUTE(.);
    
 . = ALIGN(4); 
 _ext_ram_bss_end = ABSOLUTE(.);
  } > extern_ram_seg
  /* Shared RAM */
  .dram0.bss (NOLOAD) :
  {
    
 . = ALIGN(8); 
 _bss_start = ABSOLUTE(.);
    /**

     * ldgen places all bss-related data to mapping[dram0_bss]

     * (See components/esp_system/app.lf).

     */
    *(EXCLUDE_FILE(*libbt.a *libbtdm_app.a) .bss EXCLUDE_FILE(*libbt.a *libbtdm_app.a) .bss.*)
    *(.dynbss .dynsbss .gnu.linkonce.b .gnu.linkonce.b.* .gnu.linkonce.sb .gnu.linkonce.sb.* .gnu.linkonce.sb2 .gnu.linkonce.sb2.* .sbss .sbss.* .sbss2 .sbss2.* .scommon .share.mem)
    *(.ext_ram.bss .ext_ram.bss.*)
    *(EXCLUDE_FILE(*libbt.a *libbtdm_app.a) COMMON)
    . = ALIGN(4);
    _bt_bss_start = ABSOLUTE(.);
    *libbt.a:(.bss .bss.*)
    . = ALIGN(4);
    _bt_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_common_start = ABSOLUTE(.);
    *libbt.a:(COMMON)
    . = ALIGN(4);
    _bt_common_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_controller_bss_start = ABSOLUTE(.);
    *libbtdm_app.a:(.bss .bss.*)
    . = ALIGN(4);
    _bt_controller_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_controller_common_start = ABSOLUTE(.);
    *libbtdm_app.a:(COMMON)
    . = ALIGN(4);
    _bt_controller_common_end = ABSOLUTE(.);
    
 . = ALIGN(8); 
 _bss_end = ABSOLUTE(.);
  } > dram0_0_seg
  ASSERT(((_bss_end - ORIGIN(dram0_0_seg)) <= LENGTH(dram0_0_seg)),
          "DRAM segment data does not fit.")
  .flash.appdesc : ALIGN(0x10)
  {
    /**

     * Mark flash.rodata start.

     * This can be used for mmu driver to maintain virtual address

     */
    _rodata_reserved_start = ABSOLUTE(.);
    _rodata_start = ABSOLUTE(.);
    /* !DO NOT PUT ANYTHING BEFORE THIS! */
    /* Should be the first.  App version info. */
    *(.rodata_desc .rodata_desc.*)
    /* Should be the second. Custom app version info. */
    *(.rodata_custom_desc .rodata_custom_desc.*)
    /**

     * Create an empty gap within this section. Thanks to this, the end of this

     * section will match .flah.rodata's begin address. Thus, both sections

     * will be merged when creating the final bin image.

     */
    . = ALIGN(ALIGNOF(.flash.rodata));
  } > default_rodata_seg
  ASSERT((ADDR(.flash.rodata) == ADDR(.flash.appdesc) + SIZEOF(.flash.appdesc)), "The gap between .flash.appdesc and .flash.rodata must not exist to produce the final bin image.")
  .flash.rodata : ALIGN(0x10)
  {
    _flash_rodata_start = ABSOLUTE(.);
    *(EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:rtc_clk.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:wdt_hal_iram.* *liblog.a:log_lock.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .rodata EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:rtc_clk.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:wdt_hal_iram.* *liblog.a:log_lock.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .rodata.* EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:rtc_clk.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:wdt_hal_iram.* *liblog.a:log_lock.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .sdata2 EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:rtc_clk.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:wdt_hal_iram.* *liblog.a:log_lock.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .sdata2.* EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:rtc_clk.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:wdt_hal_iram.* *liblog.a:log_lock.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .srodata EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:rtc_clk.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:wdt_hal_iram.* *liblog.a:log_lock.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .srodata.*)
    *(.rodata_wlog_error .rodata_wlog_error.*)
    *(.rodata_wlog_info .rodata_wlog_info.*)
    *(.rodata_wlog_warning .rodata_wlog_warning.*)
    *(.irom1.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    /* C++ exception handlers table.  */
    
 . = ALIGN(4); 
 __XT_EXCEPTION_TABLE_ = ABSOLUTE(.);
    *(.xt_except_table)
    *(.gcc_except_table .gcc_except_table.*)
    *(.gnu.linkonce.e.*)
    
 . = ALIGN(4); 
 __XT_EXCEPTION_DESCS_ = ABSOLUTE(.);
    *(.xt_except_desc)
    *(.gnu.linkonce.h.*)
    __XT_EXCEPTION_DESCS_END__ = ABSOLUTE(.);
    *(.xt_except_desc_end)
    /**

     * C++ constructor tables.

     *

     * Excluding crtbegin.o/crtend.o since IDF doesn't use the toolchain crt.

     */
    
 . = ALIGN(4); 
 __init_array_start = ABSOLUTE(.);
    KEEP (*(EXCLUDE_FILE (*crtend.* *crtbegin.*) .ctors SORT(.ctors.*)))
    __init_array_end = ABSOLUTE(.);
    /* Addresses of memory regions reserved via SOC_RESERVE_MEMORY_REGION() */
    
 . = ALIGN(4); 
 soc_reserved_memory_region_start = ABSOLUTE(.);
    KEEP (*(.reserved_memory_address))
    soc_reserved_memory_region_end = ABSOLUTE(.);
    /* System init functions registered via ESP_SYSTEM_INIT_FN */
    
 . = ALIGN(4); 
 _esp_system_init_fn_array_start = ABSOLUTE(.);
    KEEP (*(SORT_BY_INIT_PRIORITY(.esp_system_init_fn.*)))
    _esp_system_init_fn_array_end = ABSOLUTE(.);
    _rodata_end = ABSOLUTE(.);
    /* Literals are also RO data. */
    _lit4_start = ABSOLUTE(.);
    *(*.lit4)
    *(.lit4.*)
    *(.gnu.linkonce.lit4.*)
    _lit4_end = ABSOLUTE(.);
    /* TLS data. */
    
 . = ALIGN(4); 
 _thread_local_start = ABSOLUTE(.);
    *(.tdata)
    *(.tdata.*)
    *(.tbss)
    *(.tbss.*)
    _thread_local_end = ABSOLUTE(.);
  } > default_rodata_seg
  _flash_rodata_align = ALIGNOF(.flash.rodata);
  /**

   * This section contains all the rodata that is not used

   * at runtime, helping to avoid an increase in binary size.

   */
  .flash.rodata_noload (NOLOAD) :
  {
    /**

     * This symbol marks the end of flash.rodata. It can be utilized by the MMU

     * driver to maintain the virtual address.

     * NOLOAD rodata may not be included in this section.

     */
    _rodata_reserved_end = ABSOLUTE(.);
    *(.rodata_wlog_debug .rodata_wlog_debug.*)
    *(.rodata_wlog_verbose .rodata_wlog_verbose.*)
  } > default_rodata_seg
  .flash.text :
  {
    _stext = .;
    /**

     * Mark the start of flash.text.

     * This can be used by the MMU driver to maintain the virtual address.

     */
    _instruction_reserved_start = ABSOLUTE(.);
    _text_start = ABSOLUTE(.);
    *(EXCLUDE_FILE(*libesp_ringbuf.a *libfreertos.a *libgcov.a *librtc.a *libxt_hal.a *libxtensa.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libesp_driver_gptimer.a:gptimer.* *libesp_driver_spi.a:spi_dma.* *libesp_event.a:default_event_loop.* *libesp_event.a:esp_event.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:periph_ctrl.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:rtc_init.* *libesp_hw_support.a:rtc_sleep.* *libesp_hw_support.a:rtc_time.* *libesp_hw_support.a:rtc_wdt.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_system.a:esp_err.* *libesp_system.a:esp_system_chip.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libgcc.a:lib2funcs.* *libhal.a:cache_hal_esp32.* *libhal.a:gpio_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:timer_hal.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log_lock.* *liblog.a:log_timestamp.* *liblog.a:log_write.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .literal EXCLUDE_FILE(*libesp_ringbuf.a *libfreertos.a *libgcov.a *librtc.a *libxt_hal.a *libxtensa.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libesp_driver_gptimer.a:gptimer.* *libesp_driver_spi.a:spi_dma.* *libesp_event.a:default_event_loop.* *libesp_event.a:esp_event.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:periph_ctrl.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:rtc_init.* *libesp_hw_support.a:rtc_sleep.* *libesp_hw_support.a:rtc_time.* *libesp_hw_support.a:rtc_wdt.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_system.a:esp_err.* *libesp_system.a:esp_system_chip.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libgcc.a:lib2funcs.* *libhal.a:cache_hal_esp32.* *libhal.a:gpio_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:timer_hal.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log_lock.* *liblog.a:log_timestamp.* *liblog.a:log_write.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .literal.* EXCLUDE_FILE(*libesp_ringbuf.a *libfreertos.a *libgcov.a *librtc.a *libxt_hal.a *libxtensa.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libesp_driver_gptimer.a:gptimer.* *libesp_driver_spi.a:spi_dma.* *libesp_event.a:default_event_loop.* *libesp_event.a:esp_event.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:periph_ctrl.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:rtc_init.* *libesp_hw_support.a:rtc_sleep.* *libesp_hw_support.a:rtc_time.* *libesp_hw_support.a:rtc_wdt.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_system.a:esp_err.* *libesp_system.a:esp_system_chip.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libgcc.a:lib2funcs.* *libhal.a:cache_hal_esp32.* *libhal.a:gpio_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:timer_hal.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log_lock.* *liblog.a:log_timestamp.* *liblog.a:log_write.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .text EXCLUDE_FILE(*libesp_ringbuf.a *libfreertos.a *libgcov.a *librtc.a *libxt_hal.a *libxtensa.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libesp_driver_gptimer.a:gptimer.* *libesp_driver_spi.a:spi_dma.* *libesp_event.a:default_event_loop.* *libesp_event.a:esp_event.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:periph_ctrl.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:rtc_init.* *libesp_hw_support.a:rtc_sleep.* *libesp_hw_support.a:rtc_time.* *libesp_hw_support.a:rtc_wdt.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_system.a:esp_err.* *libesp_system.a:esp_system_chip.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libgcc.a:lib2funcs.* *libhal.a:cache_hal_esp32.* *libhal.a:gpio_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:timer_hal.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log_lock.* *liblog.a:log_timestamp.* *liblog.a:log_write.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .text.*)
    *(.wifi0iram .wifi0iram.*)
    *(.wifiextrairam .wifiextrairam.*)
    *(.wifiorslpiram .wifiorslpiram.*)
    *(.wifirxiram .wifirxiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(.wifislprxiram .wifislprxiram.*)
    *libcoexist.a:(.coexiram .coexiram.*)
    *libcoexist.a:(.coexsleepiram .coexsleepiram.*)
    *libesp_driver_gptimer.a:gptimer.*(.literal.gptimer_del_timer .literal.gptimer_destroy .literal.gptimer_disable .literal.gptimer_enable .literal.gptimer_get_captured_count .literal.gptimer_get_raw_count .literal.gptimer_get_resolution .literal.gptimer_new_timer .literal.gptimer_register_event_callbacks .literal.gptimer_register_to_group .literal.gptimer_set_alarm_action .literal.gptimer_set_raw_count .literal.gptimer_start .literal.gptimer_stop .literal.gptimer_unregister_from_group .text .text.gptimer_del_timer .text.gptimer_destroy .text.gptimer_disable .text.gptimer_enable .text.gptimer_get_captured_count .text.gptimer_get_raw_count .text.gptimer_get_resolution .text.gptimer_new_timer .text.gptimer_register_event_callbacks .text.gptimer_register_to_group .text.gptimer_set_alarm_action .text.gptimer_set_raw_count .text.gptimer_start .text.gptimer_stop .text.gptimer_unregister_from_group)
    *libesp_driver_spi.a:spi_dma.*(.literal.spi_dma_enable_burst .text .text.spi_dma_enable_burst)
    *libesp_event.a:default_event_loop.*(.literal.esp_event_handler_instance_register .literal.esp_event_handler_instance_unregister .literal.esp_event_handler_register .literal.esp_event_handler_unregister .literal.esp_event_loop_create_default .literal.esp_event_loop_delete_default .literal.esp_event_post .text .text.esp_event_handler_instance_register .text.esp_event_handler_instance_unregister .text.esp_event_handler_register .text.esp_event_handler_unregister .text.esp_event_loop_create_default .text.esp_event_loop_delete_default .text.esp_event_post)
    *libesp_event.a:esp_event.*(.literal.base_node_add_handler .literal.base_node_remove_all_handler .literal.base_node_remove_handler .literal.esp_event_handler_instance_register_with .literal.esp_event_handler_instance_unregister_with .literal.esp_event_handler_register_with .literal.esp_event_handler_register_with_internal .literal.esp_event_handler_unregister_with .literal.esp_event_handler_unregister_with_internal .literal.esp_event_loop_create .literal.esp_event_loop_delete .literal.esp_event_loop_run .literal.esp_event_loop_run_task .literal.esp_event_post_to .literal.find_and_unregister_handler .literal.handler_instances_add .literal.handler_instances_remove .literal.handler_instances_remove_all .literal.loop_node_add_handler .literal.loop_node_remove_all_handler .literal.loop_node_remove_handler .literal.loop_remove_handler .text .text.base_node_add_handler .text.base_node_remove_all_handler .text.base_node_remove_handler .text.esp_event_dump .text.esp_event_handler_instance_register_with .text.esp_event_handler_instance_unregister_with .text.esp_event_handler_register_with .text.esp_event_handler_register_with_internal .text.esp_event_handler_unregister_with .text.esp_event_handler_unregister_with_internal .text.esp_event_loop_create .text.esp_event_loop_delete .text.esp_event_loop_run .text.esp_event_loop_run_task .text.esp_event_post_to .text.find_and_unregister_handler .text.handler_execute .text.handler_instances_add .text.handler_instances_remove .text.handler_instances_remove_all .text.loop_node_add_handler .text.loop_node_remove_all_handler .text.loop_node_remove_handler .text.loop_remove_handler)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_set_watchpoint .text .text.esp_cpu_clear_breakpoint .text.esp_cpu_clear_watchpoint .text.esp_cpu_set_breakpoint .text.esp_cpu_set_watchpoint)
    *libesp_hw_support.a:periph_ctrl.*(.literal.periph_ll_disable_clk_set_rst .literal.periph_ll_enable_clk_clear_rst .literal.periph_ll_get_clk_en_mask .literal.periph_ll_get_rst_en_mask .literal.periph_ll_reset .literal.periph_ll_wifi_module_disable_clk_set_rst .literal.periph_ll_wifi_module_enable_clk_clear_rst .literal.periph_module_disable .literal.periph_module_enable .literal.periph_rcc_acquire_enter .literal.periph_rcc_acquire_exit .literal.periph_rcc_release_enter .literal.periph_rcc_release_exit .text .text.periph_ll_disable_clk_set_rst .text.periph_ll_enable_clk_clear_rst .text.periph_ll_get_clk_en_mask .text.periph_ll_get_rst_en_mask .text.periph_ll_reset .text.periph_ll_wifi_module_disable_clk_set_rst .text.periph_ll_wifi_module_enable_clk_clear_rst .text.periph_module_disable .text.periph_module_enable .text.periph_rcc_acquire_enter .text.periph_rcc_acquire_exit .text.periph_rcc_release_enter .text.periph_rcc_release_exit)
    *libesp_hw_support.a:rtc_init.*(.literal.rtc_init .text .text.rtc_init)
    *libesp_hw_support.a:sar_periph_ctrl.*(.literal.s_sar_power_acquire .literal.s_sar_power_release .literal.sar_periph_ctrl_adc_continuous_power_acquire .literal.sar_periph_ctrl_adc_continuous_power_release .literal.sar_periph_ctrl_adc_oneshot_power_acquire .literal.sar_periph_ctrl_adc_oneshot_power_release .literal.sar_periph_ctrl_init .literal.sar_periph_ctrl_power_disable .literal.sar_periph_ctrl_pwdet_power_acquire .literal.sar_periph_ctrl_pwdet_power_release .text .text.s_sar_power_acquire .text.s_sar_power_release .text.sar_periph_ctrl_adc_continuous_power_acquire .text.sar_periph_ctrl_adc_continuous_power_release .text.sar_periph_ctrl_adc_oneshot_power_acquire .text.sar_periph_ctrl_adc_oneshot_power_release .text.sar_periph_ctrl_init .text.sar_periph_ctrl_power_disable .text.sar_periph_ctrl_pwdet_power_acquire .text.sar_periph_ctrl_pwdet_power_release)
    *libesp_system.a:esp_system_chip.*(.literal.esp_get_free_heap_size .literal.esp_get_free_internal_heap_size .literal.esp_get_idf_version .literal.esp_get_minimum_free_heap_size .text .text.esp_get_free_heap_size .text.esp_get_free_internal_heap_size .text.esp_get_idf_version .text.esp_get_minimum_free_heap_size)
    *libfreertos.a:app_startup.*(.literal .literal.* .text .text.*)
    *libfreertos.a:event_groups.*(.literal.prvTestWaitCondition .text.prvTestWaitCondition)
    *libfreertos.a:event_groups.*(.literal.vEventGroupClearBitsCallback .text.vEventGroupClearBitsCallback)
    *libfreertos.a:event_groups.*(.literal.vEventGroupDelete .text.vEventGroupDelete)
    *libfreertos.a:event_groups.*(.literal.vEventGroupSetBitsCallback .text.vEventGroupSetBitsCallback)
    *libfreertos.a:event_groups.*(.literal.xEventGroupClearBits .text.xEventGroupClearBits)
    *libfreertos.a:event_groups.*(.literal.xEventGroupCreate .text.xEventGroupCreate)
    *libfreertos.a:event_groups.*(.literal.xEventGroupCreateStatic .text.xEventGroupCreateStatic)
    *libfreertos.a:event_groups.*(.literal.xEventGroupSetBits .text.xEventGroupSetBits)
    *libfreertos.a:event_groups.*(.literal.xEventGroupSync .text.xEventGroupSync)
    *libfreertos.a:event_groups.*(.literal.xEventGroupWaitBits .text.xEventGroupWaitBits)
    *libfreertos.a:freertos_compatibility.*(.literal .literal.* .text .text.*)
    *libfreertos.a:heap_idf.*(.literal.pvPortMalloc .text.pvPortMalloc)
    *libfreertos.a:heap_idf.*(.literal.vPortFree .text.vPortFree)
    *libfreertos.a:heap_idf.*(.literal.xPortCheckValidTCBMem .text.xPortCheckValidTCBMem)
    *libfreertos.a:heap_idf.*(.literal.xPortGetFreeHeapSize .text.xPortGetFreeHeapSize)
    *libfreertos.a:heap_idf.*(.literal.xPortGetMinimumEverFreeHeapSize .text.xPortGetMinimumEverFreeHeapSize)
    *libfreertos.a:heap_idf.*(.literal.xPortcheckValidStackMem .text.xPortcheckValidStackMem)
    *libfreertos.a:idf_additions.*(.literal .literal.* .text .text.*)
    *libfreertos.a:list.*(.literal.vListInitialise .text.vListInitialise)
    *libfreertos.a:list.*(.literal.vListInitialiseItem .text.vListInitialiseItem)
    *libfreertos.a:port.*(.literal.pxPortInitialiseStack .text.pxPortInitialiseStack)
    *libfreertos.a:port.*(.literal.vPortCleanUpCoprocArea .text.vPortCleanUpCoprocArea)
    *libfreertos.a:port.*(.literal.vPortEndScheduler .text.vPortEndScheduler)
    *libfreertos.a:port.*(.literal.vPortTCBPreDeleteHook .text.vPortTCBPreDeleteHook)
    *libfreertos.a:port.*(.literal.vPortTLSPointersDelCb .text.vPortTLSPointersDelCb)
    *libfreertos.a:port.*(.literal.xPortGetTickRateHz .text.xPortGetTickRateHz)
    *libfreertos.a:port.*(.literal.xPortStartScheduler .text.xPortStartScheduler)
    *libfreertos.a:port_common.*(.literal.vApplicationGetIdleTaskMemory .text.vApplicationGetIdleTaskMemory)
    *libfreertos.a:port_common.*(.literal.vApplicationGetTimerTaskMemory .text.vApplicationGetTimerTaskMemory)
    *libfreertos.a:port_systick.*(.literal.vPortSetupTimer .text.vPortSetupTimer)
    *libfreertos.a:queue.*(.literal.prvGetDisinheritPriorityAfterTimeout .text.prvGetDisinheritPriorityAfterTimeout)
    *libfreertos.a:queue.*(.literal.prvInitialiseMutex .text.prvInitialiseMutex)
    *libfreertos.a:queue.*(.literal.prvInitialiseNewQueue .text.prvInitialiseNewQueue)
    *libfreertos.a:queue.*(.literal.uxQueueMessagesWaiting .text.uxQueueMessagesWaiting)
    *libfreertos.a:queue.*(.literal.uxQueueSpacesAvailable .text.uxQueueSpacesAvailable)
    *libfreertos.a:queue.*(.literal.vQueueDelete .text.vQueueDelete)
    *libfreertos.a:queue.*(.literal.vQueueWaitForMessageRestricted .text.vQueueWaitForMessageRestricted)
    *libfreertos.a:queue.*(.literal.xQueueAddToSet .text.xQueueAddToSet)
    *libfreertos.a:queue.*(.literal.xQueueCreateCountingSemaphore .text.xQueueCreateCountingSemaphore)
    *libfreertos.a:queue.*(.literal.xQueueCreateCountingSemaphoreStatic .text.xQueueCreateCountingSemaphoreStatic)
    *libfreertos.a:queue.*(.literal.xQueueCreateMutex .text.xQueueCreateMutex)
    *libfreertos.a:queue.*(.literal.xQueueCreateMutexStatic .text.xQueueCreateMutexStatic)
    *libfreertos.a:queue.*(.literal.xQueueCreateSet .text.xQueueCreateSet)
    *libfreertos.a:queue.*(.literal.xQueueGenericCreate .text.xQueueGenericCreate)
    *libfreertos.a:queue.*(.literal.xQueueGenericCreateStatic .text.xQueueGenericCreateStatic)
    *libfreertos.a:queue.*(.literal.xQueueGenericReset .text.xQueueGenericReset)
    *libfreertos.a:queue.*(.literal.xQueueGenericSend .text.xQueueGenericSend)
    *libfreertos.a:queue.*(.literal.xQueueGetMutexHolder .text.xQueueGetMutexHolder)
    *libfreertos.a:queue.*(.literal.xQueueGiveMutexRecursive .text.xQueueGiveMutexRecursive)
    *libfreertos.a:queue.*(.literal.xQueuePeek .text.xQueuePeek)
    *libfreertos.a:queue.*(.literal.xQueueReceive .text.xQueueReceive)
    *libfreertos.a:queue.*(.literal.xQueueRemoveFromSet .text.xQueueRemoveFromSet)
    *libfreertos.a:queue.*(.literal.xQueueSelectFromSet .text.xQueueSelectFromSet)
    *libfreertos.a:queue.*(.literal.xQueueSemaphoreTake .text.xQueueSemaphoreTake)
    *libfreertos.a:queue.*(.literal.xQueueTakeMutexRecursive .text.xQueueTakeMutexRecursive)
    *libfreertos.a:stream_buffer.*(.literal.prvInitialiseNewStreamBuffer .text.prvInitialiseNewStreamBuffer)
    *libfreertos.a:stream_buffer.*(.literal.prvReadBytesFromBuffer .text.prvReadBytesFromBuffer)
    *libfreertos.a:stream_buffer.*(.literal.prvWriteBytesToBuffer .text.prvWriteBytesToBuffer)
    *libfreertos.a:stream_buffer.*(.literal.vStreamBufferDelete .text.vStreamBufferDelete)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferBytesAvailable .text.xStreamBufferBytesAvailable)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferGenericCreate .text.xStreamBufferGenericCreate)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferGenericCreateStatic .text.xStreamBufferGenericCreateStatic)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferIsEmpty .text.xStreamBufferIsEmpty)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferIsFull .text.xStreamBufferIsFull)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferNextMessageLengthBytes .text.xStreamBufferNextMessageLengthBytes)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferReceive .text.xStreamBufferReceive)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferReset .text.xStreamBufferReset)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferSend .text.xStreamBufferSend)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferSetTriggerLevel .text.xStreamBufferSetTriggerLevel)
    *libfreertos.a:tasks.*(.literal.eTaskGetState .text.eTaskGetState)
    *libfreertos.a:tasks.*(.literal.pcTaskGetName .text.pcTaskGetName)
    *libfreertos.a:tasks.*(.literal.prvAddCurrentTaskToDelayedList .text.prvAddCurrentTaskToDelayedList)
    *libfreertos.a:tasks.*(.literal.prvAddNewTaskToReadyList .text.prvAddNewTaskToReadyList)
    *libfreertos.a:tasks.*(.literal.prvCheckTasksWaitingTermination .text.prvCheckTasksWaitingTermination)
    *libfreertos.a:tasks.*(.literal.prvDeleteTCB .text.prvDeleteTCB)
    *libfreertos.a:tasks.*(.literal.prvIdleTask .text.prvIdleTask)
    *libfreertos.a:tasks.*(.literal.prvInitialiseNewTask .text.prvInitialiseNewTask)
    *libfreertos.a:tasks.*(.literal.prvInitialiseTaskLists .text.prvInitialiseTaskLists)
    *libfreertos.a:tasks.*(.literal.prvReleaseKernelLock .text.prvReleaseKernelLock)
    *libfreertos.a:tasks.*(.literal.prvSearchForNameWithinSingleList .text.prvSearchForNameWithinSingleList)
    *libfreertos.a:tasks.*(.literal.prvTakeKernelLock .text.prvTakeKernelLock)
    *libfreertos.a:tasks.*(.literal.prvTaskCheckFreeStackSpace .text.prvTaskCheckFreeStackSpace)
    *libfreertos.a:tasks.*(.literal.prvTaskPriorityRaise .text.prvTaskPriorityRaise)
    *libfreertos.a:tasks.*(.literal.prvTaskPriorityRestore .text.prvTaskPriorityRestore)
    *libfreertos.a:tasks.*(.literal.pvTaskGetThreadLocalStoragePointer .text.pvTaskGetThreadLocalStoragePointer)
    *libfreertos.a:tasks.*(.literal.pvTaskIncrementMutexHeldCount .text.pvTaskIncrementMutexHeldCount)
    *libfreertos.a:tasks.*(.literal.pxGetTaskListByIndex .text.pxGetTaskListByIndex)
    *libfreertos.a:tasks.*(.literal.pxTaskGetStackStart .text.pxTaskGetStackStart)
    *libfreertos.a:tasks.*(.literal.ulTaskGenericNotifyTake .text.ulTaskGenericNotifyTake)
    *libfreertos.a:tasks.*(.literal.ulTaskGenericNotifyValueClear .text.ulTaskGenericNotifyValueClear)
    *libfreertos.a:tasks.*(.literal.uxTaskGetNumberOfTasks .text.uxTaskGetNumberOfTasks)
    *libfreertos.a:tasks.*(.literal.uxTaskGetSnapshotAll .text.uxTaskGetSnapshotAll)
    *libfreertos.a:tasks.*(.literal.uxTaskGetStackHighWaterMark .text.uxTaskGetStackHighWaterMark)
    *libfreertos.a:tasks.*(.literal.uxTaskGetStackHighWaterMark2 .text.uxTaskGetStackHighWaterMark2)
    *libfreertos.a:tasks.*(.literal.uxTaskPriorityGet .text.uxTaskPriorityGet)
    *libfreertos.a:tasks.*(.literal.uxTaskResetEventItemValue .text.uxTaskResetEventItemValue)
    *libfreertos.a:tasks.*(.literal.vTaskDelay .text.vTaskDelay)
    *libfreertos.a:tasks.*(.literal.vTaskDelete .text.vTaskDelete)
    *libfreertos.a:tasks.*(.literal.vTaskEndScheduler .text.vTaskEndScheduler)
    *libfreertos.a:tasks.*(.literal.vTaskInternalSetTimeOutState .text.vTaskInternalSetTimeOutState)
    *libfreertos.a:tasks.*(.literal.vTaskMissedYield .text.vTaskMissedYield)
    *libfreertos.a:tasks.*(.literal.vTaskPlaceOnEventList .text.vTaskPlaceOnEventList)
    *libfreertos.a:tasks.*(.literal.vTaskPlaceOnEventListRestricted .text.vTaskPlaceOnEventListRestricted)
    *libfreertos.a:tasks.*(.literal.vTaskPlaceOnUnorderedEventList .text.vTaskPlaceOnUnorderedEventList)
    *libfreertos.a:tasks.*(.literal.vTaskPriorityDisinheritAfterTimeout .text.vTaskPriorityDisinheritAfterTimeout)
    *libfreertos.a:tasks.*(.literal.vTaskPrioritySet .text.vTaskPrioritySet)
    *libfreertos.a:tasks.*(.literal.vTaskRemoveFromUnorderedEventList .text.vTaskRemoveFromUnorderedEventList)
    *libfreertos.a:tasks.*(.literal.vTaskResume .text.vTaskResume)
    *libfreertos.a:tasks.*(.literal.vTaskSetThreadLocalStoragePointer .text.vTaskSetThreadLocalStoragePointer)
    *libfreertos.a:tasks.*(.literal.vTaskSetThreadLocalStoragePointerAndDelCallback .text.vTaskSetThreadLocalStoragePointerAndDelCallback)
    *libfreertos.a:tasks.*(.literal.vTaskSetTimeOutState .text.vTaskSetTimeOutState)
    *libfreertos.a:tasks.*(.literal.vTaskStartScheduler .text.vTaskStartScheduler)
    *libfreertos.a:tasks.*(.literal.vTaskSuspend .text.vTaskSuspend)
    *libfreertos.a:tasks.*(.literal.vTaskSuspendAll .text.vTaskSuspendAll)
    *libfreertos.a:tasks.*(.literal.xTaskAbortDelay .text.xTaskAbortDelay)
    *libfreertos.a:tasks.*(.literal.xTaskCheckForTimeOut .text.xTaskCheckForTimeOut)
    *libfreertos.a:tasks.*(.literal.xTaskCreatePinnedToCore .text.xTaskCreatePinnedToCore)
    *libfreertos.a:tasks.*(.literal.xTaskCreateStaticPinnedToCore .text.xTaskCreateStaticPinnedToCore)
    *libfreertos.a:tasks.*(.literal.xTaskDelayUntil .text.xTaskDelayUntil)
    *libfreertos.a:tasks.*(.literal.xTaskGenericNotify .text.xTaskGenericNotify)
    *libfreertos.a:tasks.*(.literal.xTaskGenericNotifyStateClear .text.xTaskGenericNotifyStateClear)
    *libfreertos.a:tasks.*(.literal.xTaskGenericNotifyWait .text.xTaskGenericNotifyWait)
    *libfreertos.a:tasks.*(.literal.xTaskGetCoreID .text.xTaskGetCoreID)
    *libfreertos.a:tasks.*(.literal.xTaskGetCurrentTaskHandle .text.xTaskGetCurrentTaskHandle)
    *libfreertos.a:tasks.*(.literal.xTaskGetCurrentTaskHandleForCore .text.xTaskGetCurrentTaskHandleForCore)
    *libfreertos.a:tasks.*(.literal.xTaskGetHandle .text.xTaskGetHandle)
    *libfreertos.a:tasks.*(.literal.xTaskGetIdleTaskHandle .text.xTaskGetIdleTaskHandle)
    *libfreertos.a:tasks.*(.literal.xTaskGetIdleTaskHandleForCore .text.xTaskGetIdleTaskHandleForCore)
    *libfreertos.a:tasks.*(.literal.xTaskGetNext .text.xTaskGetNext)
    *libfreertos.a:tasks.*(.literal.xTaskPriorityDisinherit .text.xTaskPriorityDisinherit)
    *libfreertos.a:tasks.*(.literal.xTaskPriorityInherit .text.xTaskPriorityInherit)
    *libfreertos.a:tasks.*(.literal.xTaskResumeAll .text.xTaskResumeAll)
    *libfreertos.a:timers.*(.literal.pcTimerGetName .text.pcTimerGetName)
    *libfreertos.a:timers.*(.literal.prvCheckForValidListAndQueue .text.prvCheckForValidListAndQueue)
    *libfreertos.a:timers.*(.literal.prvGetNextExpireTime .text.prvGetNextExpireTime)
    *libfreertos.a:timers.*(.literal.prvInitialiseNewTimer .text.prvInitialiseNewTimer)
    *libfreertos.a:timers.*(.literal.prvInsertTimerInActiveList .text.prvInsertTimerInActiveList)
    *libfreertos.a:timers.*(.literal.prvProcessExpiredTimer .text.prvProcessExpiredTimer)
    *libfreertos.a:timers.*(.literal.prvProcessReceivedCommands .text.prvProcessReceivedCommands)
    *libfreertos.a:timers.*(.literal.prvProcessTimerOrBlockTask .text.prvProcessTimerOrBlockTask)
    *libfreertos.a:timers.*(.literal.prvReloadTimer .text.prvReloadTimer)
    *libfreertos.a:timers.*(.literal.prvSampleTimeNow .text.prvSampleTimeNow)
    *libfreertos.a:timers.*(.literal.prvSwitchTimerLists .text.prvSwitchTimerLists)
    *libfreertos.a:timers.*(.literal.prvTimerTask .text.prvTimerTask)
    *libfreertos.a:timers.*(.literal.pvTimerGetTimerID .text.pvTimerGetTimerID)
    *libfreertos.a:timers.*(.literal.uxTimerGetReloadMode .text.uxTimerGetReloadMode)
    *libfreertos.a:timers.*(.literal.vTimerSetReloadMode .text.vTimerSetReloadMode)
    *libfreertos.a:timers.*(.literal.vTimerSetTimerID .text.vTimerSetTimerID)
    *libfreertos.a:timers.*(.literal.xTimerCreate .text.xTimerCreate)
    *libfreertos.a:timers.*(.literal.xTimerCreateStatic .text.xTimerCreateStatic)
    *libfreertos.a:timers.*(.literal.xTimerCreateTimerTask .text.xTimerCreateTimerTask)
    *libfreertos.a:timers.*(.literal.xTimerGenericCommand .text.xTimerGenericCommand)
    *libfreertos.a:timers.*(.literal.xTimerGetExpiryTime .text.xTimerGetExpiryTime)
    *libfreertos.a:timers.*(.literal.xTimerGetPeriod .text.xTimerGetPeriod)
    *libfreertos.a:timers.*(.literal.xTimerGetTimerDaemonTaskHandle .text.xTimerGetTimerDaemonTaskHandle)
    *libfreertos.a:timers.*(.literal.xTimerIsTimerActive .text.xTimerIsTimerActive)
    *libfreertos.a:timers.*(.literal.xTimerPendFunctionCall .text.xTimerPendFunctionCall)
    *libhal.a:gpio_hal.*(.literal.gpio_hal_intr_disable .literal.gpio_hal_intr_enable_on_core .text .text.gpio_hal_intr_disable .text.gpio_hal_intr_enable_on_core)
    *libhal.a:timer_hal.*(.literal.timer_hal_deinit .literal.timer_hal_init .text .text.timer_hal_deinit .text.timer_hal_init .text.timer_hal_set_counter_value)
    *libheap.a:multi_heap.*(.literal.multi_heap_check .literal.multi_heap_dump .literal.multi_heap_dump_tlsf .literal.multi_heap_get_info_impl .literal.multi_heap_register_impl .literal.multi_heap_reset_minimum_free_bytes .literal.multi_heap_restore_minimum_free_bytes .literal.multi_heap_walk .text .text.multi_heap_check .text.multi_heap_dump .text.multi_heap_dump_tlsf .text.multi_heap_free_size_impl .text.multi_heap_get_info_impl .text.multi_heap_get_info_tlsf .text.multi_heap_minimum_free_size_impl .text.multi_heap_register_impl .text.multi_heap_reset_minimum_free_bytes .text.multi_heap_restore_minimum_free_bytes .text.multi_heap_walk)
    *libheap.a:tlsf.*(.literal.control_construct .literal.default_walker .literal.integrity_walker .literal.tlsf_add_pool .literal.tlsf_check .literal.tlsf_check_pool .literal.tlsf_create .literal.tlsf_create_with_pool .literal.tlsf_fit_size .literal.tlsf_malloc_addr .literal.tlsf_remove_pool .literal.tlsf_walk_pool .text .text.control_construct .text.default_walker .text.integrity_walker .text.tlsf_add_pool .text.tlsf_check .text.tlsf_check_pool .text.tlsf_create .text.tlsf_create_with_pool .text.tlsf_destroy .text.tlsf_fit_size .text.tlsf_malloc_addr .text.tlsf_pool_overhead .text.tlsf_remove_pool .text.tlsf_walk_pool)
    *liblog.a:log_timestamp.*(.text)
    *liblog.a:log_write.*(.literal.esp_log_set_vprintf .literal.esp_log_writev .text .text.esp_log_set_vprintf .text.esp_log_writev)
    *libphy.a:(.phyiram .phyiram.*)
    *libxtensa.a:xt_trax.*(.literal .literal.* .text .text.*)
    *libxtensa.a:xtensa_intr.*(.literal .literal.* .text .text.*)
    *(.stub)
    *(.gnu.warning)
    *(.gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */
    /**

     * CPU will try to prefetch up to 16 bytes of of instructions.

     * This means that any configuration (e.g. MMU, PMS) must allow

     * safe access to up to 16 bytes after the last real instruction,

     * add dummy bytes to ensure this.

     */
    . += 16;
    _text_end = ABSOLUTE(.);
    /**

     * Mark the flash.text end.

     * This can be used for MMU driver to maintain virtual address.

     */
    _instruction_reserved_end = ABSOLUTE(.);
    _etext = .;
    /**

     * Similar to _iram_start, this symbol goes here so it is

     * resolved by addr2line in preference to the first symbol in

     * the flash.text segment.

     */
    _flash_cache_start = ABSOLUTE(0);
  } > default_code_seg
  /* Marks the end of IRAM code segment */
  .iram0.text_end (NOLOAD) :
  {
    
 . = ALIGN(4); 
 _iram_text_end = ABSOLUTE(.);
  } > iram0_0_seg
  .iram0.data :
  {
    
 . = ALIGN(4); 
 _iram_data_start = ABSOLUTE(.);
    *(.iram.data .iram.data.*)
    _coredump_iram_start = ABSOLUTE(.);
    *(.iram2.coredump .iram2.coredump.*)
    _coredump_iram_end = ABSOLUTE(.);
    _iram_data_end = ABSOLUTE(.);
  } > iram0_0_seg
  .iram0.bss (NOLOAD) :
  {
    
 . = ALIGN(4); 
 _iram_bss_start = ABSOLUTE(.);
    *(.iram.bss .iram.bss.*)
    _iram_bss_end = ABSOLUTE(.);
    
 . = ALIGN(4); 
 _iram_end = ABSOLUTE(.);
  } > iram0_0_seg
  /* Marks the end of data, bss and possibly rodata  */
  .dram0.heap_start (NOLOAD) :
  {
    /* Lowest possible start address for the heap */
    
 . = ALIGN(8); 
 _heap_low_start = ABSOLUTE(.);
  } > dram0_0_seg
  /* DWARF 1 */
  .debug 0 : { *(.debug) }
  .line 0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo 0 : { *(.debug_srcinfo) }
  .debug_sfnames 0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges 0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info 0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev 0 : { *(.debug_abbrev) }
  .debug_line 0 : { *(.debug_line) }
  .debug_frame 0 : { *(.debug_frame) }
  .debug_str 0 : { *(.debug_str) }
  .debug_loc 0 : { *(.debug_loc) }
  .debug_macinfo 0 : { *(.debug_macinfo) }
  .debug_pubtypes 0 : { *(.debug_pubtypes) }
  /* DWARF 3 */
  .debug_ranges 0 : { *(.debug_ranges) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames 0 : { *(.debug_varnames) }
  /* GNU DWARF 2 extensions */
  .debug_gnu_pubnames 0 : { *(.debug_gnu_pubnames) }
  .debug_gnu_pubtypes 0 : { *(.debug_gnu_pubtypes) }
  /* DWARF 4 */
  .debug_types 0 : { *(.debug_types) }
  /* DWARF 5 */
  .debug_addr 0 : { *(.debug_addr) }
  .debug_line_str 0 : { *(.debug_line_str) }
  .debug_loclists 0 : { *(.debug_loclists) }
  .debug_macro 0 : { *(.debug_macro) }
  .debug_names 0 : { *(.debug_names) }
  .debug_rnglists 0 : { *(.debug_rnglists) }
  .debug_str_offsets 0 : { *(.debug_str_offsets) }
  .comment 0 : { *(.comment) }
  .note.GNU-stack 0: { *(.note.GNU-stack) }
/**

 * .xt.prop and .xt.lit sections will be used by the debugger and disassembler

 * to get more information about raw data present in the code.

 * Indeed, it may be required to add some padding at some points in the code

 * in order to align a branch/jump destination on a particular bound.

 * Padding these instructions will generate null bytes that shall be

 * interpreted as data, and not code by the debugger or disassembler.

 * This section will only be present in the ELF file, not in the final binary

 * For more details, check GCC-212

 */
  .xtensa.info 0: { *(.xtensa.info) }
  .xt.prop 0 : { *(.xt.prop .xt.prop.* .gnu.linkonce.prop.*) }
  .xt.lit 0 : { *(.xt.lit .xt.lit.* .gnu.linkonce.p.*) }
  /DISCARD/ :
  {
   *(.fini)
   *(.eh_frame_hdr)
   *(.eh_frame)
  }
}
ASSERT(((_iram_end - ORIGIN(iram0_0_seg)) <= LENGTH(iram0_0_seg)),
          "IRAM0 segment data does not fit.")
ASSERT(((_heap_low_start - ORIGIN(dram0_0_seg)) <= LENGTH(dram0_0_seg)),
          "DRAM segment data does not fit.")
