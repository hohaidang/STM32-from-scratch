/*
 * core_cm4.h
 *
 *  Created on: Aug 17, 2020
 *      Author: prnsoft
 */

#ifndef INC_CORE_CM4_H_
#define INC_CORE_CM4_H_

#include <stdint.h>
using u8 = uint8_t;
using u32 = uint32_t;

#define __vo volatile

typedef struct {
    __vo u32 CSR;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
    __vo u32 RVR;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
    __vo u32 CVR;                   /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
    __vo u32 CALIB;                 /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Reg_t;

/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
    __vo u32 ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
    __vo u32 RESERVED0[24U];
    __vo u32 ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
    __vo u32 RSERVED1[24U];
    __vo u32 ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
    __vo u32 RESERVED2[24U];
    __vo u32 ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
    __vo u32 RESERVED3[24U];
    __vo u32 IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
    __vo u32 RESERVED4[56U];
    __vo u32 IPR[60U];              /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (32Bit) */
    __vo u32 RESERVED5[644U];
    __vo u32 STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Reg_t;

/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
  __vo u32 CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __vo u32 ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __vo u32 VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __vo u32 AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __vo u32 SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __vo u32 CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __vo u32 SHPR[3];                /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __vo u32 SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __vo u32 CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __vo u32 HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __vo u32 DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __vo u32 MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __vo u32 BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __vo u32 AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __vo u32 PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __vo u32 DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __vo u32 ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __vo u32 MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __vo u32 ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
  __vo u32 RESERVED0[5U];
  __vo u32 CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Reg_t;

#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define SysTick_BASEADDR    (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define NVIC_BASEADDR       (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define SCB_BASEADDR        (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

#define SYSTICK             ((SysTick_Reg_t   *)     SysTick_BASEADDR  )   /*!< SysTick configuration struct */
#define NVIC                ((NVIC_Reg_t      *)     NVIC_BASEADDR     )   /*!< NVIC configuration struct */
#define SCB                 ((SCB_Reg_t       *)     SCB_BASEADDR      )   /*!< SCB configuration struct */

/*
 * Bit position definitions SysTick_CSR
 * */
#define SYSTICK_CSR_ENA_Pos             (0u)
#define SYSTICK_CSR_ENA_Msk             (0x1UL << SYSTICK_CSR_ENA_Pos)
#define SYSTICK_CSR_ENA                 (SYSTICK_CSR_ENA_Msk)

#define SYSTICK_CSR_TICKINT_Pos         (1u)
#define SYSTICK_CSR_TICKINT_Msk         (0x1UL << SYSTICK_CSR_TICKINT_Pos)
#define SYSTICK_CSR_TICKINT             (SYSTICK_CSR_TICKINT_Msk)

#define SYSTICK_CSR_CLKSOURCE_Pos       (2u)
#define SYSTICK_CSR_CLKSOURCE_Msk       (0x1UL << SYSTICK_CSR_CLKSOURCE_Pos)
#define SYSTICK_CSR_CLKSOURCE           (SYSTICK_CSR_CLKSOURCE_Msk)

#define SYSTICK_CSR_COUNTFLAG_Pos       (16u)
#define SYSTICK_CSR_COUNTFLAG_Msk       (0x1UL << SYSTICK_CSR_COUNTFLAG_Pos)
#define SYSTICK_CSR_COUNTFLAG           (SYSTICK_CSR_COUNTFLAG_Msk)

// Cortex M4 just have 16 priorities NVIC (4 most significant bits)
// https://community.arm.com/developer/ip-products/system/b/embedded-blog/posts/cutting-through-the-confusion-with-arm-cortex-m-interrupt-priorities
#define NO_PR_BITS_IMPLEMENTED          4

#endif /* INC_CORE_CM4_H_ */
