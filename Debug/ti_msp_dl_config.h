/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_Motor */
#define PWM_Motor_INST                                                     TIMG8
#define PWM_Motor_INST_IRQHandler                               TIMG8_IRQHandler
#define PWM_Motor_INST_INT_IRQN                                 (TIMG8_INT_IRQn)
#define PWM_Motor_INST_CLK_FREQ                                         40000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_Motor_C0_PORT                                             GPIOB
#define GPIO_PWM_Motor_C0_PIN                                      DL_GPIO_PIN_6
#define GPIO_PWM_Motor_C0_IOMUX                                  (IOMUX_PINCM23)
#define GPIO_PWM_Motor_C0_IOMUX_FUNC                 IOMUX_PINCM23_PF_TIMG8_CCP0
#define GPIO_PWM_Motor_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_Motor_C1_PORT                                             GPIOB
#define GPIO_PWM_Motor_C1_PIN                                      DL_GPIO_PIN_7
#define GPIO_PWM_Motor_C1_IOMUX                                  (IOMUX_PINCM24)
#define GPIO_PWM_Motor_C1_IOMUX_FUNC                 IOMUX_PINCM24_PF_TIMG8_CCP1
#define GPIO_PWM_Motor_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for PIT_FOR_CONTROL */
#define PIT_FOR_CONTROL_INST                                             (TIMG0)
#define PIT_FOR_CONTROL_INST_IRQHandler                         TIMG0_IRQHandler
#define PIT_FOR_CONTROL_INST_INT_IRQN                           (TIMG0_INT_IRQn)
#define PIT_FOR_CONTROL_INST_LOAD_VALUE                                  (2499U)




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C0
#define I2C_OLED_INST_IRQHandler                                 I2C0_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C0_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOA
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_0
#define GPIO_I2C_OLED_IOMUX_SDA                                   (IOMUX_PINCM1)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                    IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOA
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_1
#define GPIO_I2C_OLED_IOMUX_SCL                                   (IOMUX_PINCM2)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                    IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           40000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOB
#define GPIO_UART_0_TX_PORT                                                GPIOB
#define GPIO_UART_0_RX_PIN                                         DL_GPIO_PIN_1
#define GPIO_UART_0_TX_PIN                                         DL_GPIO_PIN_0
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM13)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM12)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM13_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM12_PF_UART0_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_40_MHZ_9600_BAUD                                       (260)
#define UART_0_FBRD_40_MHZ_9600_BAUD                                        (27)




/* Defines for SPI_IMU660RB */
#define SPI_IMU660RB_INST                                                  SPI0
#define SPI_IMU660RB_INST_IRQHandler                            SPI0_IRQHandler
#define SPI_IMU660RB_INST_INT_IRQN                                SPI0_INT_IRQn
#define GPIO_SPI_IMU660RB_PICO_PORT                                       GPIOA
#define GPIO_SPI_IMU660RB_PICO_PIN                                DL_GPIO_PIN_9
#define GPIO_SPI_IMU660RB_IOMUX_PICO                            (IOMUX_PINCM20)
#define GPIO_SPI_IMU660RB_IOMUX_PICO_FUNC            IOMUX_PINCM20_PF_SPI0_PICO
#define GPIO_SPI_IMU660RB_POCI_PORT                                       GPIOA
#define GPIO_SPI_IMU660RB_POCI_PIN                               DL_GPIO_PIN_10
#define GPIO_SPI_IMU660RB_IOMUX_POCI                            (IOMUX_PINCM21)
#define GPIO_SPI_IMU660RB_IOMUX_POCI_FUNC            IOMUX_PINCM21_PF_SPI0_POCI
/* GPIO configuration for SPI_IMU660RB */
#define GPIO_SPI_IMU660RB_SCLK_PORT                                       GPIOA
#define GPIO_SPI_IMU660RB_SCLK_PIN                               DL_GPIO_PIN_11
#define GPIO_SPI_IMU660RB_IOMUX_SCLK                            (IOMUX_PINCM22)
#define GPIO_SPI_IMU660RB_IOMUX_SCLK_FUNC            IOMUX_PINCM22_PF_SPI0_SCLK



/* Defines for AIN1: GPIOB.8 with pinCMx 25 on package pin 60 */
#define GPIO_Motor_AIN1_PORT                                             (GPIOB)
#define GPIO_Motor_AIN1_PIN                                      (DL_GPIO_PIN_8)
#define GPIO_Motor_AIN1_IOMUX                                    (IOMUX_PINCM25)
/* Defines for AIN2: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_Motor_AIN2_PORT                                             (GPIOA)
#define GPIO_Motor_AIN2_PIN                                     (DL_GPIO_PIN_12)
#define GPIO_Motor_AIN2_IOMUX                                    (IOMUX_PINCM34)
/* Defines for BIN1: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_Motor_BIN1_PORT                                             (GPIOB)
#define GPIO_Motor_BIN1_PIN                                      (DL_GPIO_PIN_9)
#define GPIO_Motor_BIN1_IOMUX                                    (IOMUX_PINCM26)
/* Defines for BIN2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_Motor_BIN2_PORT                                             (GPIOA)
#define GPIO_Motor_BIN2_PIN                                     (DL_GPIO_PIN_13)
#define GPIO_Motor_BIN2_IOMUX                                    (IOMUX_PINCM35)
/* Defines for E1A: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_Encoder_E1A_PORT                                            (GPIOB)
// groups represented: ["GPIO_IMU660RB","GPIO_Encoder"]
// pins affected: ["PIN_IMU660RB_INT1","E1A"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_Encoder_E1A_IIDX                               (DL_GPIO_IIDX_DIO15)
#define GPIO_Encoder_E1A_PIN                                    (DL_GPIO_PIN_15)
#define GPIO_Encoder_E1A_IOMUX                                   (IOMUX_PINCM32)
/* Defines for E1B: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_Encoder_E1B_PORT                                            (GPIOB)
#define GPIO_Encoder_E1B_PIN                                    (DL_GPIO_PIN_16)
#define GPIO_Encoder_E1B_IOMUX                                   (IOMUX_PINCM33)
/* Defines for E2A: GPIOA.14 with pinCMx 36 on package pin 7 */
#define GPIO_Encoder_E2A_PORT                                            (GPIOA)
// pins affected by this interrupt request:["E2A"]
#define GPIO_Encoder_GPIOA_INT_IRQN                             (GPIOA_INT_IRQn)
#define GPIO_Encoder_GPIOA_INT_IIDX             (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_Encoder_E2A_IIDX                               (DL_GPIO_IIDX_DIO14)
#define GPIO_Encoder_E2A_PIN                                    (DL_GPIO_PIN_14)
#define GPIO_Encoder_E2A_IOMUX                                   (IOMUX_PINCM36)
/* Defines for E2B: GPIOA.17 with pinCMx 39 on package pin 10 */
#define GPIO_Encoder_E2B_PORT                                            (GPIOA)
#define GPIO_Encoder_E2B_PIN                                    (DL_GPIO_PIN_17)
#define GPIO_Encoder_E2B_IOMUX                                   (IOMUX_PINCM39)
/* Port definition for Pin Group GPIO_IMU660RB */
#define GPIO_IMU660RB_PORT                                               (GPIOB)

/* Defines for PIN_IMU660RB_INT1: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_IMU660RB_PIN_IMU660RB_INT1_IIDX                (DL_GPIO_IIDX_DIO13)
#define GPIO_IMU660RB_PIN_IMU660RB_INT1_PIN                     (DL_GPIO_PIN_13)
#define GPIO_IMU660RB_PIN_IMU660RB_INT1_IOMUX                    (IOMUX_PINCM30)
/* Defines for PIN_IMU660RB_CS: GPIOB.14 with pinCMx 31 on package pin 2 */
#define GPIO_IMU660RB_PIN_IMU660RB_CS_PIN                       (DL_GPIO_PIN_14)
#define GPIO_IMU660RB_PIN_IMU660RB_CS_IOMUX                      (IOMUX_PINCM31)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_Motor_init(void);
void SYSCFG_DL_PIT_FOR_CONTROL_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_SPI_IMU660RB_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
