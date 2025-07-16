#pragma once

#define LSM6DSO_FUNC_CFG_ACCESS_REG 0x01                /* R/W 00000001 00000000 */
#define LSM6DSO_PIN_CTRL_REG 0x02                       /* R/W 00000010 00111111 */
// #define RESERVED - 03-06
#define LSM6DSO_FIFO_CTRL1_REG 0x07                     /* R/W 00000111 00000000 */
#define LSM6DSO_FIFO_CTRL2_REG 0x08                     /* R/W 00001000 00000000 */
#define LSM6DSO_FIFO_CTRL3_REG 0x09                     /* R/W 00001001 00000000 */
#define LSM6DSO_FIFO_CTRL4_REG 0x0A                     /* R/W 00001010 00000000 */
#define LSM6DSO_COUNTER_BDR_REG1_REG 0x0B               /* R/W 00001011 00000000 */
#define LSM6DSO_COUNTER_BDR_REG2_REG 0x0C               /* R/W 00001100 00000000 */
#define LSM6DSO_INT1_CTRL_REG 0x0D                      /* R/W 00001101 00000000 */
#define LSM6DSO_INT2_CTRL_REG 0x0E                      /* R/W 00001110 00000000 */
#define LSM6DSO_WHO_AM_I_REG 0x0F                       /* R 00001111 01101100 R (SPI2) */
#define LSM6DSO_CTRL1_XL_REG 0x10                       /* R/W 00010000 00000000 R (SPI2) */
#define LSM6DSO_CTRL2_G_REG 0x11                        /* R/W 00010001 00000000 R (SPI2) */
#define LSM6DSO_CTRL3_C_REG 0x12                        /* R/W 00010010 00000100 R (SPI2) */
#define LSM6DSO_CTRL4_C_REG 0x13                        /* R/W 00010011 00000000 R (SPI2) */
#define LSM6DSO_CTRL5_C_REG 0x14                        /* R/W 00010100 00000000 R (SPI2) */
#define LSM6DSO_CTRL6_C_REG 0x15                        /* R/W 00010101 00000000 R (SPI2) */
#define LSM6DSO_CTRL7_G_REG 0x16                        /* R/W 00010110 00000000 R (SPI2) */
#define LSM6DSO_CTRL8_XL_REG 0x17                       /* R/W 0001 0111 00000000 R (SPI2) */
#define LSM6DSO_CTRL9_XL_REG 0x18                       /* R/W 00011000 11100000 R (SPI2) */
#define LSM6DSO_CTRL10_C_REG 0x19                       /* R/W 00011001 00000000 R (SPI2) */
#define LSM6DSO_ALL_INT_SRC_REG 0x1A                    /* R 00011010 output */
#define LSM6DSO_WAKE_UP_SRC_REG 0x1B                    /* R 00011011 output */
#define LSM6DSO_TAP_SRC_REG 0x1C                        /* R 00011100 output */
#define LSM6DSO_D6D_SRC_REG 0x1D                        /* R 00011101 output */
#define LSM6DSO_STATUS_REG_STATUS_SPIAux_REG 0x1E       /* R 00011110 output */
#define LSM6DSO_RESERVED_REG 0x1F                       /* - 1F 00011111 */
#define LSM6DSO_OUT_TEMP_L_REG 0x20                     /* R 00100000 output */
#define LSM6DSO_OUT_TEMP_H_REG 0x21                     /* R 00100001 output */
#define LSM6DSO_OUTX_L_G_REG 0x22                       /* R 00100010 output */
#define LSM6DSO_OUTX_H_G_REG 0x23                       /* R 00100011 output */
#define LSM6DSO_OUTY_L_G_REG 0x24                       /* R 00100100 output */
#define LSM6DSO_OUTY_H_G_REG 0x25                       /* R 00100101 output */
#define LSM6DSO_OUTZ_L_G_REG 0x26                       /* R 00100110 output */
#define LSM6DSO_OUTZ_H_G_REG 0x27                       /* R 00100111 output */
#define LSM6DSO_OUTX_L_A_REG 0x28                       /* R 00101000 output */
#define LSM6DSO_OUTX_H_A_REG 0x29                       /* R 00101001 output */
#define LSM6DSO_OUTY_L_A_REG 0x2A                       /* R 00101010 output */
#define LSM6DSO_OUTY_H_A_REG 0x2B                       /* R 00101011 output */
#define LSM6DSO_OUTZ_L_A_REG 0x2C                       /* R 00101100 output */
#define LSM6DSO_OUTZ_H_A_REG 0x2D                       /* R 00101101 output */
// #define RESERVED - 2E-34
#define LSM6DSO_EMB_FUNC_STATUS_MAINPAGE_REG 0x35       /* R 00110101 output */
#define LSM6DSO_FSM_STATUS_A_MAINPAGE_REG 0x36          /* R 00110110 output */
#define LSM6DSO_FSM_STATUS_B_MAINPAGE_REG 0x37          /* R 00110111 output */
// #define RESERVED - 38
#define LSM6DSO_STATUS_MASTER_MAINPAGE_REG 0x39         /* R 00111001 output */
#define LSM6DSO_FIFO_STATUS1_REG 0x3A                   /* R 00111010 output */
#define LSM6DSO_FIFO_STATUS2_REG 0x3B                   /* R 00111011 output */
// #define RESERVED - 3C-3F
#define LSM6DSO_TIMESTAMP0_REG 0x40                     /* R 01000000 output R (SPI2) */
#define LSM6DSO_TIMESTAMP1_REG 0x41                     /* R 01000001 output R (SPI2) */
#define LSM6DSO_TIMESTAMP2_REG 0x42                     /* R 01000010 output R (SPI2) */
#define LSM6DSO_TIMESTAMP3_REG 0x43                     /* R 01000011 output R (SPI2) */
// #define RESERVED - 44-55
#define LSM6DSO_TAP_CFG0_REG 0x56                       /* R/W 01010110 00000000 */
#define LSM6DSO_TAP_CFG1_REG 0x57                       /* R/W 01010111 00000000 */
#define LSM6DSO_TAP_CFG2_REG 0x58                       /* R/W 01011000 00000000 */
#define LSM6DSO_TAP_THS_6D_REG 0x59                     /* R/W 01011001 00000000 */
#define LSM6DSO_INT_DUR2_REG 0x5A                       /* R/W 01011010 00000000 */
#define LSM6DSO_WAKE_UP_THS_REG 0x5B                    /* R/W 01011011 00000000 */
#define LSM6DSO_WAKE_UP_DUR_REG 0x5C                    /* R/W 01011100 00000000 */
#define LSM6DSO_FREE_FALL_REG 0x5D                      /* R/W 01011101 00000000 */
#define LSM6DSO_MD1_CFG_REG 0x5E                        /* R/W 01011110 00000000 */
#define LSM6DSO_MD2_CFG_REG 0x5F                        /* R/W 01011111 00000000 */
// #define RESERVED - 60-61
#define LSM6DSO_I3C_BUS_AVB_REG 0x62                    /* R/W 01100010 00000000 */
#define LSM6DSO_INTERNAL_FREQ_FINE_REG 0x63             /* R 01100011 output */
// #define RESERVED - 64-6E
#define LSM6DSO_INT_OIS_REG 0x6F                        /* R 01101111 00000000 R/W (SPI2) */
#define LSM6DSO_CTRL1_OIS_REG 0x70                      /* R 01110000 00000000 R/W (SPI2) */
#define LSM6DSO_CTRL2_OIS_REG 0x71                      /* R 01110001 00000000 R/W (SPI2) */
#define LSM6DSO_CTRL3_OIS_REG 0x72                      /* R 01110010 00000000 R/W (SPI2) */
#define LSM6DSO_X_OFS_USR_REG 0x73                      /* R/W 01110011 00000000 */
#define LSM6DSO_Y_OFS_USR_REG 0x74                      /* R/W 01110100 00000000 */
#define LSM6DSO_Z_OFS_USR_REG 0x75                      /* R/W 01110101 00000000 */
// #define RESERVED - 76-77
#define LSM6DSO_FIFO_DATA_OUT_TAG_REG 0x78              /* R 01111000 output */
#define LSM6DSO_FIFO_DATA_OUT_X_L_REG 0x79              /* R 01111001 output */
#define LSM6DSO_FIFO_DATA_OUT_X_H_REG 0x7A              /* R 01111010 output */
#define LSM6DSO_FIFO_DATA_OUT_Y_L_REG 0x7B              /* R 01111011 output */
#define LSM6DSO_FIFO_DATA_OUT_Y_H_REG 0x7C              /* R 01111100 output */
#define LSM6DSO_FIFO_DATA_OUT_Z_L_REG 0x7D              /* R 01111101 output */
#define LSM6DSO_FIFO_DATA_OUT_Z_H_REG 0x7E              /* R 01111110 output */
