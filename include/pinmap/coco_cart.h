/* FujiNet Hardware Pin Mapping */
#ifdef PINMAP_COCO_CART

#define PIN_UART1_RX            GPIO_NUM_13 // RS232 HDSEL
#define PIN_UART1_TX            GPIO_NUM_21 // RS232 DRV2
#define PIN_UART2_RX            GPIO_NUM_13
#define PIN_UART2_TX            GPIO_NUM_21

#define PIN_BUTTON_B            GPIO_NUM_NC // No Button B

#define PIN_LED_BUS             GPIO_NUM_12
#define PIN_LED_BT              GPIO_NUM_NC // No BT LED

#include "common.h"

/* Coco */
#define PIN_CASS_MOTOR          GPIO_NUM_34 // Second motor pin is tied to +3V
#define PIN_CASS_DATA_IN        GPIO_NUM_33
#define PIN_CASS_DATA_OUT       GPIO_NUM_26
#define PIN_CD                  GPIO_NUM_22 // same as atari PROC
#define PIN_EPROM_A14           GPIO_NUM_36 // Used to set the serial baud rate
#define PIN_EPROM_A15           GPIO_NUM_39 // based on the HDB-DOS image selected
#endif /* PINMAP_COCO_DEVKITC */
