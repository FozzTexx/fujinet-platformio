/* FujiNet Hardware Pin Mapping */
#ifdef PINMAP_A2_REV0

/* SD Card */
#define PIN_CARD_DETECT 12 // fnSystem.h
#define PIN_CARD_DETECT_FIX 15 // fnSystem.h
#define PIN_SD_HOST_CS GPIO_NUM_5 //fnFsSD.cpp
#define PIN_SD_HOST_MISO GPIO_NUM_19
#define PIN_SD_HOST_MOSI GPIO_NUM_23
#define PIN_SD_HOST_SCK GPIO_NUM_18

/* UART */
#define PIN_UART0_RX 3 // fnUART.cpp
#define PIN_UART0_TX 1
#define PIN_UART1_RX 9
#define PIN_UART1_TX 10
#define PIN_UART2_RX 33
#define PIN_UART2_TX 21

/* Buttons */
#define PIN_BUTTON_A 0 // keys.cpp
#define PIN_BUTTON_B -1 // No Button B
#define PIN_BUTTON_C 14

/* LEDs */
#define PIN_LED_WIFI 2 // led.cpp
#define PIN_LED_BUS 12
#define PIN_LED_BT -1 // No BT LED

/* Audio Output */
#define PIN_DAC1 25 // samlib.h

/* IWM Bus Pins */
#define SP_REQ      32
#define SP_PHI0     32
#define SP_PHI1     33
#define SP_PHI2     34
#define SP_PHI3     35
#define SP_WRPROT   27
#define SP_ACK      27
#define SP_RDDATA   4 // tri-state gate enable line
#define SP_WRDATA   22
// TODO: go through each line and make sure the code is OK for each one before moving to next
#define SP_WREQ     26
#define SP_DRIVE1   36
#define SP_DRIVE2   21
#define SP_EN35     39
#define SP_HDSEL    13

#endif /* PINMAP_A2_REV0 */
