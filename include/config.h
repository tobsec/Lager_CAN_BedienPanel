#ifndef CONFIG_H
#define CONFIG_H

#define OUT_CHANNELS      5u

#define RXBUF_LEN         32u // needs to be a power of two!
#define TXBUF_LEN         32u // needs to be a power of two!

// CAN IDs
#define THIS_ID           21u // unique ID of this device
#define BROADCAST_ID      31u
#define LIGHTACTOR_11_ID  11u

// Uncomment to enable interrupt-based CAN reception
#define CAN_USE_INTERRUPTS  // Comment this out to use polling mode

// #define MCP_CLOCK_8MHZ
#define MCP_CLOCK_16MHZ

#define MCP_CLOCKOUT_ENABLE

#define OUT_VALUE_START  \
{                        \
    255u, /* OUT0 */     \
    255u, /* OUT1 */     \
    255u, /* OUT2 */     \
    255u, /* OUT3 */     \
    255u  /* OUT4 */     \
}

#endif
