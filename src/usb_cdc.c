#include "usb_cdc.h"
#include "stm32f405xx_min.h"

#include <string.h>

/*
 * USB CDC ACM implementation for STM32F405 OTG_FS.
 *
 * Register addresses from RM0090 rev 19, Chapter 34 (USB on-the-go full-speed).
 * Synopsys OTG USB 2.0 HS/FS controller.
 *
 * Endpoint assignment:
 *   EP0       Control (64-byte max packet)
 *   EP1 IN    Interrupt, 8 bytes  — CDC serial-state notification (unused)
 *   EP2 IN    Bulk, 64 bytes      — CDC data TX (MCU → PC)
 *   EP2 OUT   Bulk, 64 bytes      — CDC data RX (PC → MCU, discarded)
 *
 * FIFO layout (OTG_FS has 1.25 KB = 320 words of SRAM):
 *   Global RX FIFO   : words 0x000–0x07F  (128 words = 512 bytes)
 *   EP0 TX FIFO      : words 0x080–0x08F  ( 16 words =  64 bytes)
 *   EP1 TX FIFO      : words 0x090–0x097  (  8 words =  32 bytes)
 *   EP2 TX FIFO      : words 0x098–0x0D7  ( 64 words = 256 bytes)
 *   Total                                  216 words (< 320 max)
 */

/* ---- OTG_FS register base ---- */
#define OTG_BASE            0x50000000UL

/* Global registers */
#define OTG_GOTGCTL         (*((__IO uint32_t *)(OTG_BASE + 0x000U)))
#define OTG_GAHBCFG         (*((__IO uint32_t *)(OTG_BASE + 0x008U)))
#define OTG_GUSBCFG         (*((__IO uint32_t *)(OTG_BASE + 0x00CU)))
#define OTG_GRSTCTL         (*((__IO uint32_t *)(OTG_BASE + 0x010U)))
#define OTG_GINTSTS         (*((__IO uint32_t *)(OTG_BASE + 0x014U)))
#define OTG_GINTMSK         (*((__IO uint32_t *)(OTG_BASE + 0x018U)))
#define OTG_GRXSTSP         (*((__IO uint32_t *)(OTG_BASE + 0x020U)))  /* pop */
#define OTG_GRXFSIZ         (*((__IO uint32_t *)(OTG_BASE + 0x024U)))
#define OTG_DIEPTXF0        (*((__IO uint32_t *)(OTG_BASE + 0x028U)))  /* EP0 TX FIFO */
#define OTG_DIEPTXF(n)      (*((__IO uint32_t *)(OTG_BASE + 0x100U + ((n)-1U)*4U)))
#define OTG_GCCFG           (*((__IO uint32_t *)(OTG_BASE + 0x038U)))
#define OTG_CID             (*((__IO uint32_t *)(OTG_BASE + 0x03CU)))

/* Device registers */
#define OTG_DCFG            (*((__IO uint32_t *)(OTG_BASE + 0x800U)))
#define OTG_DCTL            (*((__IO uint32_t *)(OTG_BASE + 0x804U)))
#define OTG_DSTS            (*((__IO uint32_t *)(OTG_BASE + 0x808U)))
#define OTG_DIEPMSK         (*((__IO uint32_t *)(OTG_BASE + 0x810U)))
#define OTG_DOEPMSK         (*((__IO uint32_t *)(OTG_BASE + 0x814U)))
#define OTG_DAINT           (*((__IO uint32_t *)(OTG_BASE + 0x818U)))
#define OTG_DAINTMSK        (*((__IO uint32_t *)(OTG_BASE + 0x81CU)))
#define OTG_DIEPEMPMSK      (*((__IO uint32_t *)(OTG_BASE + 0x834U)))

/* Device IN endpoint registers: base 0x900, step 0x20 */
#define OTG_DIEPCTL(n)      (*((__IO uint32_t *)(OTG_BASE + 0x900U + (n)*0x20U)))
#define OTG_DIEPINT(n)      (*((__IO uint32_t *)(OTG_BASE + 0x908U + (n)*0x20U)))
#define OTG_DIEPTSIZ(n)     (*((__IO uint32_t *)(OTG_BASE + 0x910U + (n)*0x20U)))
#define OTG_DTXFSTS(n)      (*((__IO uint32_t *)(OTG_BASE + 0x918U + (n)*0x20U)))

/* Device OUT endpoint registers: base 0xB00, step 0x20 */
#define OTG_DOEPCTL(n)      (*((__IO uint32_t *)(OTG_BASE + 0xB00U + (n)*0x20U)))
#define OTG_DOEPINT(n)      (*((__IO uint32_t *)(OTG_BASE + 0xB08U + (n)*0x20U)))
#define OTG_DOEPTSIZ(n)     (*((__IO uint32_t *)(OTG_BASE + 0xB10U + (n)*0x20U)))

/* Power and clock gate */
#define OTG_PCGCCTL         (*((__IO uint32_t *)(OTG_BASE + 0xE00U)))

/* FIFO data registers (one per endpoint) */
#define OTG_DFIFO(n)        (*((__IO uint32_t *)(OTG_BASE + 0x1000U + (n)*0x1000U)))

/* ---- Bit definitions ---- */
#define GAHBCFG_GINT        BIT(0)   /* global interrupt enable */
#define GUSBCFG_FDMOD       BIT(30)  /* force device mode */
#define GUSBCFG_PHYSEL      BIT(6)   /* full-speed phy select (transceiver) */
#define GUSBCFG_TRDT_MASK   (0xFUL << 10)
#define GUSBCFG_TRDT(n)     (((n) & 0xFUL) << 10)
#define GRSTCTL_CSRST       BIT(0)   /* core soft reset */
#define GRSTCTL_AHBIDL      BIT(31)  /* AHB master idle */
#define GCCFG_NOVBUSSENS    BIT(21)  /* bypass VBUS sense  (unused) */
#define GCCFG_VBUSBSEN      BIT(19)  /* VBUS B-session sense enable */
#define GCCFG_PWRDWN        BIT(16)  /* 1 = transceiver active (disables power-down) */
/* GOTGCTL: force B-session valid so enumeration works without checking VBUS pin */
#define GOTGCTL_BVALOVAL    BIT(7)
#define GOTGCTL_BVALOEN     BIT(6)
#define DCFG_DSPD_FS        (3UL << 0) /* device full speed */
#define DCTL_SDIS           BIT(1)   /* soft disconnect */
#define DCTL_GINSTS         BIT(0)
#define GINTSTS_USBRST      BIT(12)
#define GINTSTS_ENUMDNE     BIT(13)
#define GINTSTS_RXFLVL      BIT(4)
#define GINTSTS_IEPINT      BIT(18)
#define GINTSTS_OEPINT      BIT(19)
#define GINTSTS_SOF         BIT(3)
#define DIEPINT_XFRC        BIT(0)   /* transfer complete */
#define DOEPINT_XFRC        BIT(0)
#define DOEPINT_STUP        BIT(3)   /* SETUP received */
#define DOEPCTL_EPENA       BIT(31)
#define DOEPCTL_CNAK        BIT(26)
#define DOEPCTL_SNAK        BIT(27)
#define DOEPCTL_SD0PID      BIT(28)  /* set DATA0 PID */
#define DOEPCTL_USBAEP      BIT(15)  /* USB active endpoint */
#define DIEPCTL_EPENA       BIT(31)
#define DIEPCTL_CNAK        BIT(26)
#define DIEPCTL_SNAK        BIT(27)
#define DIEPCTL_SD0PID      BIT(28)
#define DIEPCTL_USBAEP      BIT(15)
#define DIEPCTL_TXFNUM(n)   (((n) & 0xFUL) << 22)
#define DIEPCTL_EPTYP_BULK  (2UL << 18)
#define DIEPCTL_EPTYP_INTR  (3UL << 18)
#define DIEPCTL_MPSIZ_64    64UL
#define GRXSTSP_EPNUM(r)    ((r) & 0xFUL)
#define GRXSTSP_BCNT(r)     (((r) >> 4) & 0x7FFUL)
#define GRXSTSP_PKTSTS(r)   (((r) >> 17) & 0xFUL)
#define PKTSTS_GNAK         1U
#define PKTSTS_OUT_RX       2U
#define PKTSTS_OUT_DONE     3U
#define PKTSTS_SETUP_DONE   4U
#define PKTSTS_SETUP_RX     6U

/* OTG_FS IRQ number */
#define OTG_FS_IRQn         67U

/* RCC AHB2 for OTG_FS */
#define RCC_AHB2ENR_OTGFSEN BIT(7)
#define RCC_AHB2RSTR_OTGFSRST BIT(7)
/* RCC AHB2 registers (offset from RCC base) */
#define RCC_AHB2ENR         (*((__IO uint32_t *)(RCC_BASE + 0x34U)))
#define RCC_AHB2RSTR        (*((__IO uint32_t *)(RCC_BASE + 0x14U)))

/* ---- USB Descriptors ---- */

/* EP0 max packet size: 64 bytes for full-speed */
#define EP0_MPS     64U
#define EP2_MPS     64U  /* CDC bulk data endpoint max packet */
#define EP1_MPS      8U  /* CDC notification interrupt endpoint max packet */

static const uint8_t k_device_desc[] = {
    18,                 /* bLength */
    0x01,               /* bDescriptorType = DEVICE */
    0x00, 0x02,         /* bcdUSB = USB 2.0 */
    0x02,               /* bDeviceClass = CDC */
    0x00,               /* bDeviceSubClass */
    0x00,               /* bDeviceProtocol */
    EP0_MPS,            /* bMaxPacketSize0 */
    0x83, 0x04,         /* idVendor  = 0x0483 (STMicroelectronics) */
    0x40, 0x57,         /* idProduct = 0x5740 (CDC Virtual COM Port) */
    0x00, 0x01,         /* bcdDevice = 1.00 */
    0x01,               /* iManufacturer */
    0x02,               /* iProduct */
    0x03,               /* iSerialNumber */
    0x01,               /* bNumConfigurations */
};

/*
 * Configuration descriptor (9) +
 * Interface 0 CDC control (9) +
 * CDC Header FD (5) + Call Mgmt FD (5) + ACM FD (4) + Union FD (5) = 28 functional
 * EP1 IN interrupt (7) +
 * Interface 1 CDC data (9) +
 * EP2 IN bulk (7) + EP2 OUT bulk (7) = 67 bytes total
 */
static const uint8_t k_config_desc[] = {
    /* Configuration Descriptor */
    9,
    0x02,       /* CONFIGURATION */
    67, 0,      /* wTotalLength LE */
    2,          /* bNumInterfaces */
    1,          /* bConfigurationValue */
    0,          /* iConfiguration */
    0x80,       /* bmAttributes: bus-powered */
    50,         /* bMaxPower: 100 mA */

    /* Interface 0: CDC Communication */
    9,
    0x04,       /* INTERFACE */
    0,          /* bInterfaceNumber */
    0,          /* bAlternateSetting */
    1,          /* bNumEndpoints */
    0x02,       /* bInterfaceClass: CDC */
    0x02,       /* bInterfaceSubClass: ACM */
    0x00,       /* bInterfaceProtocol */
    0,          /* iInterface */

    /* CDC Header Functional Descriptor */
    5, 0x24, 0x00, 0x10, 0x01,

    /* Call Management Functional Descriptor */
    5, 0x24, 0x01,
    0x00,       /* bmCapabilities: no call management */
    1,          /* bDataInterface */

    /* ACM Functional Descriptor */
    4, 0x24, 0x02,
    0x02,       /* bmCapabilities: SET/GET_LINE_CODING, SET_CONTROL_LINE_STATE */

    /* Union Functional Descriptor */
    5, 0x24, 0x06,
    0,          /* bMasterInterface */
    1,          /* bSlaveInterface0 */

    /* Endpoint 1 IN: Interrupt, 8 bytes, 16 ms interval */
    7,
    0x05,       /* ENDPOINT */
    0x81,       /* EP1 IN */
    0x03,       /* Interrupt */
    EP1_MPS, 0,
    16,         /* bInterval */

    /* Interface 1: CDC Data */
    9,
    0x04,       /* INTERFACE */
    1,          /* bInterfaceNumber */
    0,          /* bAlternateSetting */
    2,          /* bNumEndpoints */
    0x0A,       /* bInterfaceClass: CDC-Data */
    0x00,
    0x00,
    0,

    /* Endpoint 2 IN: Bulk, 64 bytes */
    7,
    0x05,       /* ENDPOINT */
    0x82,       /* EP2 IN */
    0x02,       /* Bulk */
    EP2_MPS, 0,
    0,          /* bInterval */

    /* Endpoint 2 OUT: Bulk, 64 bytes */
    7,
    0x05,       /* ENDPOINT */
    0x02,       /* EP2 OUT */
    0x02,       /* Bulk */
    EP2_MPS, 0,
    0,
};

static const uint8_t k_lang_desc[] = {
    4,
    0x03,       /* STRING */
    0x09, 0x04, /* English (US) */
};

static const uint8_t k_mfr_desc[] = {
    14,
    0x03,
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, 'F', 0,
};

static const uint8_t k_prod_desc[] = {
    42,
    0x03,
    'I', 0, 'M', 0, 'U', 0, ' ', 0, 'S', 0, 'e', 0, 'n', 0, 's', 0,
    'o', 0, 'r', 0, ' ', 0, 'M', 0, 'o', 0, 'd', 0, 'u', 0, 'l', 0,
    'e', 0, ' ', 0, 'v', 0, '1', 0,
};

static const uint8_t k_serial_desc[] = {
    10,
    0x03,
    'S', 0, 'B', 0, 'F', 0, '4', 0,
};

/* ---- TX ring buffer ---- */
#define USB_TX_SIZE 512U
static volatile uint8_t  g_tx_buf[USB_TX_SIZE];
static volatile uint32_t g_tx_head;   /* producer */
static volatile uint32_t g_tx_tail;   /* consumer (ISR) */

/* ---- State ---- */
typedef enum { DEV_DEFAULT, DEV_ADDRESSED, DEV_CONFIGURED } usb_dev_state_t;

static volatile usb_dev_state_t g_dev_state;
static volatile uint8_t  g_dev_addr;        /* pending SET_ADDRESS value */
static volatile bool     g_addr_pending;
static volatile bool     g_ep2_in_busy;     /* EP2 IN transfer in progress */

/* SETUP packet storage (8 bytes, read from FIFO) */
static uint8_t g_setup[8];

/* ---- Helper: send data on EP0 IN ---- */
static const uint8_t *g_ep0_in_data;
static uint16_t       g_ep0_in_remain;
static uint16_t       g_ep0_in_total;

static void ep0_in_start(const uint8_t *data, uint16_t total, uint16_t wLength)
{
    uint16_t send = (total < wLength) ? total : wLength;
    g_ep0_in_data   = data;
    g_ep0_in_remain = send;
    g_ep0_in_total  = send;

    uint16_t pkt = (send > EP0_MPS) ? EP0_MPS : send;
    OTG_DIEPTSIZ(0) = (1UL << 19) | pkt;  /* 1 packet, pkt bytes */
    OTG_DIEPCTL(0)  |= DIEPCTL_EPENA | DIEPCTL_CNAK;

    /* Push first packet into TX FIFO word-by-word */
    for (uint16_t i = 0; i < pkt; ) {
        uint32_t word = 0;
        for (uint8_t b = 0; b < 4 && i < pkt; b++, i++) {
            word |= (uint32_t)data[i] << (b * 8U);
        }
        OTG_DFIFO(0) = word;
    }
    g_ep0_in_data   += pkt;
    g_ep0_in_remain -= pkt;
}

/* ---- SETUP packet handler ---- */
static void handle_setup(void)
{
    const uint8_t  bmRT   = g_setup[0];
    const uint8_t  bReq   = g_setup[1];
    const uint16_t wValue = (uint16_t)(g_setup[2] | ((uint16_t)g_setup[3] << 8));
    const uint16_t wIndex = (uint16_t)(g_setup[4] | ((uint16_t)g_setup[5] << 8));
    const uint16_t wLength= (uint16_t)(g_setup[6] | ((uint16_t)g_setup[7] << 8));

    (void)wIndex;

    /* Standard device/interface requests */
    if (bmRT == 0x80) {
        /* Device-to-host, device recipient */
        if (bReq == 0x06) {
            /* GET_DESCRIPTOR */
            const uint8_t dtype = (uint8_t)(wValue >> 8);
            const uint8_t dindex= (uint8_t)(wValue & 0xFF);
            if (dtype == 0x01 && dindex == 0) {
                ep0_in_start(k_device_desc, sizeof(k_device_desc), wLength);
                return;
            }
            if (dtype == 0x02 && dindex == 0) {
                ep0_in_start(k_config_desc, sizeof(k_config_desc), wLength);
                return;
            }
            if (dtype == 0x03) {
                if (dindex == 0) { ep0_in_start(k_lang_desc,   sizeof(k_lang_desc),   wLength); return; }
                if (dindex == 1) { ep0_in_start(k_mfr_desc,    sizeof(k_mfr_desc),    wLength); return; }
                if (dindex == 2) { ep0_in_start(k_prod_desc,   sizeof(k_prod_desc),   wLength); return; }
                if (dindex == 3) { ep0_in_start(k_serial_desc, sizeof(k_serial_desc), wLength); return; }
            }
        }
        if (bReq == 0x00) {
            /* GET_STATUS: return 0x0000 */
            static const uint8_t status[2] = { 0, 0 };
            ep0_in_start(status, 2, wLength);
            return;
        }
    }

    if (bmRT == 0x00) {
        /* Host-to-device, device recipient */
        if (bReq == 0x05) {
            /* SET_ADDRESS */
            g_dev_addr    = (uint8_t)(wValue & 0x7F);
            g_addr_pending = true;
            /* Send ZLP on EP0 IN, then apply address after XFRC */
            OTG_DIEPTSIZ(0) = (1UL << 19);  /* 1 pkt, 0 bytes */
            OTG_DIEPCTL(0)  |= DIEPCTL_EPENA | DIEPCTL_CNAK;
            return;
        }
        if (bReq == 0x09) {
            /* SET_CONFIGURATION */
            g_dev_state = DEV_CONFIGURED;
            /* Enable EP1 IN (interrupt) and EP2 IN/OUT (bulk) */
            OTG_DIEPCTL(1) = DIEPCTL_USBAEP | DIEPCTL_SD0PID
                           | DIEPCTL_TXFNUM(1)
                           | DIEPCTL_EPTYP_INTR
                           | EP1_MPS;
            OTG_DIEPCTL(2) = DIEPCTL_USBAEP | DIEPCTL_SD0PID
                           | DIEPCTL_TXFNUM(2)
                           | DIEPCTL_EPTYP_BULK
                           | EP2_MPS;
            OTG_DOEPCTL(1) = DOEPCTL_USBAEP | DOEPCTL_SD0PID
                           | DIEPCTL_EPTYP_BULK
                           | EP2_MPS;
            OTG_DOEPTSIZ(1) = (1UL << 19) | EP2_MPS;
            OTG_DOEPCTL(1) |= DOEPCTL_EPENA | DOEPCTL_CNAK;
            /* Unmask EP2 TX-FIFO-empty interrupt for data pump */
            OTG_DIEPEMPMSK |= BIT(2);
            /* ZLP status */
            OTG_DIEPTSIZ(0) = (1UL << 19);
            OTG_DIEPCTL(0)  |= DIEPCTL_EPENA | DIEPCTL_CNAK;
            return;
        }
    }

    if (bmRT == 0x01) {
        /* Host-to-device, interface recipient — CDC class requests */
        if (bReq == 0x09) {
            /* GET_INTERFACE */
        }
    }

    /* CDC class requests (bmRT = 0x21 = class|interface|H-to-D) */
    if (bmRT == 0x21) {
        if (bReq == 0x20 || bReq == 0x22) {
            /* SET_LINE_CODING (0x20) or SET_CONTROL_LINE_STATE (0x22):
               accept and send ZLP — we don't need line parameters since it's
               all digital inside the MCU.                                   */
            if (wLength > 0) {
                /* Drain EP0 OUT data (SET_LINE_CODING sends 7 bytes) */
                OTG_DOEPTSIZ(0) = (1UL << 29) | (1UL << 19) | EP0_MPS;
                OTG_DOEPCTL(0)  |= DOEPCTL_EPENA | DOEPCTL_CNAK;
                return; /* Status ZLP sent after OUT data is received */
            }
            OTG_DIEPTSIZ(0) = (1UL << 19);
            OTG_DIEPCTL(0)  |= DIEPCTL_EPENA | DIEPCTL_CNAK;
            return;
        }
        if (bReq == 0x21) {
            /* GET_LINE_CODING: return 115200-8N1 */
            static const uint8_t linecoding[] = {
                0x00, 0xC2, 0x01, 0x00,  /* dwDTERate = 115200 LE */
                0x00,                    /* bCharFormat = 1 stop bit */
                0x00,                    /* bParityType = none */
                0x08,                    /* bDataBits = 8 */
            };
            ep0_in_start(linecoding, sizeof(linecoding), wLength);
            return;
        }
    }

    /* Stall unhandled requests */
    OTG_DIEPCTL(0) |= BIT(21);  /* STALL */
    OTG_DOEPCTL(0) |= BIT(21);
}

/* ---- EP2 IN: push bytes from TX ring buffer into USB FIFO ---- */
static void ep2_pump(void)
{
    if (g_dev_state != DEV_CONFIGURED) { return; }
    if (g_ep2_in_busy) { return; }

    /* How many bytes are waiting? */
    const uint32_t avail = g_tx_head - g_tx_tail;
    if (avail == 0U) { return; }

    /* Cap at one EP2 max packet (64 bytes) per transfer */
    uint32_t pkt_len = (avail > EP2_MPS) ? EP2_MPS : avail;

    /* Check FIFO space (DTXFSTS gives free 32-bit words) */
    uint32_t fifo_free = OTG_DTXFSTS(2) & 0xFFFFU;
    uint32_t words_needed = (pkt_len + 3U) / 4U;
    if (fifo_free < words_needed) { return; }

    g_ep2_in_busy = true;

    OTG_DIEPTSIZ(2) = (1UL << 19) | pkt_len;
    OTG_DIEPCTL(2)  |= DIEPCTL_EPENA | DIEPCTL_CNAK;

    /* Write data to FIFO word-by-word */
    uint32_t tail = g_tx_tail;
    for (uint32_t i = 0; i < pkt_len; ) {
        uint32_t word = 0;
        for (uint8_t b = 0; b < 4U && i < pkt_len; b++, i++) {
            word |= (uint32_t)g_tx_buf[tail & (USB_TX_SIZE - 1U)] << (b * 8U);
            tail++;
        }
        OTG_DFIFO(2) = word;
    }
    g_tx_tail = tail;
}

/* ---- Core soft reset ---- */
static void otg_core_reset(void)
{
    /* Wait for AHB master to be idle */
    uint32_t to = 1000000U;
    while (((OTG_GRSTCTL & GRSTCTL_AHBIDL) == 0U) && (to-- > 0U)) {}

    OTG_GRSTCTL |= GRSTCTL_CSRST;
    to = 1000000U;
    while (((OTG_GRSTCTL & GRSTCTL_CSRST) != 0U) && (to-- > 0U)) {}

    /* Short settle delay */
    to = 3000U;
    while (to-- > 0U) { __asm volatile("nop"); }
}

/* ---- Initialise OTG_FS peripheral ---- */
void usb_cdc_init(void)
{
    g_dev_state   = DEV_DEFAULT;
    g_dev_addr    = 0;
    g_addr_pending = false;
    g_ep2_in_busy  = false;
    g_ep0_in_data   = NULL;
    g_ep0_in_remain = 0;
    g_tx_head = 0;
    g_tx_tail = 0;

    /* Enable OTG_FS clock */
    RCC->AHB1ENR |= BIT(0);   /* GPIOA is already enabled, but ensure it */
    RCC_AHB2ENR  |= RCC_AHB2ENR_OTGFSEN;
    (void)RCC_AHB2ENR;

    /* PA11 = OTG_FS DM (AF10), PA12 = OTG_FS DP (AF10) */
    /* MODER: alternate function (2) */
    GPIOA->MODER = (GPIOA->MODER
                 & ~((3UL << (11U * 2U)) | (3UL << (12U * 2U))))
                 | ((2UL << (11U * 2U)) | (2UL << (12U * 2U)));
    /* AFR[1]: AF10 for PA11 and PA12 */
    GPIOA->AFR[1] = (GPIOA->AFR[1]
                  & ~((0xFUL << ((11U-8U)*4U)) | (0xFUL << ((12U-8U)*4U))))
                  | ((10UL << ((11U-8U)*4U)) | (10UL << ((12U-8U)*4U)));
    /* Speed: very high (3) */
    GPIOA->OSPEEDR |= (3UL << (11U * 2U)) | (3UL << (12U * 2U));

    /* Force device mode before core reset so the core wakes as a device */
    OTG_GUSBCFG |= GUSBCFG_FDMOD | GUSBCFG_PHYSEL;

    /* Force B-session valid so the controller sees VBUS without the pin.
       Required when VBUS sensing (VBUSBSEN) is used with NOVBUSSENS=0.
       Reference: libopencm3 dwc_otg, STM32 HAL USB_DevInit.               */
    OTG_GOTGCTL |= GOTGCTL_BVALOEN | GOTGCTL_BVALOVAL;

    /* Core reset */
    otg_core_reset();

    /* Select FS PHY, force device mode, set turnaround time.
       TRDT=6 for AHB ≥ 30 MHz (RM0090 Table 205, used by libopencm3/HAL). */
    OTG_GUSBCFG = GUSBCFG_FDMOD | GUSBCFG_PHYSEL | GUSBCFG_TRDT(6);

    /* Delay ≥ 25 ms for mode change to take effect (RM0090 §34.17.1) */
    for (volatile uint32_t i = 0; i < 4200000U; i++) { __asm volatile("nop"); }

    /* Power up the transceiver; use VBUSBSEN so VBUS sensing path is active.
       Combined with BVALOEN/BVALOVAL above, this satisfies the controller.  */
    OTG_GCCFG = GCCFG_PWRDWN | GCCFG_VBUSBSEN;

    /* Configure device: full-speed */
    OTG_DCFG = DCFG_DSPD_FS;

    /* FIFO sizes */
    OTG_GRXFSIZ     = 128UL;                    /* 128 words for global RX */
    OTG_DIEPTXF0    = (16UL << 16) | 128UL;    /* EP0 TX: depth=16, start=128 */
    OTG_DIEPTXF(1)  = ( 8UL << 16) | 144UL;    /* EP1 TX: depth=8,  start=144 */
    OTG_DIEPTXF(2)  = (64UL << 16) | 152UL;    /* EP2 TX: depth=64, start=152 */

    /* Flush all TX FIFOs (TXFNUM=0x10 = all, TXFFLSH bit 5) */
    OTG_GRSTCTL = (0x10UL << 6) | BIT(5);
    {
        uint32_t t = 200000U;
        while (((OTG_GRSTCTL & BIT(5)) != 0U) && (t-- > 0U)) {}
    }

    /* Flush RX FIFO (RXFFLSH bit 4) */
    OTG_GRSTCTL = BIT(4);
    {
        uint32_t t = 200000U;
        while (((OTG_GRSTCTL & BIT(4)) != 0U) && (t-- > 0U)) {}
    }

    /* Brief settle after FIFO flush */
    for (volatile uint32_t i = 0; i < 200U; i++) { __asm volatile("nop"); }

    /* Configure EP0 */
    OTG_DOEPCTL(0) = 0;   /* 64-byte EP0 (bits [1:0] = 0b00 = 64B) */
    OTG_DOEPTSIZ(0) = (1UL << 29) | (1UL << 19) | EP0_MPS; /* STUPCNT=1, PKTCNT=1, XFRSIZ */
    OTG_DOEPCTL(0)  |= DOEPCTL_EPENA | DOEPCTL_CNAK;

    /* Clear all pending interrupt flags before unmasking. */
    OTG_GINTSTS = 0xBFFFFFFFUL;

    /* Enable interrupts we care about */
    OTG_GINTMSK = GINTSTS_USBRST   /* USB reset    */
                | GINTSTS_ENUMDNE  /* Enum done    */
                | GINTSTS_RXFLVL   /* RX FIFO NE   */
                | GINTSTS_IEPINT   /* IN ep int    */
                | GINTSTS_OEPINT;  /* OUT ep int   */

    OTG_DIEPMSK  = DIEPINT_XFRC;              /* IN xfr complete */
    OTG_DOEPMSK  = DOEPINT_XFRC | DOEPINT_STUP;
    OTG_DAINTMSK = 0x00070007UL;              /* EP 0/1/2 IN and OUT */
    OTG_DIEPEMPMSK = 0;

    /* Enable global interrupt in OTG controller */
    OTG_GAHBCFG = GAHBCFG_GINT;

    /* Enable OTG_FS IRQ (IRQ 67 = ISER[2] bit 3) */
    nvic_enable_irq(OTG_FS_IRQn);

    /* Soft-connect: clear SDIS to allow USB pull-up on D+ */
    OTG_DCTL &= ~DCTL_SDIS;
}

/* ---- IRQ handler ---- */
void OTG_FS_IRQHandler(void);   /* forward declaration */

void usb_cdc_irq(void)
{
    const uint32_t gintsts = OTG_GINTSTS;

    /* USB Reset */
    if (gintsts & GINTSTS_USBRST) {
        OTG_GINTSTS = GINTSTS_USBRST;
        g_dev_state    = DEV_DEFAULT;
        g_dev_addr     = 0;
        g_addr_pending = false;
        g_ep2_in_busy  = false;
        g_ep0_in_remain = 0;
        OTG_DCFG &= ~(0x7FUL << 4);  /* clear device address */
        OTG_DIEPCTL(1) = 0;
        OTG_DIEPCTL(2) = 0;
        OTG_DOEPCTL(1) = 0;
        OTG_DIEPEMPMSK = 0;
        /* Re-enable EP0 OUT */
        OTG_DOEPTSIZ(0) = (1UL << 29) | (1UL << 19) | EP0_MPS;
        OTG_DOEPCTL(0)  |= DOEPCTL_EPENA | DOEPCTL_CNAK;
        return;
    }

    /* Enumeration done */
    if (gintsts & GINTSTS_ENUMDNE) {
        OTG_GINTSTS = GINTSTS_ENUMDNE;
        /* Speed is always FS here; set TRDT for 48 MHz AHB */
        OTG_GUSBCFG = (OTG_GUSBCFG & ~GUSBCFG_TRDT_MASK) | GUSBCFG_TRDT(6);
        return;
    }

    /* RX FIFO non-empty */
    if (gintsts & GINTSTS_RXFLVL) {
        /* Mask RXFLVL while we process it */
        OTG_GINTMSK &= ~GINTSTS_RXFLVL;

        const uint32_t status = OTG_GRXSTSP;
        const uint8_t  pktsts = (uint8_t)GRXSTSP_PKTSTS(status);
        const uint16_t bcnt   = (uint16_t)GRXSTSP_BCNT(status);
        const uint8_t  epnum  = (uint8_t)GRXSTSP_EPNUM(status);

        if (pktsts == PKTSTS_SETUP_RX && epnum == 0) {
            /* Read 8 bytes (2 words) from FIFO into g_setup */
            uint32_t w0 = OTG_DFIFO(0);
            uint32_t w1 = OTG_DFIFO(0);
            g_setup[0] = (uint8_t)(w0);
            g_setup[1] = (uint8_t)(w0 >> 8);
            g_setup[2] = (uint8_t)(w0 >> 16);
            g_setup[3] = (uint8_t)(w0 >> 24);
            g_setup[4] = (uint8_t)(w1);
            g_setup[5] = (uint8_t)(w1 >> 8);
            g_setup[6] = (uint8_t)(w1 >> 16);
            g_setup[7] = (uint8_t)(w1 >> 24);
        } else if (pktsts == PKTSTS_OUT_RX) {
            /* Drain received OUT data (discard) */
            for (uint16_t i = 0; i < bcnt; i += 4U) {
                (void)OTG_DFIFO(epnum);
            }
        }
        /* Other pktsts values: no action needed */

        OTG_GINTMSK |= GINTSTS_RXFLVL;
        return;
    }

    /* IN endpoint interrupt */
    if (gintsts & GINTSTS_IEPINT) {
        const uint32_t daint = OTG_DAINT;

        if (daint & BIT(0)) {
            /* EP0 IN */
            const uint32_t diepint0 = OTG_DIEPINT(0);
            OTG_DIEPINT(0) = diepint0;
            if (diepint0 & DIEPINT_XFRC) {
                /* Apply SET_ADDRESS after ZLP completes */
                if (g_addr_pending) {
                    OTG_DCFG = (OTG_DCFG & ~(0x7FUL << 4))
                             | ((uint32_t)g_dev_addr << 4);
                    g_dev_addr    = 0;
                    g_addr_pending = false;
                    g_dev_state   = DEV_ADDRESSED;
                }
                /* Continue sending if more data in this transfer */
                if (g_ep0_in_remain > 0U) {
                    const uint16_t pkt = (g_ep0_in_remain > EP0_MPS)
                                       ? EP0_MPS : (uint16_t)g_ep0_in_remain;
                    OTG_DIEPTSIZ(0) = (1UL << 19) | pkt;
                    OTG_DIEPCTL(0)  |= DIEPCTL_EPENA | DIEPCTL_CNAK;
                    for (uint16_t i = 0; i < pkt; ) {
                        uint32_t word = 0;
                        for (uint8_t b = 0; b < 4 && i < pkt; b++, i++) {
                            word |= (uint32_t)g_ep0_in_data[i] << (b * 8U);
                        }
                        OTG_DFIFO(0) = word;
                    }
                    g_ep0_in_data   += pkt;
                    g_ep0_in_remain -= pkt;
                } else {
                    /* Re-arm EP0 OUT for next SETUP */
                    OTG_DOEPTSIZ(0) = (1UL << 29) | (1UL << 19) | EP0_MPS;
                    OTG_DOEPCTL(0)  |= DOEPCTL_EPENA | DOEPCTL_CNAK;
                }
            }
        }

        if (daint & BIT(2)) {
            /* EP2 IN: data transfer complete */
            const uint32_t diepint2 = OTG_DIEPINT(2);
            OTG_DIEPINT(2) = diepint2;
            if (diepint2 & DIEPINT_XFRC) {
                g_ep2_in_busy = false;
                /* Immediately try to pump more data */
                ep2_pump();
            }
        }
    }

    /* OUT endpoint interrupt */
    if (gintsts & GINTSTS_OEPINT) {
        const uint32_t daint = OTG_DAINT;

        if (daint & BIT(16)) {
            /* EP0 OUT */
            const uint32_t doepint0 = OTG_DOEPINT(0);
            OTG_DOEPINT(0) = doepint0;
            if (doepint0 & DOEPINT_STUP) {
                /* SETUP packet received */
                handle_setup();
            }
            if (doepint0 & DOEPINT_XFRC) {
                /* OUT data received (e.g. SET_LINE_CODING body) */
                /* Re-arm OUT EP0 */
                OTG_DOEPTSIZ(0) = (1UL << 29) | (1UL << 19) | EP0_MPS;
                OTG_DOEPCTL(0)  |= DOEPCTL_EPENA | DOEPCTL_CNAK;
                /* Send status ZLP for the class command */
                OTG_DIEPTSIZ(0) = (1UL << 19);
                OTG_DIEPCTL(0)  |= DIEPCTL_EPENA | DIEPCTL_CNAK;
            }
        }
    }
}

void OTG_FS_IRQHandler(void)
{
    usb_cdc_irq();
}

/* ---- Public API ---- */

bool usb_cdc_ready(void)
{
    return g_dev_state == DEV_CONFIGURED;
}

size_t usb_cdc_write(const uint8_t *data, size_t len)
{
    if ((data == NULL) || (len == 0U)) { return 0U; }

    const uint32_t space = USB_TX_SIZE
                         - (uint32_t)(g_tx_head - g_tx_tail);
    const uint32_t copy  = (len < space) ? (uint32_t)len : space;

    for (uint32_t i = 0; i < copy; i++) {
        g_tx_buf[g_tx_head & (USB_TX_SIZE - 1U)] = data[i];
        g_tx_head++;
    }
    return copy;
}

void usb_cdc_putchar(char c)
{
    if (c == '\n') { usb_cdc_putchar('\r'); }
    const uint8_t b = (uint8_t)c;
    usb_cdc_write(&b, 1U);
}

void usb_cdc_poll(void)
{
    if (g_dev_state == DEV_CONFIGURED) {
        ep2_pump();
    }
}
