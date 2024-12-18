/* GameCube --- read a GameCube controller on PIC32         2020-11-12 */
/* Copyright (c) 2020 John Honniball. All rights reserved              */

/*
 * Created: 2020-11-12 23:12
 */


// PIC32MX550F256L Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = OFF           // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_4         // USB PLL Input Divider (4x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (PLL Divide by 4)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define FPBCLK  (40000000)      // PBCLK frequency is 40MHz

#define LED1        LATEbits.LATE6
#define LED2        LATEbits.LATE7
#define LED3        LATEbits.LATE1
#define LED4        LATAbits.LATA7
#define LED5        LATAbits.LATA6

// Size of 128x32 OLED screen
#define MAXX 128
#define MAXY 32
#define MAXROWS 4

// Co-ord of centre of screen
#define CENX (MAXX / 2)
#define CENY (MAXY / 2)

// SSD1306 command bytes
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define UART_RX_BUFFER_SIZE  (128)
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK) != 0
#error UART_RX_BUFFER_SIZE must be a power of two and <= 256
#endif

#define UART_TX_BUFFER_SIZE  (128)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK) != 0
#error UART_TX_BUFFER_SIZE must be a power of two and <= 256
#endif

struct UART_RX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_RX_BUFFER_SIZE];
};

struct UART_TX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_TX_BUFFER_SIZE];
};

struct UART_BUFFER
{
    struct UART_TX_BUFFER tx;
    struct UART_RX_BUFFER rx;
};

// UART buffers
static struct UART_BUFFER U2Buf;

// Controller message buffer
typedef union {
    uint8_t asBytes[8];
    struct {
        unsigned int dpadRight:1;
        unsigned int dpadLeft:1;
        unsigned int dpadDown:1;
        unsigned int dpadUp:1;
        unsigned int butStart:1;
        unsigned int butZ:1;
        unsigned int butB:1;
        unsigned int butA:1;
        unsigned int cpadRight:1;
        unsigned int cpadLeft:1;
        unsigned int cpadDown:1;
        unsigned int cpadUp:1;
        unsigned int butR:1;
        unsigned int butL:1;
        unsigned int xxx:1;
        unsigned int yyy:1;
        int8_t joyx;
        int8_t joyy;
    } asN64;
} ControllerReply;

// The frame buffer, 1024 bytes
unsigned char Frame[MAXROWS][MAXX];

// DDS splash screen
#include "ddslogo.h"

volatile uint32_t MilliSeconds = 0;


volatile int SPINbytes = 0;
volatile uint8_t *SPIBuf = NULL;
volatile int SPIDummyReads = 0;

/* dally --- CPU busy-loop for crude time delay */

static void dally(const int loops)
{
    volatile int dally;
    
    for (dally = 0; dally < loops; dally++)
            ;
}


/* delayms --- busy-wait delay for given number of milliseconds */

static void delayms(const uint32_t interval)
{
    const uint32_t now = MilliSeconds;
    
    while ((MilliSeconds - now) < interval)
        ;
}


/* millis --- Arduino-like function to return milliseconds since start-up */

static uint32_t millis(void)
{
    return (MilliSeconds);
}


void __ISR(_TIMER_1_VECTOR, ipl7AUTO) Timer1Handler(void)
{
    MilliSeconds++;
    
    //LATAINV = _LATA_LATA4_MASK; // Toggle P7 pin 6 (500Hz))
    
    IFS0CLR = _IFS0_T1IF_MASK;  // Clear Timer 1 interrupt flag
}


void __ISR(_SPI_3_VECTOR, ipl4AUTO) SPI3Handler(void)
{
    volatile uint32_t junk;
    
    if (IFS2 & _IFS2_SPI3RXIF_MASK)
    {
        junk = SPI3BUF;
        SPIDummyReads--;
        
        IFS2CLR = _IFS2_SPI3RXIF_MASK;  // Clear SPI3 Rx interrupt flag
        
        if (SPIDummyReads == 0)
        {
            LATASET = _LATA_LATA0_MASK;

            IEC2CLR = _IEC2_SPI3RXIE_MASK;  // Disable SPI3 Rx interrupt
        }
    }
    else if (IFS2 & _IFS2_SPI3TXIF_MASK)
    {
        SPI3BUF = *SPIBuf++;        // Transmit next byte
        SPINbytes--;
        
        IFS2CLR = _IFS2_SPI3TXIF_MASK;  // Clear SPI3 Tx interrupt flag
        
        if (SPINbytes == 0)
        {
            IEC2CLR = _IEC2_SPI3TXIE_MASK;  // Disable SPI3 Tx interrupt
        }
    }
}

void __ISR(_UART_2_VECTOR, ipl1AUTO) UART2Handler(void)
{
    if (IFS1bits.U2TXIF)
    {
        if (U2Buf.tx.head != U2Buf.tx.tail) // Is there anything to send?
        {
            const uint8_t tmptail = (U2Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
            
            U2Buf.tx.tail = tmptail;

            U2TXREG = U2Buf.tx.buf[tmptail];     // Transmit one byte
        }
        else
        {
            IEC1CLR = _IEC1_U2TXIE_MASK;         // Nothing left to send; disable Tx interrupt
        }
        
        IFS1CLR = _IFS1_U2TXIF_MASK;  // Clear UART2 Tx interrupt flag
    }
    
    if (IFS1bits.U2RXIF)
    {
        const uint8_t tmphead = (U2Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
        const uint8_t ch = U2RXREG;   // Read received byte from UART
        
        if (tmphead == U2Buf.rx.tail)   // Is receive buffer full?
        {
             // Buffer is full; discard new byte
        }
        else
        {
            U2Buf.rx.head = tmphead;
            U2Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
        }
        
        IFS1CLR = _IFS1_U2RXIF_MASK;  // Clear UART2 Rx interrupt flag
    }
    
    if (IFS1bits.U2EIF)
    {
        IFS1CLR = _IFS1_U2EIF_MASK;   // Clear UART2 error interrupt flag
    }
}


static void UART1_begin(const int baud)
{
    /* Configure PPS pins */
    RPE5Rbits.RPE5R = 3;    // U1Tx on pin 3, RPE5
    U1RXRbits.U1RXR = 10;   // U1Rx on pin 6, RPC1
    
    /* Configure USART1 */
    U1MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U1STAbits.UTXEN = 1;    // Enable Tx
    U1STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U1BRG = (FPBCLK / (baud * 16)) - 1;
    
    U1MODESET = _U1MODE_ON_MASK;      // Enable USART1
}

static void UART2_begin(const int baud)
{
    U2Buf.tx.head = 0;
    U2Buf.tx.tail = 0;
    U2Buf.rx.head = 0;
    U2Buf.rx.tail = 0;
    
    /* Configure PPS pins */
    RPG0Rbits.RPG0R = 1;    // U2Tx on pin 90, RPG0
    
    /* Configure USART2 */
    U2MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U2STAbits.UTXEN = 1;    // Enable Tx
    U2STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U2BRG = (FPBCLK / (baud * 16)) - 1;
    
    IPC9bits.U2IP = 1;          // UART2 interrupt priority 1 (lowest)
    IPC9bits.U2IS = 2;          // UART2 interrupt sub-priority 2
    
    IFS1CLR = _IFS1_U2TXIF_MASK;  // Clear UART2 Tx interrupt flag
    IFS1CLR = _IFS1_U2RXIF_MASK;  // Clear UART2 Rx interrupt flag
    IFS1CLR = _IFS1_U2EIF_MASK;   // Clear UART2 error interrupt flag
    
    IEC1SET = _IEC1_U2RXIE_MASK;  // Enable UART2 Rx interrupt
    IEC1SET = _IEC1_U2EIE_MASK;   // Enable UART2 error interrupt
    
    U2MODESET = _U2MODE_ON_MASK;      // Enable USART2
}

uint8_t UART2RxByte(void)
{
    const uint8_t tmptail = (U2Buf.rx.tail + 1) & UART_RX_BUFFER_MASK;
    
    while (U2Buf.rx.head == U2Buf.rx.tail)  // Wait, if buffer is empty
        ;
    
    U2Buf.rx.tail = tmptail;
    
    return (U2Buf.rx.buf[tmptail]);
}


void UART2TxByte(const uint8_t data)
{
    const uint8_t tmphead = (U2Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
    
    while (tmphead == U2Buf.tx.tail)   // Wait, if buffer is full
        ;

    U2Buf.tx.buf[tmphead] = data;
    U2Buf.tx.head = tmphead;

    IEC1SET = _IEC1_U2TXIE_MASK;       // Enable UART2 Tx interrupt
}


bool UART2RxAvailable(void)
{
    return (U2Buf.rx.head != U2Buf.rx.tail);
}


static void UART3_begin(const int baud)
{
    /* Configure PPS pins */
    RPF1Rbits.RPF1R = 1;    // U3Tx on pin 88, RPF1
    
    /* Configure USART3 */
    U3MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U3STAbits.UTXEN = 1;    // Enable Tx
    U3STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U3BRG = (FPBCLK / (baud * 16)) - 1;
    
    U3MODESET = _U3MODE_ON_MASK;      // Enable USART3
}

static void UART4_begin(const int baud)
{
    /* Configure PPS pins */
    RPD4Rbits.RPD4R = 2;    // U4Tx on pin 81, RPD4
    
    /* Configure USART4 */
    U4MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U4STAbits.UTXEN = 1;    // Enable Tx
    U4STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U4BRG = (FPBCLK / (baud * 16)) - 1;
    
    U4MODESET = _U4MODE_ON_MASK;      // Enable USART4
}

static void UART5_begin(const int baud)
{
    /* Configure PPS pins */
    RPD12Rbits.RPD12R = 4;  // U5Tx on pin 79, RPD12
    
    /* Configure USART5 */
    U5MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U5STAbits.UTXEN = 1;    // Enable Tx
    U5STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U5BRG = (FPBCLK / (baud * 16)) - 1;
    
    U5MODESET = _U5MODE_ON_MASK;      // Enable USART5
}

void _mon_putc(const char ch)
{
    // See: https://microchipdeveloper.com/faq:81
    if (ch == '\n')
    {
        UART2TxByte('\r');
    }

    UART2TxByte(ch); // Connect stdout to UART2
}


static void SPI3_begin(const int baud)
{    
    /* Configure SPI3 */
    // SCK3 on pin 39, RF13, P1 pin 15
    SDI3Rbits.SDI3R = 0;   // SDI3 on RPD2, pin 77
    RPG8Rbits.RPG8R = 14;  // SDO3 on RPG8, pin 12, P1 pin 28
    
    SPI3BRG = ((FPBCLK / 2) / baud) - 1;
    SPI3CONbits.MSTEN = 1;  // Master mode
    SPI3CONbits.MODE16 = 0; // 8-bit mode
    SPI3CONbits.MODE32 = 0;
    SPI3CONbits.CKE = 1;
    SPI3CONbits.STXISEL = 0; // Interrupt on Tx complete
    SPI3CONbits.SRXISEL = 3; // Interrupt on Rx full
    
    TRISAbits.TRISA0 = 0;   // RA0 pin 17, P1 pin 24 as output for SS
    LATASET = _LATA_LATA0_MASK;   // De-assert SS for SPI3
    
    IPC12bits.SPI3IP = 4;          // SPI3 interrupt priority 4
    IPC12bits.SPI3IS = 1;          // SPI3 interrupt sub-priority 1
    IFS2CLR = _IFS2_SPI3TXIF_MASK;  // Clear SPI3 Tx interrupt flag
    IFS2CLR = _IFS2_SPI3RXIF_MASK;  // Clear SPI3 Rx interrupt flag
    
    SPINbytes = 0;
    SPIDummyReads = 0;
    SPIBuf = NULL;
    
    SPI3CONbits.ON = 1;
}


bool SPIwrite(uint8_t *buf, const int nbytes)
{
    if (SPIDummyReads != 0)     // SPI tranmission still in progress?
    {
        return (false);
    }
    
    if ((nbytes <= 0) || (buf == NULL))
    {
        return (false);
    }
    
    LATACLR = _LATA_LATA0_MASK;   // Assert SS for SPI3
    
    SPIBuf = buf;
    SPINbytes = nbytes;
    SPIDummyReads = nbytes;
    
    SPI3BUF = *SPIBuf++;          // Transmit first byte
    SPINbytes--;
    
    IFS2CLR = _IFS2_SPI3RXIF_MASK;  // Clear SPI3 Rx interrupt flag
    IEC2SET = _IEC2_SPI3RXIE_MASK;  // Enable SPI3 Rx interrupt
    
    if (SPINbytes > 0)
    {
        IFS2CLR = _IFS2_SPI3TXIF_MASK;  // Clear SPI3 Tx interrupt flag
        IEC2SET = _IEC2_SPI3TXIE_MASK;  // Enable SPI3 Tx interrupt
    }
    
    return (true);
}

int SPIbytesPending(void)
{
    return (SPIDummyReads);
}


/* oledData --- send a data byte to the OLED by fast hardware SPI */

inline void oledData(const uint8_t d)
{
    static char data[2];
    
    while (SPIbytesPending() > 0)
        ;
    
    LATGbits.LATG9 = 1;  // DC HIGH
    
    data[0] = d;
    SPIwrite(data, 1);
}


/* oledCmd --- send a command byte to the OLED by fast hardware SPI */

inline void oledCmd(const uint8_t c)
{
    static char cmd[2];
    
    while (SPIbytesPending() > 0)
        ;
    
    LATGbits.LATG9 = 0;  // DC LOW
    
    cmd[0] = c;
    SPIwrite(cmd, 1);
}


/* oledCmd1b --- send two command bytes to the OLED by fast hardware SPI */

inline void oledCmd1b(const uint8_t c, const uint8_t b)
{
    static char cmd[2];
    
    while (SPIbytesPending() > 0)
        ;
    
    LATGbits.LATG9 = 0;  // DC LOW
    
    cmd[0] = c;
    cmd[1] = b;
    SPIwrite(cmd, 2);
}


/* updscreen --- update the physical screen from the buffer */

static void updscreen(void)
{
    static uint8_t addrCmd[] = {SSD1306_COLUMNADDR, 0, MAXX - 1, SSD1306_PAGEADDR, 4, 7};

    while (SPIbytesPending() > 0)
        ;
    
    LATGbits.LATG9 = 0;  // DC LOW
    
    SPIwrite(addrCmd, sizeof (addrCmd));
    
    while (SPIbytesPending() > 0)
        ;
    
    LATGbits.LATG9 = 1;  // DC HIGH
    
    SPIwrite(&Frame[0][0], MAXROWS * MAXX);
    
    while (SPIbytesPending() > 0)   // Wait for SPI transaction using 'Frame' to complete
        ;
}


/* OLED_begin --- initialise the SSD1306 OLED */

void OLED_begin(const int wd, const int ht)
{
    /* Configure I/O pins on PIC32 */
    TRISEbits.TRISE8 = 0;     // RE8, pin 18, P1 pin 22, as output for RES
    TRISGbits.TRISG9 = 0;     // RG9, pin 14, P1 pin 26, as output for DC

    LATEbits.LATE8 = 1;       // RES pin HIGH initially
    LATGbits.LATG9 = 1;       // DC pin HIGH initially

    /* Start configuring the SSD1306 OLED controller */
    delayms(1);
    LATEbits.LATE8 = 0;     // Hardware reset for 10ms
    delayms(10);
    LATEbits.LATE8 = 1;

    // Init sequence for SSD1306 128x64 or 128x32 OLED module
    oledCmd(SSD1306_DISPLAYOFF);                    // 0xAE
    oledCmd1b(SSD1306_SETDISPLAYCLOCKDIV, 0x80);    // 0xD5, the suggested ratio 0x80
    oledCmd1b(SSD1306_SETMULTIPLEX, 0x3F);          // 0xA8
    oledCmd1b(SSD1306_SETDISPLAYOFFSET, 0x00);      // 0xD3, no offset
    oledCmd(SSD1306_SETSTARTLINE | 0x0);            // line #0
    oledCmd1b(SSD1306_CHARGEPUMP, 0x14);            // 0x8D
    oledCmd1b(SSD1306_MEMORYMODE, 0x00);            // 0x20, 0x00 act like ks0108                                 // 
    oledCmd(SSD1306_SEGREMAP | 0x1);
    oledCmd(SSD1306_COMSCANDEC);
    
    if (ht < 64)
    {
        oledCmd1b(SSD1306_SETCOMPINS, 0x02);        // 0xDA
    }
    else
    {
        oledCmd1b(SSD1306_SETCOMPINS, 0x12);        // 0xDA
    }
    
    oledCmd1b(SSD1306_SETCONTRAST, 0xCF);           // 0x81
    oledCmd1b(SSD1306_SETPRECHARGE, 0xF1);          // 0xd9
    oledCmd1b(SSD1306_SETVCOMDETECT, 0x40);         // 0xDB
    oledCmd(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    oledCmd(SSD1306_NORMALDISPLAY);                 // 0xA6

    oledCmd(SSD1306_DISPLAYON); // Turn on OLED panel
}


/* greyFrame --- clear entire frame to checkerboard pattern */

void greyFrame(void)
{
    int r, c;

    for (r = 0; r < MAXROWS; r++)
    {
        for (c = 0; c < MAXX; c += 2)
        {
            Frame[r][c] = 0xaa;
            Frame[r][c + 1] = 0x55;
        }
    }
}


/* setPixel --- set a single pixel */

void setPixel(const unsigned int x, const unsigned int y)
{
    if ((x < MAXX) && (y < MAXY))
        Frame[y / 8][x] |= 1 << (y & 7);
    else
    {
//      Serial.print("setPixel(");
//      Serial.print(x);
//      Serial.print(",");
//      Serial.print(y);
//      Serial.println(")");
    }
}


/* clrPixel --- clear a single pixel */

void clrPixel(const unsigned int x, const unsigned int y)
{
    if ((x < MAXX) && (y < MAXY))
        Frame[y / 8][x] &= ~(1 << (y & 7));
    else
    {
//      Serial.print("clrPixel(");
//      Serial.print(x);
//      Serial.print(",");
//      Serial.print(y);
//      Serial.println(")");
    }
}


/* setVline --- draw vertical line */

void setVline(const unsigned int x, const unsigned int y1, const unsigned int y2)
{
    unsigned int y;

    for (y = y1; y <= y2; y++)
        setPixel(x, y);
}


/* clrVline --- draw vertical line */

void clrVline(const unsigned int x, const unsigned int y1, const unsigned int y2)
{
    unsigned int y;

    for (y = y1; y <= y2; y++)
        clrPixel(x, y);
}


/* setHline --- set pixels in a horizontal line */

void setHline(const unsigned int x1, const unsigned int x2, const unsigned int y)
{
    unsigned int x;
    unsigned int row;
    unsigned char b;

    row = y / 8;
    b = 1 << (y  & 7);

    for (x = x1; x <= x2; x++)
        Frame[row][x] |= b;
}


/* clrHline --- clear pixels in a horizontal line */

void clrHline(const unsigned int x1, const unsigned int x2, const unsigned int y)
{
    unsigned int x;
    unsigned int row;
    unsigned char b;

    row = y / 8;
    b = ~(1 << (y  & 7));

    for (x = x1; x <= x2; x++)
      Frame[row][x] &= b;
}


/* setRect --- set pixels in a (non-filled) rectangle */

void setRect(const int x1, const int y1, const int x2, const int y2)
{
    setHline(x1, x2, y1);
    setVline(x2, y1, y2);
    setHline(x1, x2, y2);
    setVline(x1, y1, y2);
}


/* fillRect --- set pixels in a filled rectangle */

void fillRect(const int x1, const int y1, const int x2, const int y2, const int ec, const int fc)
{
    int y;

    for (y = y1; y <= y2; y++)
        if (fc == 0)
            clrHline(x1, x2, y);
        else if (fc == 1)
            setHline(x1, x2, y);

    if (ec == 1)
    {
        setHline(x1, x2, y1);
        setVline(x2, y1, y2);
        setHline(x1, x2, y2);
        setVline(x1, y1, y2);
    }
    else if (ec == 0)
    {
        clrHline(x1, x2, y1);
        clrVline(x2, y1, y2);
        clrHline(x1, x2, y2);
        clrVline(x1, y1, y2);
    }
}


/* PPS_begin --- map Peripheral Pin Select to suit dev board */

static void PPS_begin(void)
{
    /* Configure USART1 */
    RPE5Rbits.RPE5R = 3;    // U1Tx on pin 3, RPE5
    U1RXRbits.U1RXR = 10;   // U1Rx on pin 6, RPC1
    
    /* Configure USART2 */
    RPG0Rbits.RPG0R = 1;    // U2Tx on pin 90, RPG0
    U2RXRbits.U2RXR = 12;   // U2Rx on pin 89, RPG1 (5V tolerant)
    
    /* Configure USART3 */
    RPF1Rbits.RPF1R = 1;    // U3Tx on pin 88, RPF1
    U3RXRbits.U3RXR = 4;    // U3Rx on pin 87, RPF0
    
    /* Configure USART4 */
    RPD4Rbits.RPD4R = 2;    // U4Tx on pin 81, RPD4
    U4RXRbits.U4RXR = 6;    // U4Rx on pin 82, RPD5 (5V tolerant)
    
    /* Configure USART5 */
    RPD12Rbits.RPD12R = 4;  // U5Tx on pin 79, RPD12
    U5RXRbits.U5RXR = 0;    // U5Rx on pin 76, RPD1
    
    /* Configure OC pins (PWM) */
    RPD8Rbits.RPD8R = 12; // OC1 on pin 68, P7 pin 10 (LED PWM)
    RPD0Rbits.RPD0R = 11; // OC2 on pin 72, P7 pin 14 (tone)
    
    /* Configure SPI2 */
    // SCK2 on pin 10, RG6, P1 pin 32
    SDI2Rbits.SDI2R = 0;   // SDI2 on RPD3, pin 78
    RPC13Rbits.RPC13R = 6; // SDO2 on RPC13, pin 73, P7 pin 16
    
    /* Configure SPI3 */
    // SCK3 on pin 39, RF13, P1 pin 15
    SDI3Rbits.SDI3R = 0;   // SDI3 on RPD2, pin 77
    RPG8Rbits.RPG8R = 14;  // SDO3 on RPG8, pin 12, P1 pin 28
}


/* TRIS_begin --- switch GPIO pins to input or output as required */

static void TRIS_begin(void)
{
    TRISEbits.TRISE6 = 0;   // LED1 pin 4 as output
    TRISEbits.TRISE7 = 0;   // LED2 pin 5 as output
    TRISEbits.TRISE1 = 0;   // LED3 pin 94 as output
    TRISAbits.TRISA7 = 0;   // LED4 pin 92 as output
    TRISAbits.TRISA6 = 0;   // LED5 pin 91 as output
    
    TRISGbits.TRISG15 = 0;  // U1EN pin 1 as output
    TRISEbits.TRISE2 = 0;   // U2EN pin 98 as output
    TRISGbits.TRISG12 = 0;  // U3EN pin 96 as output
    TRISDbits.TRISD13 = 0;  // U4EN pin 80 as output
    TRISDbits.TRISD11 = 0;  // U5EN pin 71 as output
    
    TRISEbits.TRISE3 = 1;   // U1FAULT pin 99 as input
    TRISGbits.TRISG13 = 1;  // U2FAULT pin 97 as input
    TRISGbits.TRISG14 = 1;  // U3FAULT pin 95 as input
    TRISDbits.TRISD3 = 1;   // U4FAULT pin 78 as input
    TRISDbits.TRISD10 = 1;  // U5FAULT pin 70 as input
    TRISFbits.TRISF5 = 1;   // Main current FAULT pin 50 as input
    TRISFbits.TRISF4 = 1;   // Motor current FAULT pin 49 as input
    
    ANSELEbits.ANSE4 = 1;   // U1 current sense pin 100, RE4, AN21, analog
    ANSELEbits.ANSE0 = 1;   // U2 current sense pin 93, RE0, AN46, analog
    ANSELDbits.ANSD7 = 1;   // U3 current sense pin 84, RD7, AN43, analog
    ANSELDbits.ANSD6 = 1;   // U4 current sense pin 83, RD6, AN42, analog
    ANSELDbits.ANSD2 = 1;   // U5 current sense pin 77, RD2, AN25, analog
    ANSELDbits.ANSD14 = 1;  // Main current sense pin 47, RD14, AN36, analog
    ANSELDbits.ANSD15 = 1;  // Motor current sense pin 48, RD15, AN37, analog (blocks SCK4)

    TRISEbits.TRISE4 = 1;   // U1 current sense pin 100, RE4, AN21, input
    TRISEbits.TRISE0 = 1;   // U2 current sense pin 93, RE0, AN46, input
    TRISDbits.TRISD7 = 1;   // U3 current sense pin 84, RD7, AN43, input
    TRISDbits.TRISD6 = 1;   // U4 current sense pin 83, RD6, AN42, input
    TRISDbits.TRISD2 = 1;   // U5 current sense pin 77, RD2, AN25, input
    TRISDbits.TRISD14 = 1;  // Main current sense pin 47, RD14, AN36, input
    TRISDbits.TRISD15 = 1;  // Motor current sense pin 48, RD15, AN37, input (blocks SCK4)
    
    TRISBbits.TRISB15 = 0;  // HV_EN pin 44 as output
    
    // TODO: Add head LEDs and Add-On ports on daughter board
    
    ANSELBbits.ANSB6 = 1;   // Pot on pin 26, P1-37, RB6, AN6 analog
    TRISBbits.TRISB6 = 1;   // Pot on pin 26, P1-37, RB6, AN6 input
    
    TRISAbits.TRISA4 = 0;   // RA4 pin 60, P7 pin 6 as output (command pin)
    TRISAbits.TRISA5 = 1;   // RA5 pin 61, P7 pin 8 as input (reply pin)
    TRISDbits.TRISD0 = 0;   // RD0 pin 72, P7 pin 14 as output (SYNC signal)
}


int ReadController(const uint8_t cmd, const int nBits, uint8_t buf[])
{
    int i;
    uint32_t mask;
    const int expectedBits = nBits + 1;
    int actualBits = 0;
    int oneTime, zeroTime;
    int w, h, l;
    uint8_t pulseh[66];
    uint8_t pulsel[66];
    
    __builtin_disable_interrupts();      // Global interrupt disable
        
    mask = 0x80;

    for (i = 0; i < 8; i++)
    {
        if (cmd & mask)
        {
            LATACLR = _LATA_LATA4_MASK;  // 1us LOW

            dally(2);
            __asm__("NOP");
            __asm__("NOP");
            __asm__("NOP");
            __asm__("NOP");

            LATASET = _LATA_LATA4_MASK;  // 3us HIGH

            dally(8);
        }
        else
        {
            LATACLR = _LATA_LATA4_MASK;  // 3us LOW

            dally(9);

            LATASET = _LATA_LATA4_MASK;  // 1us HIGH

            dally(1);
            __asm__("NOP");
            __asm__("NOP");
            __asm__("NOP");
            __asm__("NOP");
            __asm__("NOP");
            __asm__("NOP");
        }

        mask >>= 1;
    }

    // Send stop bit, 1us LOW
    __asm__("NOP");
    __asm__("NOP");
    __asm__("NOP");
    __asm__("NOP");
    LATACLR = _LATA_LATA4_MASK;  // 1us LOW

    dally(2);

    LATASET = _LATA_LATA4_MASK;

    dally(1);
            
    for (w = 0; w < 256; w++)       // Wait for controller to pull line LOW
    {
        if (PORTAbits.RA5 == 0)
            break;
    }

    for (i = 0; i < expectedBits; i++)
    {
        for (l = 0; l < 64; l++)
            if (PORTAbits.RA5 != 0)
                break;

        for (h = 0; h < 64; h++)
            if (PORTAbits.RA5 == 0)
                break;

        pulseh[i] = h;
        pulsel[i] = l;
    }

    __builtin_enable_interrupts();       // Global interrupt enable

    memset(buf, 0, nBits / 8);

    oneTime = (pulsel[1] + pulseh[1]) / 3;
    zeroTime = oneTime * 2;
    
    if (w < 255)
    {
        for (i = 0; i < expectedBits; i++)
        {
            int bitVal;

            if (pulsel[i] > zeroTime)
            {
                bitVal = 0;
            }
            else if (pulsel[i] < oneTime)
            {
                bitVal = 1;
                buf[i / 8] |= 0x80 >> (i % 8);
            }
            else
            {
                bitVal = 2;
                actualBits = i;
            }

            //printf("%d: %2d %2d %2d %d\n", i, pulsel[i] + pulseh[i], pulsel[i], pulseh[i], bitVal);
        }

        //printf("cmd = %d, w = %d\n", cmd, w);
        
        return (actualBits);
    }
    else
    {
        //printf("cmd = %d, w = %d: No response from controller\n", cmd, w);
        
        return (0);
    }
}


/* initMCU --- set up the microcontroller in general */

void initMCU(void)
{
    /* Configure interrupts */
    INTCONSET = _INTCON_MVEC_MASK; // Multi-vector mode
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

void initMillisecondTimer(void)
{
    /* Configure Timer 1 for 1kHz/1ms interrupts */
    T1CONbits.TCKPS = 0;        // Timer 1 prescale: 1
    
    TMR1 = 0x00;                // Clear Timer 1 counter
    PR1 = (FPBCLK / 1000) - 1;  // Interrupt every millisecond (1kHz)
    
    IPC1bits.T1IP = 7;          // Timer 1 interrupt priority 7 (highest)
    IPC1bits.T1IS = 1;          // Timer 1 interrupt sub-priority 1
    IFS0CLR = _IFS0_T1IF_MASK;  // Clear Timer 1 interrupt flag
    IEC0SET = _IEC0_T1IE_MASK;  // Enable Timer 1 interrupt
    
    T1CONSET = _T1CON_ON_MASK;  // Enable Timer 1
}


void main(void)
{
    ControllerReply buf;
    
    initMCU();
    initMillisecondTimer();
    
    /* Set up peripherals to match pin connections on PCB */
    PPS_begin();
    
    /* Configure tri-state registers */
    TRIS_begin();
    
    LATBbits.LATB15 = 0;  // HV_EN pin 44 LOW (6V regulator OFF)
    
    /* Switch off the MOSFETs */
    LATGbits.LATG15 = 0;  // U1EN pin 1 OFF
    LATEbits.LATE2 = 0;   // U2EN pin 98 OFF
    LATGbits.LATG12 = 0;  // U3EN pin 96 OFF
    LATDbits.LATD13 = 0;  // U4EN pin 80 OFF
    LATDbits.LATD11 = 0;  // U5EN pin 71 OFF
    
    UART1_begin(9600);
    UART2_begin(9600);
    UART3_begin(9600);
    UART4_begin(9600);
    UART5_begin(9600);
    
    SPI3_begin(1000000);
    
    __builtin_enable_interrupts();   // Global interrupt enable
    
    OLED_begin(MAXX, MAXY);
    
    greyFrame();
    
    updscreen();

    delayms(50);
    
    puts("GameCube");
    
    LATASET = _LATA_LATA4_MASK; // Set RA4 HIGH initially
    
    while (1)
    {
        memcpy(Frame, DdsLogo, sizeof (Frame));
        
        if (ReadController(0x01, 32, buf.asBytes) > 0)
        {
            char butStr[17];
            
            butStr[0]  = buf.asN64.butA      ? 'A' : '.';
            butStr[1]  = buf.asN64.butB      ? 'B' : '.';
            butStr[2]  = buf.asN64.butZ      ? 'Z' : '.';
            butStr[3]  = buf.asN64.butL      ? 'L' : '.';
            butStr[4]  = buf.asN64.butR      ? 'R' : '.';
            butStr[5]  = buf.asN64.butStart  ? 'S' : '.';
            butStr[6]  = buf.asN64.dpadLeft  ? 'L' : '.';
            butStr[7]  = buf.asN64.dpadRight ? 'R' : '.';
            butStr[8]  = buf.asN64.dpadUp    ? 'U' : '.';
            butStr[9]  = buf.asN64.dpadDown  ? 'D' : '.';
            butStr[10] = buf.asN64.cpadLeft  ? 'L' : '.';
            butStr[11] = buf.asN64.cpadRight ? 'R' : '.';
            butStr[12] = buf.asN64.cpadUp    ? 'U' : '.';
            butStr[13] = buf.asN64.cpadDown  ? 'D' : '.';
            butStr[14] = '\0';
            
            LED1 = buf.asN64.butA ? 0 : 1;
            LED2 = buf.asN64.butB ? 0 : 1;
            LED3 = buf.asN64.butZ ? 0 : 1;
            LED4 = buf.asN64.butL ? 0 : 1;
            LED5 = buf.asN64.butR ? 0 : 1;
            
            if (buf.asN64.butA)
                fillRect(0, 0, 8, 8, 1, 1);
            
            if (buf.asN64.butB)
                fillRect(8, 0, 16, 8, 1, 1);
            
            if (buf.asN64.butZ)
                fillRect(16, 0, 24, 8, 1, 1);
            
            if (buf.asN64.butL)
                fillRect(24, 0, 32, 8, 1, 1);
            
            if (buf.asN64.butR)
                fillRect(32, 0, 40, 8, 1, 1);
            
            if (buf.asN64.cpadLeft)
                fillRect(40, 0, 48, 8, 1, 1);
            
            if (buf.asN64.cpadRight)
                fillRect(48, 0, 56, 8, 1, 1);
            
            if (buf.asN64.cpadUp)
                fillRect(56, 0, 64, 8, 1, 1);
            
            if (buf.asN64.cpadDown)
                fillRect(64, 0, 72, 8, 1, 1);
            
            setVline(64 + buf.asN64.joyx, 8, 15);
            setVline(64 + buf.asN64.joyy, 16, 23);
            
            printf("%s %d %d\n", butStr, buf.asN64.joyx, buf.asN64.joyy);
        }
        else
        {
            LED1 = 1;
            LED2 = 1;
            LED3 = 1;
            LED4 = 1;
            LED5 = 1;
            
            printf("No reply\n");
        }
        
        updscreen();
        
        delayms(50);
    }
}

