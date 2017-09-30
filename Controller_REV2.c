/*
                            DRONENET: THE QUAD CHRONICLES
                             UNIVERSITY OF CENTRAL FLORIDA
                                PLATFORM CONTROL TERMINAL
                    CODED IN MPLABX -- LINUX-64 -- PIC32MX320F128H
                                BRANDON FRAZER
                    REV6: Updated with RX Parser from Brohemoth
                    REV5: PORTED FROM ARDUINO/CHIPKIT --> MPLABX

                            Platform Controller Terminal
                            ::  Turret Controls	::
                            ::  Motor Controls	::

                                     ,-.
                                    / \  `.  __..-,O
				   :   \ --''_..-'.'
				   |    . .-' `. '.
				   :     .     .`.'
                                    \     `.  /  ..
                                     \      `.   ' .
                                      `,       `.   \
                                     ,|,`.        `-.\
                                    '.||  ``-...__..-`
                                     |  |
                                     |__|
                                     /||\
                                    //||\\
				   // || \\
				__//__||__\\__
			   '--------------' SSt

             _____  ______  _____  __   _ ______  _____ _______
               |   |_____/ |     | | \  | |_____]   |      |
             __|__ |    \_ |_____| |  \_| |_____] __|__    |

*/


#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <math.h>
#include <plib.h>

//--------------- PIC32 Config. Stuffies ------------//
// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = ON            // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)


#define GetSystemClock()       (80000000ul) //80MHz System Clock
#define GetPeripheralClock()   (40000000ul) //40MHz Periphal Clock (otherwise have to double baud)
#define TOGGLE_RATE             2
#define CORE_TICK_RATE (GetSystemClock() / 2 / TOGGLE_RATE) // 1mS


//-------------Define UART Baud Rates------------//
/* Dont forget to define the PERIPHAL Clock above Otherwise
    your Baudrate will be Half of what it should be */
#define UART1_BAUD    115200
#define UART2_BAUD    57600
#define BUFF_SIZE      1024

//------------ 4DLCD Genie Command/Reply ------------//
#define	LCD_READ_OBJ        0X00
#define LCD_WRITE           0x01
#define LCD_WRITE_ASCII     0x02
#define	LCD_WRITE_STRU      0x03
#define	LCD_CONTRAST        0x04
#define	LCD_REPORT_OBJ      0x05
#define	LCD_REPORT_EVENT    0x07

//---------------4DLCD Genie Object Codes------------//
#define LCD_DIPSWITCH       0x00
#define LCD_KNOB            0x01
#define LCD_ROCKER          0x02
#define LCD_SLIDER          0x04
#define LCD_TRACKBAR        0x05
#define LCD_WINBUTTON       0x06
#define LCD_ANGULAR_METER   0x07
#define LCD_COOL_GAUGE      0x08
#define LCD_CUSTOM_DIGITS   0x09
#define LCD_FORM            0x0A
#define LCD_GAUGE           0x0B
#define LCD_IMAGE           0x0C
#define LCD_KEYBOARD        0x0D
#define LCD_LED             0x0E
#define LCD_LED_DIGITS      0x0F
#define LCD_METER           0x10
#define LCD_STRINGS         0x11
#define LCD_THERMOMETER     0x12
#define LCD_USER_LED        0x13
#define LCD_VIDEO           0x14
#define LCD_STATIC_TEXT     0x15
#define LCD_SOUND           0x16
#define LCD_TIMER           0x17
#define	LCD_SPECTRUM        0x18
#define	LCD_SCOPE           0x19
#define	LCD_TANK            0x1A
#define	LCD_USERIMAGES      0x1B
#define	LCD_PINOUTPUT       0x1C
#define	LCD_PININPUT        0x1D
#define	LCD_4DBUTTON        0x1E
#define	LCD_ANIBUTTON       0x1F
#define	LCD_COLORPICKER     0x20
#define	LCD_USERBUTTON      0x21

#define ON               0x01
#define OFF              0x00
#define TRUE             0x01
#define FALSE            0x00

//------------- 4DLCD OBject Def ------------------//
/*These are my defined objects in the actual
 * LCD GUI I made to make coding easier */
#define FWD_4DB             0x00
#define RVS_4DB             0X01
#define RGT_4DB             0X03
#define LFT_4DB             0X02
#define FWD_LD              0X00
#define RVS_LD              0X06
#define RGT_LD              0X07
#define LFT_LD              0X08
#define M1                  0X00
#define M2                  0X01
#define M3                  0X02
#define M4                  0X03
#define SPD_CTRL            0x04
#define PAN_KNOB            0x01
#define TILT_KNOB           0x02
#define DIR_STR             0x00

//-----------------Inputs------------------------------//
/* Real-World Inputs from the custom  made controller */
#define JSB    _RE7     //Joystick Back (reverse)
#define JSL    _RE4     //Joystick Forward
#define JSR    _RE5     //Joystick Right
#define JSF    _RE6     //Joystick Left
#define BBT    _RE3     //White Button
#define GBT    _RG6     //Green Button
#define WBT    _RG8     //Blue Button
#define RBT    _RB4     //Red Button

//------------------Outputs----------------------------//
#define LD1     _RF0     //LED1
#define LD2     _RF1     //LED2
#define LD3     _RE0     //LED3
#define LD4     _RE1     //LED4
#define MXR_S1  _RB12   //RX MUX S1
#define MXR_S0  _RB13   //RX MUX S0
#define MXT_S1  _RB14   //TX MUX S1
#define MXT_S0  _RB15   //TX MUX S0
#define LDB     _RE2    //White Button LED
#define LDG     _RG7    //Green Button LED
#define LDW     _RB5    //Blue Button LED
#define LDR     _RB3    //Red Button LED

//----------------Analog Stuff----------------------------//
#define SPD_POT         2       // Speed Control Slide pot
#define SPD2_POT        8       //Servo Pan Pot
#define TILT_POT        9       //Servo Tilt Pot
#define SP_POT          10     //Spare Pot
#define LOCKING_SERVO   11      //Locking Arm Servo Control
#define AINPUTS 0xF0FB      // Analog inputs (mask for analog config)

//--------------UART MUX HANDLER-------------------------//
#define XBEE        0x0101
#define FPS         0x0000
#define KEYPAD      0x1010
#define SPARE_UART  0X1111


//---------------Serial Print Messages-----------------//
char INIT[]     = "Hello";
char FWD[]      = "F";
char RVS[]      = "B";
char LFT[]      = "L";
char RGT[]      = "R";
char STRL[]     = "L";
char STRR[]     = "R";
char PVTL[]     = "Q";
char PVTR[]     = "W";
char TLFT[]     = "E";
char TRGT[]     = "P";
char DBR[]      = "C";
char DFL[]      = "V";
char DFR[]      = "Z";
char DBL[]      = "X";
char NEUTRAL[]  = "N";
char ANA_READ[] = "T";
char DEBUG_DN[] = "D0";
char DEBUG_EN[] = "D1";
char TEST[]     = "1234";
char MOTOR_MCU[]  = "M";

char PERIPH_MCU[] = "P";
char GPRO_PAN[]   = "0";
char GPRO_TILT[]  = "1";
char TURRET_PAN[] = "2";
char TURRET_TILT[]    = "3";
char MAGL[]       = "4";
char MAGR[]       = "5";
char NEO_EYE[]    =  "6";
char NEO_RUNWAY[] = "7";
char MP3_SEL[]    = "8";
char TURRET_FIRE[]    = "9";


//---------------LCD Print Variables-----------------//
unsigned char LCD_INDEX     = 0x04;
unsigned char LCD_MSB       = 0x00;
unsigned char LCD_VALUE     = 0x0A;
unsigned char LCD_CHECKSUM  = 0x00;
char FWD_STR[]  = "  Moving Forward";
char RVS_STR[]  = "  Moving Reverse";
char RGT_STR[]  = "   Moving Right";
char LFT_STR[]  = "   Moving Left";
char STRL_STR[] = "Straffing Left";
char STRR_STR[] = "Straffing Right";
char DFL_STR[]  = "Diag Front Left";
char DFR_STR[]  = "Diag Front Right";
char DBL_STR[]  = "Diag Rear Left";
char DBR_STR[]  = "Diag Rear Right";

int MOTORS_ENABLED      = 0;
int SEND_INITIAL_SPEED  = 0;
int ANA_THRESH          = 4;
int ser_print_counter   = 0;
int ser_print_max       = 4;
int lcd_print_counter   = 4;
int lcd_print_max       = 1;
unsigned int A2         = 0;
unsigned int A2_LCD     = 0;
unsigned int A2_OLD     = 0;
unsigned int M1_READ    = 0;
unsigned int average_speed = 0;
unsigned int average_speed_old = 0;
unsigned int M1_OLD     = 0;
unsigned int M1_LCD     = 0;
char Enable_Neutral     = 0;
char A2_XBEE[]          = " ";
char M1_XBEE[]          = " ";
char PAN_XBEE[]          = " ";
char TILT_XBEE[]          = " ";
char LOCKING_ARM_XBEE[]   = " ";
unsigned int LOCKING_ARM_LCD       = 0;
unsigned int  PAN_LCD             = 0;
unsigned int TILT_LCD           = 0;
char rx_1;
//char rx_2;

//char XBEE_RX[BUFF_SIZE];
UINT32 XBEE_RX;
char XBEE_STR[128];
UINT8   rx_buff[BUFF_SIZE];
UINT8    rx_buff1[BUFF_SIZE];
UINT32 rx_compare = "hi\r\n";
unsigned int STRAFFE_ENABLE = OFF;
unsigned int MECANUM_TOGGLE = 0;
unsigned int SPEED_TOGGLE   = 0;
unsigned int SIMPLE_SPEED   = 3;
unsigned int G_STRAFFE      = 2;
unsigned int G_TOGGLE       = 0;
unsigned int SPEED_CTRL_MODE = 3;
unsigned int LR_SPEED         = 1;
int INTERUPT_TOGGLE = OFF;
int clock_counter = 0;
UINT32 motor_number;
char please_work;
unsigned int fake_counter = 0x00;
int RANGE_ARRAY = 0x05;
int PAN  = 0;
int TILT = 0;
int TILT_OLD;
int PAN_OLD;
int LOCKING_ARM = 0;
int LOCKING_ARM_OLD;
int RANGE_COUNTER = 5;
int TAG_FOUND = FALSE;
int VERIFIED = FALSE;
    int device_tag = 0;

    int POT_TOGGLE = 0;
int POT_CTRL_MODE = 0;
int GOPRO_MODE = 1;
int LOCKING_SYS = 2;
/********************************************************/
/*               UART1 Transmit Handler                 */
/********************************************************/
void U1_print(char *buffer)
{
   while(*buffer != (char)0)
   {
      while(!UARTTransmitterIsReady(UART1));
      UARTSendDataByte(UART1, *buffer++);
   }
    while(!UARTTransmissionHasCompleted(UART1));
   UARTSendDataByte(UART1, '\r');
   UARTSendDataByte(UART1, '\n');
}


/********************************************************/
/*               UART1 CHAR Handler                 */
/********************************************************/
char U1_READ(void)
{
    while(!U1STAbits.URXDA);
    return U1RXREG;
}

/********************************************************/
/*               UART1 STR Handler                      */
/********************************************************/

char *U1_STR_READ(char *s, int len)
{
    char *p = s;
    do
    {
        *s = U1_READ();
        if (*s == '\r')
            break;
        s++;
        len--;
    } while(len>1);
    *s = '\0';

    return p;
}

/********************************************************/
/*               UART2 Transmit Handler                 */
/********************************************************/
void U2_PRINT(char *buffer)
{
   while(*buffer != (char)0)
   {
      while(!UARTTransmitterIsReady(UART2));
      UARTSendDataByte(UART2, *buffer++);
      LD1=~LD1;
   }
    while(!UARTTransmissionHasCompleted(UART2));
   UARTSendDataByte(UART2, '\r');
   UARTSendDataByte(UART2, '\n');
    LD1=OFF;
}

/********************************************************/
/*              XBEE UART2 Transmit Handler             */
/********************************************************/
void XBEE_PRINT(char *buffer)
{
  //Same function as U2_PRINT but without /r /n
   while(*buffer != (char)0)
   {
      while(!UARTTransmitterIsReady(UART2));
      UARTSendDataByte(UART2, *buffer++);
   }
    while(!UARTTransmissionHasCompleted(UART2));
}

/********************************************************/
/*               UART2 Receive  Handler                 */
/********************************************************/
char U2_READ(void)
{
    while(!U2STAbits.URXDA);
    return U2RXREG;

}
/********************************************************/
/*               UART2 STR Handler                      */
/********************************************************/
char *U2_STR_READ(char *s, int len)
{
    char *p = s;
    do
    {
        *s = U2_READ();
        if( *s == '\n')
            continue;
        if (*s == '\r') //|| (*s == '\n'))
            break;
        s++;
        len--;
    } while(len>1);
    *s = '\0';
    return p;
}

UINT32 GetMenuChoice(void)
{
    UINT8  menu_item;

    while(!UARTReceivedDataIsAvailable(UART2))
        ;

    menu_item = UARTGetDataByte(UART2);

   menu_item -= '0'; //converts to integer

    return (UINT32)menu_item;

}

// *****************************************************************************
// void UARTTxBuffer(char *buffer, UINT32 size)
// *****************************************************************************
void SendDataBuffer(const char *buffer, UINT32 size)
{
    while(size)
    {
        while(!UARTTransmitterIsReady(UART2))
            ;

        UARTSendDataByte(UART2, *buffer);

        buffer++;
        size--;
    }

    while(!UARTTransmissionHasCompleted(UART2))
        ;
}

// *****************************************************************************
// UINT32 GetDataBuffer(char *buffer, UINT32 max_size)
// *****************************************************************************
UINT32 GetDataBuffer(char *buffer, UINT32 max_size)
{
    UINT32 num_char;

    num_char = 0;

    while(num_char < max_size)
    {
        UINT8 character;

        while(!UARTReceivedDataIsAvailable(UART2))
            ;

        character = UARTGetDataByte(UART2);

        if(character == '\r')
            continue;
        if(character == '\n') //|| character == '\n')
       // if(character == '\n')
            break;

        *buffer = character;

        buffer++;
        num_char++;
    }

    return num_char;
  //  return buffer;
}

/********************************************************/
/*              UART LCD Transmit Handler               */
/********************************************************/
void LCD_PRINT(unsigned char buffer)
{
  while(!UARTTransmitterIsReady(UART1));
  UARTSendDataByte(UART1, buffer);
 while(!UARTTransmissionHasCompleted(UART1));
}



/********************************************************/
/*            UART2 MUX Mapper                          */
/********************************************************/
void UART2_MUX(int mux_state)
{
    /*     MUX SIGNAL MAPPING
     *
     *      S1  S0  U2_INPUT
     *      0   0   Finger Print Scanner
     *      0   1   PIN Keypad
     *      1   0   XBEE
     *      1   1   SPARE
     */

   switch(mux_state)
   {
       // 0. Fingerprint Scanner
       case (FPS):
            MXR_S0 = 0;
            MXR_S1 = 0;
            MXT_S0 = 0;
            MXT_S1 = 0;
            break;

        // 1. Keypad Buttons
       case(KEYPAD):
           MXR_S0 = 1;
           MXR_S1 = 0;
           MXT_S0 = 0;
           MXT_S1 = 1;
           break;

       // 2. XBEE Comms
       case (XBEE):
           _RB12 = 0;
           Delay(200);
           _RB13 = 1;
           Delay(200);
           _RB14 = 0;
           Delay(200);
           _RB15 = 1;
           Delay(200);
           break;

        //Spare
       case(SPARE_UART):
           MXR_S0 = 1;
           MXR_S1 = 1;
           MXT_S0 = 1;
           MXT_S1 = 1;
           break;

   }//end mux state

}

/********************************************************/
/*               Initialize the Digital IO             */
/********************************************************/
void Digital_Setup()
{
//----------Clear the Outputs --------------------------//

  //         B      LDR  | LDB   |   S10  |  S10   |  S00   | S01
   mPORTBClearBits(BIT_3 | BIT_5 | BIT_12 | BIT_13 | BIT_14 | BIT_15);
   //          E   LD3  |  LD4  |  LDB     | LDW
   mPORTEClearBits(BIT_0| BIT_1 | BIT_2| BIT_7);
   //          F    LD1  | LD2   | TX1   | TX2
   mPORTFClearBits(BIT_0 | BIT_1 | BIT_3 | BIT_5);
   //         G     LDG
   mPORTGClearBits(BIT_7);


//-----------Initialize all the Outputs-----------------//

  //         B      LDR  | LDB   |   S10  |  S10   |  S00   | S01
   mPORTBSetPinsDigitalOut(BIT_3 | BIT_5 | BIT_12 | BIT_13 | BIT_14 | BIT_15);
   //          E           LD3  |  LD4  | LDB   | LDW
   mPORTESetPinsDigitalOut(BIT_0| BIT_1 | BIT_2 | BIT_7);
   //          F    LD1  | LD2   | TX1   | TX2
   mPORTFSetPinsDigitalOut(BIT_0 | BIT_1 | BIT_3 | BIT_5);
   //         G     LDG
   mPORTGSetPinsDigitalOut(BIT_7);


//----------Initialize all the Inputs-------------------//

   //           B           RBT
   mPORTBSetPinsDigitalIn(BIT_4);
   //           F           RX1  | RX2
   mPORTFSetPinsDigitalIn(BIT_2 | BIT_3);

  //           E          BBT   | JSF   | JSB   |  JSR  | JSR
   mPORTESetPinsDigitalIn(BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7);

   //          G           GBT  | BBT
   mPORTGSetPinsDigitalIn(BIT_6 |BIT_8);

}

/********************************************************/
/*               UART Config                            */
/********************************************************/
void UART_Setup()
{
  // Configure UART1
   UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
   UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
   UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
   UARTSetDataRate(UART1, GetPeripheralClock(), UART1_BAUD);
   UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

   UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
   UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
   UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
   UARTSetDataRate(UART2, GetPeripheralClock(), UART2_BAUD);
   UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART1 RX Interrupt (LCD Screen)
    INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART1), INT_SUB_PRIORITY_LEVEL_0);

    // Configure UART2 RX Interrupt (XBEE and MUX)
    INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART2), INT_SUB_PRIORITY_LEVEL_0);

    // Enable multi-vector interrupts
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();

}


/********************************************************/
/*                Initialize the ADC                    */
/********************************************************/
// initialize the ADC for single conversion, select input pins
void initADC( int amask)
{
    AD1PCFG = amask;    // select analog input pins
    AD1CON1 = 0x00E0;   // auto convert after end of sampling
    AD1CSSL = 0;        // no scanning required
    AD1CON2 = 0;        // use MUXA, AVss/AVdd used as Vref+/-
    AD1CON3 = 0x1F3F;   // max sample time = 31Tad
    AD1CON1SET = 0x8000;// turn on the ADC
} //initADC

/********************************************************/
/*                  Read the ADC                        */
/********************************************************/
int readADC( int ch)
{
    AD1CHSbits.CH0SA = ch;      // select analog input channel
    AD1CON1bits.SAMP = 1;       // start sampling
    while (!AD1CON1bits.DONE);  // wait to complete conversion
    return ADC1BUF0;            // read the conversion result
} // readADC


/********************************************************/
/*                  Pan and Tilt                        */
/********************************************************/
void Servo_Controls()
{


    TILT = readADC(TILT_POT);
    PAN =  readADC(SP_POT);
    LOCKING_ARM = readADC(LOCKING_SERVO);

    if( (TILT_OLD > (TILT+ANA_THRESH)) || (TILT_OLD < (TILT-ANA_THRESH)) )
    {
        TILT_LCD = TILT >> 2;           //shift tilt to represent it as a 0-255 for PWM
        *itoa(TILT_XBEE, TILT_LCD, 10);
        XBEE_PRINT(PERIPH_MCU);

        //Pan the GOPRO
        if(POT_CTRL_MODE == GOPRO_MODE)
        {
            XBEE_PRINT(GPRO_PAN);
            U2_PRINT(TILT_XBEE);
        }

        //Lower/Raise LEFT Locking Arm
        else if(POT_CTRL_MODE == LOCKING_SYS)
        {
            XBEE_PRINT(MAGL);
            U2_PRINT(TILT_XBEE);
        }
        LCD_OBJ_WRITE(LCD_KNOB ,       0x01, TILT_LCD);
        LCD_OBJ_WRITE(LCD_LED_DIGITS,  0x04,  TILT_LCD); //write to the LCD for debugging
    }

    if( (PAN_OLD > (PAN+ANA_THRESH)) || (PAN_OLD < (PAN-ANA_THRESH)) )
    {
        PAN_LCD = PAN >> 2;           //shift tilt to represent it as a 0-255 for PWM
       *itoa(PAN_XBEE, PAN_LCD, 10);
        XBEE_PRINT(PERIPH_MCU);
        
        //Tilt the GOPRO
        if(POT_CTRL_MODE == GOPRO_MODE)
        {
            XBEE_PRINT(GPRO_TILT);
            U2_PRINT(PAN_XBEE);
        }

        //Lower/Raise the RIGHT Locking arm
        else if(POT_CTRL_MODE == LOCKING_SYS)
        {
            XBEE_PRINT(MAGR);
            U2_PRINT(PAN_XBEE);
        }

        LCD_OBJ_WRITE(LCD_LED_DIGITS, 0x03, PAN_LCD); //write to the LCD for debugging
        LCD_OBJ_WRITE(LCD_KNOB,       0x02, PAN_LCD);
        //LCD_OBJ_WRITE(LCD_SCOPE, 0x00, PAN_LCD);
    }
/*
    if( (LOCKING_ARM_OLD > (LOCKING_ARM+ANA_THRESH)) || (LOCKING_ARM_OLD < (LOCKING_ARM-ANA_THRESH)) )
    {
        LOCKING_ARM_LCD = LOCKING_ARM >> 2;           //shift tilt to represent it as a 0-255 for PWM
       *itoa(LOCKING_ARM_XBEE, LOCKING_ARM_LCD, 10);
        XBEE_PRINT(PERIPH_MCU);
        XBEE_PRINT(MAGL);
        U2_PRINT(LOCKING_ARM_XBEE);
        LCD_OBJ_WRITE(LCD_SLIDER,        0x01, LOCKING_ARM_LCD);
        if(LOCKING_ARM_LCD > 150)
        {
            LCD_OBJ_WRITE(LCD_USER_LED, 0x01, ON);
            LCD_OBJ_WRITE(LCD_USER_LED, 0X02, ON);
        }

        else
        {
            LCD_OBJ_WRITE(LCD_USER_LED, 0x01, OFF);
            LCD_OBJ_WRITE(LCD_USER_LED, 0X02, OFF);
        }

    }*/

    LOCKING_ARM_OLD = LOCKING_ARM;
    PAN_OLD = PAN;
    TILT_OLD = TILT;
}

/********************************************************/
/*               Analog Speed Control                   */
/********************************************************/
void Motor_Controls_Speed(int MOTOR_NUM)
{

    char SPEED_M1[]     = "1";
    char SPEED_M2[]     = "2";
    char SPEED_M3[]     = "3";
    char SPEED_M4[]     = "4";
    char SPEED_ALL[]    = "5";
    char SPEED_LEFT[]   = "6";
    char SPEED_RIGHT[]  = "7";
    char SPEED_FRONT[]  = "8";
    char SPEED_REAR[]   = "9";
    A2 = readADC(SPD_POT);
    M1_READ = readADC(SPD2_POT);

    switch(SPEED_CTRL_MODE)
    {
        //2 Speed Mode.
            case 1:
                if( ((A2_OLD > (A2+ANA_THRESH)) || (A2_OLD < (A2-ANA_THRESH))) && A2 > 0 )
                {
                    A2_LCD = A2>>2;
                   LCD_OBJ_WRITE(LCD_METER, M2, A2_LCD);
                   LCD_OBJ_WRITE(LCD_METER, M4, A2_LCD);
                   *itoa(A2_XBEE, A2_LCD, 10);
                    XBEE_PRINT(MOTOR_MCU);
                    XBEE_PRINT(SPEED_LEFT);
                    U2_PRINT(A2_XBEE);
                }

                if( (M1_OLD > (M1_READ+ANA_THRESH)) || (M1_OLD < (M1_READ-ANA_THRESH)) && (M1_READ > 0) )
                {
                    M1_LCD = M1_READ >> 2;
                   /*  if(M1_LCD > 100)
                    {
                        M1_LCD = 100%M1_LCD;
                    }
                    */
                    Delay(20);
                    LCD_OBJ_WRITE(LCD_METER, M1, M1_LCD);
                    LCD_OBJ_WRITE(LCD_METER, M3, M1_LCD);
                   *itoa(M1_XBEE, M1_LCD, 10);
                   XBEE_PRINT(MOTOR_MCU);
                    XBEE_PRINT(SPEED_RIGHT);
                    U2_PRINT(M1_XBEE);
                }

                //Just using this a toggle to help clear the LCD buffers
                //From continous printing
                if(M1_READ > 5 && A2 > 5)
                {
                    Enable_Neutral = ON;
                }

                //make sure the motors are off
                //Must do this first in order to change direction
                else if( (M1_READ<5) && (A2<5) && Enable_Neutral )
                {
                    Enable_Neutral = OFF;
                    Neutral_Mode();
                }

             break;

        //Straffe like a ganster (control front and rear independately)
            case 2:
                if( ((A2_OLD > (A2+ANA_THRESH)) || (A2_OLD < (A2-ANA_THRESH))) && A2 > 0 )
                    {
                        A2_LCD = A2>>2;
                        LCD_OBJ_WRITE(LCD_METER, M1, A2_LCD);
                        LCD_OBJ_WRITE(LCD_METER, M2, A2_LCD);
                        *itoa(A2_XBEE, A2_LCD, 10);
                        XBEE_PRINT(MOTOR_MCU);
                        XBEE_PRINT(SPEED_FRONT);
                        U2_PRINT(A2_XBEE);
                    }

                    if( (M1_OLD > (M1_READ+ANA_THRESH)) || (M1_OLD < (M1_READ-ANA_THRESH)) && (M1_READ > 0) )
                    {
                        M1_LCD = M1_READ >> 2;
                        Delay(50);

                        LCD_OBJ_WRITE(LCD_METER, M3, M1_LCD);
                        LCD_OBJ_WRITE(LCD_METER, M4, M1_LCD);
                        *itoa(M1_XBEE, M1_LCD, 10);
                        XBEE_PRINT(MOTOR_MCU);
                        XBEE_PRINT(SPEED_REAR);
                         U2_PRINT(M1_XBEE);
                    }

                    //Just using this a toggle to help clear the LCD buffers
                    //From continous printing
                    if(M1_READ > 5 && A2 > 5)
                    {
                        Enable_Neutral = ON;
                    }

                    //make sure the motors are off
                    //Must do this first in order to change direction
                    else if( (M1_READ<5) && (A2<5) && Enable_Neutral )
                    {
                        Enable_Neutral = OFF;
                        Neutral_Mode();
                    }
        break;

    //Simple Speed Controls (one pot for all motor speeds)
        case 3:
            if( ((A2_OLD > (A2+ANA_THRESH)) || (A2_OLD < (A2-ANA_THRESH))) && A2 > 0 )
            {
                A2_LCD = A2>>2;
                LCD_OBJ_WRITE(LCD_METER, SPD_CTRL, A2_LCD);
                LCD_OBJ_WRITE(LCD_SLIDER, 0x00, A2_LCD);
               *itoa(A2_XBEE, A2_LCD, 10);
                XBEE_PRINT(MOTOR_MCU);
                XBEE_PRINT(SPEED_ALL);
                U2_PRINT(A2_XBEE);
            }
        break;

    } //end speed control FSM

    //Update all the values to be used on next comparison
    average_speed_old = average_speed;
    M1_OLD = M1_READ;
    A2_OLD = A2;

}


/********************************************************/
/*              Send Motor Controls                     */
/********************************************************/
void Motor_FSM(char DIR_4DB, char DIR_LD, char DIR_PRINT[], char DIR_STRING[])
{
    while(ser_print_counter != ser_print_max)
    {
        //Send the pot value on state change. Prevents from having to
        //toggle the pot everytime you change direction
        if(ser_print_counter == 0)
        {
            A2 = readADC(SPD_POT);
            M1_READ = A2>>2;
            *itoa(A2_XBEE, M1_READ, 10);
            XBEE_PRINT(MOTOR_MCU);
            XBEE_PRINT("5");
            U2_PRINT(A2_XBEE);
            LCD_WRITE_STRINGS(DIR_STR, DIR_STRING);
            LCD_OBJ_WRITE(LCD_4DBUTTON, DIR_4DB, ON);
            LCD_OBJ_WRITE(LCD_LED,      DIR_LD,  ON);
        }
        
        XBEE_PRINT(MOTOR_MCU);
        U2_PRINT(DIR_PRINT);
        ser_print_counter++;
    }


    MOTORS_ENABLED = ON; //enable printing the speed
}

/********************************************************/
/*              Determine Motor Direction               */
/********************************************************/
void Motor_Controls_Direction()
{
    //Straffe Left
    if(!JSL && STRAFFE_ENABLE)
    {
        Motor_FSM(LFT_4DB, LFT_LD, STRL, STRL_STR);
    }

    //Straffe Right
    else if(!JSR && STRAFFE_ENABLE)
    {
        Motor_FSM(RGT_4DB, RGT_LD, STRR, STRR_STR);
    }

    //Diagonal Front Left
    else if ( !JSF && !JSL )
    {
        LCD_OBJ_WRITE(LCD_4DBUTTON, FWD_4DB, ON);
        LCD_OBJ_WRITE(LCD_LED, FWD_LD, ON);
        Motor_FSM(LFT_4DB, LFT_LD, DFL, DFL_STR);
    }

    //Diagonal Front Right
    else if ( !JSF && !JSR )
    {
        LCD_OBJ_WRITE(LCD_4DBUTTON, FWD_4DB, ON);
        LCD_OBJ_WRITE(LCD_LED, FWD_LD, ON);
        Motor_FSM(RGT_4DB, RGT_LD, DFR, DFR_STR);
    }

    //Diagonal Rear Left
    else if ( !JSB && !JSL )
    {
        LCD_OBJ_WRITE(LCD_4DBUTTON, RVS_4DB, ON);
        LCD_OBJ_WRITE(LCD_LED, RVS_LD, ON);
        Motor_FSM(LFT_4DB, LFT_LD, DBL, DBL_STR);
    }

    //Diagonal Rear Right
    else if ( !JSB && !JSR)
    {
        LCD_OBJ_WRITE(LCD_4DBUTTON, RVS_4DB, ON);
        LCD_OBJ_WRITE(LCD_LED, RVS_LD, ON);
        Motor_FSM(RGT_4DB, RGT_LD, DBR, DBR_STR);
    }

    //Forward
    else if ( !JSF && JSR && JSB && JSL )
    {
        LCD_OBJ_WRITE(LCD_SOUND, 0x02, 0x00);
        LD1 = 1;
        Motor_FSM(FWD_4DB, FWD_LD, FWD, FWD_STR);
    }

    //Backward (reverse)
    else if ( !JSB && JSR && JSL && JSF )
    {
     //  LCD_OBJ_WRITE(LCD_SOUND, 0x01, 0x00);
        LD4 = 1;
        Motor_FSM(RVS_4DB, RVS_LD, RVS, RVS_STR);
    }

    //Right
    else if ( !JSR && JSL && JSB && JSF && !STRAFFE_ENABLE)
    {
      //  LCD_OBJ_WRITE(LCD_SOUND, 0x02, 0x00);
        LD2 = 1;
        Motor_FSM(RGT_4DB, RGT_LD, TRGT, RGT_STR);
    }

    //Left
    else if ( !JSL && JSR && JSB && JSF && !STRAFFE_ENABLE)
    {
       // LCD_OBJ_WRITE(LCD_SOUND, 0x03, 0x00);
        LD3 = 1;
        Motor_FSM(LFT_4DB, LFT_LD, TLFT, LFT_STR);
    }

    //Neutral State
    else if (JSL && JSR && JSB && JSF && SPEED_CTRL_MODE==SIMPLE_SPEED )
    {
        Neutral_Mode();
    }
}//end motor controls

/********************************************************/
/*              Neutral Mode                            */
/********************************************************/
void Neutral_Mode()
{
    if(ser_print_counter != 0) //just using this as a latch so it doesn't keep repeating
    {
        LCD_WRITE_STRINGS(DIR_STR, "    Just Chillin       ");

        //LCD needs to see the command twice,
        //otherwise it latches onto the last state
        while(lcd_print_counter != lcd_print_max)
        {
            if(lcd_print_counter == 0)
            {
                LCD_OBJ_WRITE(LCD_LED, FWD_LD, OFF);
                LCD_OBJ_WRITE(LCD_LED, RVS_LD, OFF);
                LCD_OBJ_WRITE(LCD_LED, RGT_LD, OFF);
                LCD_OBJ_WRITE(LCD_LED, LFT_LD, OFF);
                LCD_OBJ_WRITE(LCD_4DBUTTON, FWD_4DB, OFF);
                LCD_OBJ_WRITE(LCD_4DBUTTON, RVS_4DB, OFF);
                LCD_OBJ_WRITE(LCD_4DBUTTON, RGT_4DB, OFF);
                LCD_OBJ_WRITE(LCD_4DBUTTON, LFT_4DB, OFF);
            }
            XBEE_PRINT(MOTOR_MCU);
            U2_PRINT(NEUTRAL);
            lcd_print_counter++;
        }
     }

    LD2 = 0;
    LD3 = 0;
    LD4 = 0;
    LD1 = 0;

    lcd_print_counter   = 0;
    ser_print_counter   = 0;
    MOTORS_ENABLED      = OFF;
}

/********************************************************/
/*              Driving Mode Button Handler             */
/********************************************************/
void Driving_Mode()
{
/*-------------------Enable 2 Speed Left/Right Controls----------------*/
    //Swap the speed controls to using two pots versus one
    //Each pot controls left or right motors
    switch(POT_TOGGLE)
    {
        //Enable Simple Speed Controls
        case 0:
            if(!GBT)
            {
                LDG = OFF;
                POT_TOGGLE = 1;
                LCD_WRITE_STRINGS(0x06, "GPRO CONTROL");
                POT_CTRL_MODE = GOPRO_MODE;
            }
        break;

        //Wait for next change of button state
        case 1:
            if(GBT)
            {
                POT_TOGGLE = 2;
                //LCD_OBJ_WRITE(LCD_METER, SPD_CTRL, 0x00);
                //LCD_OBJ_WRITE(LCD_SLIDER, 0x00,    0x00);
            }
        break;

        //Enable 2 Speed Mode for left and right
        case 2:
            LDG = ~LDG;
            if(!GBT)
            {
                POT_TOGGLE = 3;
                LCD_WRITE_STRINGS(0x06, "LOCKING SYSTEM");
                POT_CTRL_MODE = LOCKING_SYS;
                LDG = ON;
            }
        break;

        //Wait for next change of button state
        case 3:
            if(GBT)
            {
                POT_TOGGLE = 0;
            }
        break;

    }//end 2 speed mode FSM

/*---------------Enable Straffing like a Boss---------------------*/
    switch(G_TOGGLE)
    {
        //Enable Gangster Straffing
        case 0:
            if(!BBT)
            {
                LDB = OFF;
                G_TOGGLE = 1;
                LCD_WRITE_STRINGS(0x07, "PIVOT MODE");
                SPEED_CTRL_MODE = G_STRAFFE;
            }
        break;

        // Wait for button  to be pressed again
        case 1:
            if(BBT)
            {
                LCD_OBJ_WRITE(LCD_METER, SPD_CTRL, 0x00);
                LCD_OBJ_WRITE(LCD_SLIDER, 0x00,    0x00);
                G_TOGGLE = 2;
            }
        break;

        //Enable 2 Speed Mode for left and right
        case 2:
             if(!BBT)
            {
                LDB = OFF;
                G_TOGGLE = 3;
                LCD_WRITE_STRINGS(0x07, "2 SPEED MODE");
                SPEED_CTRL_MODE = LR_SPEED;
            }
        break;

        // Wait for button  to be pressed again
        case 3:
            if(BBT)
            {
                LCD_OBJ_WRITE(LCD_METER, SPD_CTRL, 0x00);
                LCD_OBJ_WRITE(LCD_SLIDER, 0x00,    0x00);
                G_TOGGLE = 4;
            }
        break;

        ///Switch back to Simple Mode
        case 4:
            LDB = ~LDB;     //Blink the LED while Gangster Straffe
            if(!BBT)
            {
                G_TOGGLE = 5;
                LCD_WRITE_STRINGS(0x07, "SIMPLE MODE");
                SPEED_CTRL_MODE = SIMPLE_SPEED;
                LDB = ON;
             }
        break;

        //Wait for button to be pressed again/Restart FSM
        case 5:
            if(BBT)
            {
                G_TOGGLE = 0;
                LCD_OBJ_WRITE(LCD_METER, M1, OFF);
                LCD_OBJ_WRITE(LCD_METER, M2, OFF);
                LCD_OBJ_WRITE(LCD_METER, M3, OFF);
                LCD_OBJ_WRITE(LCD_METER, M4, OFF);
            }
        break;

    }//end gangster straffe FSM

/*-------------------Enable Straffing ----------------------------*/
    switch(MECANUM_TOGGLE)
    {
        //Enable Straffe Mode
        case 0:
            if(!WBT)
            {
                MECANUM_TOGGLE = 1;
                LCD_WRITE_STRINGS(0x09, "STRAFFE MODE  ");
                STRAFFE_ENABLE = ON;
            }
        break;

        //Wait for button to be pressed
        case 1:
            if(WBT)
            {
                MECANUM_TOGGLE = 2;
            }
        break;

        //Disable Straffe and return to Regular Turning
        case 2:
            LDW = ~LDW; //Blink the LED while Straffe is Enabled

            if(!WBT)
            {
                MECANUM_TOGGLE = 3;
                LCD_WRITE_STRINGS(0x09, "STANDARD TURNS");
                STRAFFE_ENABLE = OFF;
                LDW = ON;
            }
        break;

        //Wait....again. Tired of repeating this comment.
        case 3:
            if(WBT)
            {
                MECANUM_TOGGLE = 0;
            }
        break;

    }//end straffe enable
}

/********************************************************/
/*              LCD Read  Objects                      */
/********************************************************/

void LCD_OBJ_READ()
{
    unsigned int checksum;
    unsigned int LCD_RETURN;

    LCD_PRINT(LCD_READ_OBJ);    checksum = LCD_READ_OBJ;
    LCD_PRINT(LCD_4DBUTTON);    checksum ^= LCD_4DBUTTON;
    LCD_PRINT(FWD_4DB);         checksum ^= FWD_4DB;
    LCD_PRINT(checksum);
    LCD_RETURN = U1_READ();

}


/********************************************************/
/*              LCD Write Objects                      */
/********************************************************/
/* PRE: Send the OBJECT on the LCD screen (LED, BUTTON, etc..
 *      The Index of the object (for LED #1 LCD_INDEX == 0x01
 *      The Value you Want to write to the object (0x01 Turns LED's on, 0x00
 *      turns them off. values from Analog Reads etc are also OK as long as you convert
 *      them from ints to chars.
 *
 * POST: It will simply write the object to the LCD.
 *
 */
void LCD_OBJ_WRITE(unsigned char LCD_OBJ , unsigned char LCD_INDEX, unsigned char LCD_VALUE)
{
    char lsb = 0x00;
    char msb = 0x00;

    //split the value into the MSB and LSB (part of 4D Genie Protocol)
    msb = (LCD_VALUE >> 8) & 0xFF;
    lsb = LCD_VALUE & 0xFF;

    LCD_PRINT(LCD_WRITE);
    LCD_PRINT(LCD_OBJ);
    LCD_PRINT(LCD_INDEX);
    LCD_PRINT(msb);
    LCD_PRINT(lsb);
    LCD_CHECKSUM = LCD_WRITE ^ LCD_OBJ ^ LCD_INDEX ^ msb ^ lsb; //Used for 4D Protocol
    LCD_PRINT(LCD_CHECKSUM);
}

/********************************************************/
/*              LCD Write Strings                       */
/********************************************************/
/* PRE: Pass the String Index (String Number on the screen)
 *      you want to write to.
 *
 * POST: The String should be displayed on the LCD
 *
 */
void LCD_WRITE_STRINGS(unsigned char LCD_INDEX, char *stringy)
{
    char *p;
    int len = strlen(stringy);
    unsigned int checksum;

    LCD_PRINT(LCD_WRITE_ASCII);         checksum = LCD_WRITE_ASCII;
    LCD_PRINT(LCD_INDEX);               checksum ^= LCD_INDEX;
    LCD_PRINT((unsigned char) len);     checksum ^= len; //4D needs to know the string length

    //The Genie Protocol needs to receive the String One Char at a time
    //Use the loop and update the checksum value as you go.
    for (p = stringy; *p ; ++p)
    {
        LCD_PRINT(*p);
        checksum ^= *p;
    }

    LCD_PRINT(checksum);
}

/********************************************************/
/*              LCD Write Contrast                    */
/********************************************************/
/* PRE: Pass the Contrast Value you want to set the screen
 *      to (0-15). 0 is off.
 *
 * POST: Contrast will adjust itself on the screen.
 *
 */
void LCD_WRITE_CONTRAST(unsigned char LCD_VALUE)
{
    LCD_PRINT(LCD_CONTRAST);
    LCD_PRINT(LCD_VALUE);
    LCD_CHECKSUM = LCD_CONTRAST ^ LCD_VALUE;
    LCD_PRINT(LCD_CHECKSUM);

}
/********************************************************/
/*              Simple Delay                            */
/********************************************************/
void Delay(int Delay_Time)
{
    Delay_Time = Delay_Time * 1000;
    while(Delay_Time--); //Kill time. Change this to TMR
}
volatile unsigned long gmscount;;

void delay(unsigned long msdelay)
{
   unsigned long startTime = gmscount;
   while(gmscount - startTime < msdelay);
}


void Counter(int Count_time)
{

   // Count_time = Count_time * 1000;
    clock_counter++;
    if(clock_counter == Count_time)
    {
        //U2_PRINT(ANA_READ);
        clock_counter = 0;
    }
}

/********************************************************/
/*            User Authentication Sequence              */
/********************************************************/
void FPS_SCAN()
{

  UART2_MUX(FPS);          //Disable all wireless comms (MUX)
  LCD_OBJ_WRITE(LCD_FORM, 0x05, ON);
  VERIFIED = FALSE;
  
}//end user authentication

/********************************************************/
/*              LED SETUP ROUTINE                      */
/********************************************************/
void LED_SETUP_ROUTINE()
{
   int i = 0;
   LD1 = 1;
   Delay(700);
   LD2 = 1;
   Delay(700);
   LD3 = 1;
   Delay(700);
   LD4 = 1;
   Delay(700);
   LDW = 1;
   Delay(700);
   LDG = 1;
   Delay(700);
   LDB = 1;
   Delay(700);
   LD1 = 0;
   Delay(700);
   LD2 = 0;
   Delay(700);
   LD3 = 0;
   Delay(700);
   LD4 = 0;
   Delay(700);
   LDW = 0;
   Delay(700);
   LDG = 0;
   Delay(700);
   LDB = 0;
   Delay(700);
   for (i=0; i<3; i++)
   {
       LD1 = ~LD1;
       LD2 = ~LD2;
       LD3 = ~LD3;
       LD4 = ~LD4;
       LDB = ~LDB;
       LDG = ~LDG;
       LDW = ~LDW;
       LDR = ~LDR;
       Delay(1000);
   }

       LD1 = OFF;
       LD2 = OFF;
       LD3 = OFF;
       LD4 = OFF;

}

/********************************************************/
/*                  Main Loop                           */
/********************************************************/
main()
{

/*-----------------Initialize all IO -------------------*/

   SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
   OpenCoreTimer(CORE_TICK_RATE);
   DDPCONbits.JTAGEN = 0;       //DONT FUCKING FORGET TO DISABLE THIS

   initADC( AINPUTS);      // initialize the ADC
   Digital_Setup();        // initialize the DIO
   UART_Setup();           // initialize the UART(s)

   LED_SETUP_ROUTINE();    //Flash the LEDS!
   //UART2_MUX(FPS);
   FPS_SCAN();

   //Disable the DEBUG Mode on the motor controller
   //U2_PRINT(DEBUG_DN);
   //UART2_MUX(XBEE);
   //Delay(1000);


/*-------To Infinity and Beyond! Over and Over------------*/
    while(1)
    {
        Driving_Mode();                 //Determine the speed/drive control mode
        Motor_Controls_Direction();     //Determine the Motor Direction
        Motor_Controls_Speed(5);         //Determine Motor Speed
        Servo_Controls();
        Delay(200); // ms


    }//end infinite while

}//end main

// UART 2 interrupt handler, set at priority level 2
void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void)
{
    char rx_buffer[] = "";
    int rx_2;
    int i;


	// Is this an RX interrupt?
	if(INTGetFlag(INT_SOURCE_UART_RX(UART2)))
	{
            // Clear the RX interrupt Flag
	    INTClearFlag(INT_SOURCE_UART_RX(UART2));

            if(VERIFIED == TRUE)
            {
                switch(device_tag)
                {
                    case 0:
                        device_tag = UARTGetDataByte(UART2);
                        LD1 = ~LD1;
                        break;
            //Motors 1-4 Current
                    case 0x01:
                        rx_2 = UARTGetDataByte(UART2);
                        LCD_OBJ_WRITE(LCD_COOL_GAUGE, 0x00, rx_2);
                        LD2 = ~LD2;
                        device_tag = 0;
                        break;

                    case 0x02:
                        rx_2 = UARTGetDataByte(UART2);
                        LCD_OBJ_WRITE(LCD_COOL_GAUGE, 0x01, rx_2);
                        LD2 = ~LD2;
                        device_tag = 0;
                        break;

                    case 0x03:
                        rx_2 = UARTGetDataByte(UART2);
                        LCD_OBJ_WRITE(LCD_COOL_GAUGE, 0x02, rx_2);
                        LD2 = ~LD2;
                        device_tag = 0;
                        break;

                    case 0x04:
                        rx_2 = UARTGetDataByte(UART2);
                        LCD_OBJ_WRITE(LCD_COOL_GAUGE, 0x03, rx_2);
                        LD2 = ~LD2;
                        device_tag = 0;
                        break;

            //Motor Faults
                    case 0x05:
                        rx_2 = UARTGetDataByte(UART2);
                        //LCD_OBJ_WRITE(LCD_USER_LED, 0x00, rx_2);
                        LD2 = ~LD2;
                        device_tag = 0;
                        break;

            //Range Finder
                    case 0x06:
                        rx_2 = UARTGetDataByte(UART2);
                        //if(RANGE_ARRAY == 0x09)
                        //    RANGE_ARRAY = 0x05;
                        LCD_OBJ_WRITE(LCD_LED_DIGITS, 0x00, rx_2);
                        //RANGE_ARRAY++;
                        LD2 = ~LD2;
                        device_tag = 0;
                        break;

            //PIR
                    case 0x07:
                        rx_2 = UARTGetDataByte(UART2);
                        LCD_OBJ_WRITE(LCD_USER_LED, 10, rx_2);
                        LD2 = ~LD2;
                        device_tag = 0;
                        break;

            //Bedini Motor Active
                    case 0x08:
                        rx_2 = UARTGetDataByte(UART2);
                        LCD_OBJ_WRITE(LCD_USER_LED, 0x09, rx_2);
                        LD3 = ~LD3;
                        device_tag = 0;
                        break;
                }
            }//end verified if

//***Authenticate User
        else
        {
            rx_2 = UARTGetDataByte(UART2);
              switch(rx_2)
              {
                  case 0x00:
                        LCD_WRITE_STRINGS(0x03, "Welcome Brandon");
                        LCD_OBJ_WRITE(LCD_SOUND, 0x00, 0x02); //because sounds are always fun
                        Delay(9000);
                        LCD_OBJ_WRITE(LCD_FORM, 0x00, ON);
                        VERIFIED = TRUE;
                        break;

                  case 0x01:
                      LCD_WRITE_STRINGS(0x03, "Welcome Brandon2");
                      LCD_OBJ_WRITE(LCD_SOUND, 0x00, 0x02); //because sounds are always fun
                      Delay(5000);
                      LCD_OBJ_WRITE(LCD_FORM, 0x00, ON);
                      VERIFIED = TRUE;
                      break;

                  case 0x02:
                      LCD_WRITE_STRINGS(0x03, "Welcome Raymond");
                      LCD_OBJ_WRITE(LCD_SOUND, 0x00, 0x02); //because sounds are always fun
                      Delay(5000);
                       LCD_OBJ_WRITE(LCD_FORM, 0x00, ON);
                       VERIFIED = TRUE;
                      break;

                  case 0x03:
                      LCD_WRITE_STRINGS(0x03, "Welcome Ryan");
                      LCD_OBJ_WRITE(LCD_SOUND, 0x00, 0x02); //because sounds are always fun
                      Delay(5000);
                        LCD_OBJ_WRITE(LCD_FORM, 0x00, ON);
                        VERIFIED = TRUE;
                      break;

                  case 0x04:
                      LCD_WRITE_STRINGS(0x03, "Welcome Joseph");
                           LCD_OBJ_WRITE(LCD_SOUND, 0x00, 0x02); //because sounds are always fun
                      Delay(5000);
                        LCD_OBJ_WRITE(LCD_FORM, 0x00, ON);
                        VERIFIED = TRUE;
                      break;

                  case 0x05:
                      LCD_WRITE_STRINGS(0x03, "Welcome Dr. Richie");
                      LCD_OBJ_WRITE(LCD_SOUND, 0x00, 0x02); //because sounds are always fun
                      Delay(5000);
                        LCD_OBJ_WRITE(LCD_FORM, 0x00, ON);
                        VERIFIED = TRUE;
                      break;

                  case 0x06:
                      LCD_WRITE_STRINGS(0x03, "Unknown User");
                      LCD_OBJ_WRITE(LCD_SOUND, 0x00, 0x02); //because sounds are always fun
                      Delay(5000);
                        LCD_OBJ_WRITE(LCD_FORM, 0x00, ON);
                        VERIFIED = TRUE;
                      break;

                  case 0x09:
                      LCD_OBJ_WRITE(LCD_FORM, 0x06, ON);
                      LCD_OBJ_WRITE(LCD_SOUND, 0x00, 0x06); //because sounds are always fun
                      Delay(3000);
                        VERIFIED = FALSE;
                      break;

              }// end user identification FSM

              if(VERIFIED == TRUE)
                    UART2_MUX(XBEE); //turn on the comms after verification

            }//end user authentication

           LD4= ~LD4; //Flash the LED to know we are Receiving data through UART2

        }//end flag ISR_RX

	// We don't care about TX interrupt
	if ( INTGetFlag(INT_SOURCE_UART_TX(UART2)) )
	{
            INTClearFlag(INT_SOURCE_UART_TX(UART2));
	}
}


// UART 1 interrupt handler, set at priority level 2
void __ISR(_UART1_VECTOR, ipl2) IntUart1Handler(void)
{

	// Is this an RX interrupt?
	if(INTGetFlag(INT_SOURCE_UART_RX(UART1)))
	{
            // Clear the RX interrupt Flag
	    INTClearFlag(INT_SOURCE_UART_RX(UART1));

                rx_1 = UARTGetDataByte(UART1);
                //PutCharacter(rx_1);
            LD2 = ~LD2;
	}

	// We don't care about TX interrupt
	if ( INTGetFlag(INT_SOURCE_UART_TX(UART1)) )
	{
            INTClearFlag(INT_SOURCE_UART_TX(UART1));
	}
}


 void PutCharacter(const char character)
{
        while(!UARTTransmitterIsReady(UART2))
            ;

        UARTSendDataByte(UART2, character);


        while(!UARTTransmissionHasCompleted(UART2))
            ;
}
