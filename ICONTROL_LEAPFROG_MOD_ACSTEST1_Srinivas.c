/**********************************************************************************/
// Was Previously Copyright. iControl Incorporated, 2000
// Now it is whatever Barnhart, Omair and Cassandra want with it.
/**********************************************************************************/

#memmap xmem
#class static
#define processor      'R'							// define processor type R="Rabbit" or Z="Z180"

//================ For SPI Interface =========================================
#define SPI_SER_B
#define SPI_CLK_DIVISOR			5

//============================================================================

#use fft.lib
#use SPI.LIB
#use RCM34XX.LIB

#define version	 	"01.04.06-v1.1"			// Software Version #

#define Run 'R'
#define Stop 'S'
#define Background 'B'
#define I_ACK 0x02
#define I_NAK 0x00

#define AINBUFSIZE	1023         					// size of userPort receive buffer
#define AOUTBUFSIZE	1023         					// size of transmit buffer
#define BINBUFSIZE	1023         					// size of receive buffer
#define BOUTBUFSIZE	1023         					// size of transmit buffer
#define CINBUFSIZE	1023         					// size of receive buffer
#define COUTBUFSIZE	1023         					// size of transmit buffer
#define DINBUFSIZE	1023         					// size of receive buffer
#define DOUTBUFSIZE	1023         					// size of transmit buffer
#undef  outport											// For Rabbit redefine outport
#undef  inport												// For Rabbit redefine inport
#undef  bit
#undef  res
#undef  set
#define  tm_rd	TM_RD										// For Rabbit redefine tm_rd
//============================ LLPV Allocations ================================
#define DS1 6		//port D bit 6
#define DS2 7		//port D bit 7
#define ON  0		//state to turn on led
#define OFF 1		//state to turn off led
#define ACS_Port 3	//Port D
#define Data_Port 2	//Port C
#define Jet_Baud 38400;      //Port D BaudRate

//============================ Memory Allocations ================================
#define NUM_MACROS 40			//== Macro Buffer Size Definitions
#define MAX_MACRO_SIZE 128

#define NUM_STORED_CMDS 40		//== Stored Command sizes (for time tagged commands)
#define MAX_STORED_CMD_SIZE 32

#define NUM_LABELS 12
#define MAX_LABEL_LEN 16
#define NUM_SFACTORS 16
#define MAX_NUM_SFACTORS 2
#define MAX_STORED_DATA 248	// Maximum stored record length (includes multiple sectors)
//==================================================================================

typedef struct {																								// cummulative bytes
 	char RTU_ID[4];				// RTU id																4
	char schedule;					// Schedule checking..enabled=='T'								99
 	unsigned long	baud[4];		// Baud rates for serial ports									126
	int ISRTimer;					// Counter used by MAcro Interrupt Service Routine			134
	int dataPort;					// Serial Port used by I/O device (normally 2 or 3)		138
	char RXmode;					// RXmode=0,1,2														144
	char RetryTime;				// Seconds until TX retry											156
	char discard;					// Number of retries before discard								157
	int TWindow;					// Valid Command time window										158
   char password[10];			// Password for setup menu
	char label[NUM_LABELS][MAX_LABEL_LEN];							// Analog Channel Labels	+128	222
	float sFactors[NUM_SFACTORS][MAX_NUM_SFACTORS];				// Scale factors				+128	350
	char StoredCmds[NUM_STORED_CMDS][MAX_STORED_CMD_SIZE];	// Stored Commands			+1280
	char MacroCmds[NUM_MACROS][MAX_MACRO_SIZE];					// Stored Macro commands	+5120
	unsigned int  TX_SIZE;		//== Command buffer memory allocation
	unsigned int  TX_LEN;
	unsigned int  NUM_RECORDS;	//== Stored Data memory allocation
	unsigned int  RECORD_SIZE;
	char initialized;				// flag to see if flash has ever been initialized	+1
} TFlashProtected;

TFlashProtected * const PTFlashBasePtr = (TFlashProtected*)0;	// pointer to this structure which is accesed at offset 0

TFlashProtected globals;						// structure where all globals are accessed

// global buffer used locally - data cannot be expected to be persistent between uses.
// *** IMPORTANT - If this is changed from 1280, must look at where the Store cmds and
//                 macro cmds are read and written to flash

	char Lbuf[256];

// Protected data
// Data that changes too much to be put into flash
	protected	unsigned char rebootCounter;	// counter incremented with each reboot
	protected	char portCFG[4];				// Stores special configuration for Serial ports
	protected	struct tm thistime;			// Used for scheduling and tasks (protected to prevent corruption)
	protected	char first_time;				// Flag indicating first time main is executed

	int 			userPort;						// User Serial Port(normally 0)
	unsigned 	long ulongChecksum;
	char 			TCW;												// Time Code Word
	struct 		tm time;											// Used for checking timeouts
	char 			display[4], format[4];   					//
 	char 			PC[4]; 											// flag to enable PC direct modes
   char 			menu[4];											// flag to display menu's
	int   		oldsec,oldmin;
	char 			RTU_mode, tlmtext[7];
	char 			stime[25];
	int  			LoadCnt;
	int  			index, nsamples;
	char 			portA, portB, portC, PEshadow;
	char			user_state;
	int  			cursor;
	int			count,Done;
 	char			debugMSG;
 	char			send_Data[4];									// Flag indicating serial ports that are enabled for data transmission
   unsigned 	char PScnt;			 							// Power save variables (255 max)
   unsigned		long pSEC_TIMER;								// Past value of SEC_TIMER
   char			Channel,ADC_CMD, ADC_GAIN,ADC_PTR;
///////////////////////////////////////////////////

const char  ESCAPE=27;						// Define ESC key to break loop
const char 	ENTER =13;						// define <CR> for Rabbit
const char  GainCmd[]={0x00,0x10,0x20,0x30,0x40,0x50,0x60,0x70};

xmem void Init_Device();									//=== Initilization functions
xmem void initProt();
xmem void initComm(char mode, char type, int port);
xmem void initPort(int port, unsigned long baud);
xmem void writeToFlash(void);
xmem void loadDefaults(void);
xmem void M_Allocate(void);
xmem void initADC();

xmem int adjust_rtc(int data, int cursor);			//=== Time related functions
xmem void getTime(char *buf);
xmem void once_per_second();
xmem void MSdelay(int n);
xmem void getEpoch(char *value);

			 												      		//=== Data and I/O functio
xmem void HiResAnalog(char Baddr, char ExtAddr, char channel,char *buf);
xmem void showmenu(int port);										//=== menu and user interface functions
xmem void clrscrn(int port);
xmem void check_User(int port);
xmem void locxy(int port,int row, int col);

xmem char sendMSG(int port, char *value, int numBytes);	//=== Serial Port functions
xmem char getMSG(int port, char *value, char term);
xmem char sendByte(int port, char value);
xmem char getByte(int port, char *value);
xmem void resetPort(int port);
xmem int  setPort(int port);

//========================= string defines and functions ================================//
xmem void placeString(int port, int x, int y, char *str);
xmem void clearLine(int port, int x, int y, int blanks);
xmem void placeAddress(int port, int x,int y,char protocol,char *addr);
xmem char* get_xstring(unsigned long xstring_addr, int index);

xmem  void outport(unsigned int addr, char value);
xmem  char inport(unsigned int addr);
xmem 	int tm_rd(struct tm *time);

//========================= LLPV functions ================================//
void ACS_Test();
///////////////////////////////////////////////////////////
// DS1 led on protoboard is controlled by port G bit 6
// turns on if state = 0
// turns off if state = 1
///////////////////////////////////////////////////////////
void DS1led(int state)
{
	if (state == ON)
	{
#asm
	ld		a,(PDDRShadow)			;use shadow register to keep other bit values
	res	DS1,a						;clear bit 6 only
	ioi	ld (PDDR),a				;write data to port
	ld		(PDDRShadow),a			;update shadow register
#endasm
	}
	else
	{
#asm
	ld		a,(PDDRShadow)			;use shadow register to keep other bit values
	set	DS1,a						;set bit 6 only
	ioi	ld (PDDR),a				;write data to port
	ld		(PDDRShadow),a			;update shadow register
#endasm
	}
}

///////////////////////////////////////////////////////////
// DS2 led on protoboard is controlled by port G bit 7
// turns on if state = 0
// turns off if state = 1
///////////////////////////////////////////////////////////
void DS2led(int state)
{
	if (state == ON)
	{
#asm
	ld		a,(PDDRShadow)			;use shadow register to keep other bit values
	res	DS2,a						;clear bit 7 only
	ioi	ld (PDDR),a				;write data to port
	ld		(PDDRShadow),a			;update shadow register
#endasm
	}
	else
	{
#asm
	ld		a,(PDDRShadow)			;use shadow register to keep other bit values
	set	DS2,a						;set bit 7 only
	ioi	ld (PDDR),a				;write data to port
	ld		(PDDRShadow),a			;update shadow register
#endasm
	}
}


//============================================================================//
xmem main()
{
useMainOsc();					// Start system with Main Oscilator enabled
clockDoublerOn();

if(first_time!='F' || globals.initialized!='T')	// first_time is used to trigger super_reset
	{
	loadDefaults();
	_prot_init();
	initProt();
	}
else _prot_recover();

Init_Device();					// Initialize Device configuration

//== Infinite Loop after everything is initialized
	serBputc('b');
//   serCputc('c');
//   serDputc('d');
  //       	sendByte(Jet_Port,'@');
				//sendByte(Jet_Port,'\n');
//   JetCat_Test();
   ACS_Test();
//   send_ACS();

   while(1)    					// endless loop constantly monitoring for new commands or data
	{
		//check_User(userPort); 											//Check User interface for input
		tm_rd(&thistime);													// === One second task..based in RTC
		if(thistime.tm_sec != oldsec)once_per_second();
   	DS2led(ON);
   	MSdelay(10000);
   	DS2led(OFF);
   	MSdelay(10000);
	}

} //end main

//===========================   End of Main  =====================================


//==========================  Once per second =====================================
void once_per_second()
{
		int n,port;
      char buf[255];

																	 		// Format time field
		thistime.tm_year=thistime.tm_year-100;					// Year ranges from 80-147 for Rabbit
		sprintf(stime,"%02d/%02d/%02d %d %02d:%02d:%02d",
		thistime.tm_mon, thistime.tm_mday, thistime.tm_year,
		thistime.tm_wday,
		thistime.tm_hour, thistime.tm_min, thistime.tm_sec);
		oldsec = thistime.tm_sec;         						// Save past value seconds


      placeString(userPort,22, 18, stime);      			// Write clock put userPort

}


//=============================Check User Interface=========================
xmem void check_User(int port)
{
unsigned long nl;
int 		n,m, cmd_start;
int 		space[20];
char 		string[3], text[5], ACK,TraceStat,IDPtr,valid;
int 	   len, CRCstart, CRClen;
const 	static char nextcursor[] = {1,3,3,4,6,6,7,9,9,11,11,12,14,14,15,17,17,18,1};
char 		rcmd[256], rbcmd[280];  			// rcmd (ASCII commands), rbcmd(rcmd converted to binary)
char 		*next, *ptr;
char 		CRC[2],load,temp;
char 		data,protocol,Error;

if(getByte(port,&data)==0)return;	 		// Bail out if no data is detected
	memset(rcmd, 0, sizeof(rcmd));
	memset(rbcmd, 0, sizeof(rbcmd));
	MSdelay(100);									// Wait to receive more of the message, delay 50 msec

 		switch(data)
				{
				case 'a': //==Switch to ASCII data formating
				case 'A':
                        format[port]='A';
								break;

				case 'b': //==Switch to binary mode
				case 'B': //
								format[port]='B';
								break;

				case 'm': //=="M or m" send display to hyperterminal
				case 'M':
							if(getMSG(port,rcmd,ENTER))
							{
							len=strlen(rcmd);
							PC[port]='F';           // Turn off formatted PC interface
							display[port]='F';   	// shut of display until requested

							for(m=0;m<len;m++)
								{
								if(rcmd[m]=='M' || rcmd[m]=='m')	// received 'm' main menu request
									{
									menu[port]= 'T';			// enable main menu functions
									showmenu(port);
									locxy(port,22, 20+cursor);
									sendByte(port,'^');
									return;
									}
								}
                     }
							break;

            default: break;
            }
}


// =========================== Milli second Delays =====================
xmem void MSdelay(int n)
{

unsigned long int	timer, timer2;

timer=MS_TIMER+(long)n;
timer2=timer;
while(MS_TIMER<timer)
{
	if ((MS_TIMER%500)==0) {	DS1led(ON); }
   else if ((MS_TIMER%250)==0) {	DS1led(OFF); }
};

}


//===================Formats time field for transfer via buffer ====================

xmem void getTime(char *buf)
{
tm_rd(&time);
buf[0]=time.tm_mon;
buf[1]=time.tm_mday;
buf[2]=time.tm_year;
if(buf[2]>99)buf[2]-=100;
buf[3]=time.tm_wday;
buf[4]=time.tm_hour;
buf[5]=time.tm_min;
buf[6]=time.tm_sec;
}

//=================== Check Status of Macro Commands=====================//
nodebug root interrupt void checkMacro()
{
int n;
static char data[64];
static char BrdAddr,ExtAddr;

BrdAddr=	48;    // Not really used
ExtAddr= 12;

#asm
timerb_isr::
	push	af							; save registers
	push	hl

	ioi	ld a, (TBCSR)			; load B1, B2 interrupt flags (clears flag)

	ld		hl, (count)
	inc	hl							; increment counter
	ld		(count), hl

	ld		a, 01h
	and	l							; mask off all but lowest bit of counter
	jr		z, match_0200
match_0000:
	ld		a, 80h  					; was 40h   (should be 0200h= 80h)
	ioi	ld (TBM1R), a			; set up next B1 match (at timer=0200h)
	ld		a, 00h					; 01h bit-mangled to for TBM1R layout
	ioi	ld (TBL1R), a			; NOTE:  you _need_ to reload the match
										;	register after every interrupt!
	jr		done
match_0200:
	ld		a, 00h
	ioi	ld (TBM1R), a			; set up next B1 match (at timer=0000h)
	ioi	ld (TBL1R), a			; NOTE:  you _need_ to reload the match
done:
	pop	hl							; restore registers
	pop	af
	ipres								; restore interrupts
#endasm
                              // Produces Interrupts
										// 0000h   Interrupt
                              // 1000h
                              // 2000h   Interrupt
                              // 3000h

if(count>=globals.ISRTimer)			//== Timer used by Macro's
	{
	count=0;
// Place Interrupt Scheduled Functions here

	HiResAnalog(BrdAddr,ExtAddr,'A',data);		// Get IO (A2D) 'A'll channels  48,12,'A',data)
//  Compute control laws
//  Output control command
	}
}


//=====================show menu to the dumb terminal===================
xmem void showmenu(int port)
{
	int i,temp;
   char *ptr;

 	xstring showText
						{
						"         iControl Incorporated",
						"  Digital Signal Processor (DSP) Menu ",
						"",
						" Use 'Send Text File' to send commands",
						"",
						" Keypad Command Options:",
						"",
						" 'MM<ENTER>' Refresh Main Menu",
						" 'MS<ENTER>' Setup Menu (password req'd)",
						" 'MA<ENTER>' Display Macro Status",
						" 'MP<ENTER>' Display Command Schedule",
						" 'U<ENTER>'  Upload Stored Data",
						" 'DB<ENTER>' Display DSP Binary Data ",
						" 'iDSP Version:    01.04.06-v1.1     ",
                  "",
						"",
						"",
						" Type your choice >> "
						};
	clrscrn(port);

	for(i=0;i<15;i++)
		{
 		placeString(port,i+2, 1,get_xstring(showText,i));
		}

	placeString(port,18, 1, get_xstring(showText,16));
	placeString(port,19, 1, get_xstring(showText,17));
	placeString(port, 22, 4, "Current Time");

}



//=================Position cursor on the dumb terminal=======================
xmem void locxy(int port,int row, int col)
{
	char xx[8];
	char temp;
	xx[0] = '\x1b';
	xx[1] = '[';
	temp  = row/10;
	xx[2] = temp+0x30;
	xx[3] = (row - 10*temp) + 0x30;
	xx[4] = ';';
	temp = col/10;
	xx[5] = temp+ 0x30;
	xx[6] = (col - 10*temp) + 0x30;
	xx[7] = 'H';
	sendMSG(port,xx,8);
}

//==================Clear the screen on the dumb terminal===================
xmem void clrscrn(int port)
{
	sendMSG(port,"\x1b[2J",4);
}

//=========================================================================
//								Rabbit Specific Functions                         |
//=========================================================================

#if processor=='R'

//======================Initialize Serial Ports ===========================

xmem void initPort(int port, unsigned long baud)
{
int bits,parity;

if(0x08 & portCFG[port])bits=PARAM_7BIT;
else bits=PARAM_8BIT;

parity= 0x03& portCFG[port];
if(parity==0)parity=PARAM_NOPARITY;		// No parity is the default
if(parity==1)parity=PARAM_OPARITY;		// Odd parity
if(parity==2)parity=PARAM_EPARITY;		// Even Parity

switch(port)
	{
	case 0:
		serAparity(parity);
		serAdatabits(bits);
		serAopen(baud);
		break;
	case 1:
		serBparity(parity);
		serBdatabits(bits);
		serBopen(57600);
		break;
	case 2:
		serCparity(parity);
		serCdatabits(bits);
		serCopen(38400);
		break;
	case 3:
		serDparity(parity);
		serDdatabits(bits);
		serDopen(9600);
		break;
	default: break;
	}
}
//=================== Reset Serial port receive buffers ===================

xmem void resetPort(int port)
{
	switch(port)
		{
		case 0: serArdFlush();break;
		case 1: serBrdFlush();break;
		case 2: serCrdFlush();break;
		case 3: serDrdFlush();break;
		default:break;
		}
}

//=================== Send messages via serial port ======================

char sendMSG(int port, char *value, int numBytes)
{
int n;

if(numBytes>0)
	{
	switch(port)
		{
		case 0:serAwrite(value,numBytes);break;
		case 1:serBwrite(value,numBytes);break;
		case 2:serCwrite(value,numBytes);break;
		case 3:serDwrite(value,numBytes);break;
		default:break;
		}
	}

return 1;
}

//=================== Send byte via serial port ===========================

xmem char sendByte(int port, char value)
{
	switch(port)
		{
		case 0:serAputc(value);break;
		case 1:serBputc(value);break;
		case 2:serCputc(value);break;
		case 3:serDputc(value);break;
		default:break;
		}

return 1;
}
//=================== Get messages via serial port =======================

xmem char getMSG(int port, char *value, char term )
{
int temp, n, time_cnt, past_sec;
char skip;

	n=0;
	skip='F';
	temp=0;
	time_cnt=0;
	value[0]='\0';							// Start out with zero length string
	while(temp!=(int)term)				// Continue checking port until "term" character is received
		{
		switch(port)
			{
			case 0: temp= serAgetc(); break;
			case 1: temp= serBgetc(); break;
			case 2: temp= serCgetc(); break;
			case 3: temp= serDgetc(); break;
			default: return 0; break;
			}
		if(temp!=-1 && skip=='F')		// if a valid read and not a comment
			{
			if(temp=='/')skip='T';		// encountered comment skip everything until "term"
			else
				{
				value[n]=(char)temp;
				n++;
				}
			value[n]='\0';					// Terminate string where we are
			}
		tm_rd(&time);
		if(time.tm_sec!=past_sec)		// if seconds have incremented
			{
			past_sec=time.tm_sec;		// Save current seconds
			time_cnt++;						// Increment time_out counter
			}
		if(time_cnt>20)return 0;		// If after timeout, return 0
		}
	return 1;

}
//==================== Get byte via serial port ===========================

xmem char getByte(int port, char *value)
{
int temp;
   switch(port)
   	{
   	case 0:temp=serAgetc();break;
   	case 1:temp=serBgetc();break;
   	case 2:temp=serCgetc();break;
   	case 3:temp=serDgetc();break;
   	default: temp=-1; break;
   	}

	if(temp==-1)temp=0;					// return "zero" if no data
	else
		{
		*value=(char)temp;
		temp=1;
		}
return (char)temp;
}
//====================== output byte =======================================

xmem void outport(unsigned int addr, char value) // Implement outport using Rabbit functions
{
if(addr > 0x100)				//==Must be external address
	{
	WrPortE(addr,NULL,value);			// Write value out external bus
	}
else if(addr< 0x20)						// Must be an iDAC IO bus address
	{
//eft	BusWrite(addr,value);
	}
else								//==Must be a local register or port
	{
	WrPortI(addr,NULL,value);
	}
}
//====================== input byte ========================================
xmem char inport(unsigned int addr)		// Implement inport using Rabbit library function
{
char temp;

if(addr > 0x100)							// Must be external address
	{
	temp=RdPortE(addr);					// Read the data from the external bus
	}
else if(addr< 0x30)						// Must be an iDAC IO bus address
	{
//eft 	temp=BusRead(addr);
	}
else temp= RdPortI(addr);				// Must be local register or port;
return temp;
}

//===================== Read real-time clock ==================================
xmem	int tm_rd(struct tm *time)
{
unsigned long t0;
t0 = read_rtc();
mktm(time, t0);
return 1;
}

//======================End of Rabbit Serial Library =========================
#endif



//=============================Initialize Protected Variables========================
//
//   - These variables are stored in battery backed RAM
//   - They are initialized only when first programmed, or when requested by command
//   - Send command 255 to reset protected variables
//===================================================================================

xmem void initProt()
{
int n;

	first_time= 	'F';				// Indicate this code has been executed before,


//=======Modem Configuration parameters=========================================
//=== Initialize Stored SOH Data ==============================================

	rebootCounter =	0;

//=== Init Flash with defaults if not yet initialized =======================

	readUserBlock(Lbuf,(int)&PTFlashBasePtr->initialized, 1);
	if (Lbuf[0] != 'T')
		{
		loadDefaults();
		writeToFlash();
		}
}

//================================Initialize iDAC ============================
xmem void Init_Device
()						// Initialize iDAC variables
{
char n;
	// ============= Set up default Configuration ====================
	userPort=			 	 		0;			// Define serial port address for user interface
	Channel=							0;			// Used for high speed data acquisition in ISR
	// ============== Read flash into global variables structure
	readUserBlock(&globals,0, (int)sizeof(TFlashProtected));

	memset(send_Data,1,4);

	//================ Initialize menus and formats for each port (Overwrite with command in Macro 38)

	memset(menu,'F',4);
	memset(display,'F',4);
	memset(PC,'F',4);
	memset(format,'B',4);
	memset(portCFG,0,4);


	cursor = 			  1;
	user_state= 		'O';					// Initialize User interface flag,

	tm_rd(&thistime);							// read the real time clock
													// Make sure clock is within range
	if((thistime.tm_year <100 || thistime.tm_year > 147) && processor=='R')
		{
		thistime.tm_hour = 23;
		thistime.tm_min = 59;
		thistime.tm_sec = 50;
		thistime.tm_mon = 1;
		thistime.tm_mday = 19;
		thistime.tm_year = 105;				// Equates to 2004
		thistime.tm_wday =4;					// ignored for Rabbit..computed by RTC
		tm_wr(&thistime);
		}

	// ===========set default baud rates for other serial ports
		if ( globals.baud[1]!= 0)initPort(1,globals.baud[1]);				// Initialize Port 1 serial port
		if(processor=='R')															// If processor is a Rabbit, initilize the other ports
			{
			if (globals.baud[2] != 0) initPort(2,globals.baud[2]);		// Initialize Port 2 serial port
			if (globals.baud[3] != 0) initPort(3,globals.baud[3]);		// Initialize Port 3 serial port
			}

	//============Increment our reboot counter ===============
		rebootCounter++;

	//============Setup TimerB interrupt service routine =====
		count=globals.ISRTimer-2;							// Plan on early Interrupt to run Init Macro
		SetVectIntern(0x0B, checkMacro);					// Set up interrupt for timerB
		WrPortI(TBCR, &TBCRShadow, 0x09);				// clock timer B with (use 0x09 perclk/16) or (0x01 perclk/16)
																	// set interrupt level to 1
		WrPortI(TBL1R, NULL, 0x00);
		WrPortI(TBM1R, NULL, 0x00);						// set initial match
		WrPortI(TBCSR, &TBCSRShadow, 0x03);				// enable timer B and B1 match interrupts

   //=============== Init A2D ====================================
    	initADC();

    //======= Overwrite SPI control..improve A2D performance ====
    	MSdelay(100);										// Let Macro 38 start before completing User Init

    	WrPortI(0x24,NULL,0x84);			// Set up port A for output
	   WrPortI(0x40,NULL,0x40);			// Enable Periphial power on version 1.4 motherboards
	   WrPortI(0x55,NULL,0x55);			// Setup PortC for normal Serial port use (4 channels)
	   WrPortI(0x65,NULL,0x00);			// Set up port D for normal I/O
	   WrPortI(0x66,NULL,0x00);			// Set up port D as driven high or low
	   WrPortI(0x67,NULL,0xff);			// Set up port D for output
      WrPortI(0x77,NULL,0x12);			// Set up port E data direction (external I/O logic)
		WrPortI(0x75,NULL,0x1a);			// Set up port E function register (PEFR) (external I/O logic)
 		WrPortI(0x81,NULL,0x38);			// Set up external I/0 block 1 (read/write) register (AeroComm register)
		WrPortI(0x84,NULL,0xf8);			// Set up external I/0 block 4 (read/write) for LED and portC
		WrPortI(0x85,NULL,0x38);			// Set up external I/0 block 5 (read/write) for ADC

   	WrPortI(PADR,NULL,0xc0);						// Raise A7 to enable GPS receiver (reset)

}	// Initialize Device

//=============== Initialize Analog Converter on RM3400 =============
xmem void initADC()
{
char data[2];

    brdInit();         						   // This Inits the 3400 board
    ADC_GAIN=1;  									// Default gain is 1
    ADC_PTR=0;                            // Pointer into Gain array is 0
    ADC_CMD= 0x88|GainCmd[ADC_PTR];
    anaIn(0,SINGLE,GAIN_1);
 	 BitWrPortI ( PDDR, &PDDRShadow, 1, 2 );		// turn off /CS (1=off)
 	 BitWrPortI ( PDDCR, &PDDCRShadow, 0, 2 );	// bit 0 = "normal" output
 	 WrPortI ( PDCR, &PDCRShadow, 0 );				// bits 0..3 = clocked by perclk/2
 	 BitWrPortI ( PDDDR, &PDDDRShadow, 1, 2 );	// bit 0 = output
 	 SPIinit();   											// Init SPI interface to support ADC
    data                                              [0]= 0x03;   									// ADC Control register address
	 data[1]= 0x24;   									// 0x24 returns MSB first for ADC data in Binary mode
	 SPIWrRd (data, Lbuf, 2 );    					// Init ADC device
}



//==================== load defaults into globals ====================
xmem void loadDefaults(void)
{
	int n;

// ============= set all values to zero to save code space.
//               Uncomment the variable that you want to initialize to non-zero
	memset(globals.RTU_ID,0,4);			// ID used for RTU functions (most significant BYTE)

//== Memory allocation =======================================================
	globals.TX_SIZE= 		 		 32;		// Transmit buffer size (number of commands)
	globals.TX_LEN= 				256;		// max command length
	globals.NUM_RECORDS= 		256; 		// Data Storage Memory allocation (number of records)
	globals.RECORD_SIZE=	 		32;		// record size (Sectors)

//=== Default Serial Port configurations ======================================
	globals.baud[0]=				57600;	//	57600	default baud rate
	globals.baud[1]=				57600;	// 57600	default baud rate
	globals.baud[2]=				57600;		//	1200	default baud rate
	globals.baud[3]=				57600;		// 1200	default baud rate

	globals.schedule=			 	 'T';		//'F' Disables schedule functions

	globals.ISRTimer=		  364;		  	// Counter used by Interrupt Service Routine (default 18 Mhz Rabbit)
	globals.RetryTime=		 2;			// Seconds until discarding TX buffered data
	globals.discard=		    3;			// Number of attempts to deliver command before discarding (used by iGate)

}

//==================== Save Globals to flash =========================
xmem int writeBlock(int addr, void* buff, int buffSize)
{
	const static char flashWriteFailed[] = "flash write failed";
	if (writeUserBlock(addr, buff, buffSize) != 0)
		{
		sendMSG(userPort,flashWriteFailed,strlen(flashWriteFailed));
		return -1;
		}
	return 0;
}

//===================== Write globals to Flash =========================
xmem void writeToFlash(void)
{
	globals.initialized='T';
	writeBlock(0, (void*)&globals, sizeof(TFlashProtected));
}

//======compare string of arbitrary length ==============================
xmem char comp(char *str1, char *str2, int bytes)
{
int n;
for(n=0;n<bytes;n++)
	{
	if(str1[n]!=str2[n])return 0;
	}
return 1;
}


//====================== Display functions ============================
xmem void placeString(int port, int x, int y, char *str)
{
char string[5];

	if (PC[port]=='T')
		{
		if (format[port]=='N')
			{
			sendMSG(port, stime, strlen(stime));
			sendMSG(port, " - ", 3);
			sendMSG(port, str, strlen(str));
			sendMSG(port, "\n\r", 2);
			}
		}
	else
		{
		if(send_Data[port])
			{
			locxy(port, x,y);
			sendMSG(port, str, strlen(str));
			}
		else if(user_state=='L' && x<4)
			{
			string[0]=17;
			string[1]=y;					// column
			string[2]=x;					// row
			sendMSG(port,string,3);
			sendMSG(port, str, strlen(str));
			}
		}
}

//===================== Clear Line =======================================
xmem void clearLine(int port, int x, int y, int blanks)
{
	int i;
	if(PC[port]=='T' || debugMSG)return;			// Only Clear lines in Menu mode
	locxy(port, x,y);
	for (i=0;i<blanks;i++) sendByte(port, ' ');
}

//====================== copy xstring to root ============================
xmem char* get_xstring(unsigned long xstring_addr, int index)
{
	unsigned long	string_addr, index_addr;
	char buff[64];

   xstring_addr += index * sizeof(long);									// calculate address in string index
   xmem2root((void *)&string_addr, xstring_addr, sizeof(long));	// get address of desired string
   xmem2root((void *)buff, string_addr, sizeof(buff));				// load string into data
   buff[sizeof(buff)-1] = '\0';												// ensure that it is null terminated
	return buff;
}

//==================== Hi Res Serial Analog ====================================
xmem void HiResAnalog(char Baddr, char ExtAddr , char channel, char *buf)
{
char command, n,stop;
unsigned int m;
char data[3];

if(channel=='A')		// All Channels
	{
	channel=0;			// Start at 0
	stop=9;
	}
else						// Pick one channel
	{
	channel--;			// Channel has input range from 1-8, but is indexed 0-7
	stop=channel+2;
	}
WrPortI(TBCSR, &TBCSRShadow, 0x00);	      	// disable timer B and B1 match interrupts
BitWrPortI ( PDDR, &PDDRShadow, 0, 2 );		// drop ADC chipselect
for(n=channel;n<stop;n++)                    // First time through starts convergence
   {
   command= ADC_CMD|n;      						// ADC_CMD= 0x88|(Gain_1-> Gain20);
   SPIWrRd (&command, &data, 3 );
   if(n)
       {
		 m=   data[1]*256+data[2];     			//  data[1]=MSB, data[2]=LSB (in int format)
       m=m>>4;
       memcpy(buf+2*(n-1),&m,2);
       }
   }
BitWrPortI ( PDDR, &PDDRShadow, 1, 2 );		// Raise ADC chip select
WrPortI(TBCSR, &TBCSRShadow, 0x03);	      	// re-enable timer B and B1 match interrupts after getting data
}

void ACS_Test()
{
  	char Jet_Addr[90], buffer[50];
   int i,j;

   int Enter1, Enter2;

   Enter1=0x0d; Enter2=0x0a;

   i=0;
//Valve 1
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201000100*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 2
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201000200*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 3
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201000400*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 4
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201000800*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 5
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201001000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 6
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201002000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 7
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201004000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 8
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201008000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 9
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201010000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 10
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201020000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 11
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201040000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 12
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201080000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(30000);
//Valve 1 & 2
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201000300*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 4 & 5
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201001800*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 7 & 8
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS020100C000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }
   MSdelay(10000);
//Valve 10 & 11
   for(j=0; j<10; j++)
   {
	   sprintf( Jet_Addr,"@01WVS0201060000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(30); i=0;
	   sprintf( Jet_Addr,"@01WVS0201000000*%c%c!", Enter1, Enter2);
	   while (Jet_Addr[i]!='!')
	   {
	      sendByte(ACS_Port,Jet_Addr[i]);
	      sendByte(Data_Port,Jet_Addr[i]);
	      i=i+1;
	   }
      MSdelay(1000);  i=0;
   }

	return;
}