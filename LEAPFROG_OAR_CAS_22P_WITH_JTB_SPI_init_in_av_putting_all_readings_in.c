/**********************************************************************************/
// Was Previously Copyright. iControl Incorporated, 2000
// Now it is whatever Barnhart, Omair, Cassandra and Aditi want with it.
/**********************************************************************************/
// Revision Control : Rev 1.5
// Updated by : 4/13/07 - Omair/Cassandra/Omair
// Updates : Transformation matrices for the IMU in the GNC
//           input profiles for 3 different flight tests for the tethered tes7
// 4-14-07 - ENDCHAN to 6
// 4-21-07 - adding the velocities not the accels in the GNC - Aditi
// 6-06-07 - New GNC3
// 6-06-07 - Never Use this one. Has changed port settings
// 6-07-07 - A variant of the 22A. For Time Testing. Changing function printfs to reduce time. Port settings back to normal.
// 6-07-07 - 22C was closed with the successful reduction of times of the Cmd Funcs and failure on CnvrtSnsrVals.
// 6-07-07 - Making new X-Interrupt to account for a new 50-50 cycle.
// 6-08-07 - New 22F to incorporate new things in X2. The 22E is with good X2 and ADC2.
// 6-08-07 - 22F does not run. X2 fails. Either Bad CnvrtSnsr or the ADC2. Though Jet_Send was Optimized.
// 6-11-07 - 22G is good. It has the X2 working perfectly. ADC robin works OK. ADC queue initialized.
// 6-11-07 - 22H: Now putting the mulitple ADCs in. First need to find out if GNC3 works.
// 6-11-07 - Also find out if making it non-Xmem helps.
// 6-11-07 - First doing the removal of extra functions and going to Internal Memory.
// 6-11-07 - No 22I. 22H has all unnecessary functions removed and all Mission mode functions should be INMEM now.
// 6-11-07 - Doing the unnecessary GNC variable removal now.
// 6-12-07 - Did the GNC_init and globals reduction. Compiles, runs, function lengths stored in xls.
// 6-12-07 - 22K now for the ACS send optimization and then to go for the stuff said above.
// 6-13-07 - mfuelburn and desired velocities were rewritten for less * and /
// 6-13-07 - Initial ADC readings in init avionics are now the sensor_offsets
// 6-14-07 - ISRTimer changed to 180 from 144
// 6-15-07 - Don't use. Only for DAQ for TIM.
// 6-28-07 - From 220 but used to play with rawdata and filter depths
// 6-29-07 - Added SPI stuff
#memmap xmem
#class static
#define processor 'R'                     // define processor type R="Rabbit" or Z="Z180"
#class auto
//================ For SPI Interface =========================================
#define SPI_SER_D     // changed from _B
#define SPI_CLOCK_MODE 0 /* jtb was not defined defaulted to 0 */
#define SPI_CLK_DIVISOR 5

//============================================================================

#use fft.lib
#use SPI.LIB
#use RCM34XX.LIB

#define version      "01.04.06-v1.1"         // Software Version #

#define Run 'R'
#define Stop 'S'
#define Background 'B'
#define I_ACK 0x02
#define I_NAK 0x00

#define AINBUFSIZE   1023                       // size of userPort receive buffer
#define AOUTBUFSIZE  1023                       // size of transmit buffer
#define BINBUFSIZE   1023                       // size of receive buffer
#define BOUTBUFSIZE  1023                       // size of transmit buffer
#define CINBUFSIZE   1023                       // size of receive buffer
#define COUTBUFSIZE  1023                       // size of transmit buffer
#define DINBUFSIZE   1023                       // size of receive buffer
#define DOUTBUFSIZE  1023                       // size of transmit buffer
#define EINBUFSIZE   1023                       // size of receive buffer
#define EOUTBUFSIZE  1023                       // size of transmit buffer
#define FINBUFSIZE   1023                       // size of receive buffer
#define FOUTBUFSIZE  1023                       // size of transmit buffer
#undef  outport                                 // For Rabbit redefine outport
#undef  inport                                  // For Rabbit redefine inport
#undef  bit
#undef  res
#undef  set
#define  tm_rd TM_RD                            // For Rabbit redefine tm_rd

//============================ Memory Allocations ================================#define NUM_MACROS 40        //== Macro Buffer Size Definitions
#define MAX_MACRO_SIZE 128
#define NUM_MACROS 40

#define NUM_STORED_CMDS 40    //== Stored Command sizes (for time tagged commands)
#define MAX_STORED_CMD_SIZE 32

#define NUM_LABELS 12
#define MAX_LABEL_LEN 16
#define NUM_SFACTORS 16
#define MAX_NUM_SFACTORS 2
#define MAX_STORED_DATA 248   // Maximum stored record length (includes multiple sectors)
//==================================================================================

typedef struct {                                                                       // cummulative bytes
   char RTU_ID[4];            // RTU id                                                4
   char schedule;             // Schedule checking..enabled=='T'                       99
   unsigned long  baud[4];    // Baud rates for serial ports                           126
   int ISRTimer;              // Counter used by MAcro Interrupt Service Routine       134
   int dataPort;              // Serial Port used by I/O device (normally 2 or 3)      138
   char RXmode;               // RXmode=0,1,2                                          144
   char RetryTime;            // Seconds until TX retry                                156
   char discard;              // Number of retries before discard                      157
   int TWindow;               // Valid Command time window                             158
   char password[10];         // Password for setup menu
   char label[NUM_LABELS][MAX_LABEL_LEN];                   // Analog Channel Labels   +128  222
   float sFactors[NUM_SFACTORS][MAX_NUM_SFACTORS];          // Scale factors           +128  350
   char StoredCmds[NUM_STORED_CMDS][MAX_STORED_CMD_SIZE];   // Stored Commands         +1280
   char MacroCmds[NUM_MACROS][MAX_MACRO_SIZE];              // Stored Macro commands   +5120
   unsigned int  TX_SIZE;     //== Command buffer memory allocation
   unsigned int  TX_LEN;
   unsigned int  NUM_RECORDS; //== Stored Data memory allocation
   unsigned int  RECORD_SIZE;
   char initialized;          // flag to see if flash has ever been initialized  +1
} TFlashProtected;

TFlashProtected * const PTFlashBasePtr = (TFlashProtected*)0;  // pointer to this structure which is accesed at offset 0

TFlashProtected globals;                  // structure where all globals are accessed

// global buffer used locally - data cannot be expected to be persistent between uses.
// *** IMPORTANT - If this is changed from 1280, must look at where the Store cmds and
//                 macro cmds are read and written to flash

   char Lbuf[256];

// Protected data
// Data that changes too much to be put into flash
   protected   unsigned char rebootCounter;  // counter incremented with each reboot
   protected   char portCFG[4];           // Stores special configuration for Serial ports
   protected   struct tm thistime;        // Used for scheduling and tasks (protected to prevent corruption)
   protected   char first_time;           // Flag indicating first time main is executed

   int         userPort;                  // User Serial Port(normally 0)
   unsigned    long ulongChecksum;
   char        TCW;                                   // Time Code Word
   struct      tm time;                               // Used for checking timeouts
   char        display[4], format[4];                 //
   char        PC[4];                                 // flag to enable PC direct modes
   char        menu[4];                               // flag to display menu's
   int         oldsec,oldmin;
   char        RTU_mode, tlmtext[7];
   char        stime[25];
   int         LoadCnt;
   int         index, nsamples;
   char        portA, portB, portC, PEshadow;
   char        user_state;
   int         cursor;
   int         count,Done;
   char        debugMSG;
   char        send_Data[4];                          // Flag indicating serial ports that are enabled for data transmission
   unsigned    char PScnt;                            // Power save variables (255 max)
   unsigned    long pSEC_TIMER;                       // Past value of SEC_TIMER
   char        Channel,ADC_CMD, ADC_GAIN,ADC_PTR;

const char  ESCAPE=27;                 // Define ESC key to break loop
const char  ENTER =13;                 // define <CR> for Rabbit
const char  GainCmd[]={0x00,0x10,0x20,0x30,0x40,0x50,0x60,0x70};

xmem void Init_Device();                           //=== Initilization functions
xmem void initProt();
xmem void initComm(char mode, char type, int port);
xmem void initPort(int port, unsigned long baud);
xmem void writeToFlash(void);
xmem void loadDefaults(void);
xmem void M_Allocate(void);
xmem void initADC();

xmem int adjust_rtc(int data, int cursor);         //=== Time related functions
xmem void getTime(char *buf);
xmem void once_per_second();
xmem void MSdelay(int n);
xmem void getEpoch(char *value);

char sendMSG(int port, char *value, int numBytes); //=== Serial Port functions
char getMSG(int port, char *value, char term);
char sendByte(int port, char value);
char getByte(int port, char *value);
xmem void resetPort(int port);
xmem int  setPort(int port);

//========================= string defines and functions ================================//
xmem void placeString(int port, int x, int y, char *str);
xmem void clearLine(int port, int x, int y, int blanks);
xmem void placeAddress(int port, int x,int y,char protocol,char *addr);
xmem char* get_xstring(unsigned long xstring_addr, int index);

xmem  void outport(unsigned int addr, char value);
xmem  char inport(unsigned int addr);
xmem  int tm_rd(struct tm *time);

//============================ LLPV Allocations ================================
#define DS1 6     //port F bit 6
#define DS2 5     //port D bit 7
#define ON  0     //state to turn on led
#define OFF 1     //state to turn off led
#define Jet_Port 5   //Port F
#define ACS_Port 2   //Port C
#define Data_Port 4  // Port E
#define Comms_Port 0 // Port A
#define Jet_Baud 57600      //Port D BaudRate
#define dT 0.050     // for GNC delta-time
#define dTdT2 dT*dT/2
#define dT_2 2/dT
#define dT3 dT/3
#define dTdTdT6 dT*dT*dT/6
#define profile0 1   // for all zero
//#define profile2   1    // 2 ft profile
//#define profile4   1   // 4 ft profile
//#define profile5   1    // 5 ft profile

#define STARTCHAN 0
#define ENDCHAN 6
#define GAINSET GAIN_1

#define MaxGNCLoopCount 100     //GNC every 10th time of the x loop
#define MaxTelemetryCount 9   //Telemetry sent every 10th iteration of GNC loop

#define ZPLUSX 1
#define ZMINUSX 7
#define ZPLUSY 4
#define ZMINUSY 10
#define MINUSXMINUSY 0
#define PLUSXMINUSY 2
#define MINUSYPLUSX 3
#define PLUSYPLUSX 5
#define PLUSXPLUSY 6
#define MINUSXPLUSY 8
#define PLUSYMINUSX 9
#define MIUNUSYMINUSX 11

//========================= LLPV X2 Assignments ============================//
#define FilterDepth 1
int InterruptX2();
void ADC2();

int RobinPosition;

//========================= LLPV functions ================================//
int InterruptXmsec();

xmem int InitAvionics();
void ADC(int iteration_Number);
void ACS_Cmd();
void Jet_Send();
xmem void GNC_init();
int GNC2(int AbortRcvd);
int GNC3(int AbrtOrder);
void ConvertSensorVals();
void ConvertSpi();
void DataFilter(int NumofSamples);
xmem void SendTelemetry();
xmem void SystemShutDown();
xmem void IdleShutDown();

xmem void IMU_Status_Report();
xmem void ACS_Test();
xmem void ACS_Test2(int IterationPerThruster);
xmem void Jet_Test();


void DS1led(int state);
void DS2led(int state);
void R_Stuff();
xmem int get_CMD(int Port_Num);

void SPIaccelADC();
//========================= LLPV variables ================================//
int TelemetryCount;
int GNCLoopCount;
float GNCJetrpm;
unsigned int facs[12];
int AbortCmdRcvd;
float ad_channel[7];
float sensor_read[7][21];
//float sensor_read[7][FilterDepth+1];
float sensor_filtrd[7];
float adc_sample_x;
float adc_sample_gyro;
float sensor_filtrd_prev[7];
// LookSee     1,2,3-accel 4,5,6-gyro 7-pressure transducer
//const float sensor_ratio_error[]={1.017, 1.017, 1.017, 1.008, 1.012, 1.016, 1.084};
float sensor_offsets[7];
//const float sensor_offsets[]={1.6459, 1.6518, 1.6074, 1.101, 1.072, 1.043, 0};
//const float sensor_offsets[]={1.6459, 1.6518, 1.6074, 1.3425, 1.3106, 1.2717, 0};
//const float sensor_UnitConvert[]={29.28318, 28.9649, 29.8552, 88/63, 88/63, 88/63, 100};
const float sensor_UnitConvert[]={29.28318, 28.9649, 29.8552, 2.700, 3.623, 2.718, 100}; //times two for the gyros
//const float sensor_UnitConvert[]={29.28318, 28.9649, 29.8552, 2.793, 2.793, 2.793, 100}; //times two for the gyros
//const float sensor_UnitConvert[]={29.28318, 28.9649, 29.8552, 176/63, 176/63, 176/63, 100}; //times two for the gyros
const int NumOfReadingsPerChannel=MaxGNCLoopCount-1;
char cmd_BYTE;
int flag_space;
int flag_slash;
int flag_d;

int In_Flight;
const char ShutDownTurbine[] = "1,TCO,0";
const char StartTurbine[] = "1,TCO,1";

   float xdes, ydes, zdes, phides, thetades, psides, fxdes, fydes,
      fzdes, fphides, fpsides, fthetades;
   float facsx, facsy, facsz, fth, facsphi, facspsi, facstheta, facsthruster;
   float xmeas, ymeas, zmeas, xdmeas, ydmeas, zdmeas, xddmeas, yddmeas, zddmeas;
   float x_integral, y_integral, z_integral, xerroro, yerroro, zerroro;
   float x_meas_integral, y_meas_integral, z_meas_integral;
   float x_des_integral, y_des_integral, z_des_integral;
   float xerrormeas, yerrormeas, zerrormeas, xerr, yerr, zerr, phierr,
      psierr, thetaerr;
   int m, rowmax, amp, ampr, ampz, xband, yband, n, nfiretotal, nfire, am, nm, pron, pta, ptb;
   float kp_x, kp_y, kp_z, kd_vx, kd_vy, kd_vz, ki_x, ki_y, ki_z;
   float kp_phi, kp_theta, kp_psi, kd_phi, kd_theta, kd_psi;
   float Ad_time, totaltime;
   float phimeas, psimeas, thetameas, phidmeas, psidmeas, thetadmeas;
   float X_force, Y_force, Z_force, Phi_force, Theta_force, Psi_force;
   float vx_des, vy_des, vz_des, vx_deso, vy_deso, vz_deso, xdeso, ydeso, zdeso;
   float vphi_des, vtheta_des, vpsi_des, vphi_deso, vtheta_deso, vpsi_deso, phideso, thetadeso, psideso;
   float vx_err, vy_err, vz_err, vphi_err, vtheta_err, vpsi_err;
   float phiband, psiband, thetaband;
   float ixo, ixf, iyo, iyf, izo, izf,alpha;
   float xmeaso, ymeaso, zmeaso, phimeaso, psimeaso, thetameaso;
   float xdmeaso, ydmeaso, zdmeaso;
   float x_err_integral, y_err_integral, z_err_integral;
   float phidmeaso, thetadmeaso, psidmeaso;
   float fthunlim, throttlechange, fthprev, maxrate, engrate, mfuelburn, fthmax;
   float mass, mcoo, mdry, mfuel, mcooburn, tremain;
   float tlandsub, zdist_cp_to_cg, dcgzoffset;

#ifdef profile2

const float X[62]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0};

const float Y[62]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0};


const float Z[62] =  {0.0, 0.1524, 0.474690582, 0.499221557, 0.511413152, 0.523093394, 0.534752445,
   0.546410617, 0.558068754, 0.569726889, 0.581385024, 0.593043159, 0.604701293,
   0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428,
   0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428,
   0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428,
   0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428,
   0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428,
   0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428,
   0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428, 0.616359428,
   0.616359428, 0.387759428, 0.159159428, 0.0, 0.0, 0.0, 0.0};

const int T[95]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,
                 23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,
                 44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62};

#endif

#ifdef profile4
const float X[64]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

const float Y[64]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

const float Z[64]= {0.0, 0.1524, 0.561569258, 0.822955687, 0.993971323, 1.109724243, 1.191683415,
   1.252977347, 1.252977347, 1.252977347, 1.252977347, 1.252977347, 1.252977347,
   1.252977347, 1.252977347, 1.252977347, 1.252977347, 1.252977347, 1.252977347,
   1.252977347, 1.252977347, 1.252977347, 1.252977347, 1.252977347, 1.252977347,
   1.252977347, 1.252977347, 1.252977347, 1.252977347, 1.252977347, 1.252977347,
   1.252977347, 0.813751145, 0.556031352, 0.556031352, 0.556031352, 0.556031352,
   0.556031352, 0.556031352, 0.556031352, 0.556031352, 0.556031352, 0.556031352,
   0.556031352, 0.556031352, 0.556031352, 0.556031352, 0.556031352, 0.556031352,
   0.556031352, 0.556031352, 0.556031352, 0.556031352, 0.556031352, 0.556031352,
   0.556031352, 0.418871352, 0.281711352, 0.144551352, 0.144551352, 0.144551352,
   0.0, 0.0, 0.0};

const int T[95]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,
                 23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,
                 44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64};

#endif

#ifdef profile5

const float X[182]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

const float Y[182]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

const float Z[182]={ 0.0,0.195072,0.715679646,1.045211853,1.257899367,1.399135089,
     1.496677271,1.567500381,1.567500381,1.567500381,1.567500381,1.567500381,
     1.567500381,1.567500381,1.567500381,1.567500381,1.567500381,1.567500381,
     1.567500381,1.567500381,1.567500381,1.567500381,1.567500381,1.567500381,
     1.567500381,1.567500381,1.567500381,1.567500381,1.567500381,1.567500381,
     1.567500381,1.567500381,1.385505416,1.285186381,1.234813059,1.214982079,
     1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,
     1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,
     1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,
     1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,
     1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,
     1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,
     1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,
     1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,
     1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,1.214982079,
     1.214982079,1.214982079,1.044349553,0.950666339,0.904038458,0.904038458,
     0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,
     0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,
     0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,
     0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,
     0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,
     0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,
     0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,
     0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,
     0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,0.904038458,
     0.904038458,0.904038458,0.876606458,0.849174458,0.821742458,0.794310458,
     0.766878458,0.739446458,0.712014458,0.684582458,0.657150458,0.629718458,
     0.602286458,0.574854458,0.547422458,0.519990458,0.492558458,0.465126458,
     0.437694458,0.410262458,0.382830458,0.355398458,0.327966458,0.300534458,
     0.273102458,0.245670458,0.218238458,0.190806458,0.163374458,0.135942458,
     0.135942458,0.0,0.0};

const int T[182]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,
    23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,
    48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,
    74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,
    99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,
    118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,
    137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,
    156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,
    175,176,177,178,179,180,181,182};

    #endif


#ifdef profile0

const float X[95]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

const float Y[95]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

const float Z[95]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

const int T[95]={0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,
   50,52,54,56,58,60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,
   90,92,94,96,98,100,102,104,106,108,110,112,114,116,118,120,122,124,126,128,
   130,132,134,136,138,140,142,144,146,148,150,152,154,156,158,160,162,164,166,
   168,170,172,174,176,178,180,182,184,186,188};

#endif


//============================================================================//
xmem main()
{
   char temp_ser, temp, tempch;
   char Buffer[20];

RobinPosition=0;
//useMainOsc();               // Start system with Main Oscilator enabledclockDoublerOn();

if(first_time!='F' || globals.initialized!='T') // first_time is used to trigger super_reset
   {
   loadDefaults();
   _prot_init();
   initProt();
   }
else _prot_recover();

printf("\nBefore init device");
Init_Device();
printf("\nAfter init device");

   //initially start up A/D oscillator and charge up cap
   anaIn(0,SINGLE,GAINSET);

   //Open Ports
	printf("\nBefore open ports");
   serEopen(115200);
   serFopen(9600);
   serEwrFlush();
   serErdFlush();
   serFwrFlush();
   serFrdFlush();

   serAputc('A');
   serBputc('B');
   serCputc('C');
   serDputc('D');
   serEputc('E');
   serFputc('F');
	printf("\nAfter open ports");
   flag_space=0; flag_slash=0; flag_d=0;
// sprintf(TM_Frame, "\\d 123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOP*\n\r");
//   sprintf(ACS_TM, "boo");

// while ((temp_ser=serEgetc())!='C')           //Wait till we get a Start order on Comms
//   {}
DS2led(ON);
MSdelay(100);
DS2led(OFF);
      sprintf(Buffer, "mm\n\rds\n\r");
      //		sendMSG(Comms_Port, Buffer, 8);  //4/20/12
 		//		sendMSG(Data_Port, Buffer, 8);   //4/20/12

//Initialize LED Port on the newboard.
   BitWrPortI(PFDDR, &PFDDRShadow, 1, 6);
   BitWrPortI(PFDDR, &PFDDRShadow, 1, 5);

   printf("\nInitialized LEDS\n");
   /*while ((temp_ser=serEgetc())!='S')           //Wait till we get a Start order on Comms
   {
      if (temp_ser=='b')   ACS_Test();
      else if (temp_ser=='B') ACS_Test2(5);
      else if (temp_ser=='I') IMU_Status_Report();
      else if (temp_ser=='J') Jet_Test();
      else if (temp_ser=='X') R_Stuff();
      else if (temp_ser=='a') ADC2();
      else if (temp_ser=='j') Jet_Send();
      else if (temp_ser=='A') ACS_Cmd();
      else if (temp_ser=='c') ConvertSensorVals();
      else if (temp_ser=='d') DataFilter(1);
      else if (temp_ser=='G') GNC2(1);
      else if (temp_ser=='3') {GNC_init(); GNC3(1); }
      else if (temp_ser=='N') SPIaccelADC();
      else if (temp_ser=='S') break;
   }*/
	printf("\nBefore init avionics");
   InitAvionics();
	printf("\nAfter init avionics");
   while(1)
   {
		printf("\nBefore");
		IMU_Status_Report();
		ADC2();
		IMU_Status_Report();
		printf("\nAfter");
   }
   /*while(In_Flight)                 // endless loop constantly monitoring for new commands or data
   {
   }*/

   SystemShutDown();
} //end main

//===========================   End of Main  =====================================

void R_Stuff()
{
   int temp, i;
   char Send_Buf[30];
   i=0;
/*rmv*/   DS1led(ON);
   for (temp=0; temp<100; temp++)
   {
      sendMSG(Data_Port, "123456789\n", 10);
   }
/*rmv*/   DS1led(OFF);
   MSdelay(5);
/*rmv*/   DS1led(ON);
   for (temp=0; temp<100; temp++)
   {
      i=0;
      sprintf(Send_Buf,"abcdefghi\n!");
      while (Send_Buf[i]!='!')
      {
         sendByte(Data_Port,Send_Buf[i]);
         i++;
      }
   }
/*rmv*/   DS1led(OFF);
   MSdelay(5);
   sprintf(Send_Buf,"012345678\n!");
/*rmv*/   DS1led(ON);
   for (temp=0; temp<100; temp++)
   {
      i=0;
      while (Send_Buf[i]!='!')
      {
         sendByte(Data_Port,Send_Buf[i]);
         i++;
      }
   }

   /*
   sendMSG(Data_Port, "\nON", 3);
   while (1)
   {
//    temp=get_CMD(Data_Port);
      temp=get_CMD(Comms_Port);
      if(temp==1)
      {
         sendByte(Data_Port, cmd_BYTE);
      }
   } */
/*rmv*/   DS1led(OFF);
   return;
}

// =========================== Milli second Delays =====================
xmem void MSdelay(int n)
{
   unsigned long int timer, timer2;

   timer=MS_TIMER+(long)n;
   timer2=timer;
   while(MS_TIMER<timer)
   {
   }
}

//=================== Check Status of Macro Commands=====================//
nodebug root interrupt void checkMacro()
{
int n;
static char data[64];
static char BrdAddr,ExtAddr;

BrdAddr= 48;    // Not really used
ExtAddr= 12;

#asm
timerb_isr::
   push  af                   ; save registers
   push  hl

   ioi   ld a, (TBCSR)        ; load B1, B2 interrupt flags (clears flag)

   ld    hl, (count)
   inc   hl                   ; increment counter
   ld    (count), hl

   ld    a, 01h
   and   l                    ; mask off all but lowest bit of counter
   jr    z, match_0200
match_0000:
   ld    a, 80h               ; was 40h   (should be 0200h= 80h)
   ioi   ld (TBM1R), a        ; set up next B1 match (at timer=0200h)
   ld    a, 00h               ; 01h bit-mangled to for TBM1R layout
   ioi   ld (TBL1R), a        ; NOTE:  you _need_ to reload the match
                              ;  register after every interrupt!
   jr    done
match_0200:
   ld    a, 00h
   ioi   ld (TBM1R), a        ; set up next B1 match (at timer=0000h)
   ioi   ld (TBL1R), a        ; NOTE:  you _need_ to reload the match
done:
   ld    hl, PADR
   ioi   ld (hl), 0xFF        ; send a pulse to port A
   ioi   ld (hl), 0x00

   pop   hl                   ; restore registers
   pop   af
   ipres                      ; restore interrupts

#endasm
                              // Produces Interrupts
                              // 0000h   Interrupt
                              // 1000h
                              // 2000h   Interrupt
                              // 3000h

if(count>=globals.ISRTimer)         //== Timer used by Macro's
   {
   count=0;
/*rmv*///   InterruptXmsec();
   InterruptX2();
   }
}

//======================Initialize Serial Ports ===========================

xmem void initPort(int port, unsigned long baud)
{
int bits,parity;

if(0x08 & portCFG[port])bits=PARAM_7BIT;
else bits=PARAM_8BIT;

parity= 0x03& portCFG[port];
if(parity==0)parity=PARAM_NOPARITY;    // No parity is the default
if(parity==1)parity=PARAM_OPARITY;     // Odd parity
if(parity==2)parity=PARAM_EPARITY;     // Even Parity

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
      serDopen(19200);
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
      case 4:serEwrite(value,numBytes);break;
      case 5:serFwrite(value,numBytes);break;
      default:break;
      }
   }

return 1;
}

//=================== Send byte via serial port ===========================

char sendByte(int port, char value)
{
   switch(port)
      {
      case 0:serAputc(value);break;
      case 1:serBputc(value);break;
      case 2:serCputc(value);break;
      case 3:serDputc(value);break;
      case 4:serEputc(value);break;
      case 5:serFputc(value);break;
      default:break;
      }

return 1;
}
//=================== Get messages via serial port =======================

char getMSG(int port, char *value, char term )
{
int temp, n, time_cnt, past_sec;
char skip;

   n=0;
   skip='F';
   temp=0;
   time_cnt=0;
   value[0]='\0';                   // Start out with zero length string
   while(temp!=(int)term)           // Continue checking port until "term" character is received
      {
      switch(port)
         {
         case 0: temp= serAgetc(); break;
         case 1: temp= serBgetc(); break;
         case 2: temp= serCgetc(); break;
         case 3: temp= serDgetc(); break;
         case 4: temp= serEgetc(); break;
         case 5: temp= serFgetc(); break;
         default: return 0; break;
         }
      if(temp!=-1 && skip=='F')     // if a valid read and not a comment
         {
         if(temp=='/')skip='T';     // encountered comment skip everything until "term"
         else
            {
            value[n]=(char)temp;
            n++;
            }
         value[n]='\0';             // Terminate string where we are
         }
      tm_rd(&time);
      if(time.tm_sec!=past_sec)     // if seconds have incremented
         {
         past_sec=time.tm_sec;      // Save current seconds
         time_cnt++;                // Increment time_out counter
         }
      if(time_cnt>20)return 0;      // If after timeout, return 0
      }
   return 1;

}
//==================== Get byte via serial port ===========================

char getByte(int port, char *value)
{
int temp;
   switch(port)
      {
      case 0:temp=serAgetc();break;
      case 1:temp=serBgetc();break;
      case 2:temp=serCgetc();break;
      case 3:temp=serDgetc();break;
      case 4: temp= serEgetc(); break;
      case 5: temp= serFgetc(); break;
      default: temp=-1; break;
      }

   if(temp==-1)temp=0;              // return "zero" if no data
   else
      {
      *value=(char)temp;
      temp=1;
      }
return (char)temp;
}

//================================Initialize iDAC ============================
xmem void Init_Device()                // Initialize iDAC variables
{


char n;

   // ============= Set up default Configuration ====================
   userPort=                  0;       // Define serial port address for user interface
   Channel=                   0;       // Used for high speed data acquisition in ISR
   // ============== Read flash into global variables structure
   readUserBlock(&globals,0, (int)sizeof(TFlashProtected));

   memset(send_Data,1,4);

   //================ Initialize menus and formats for each port (Overwrite with command in Macro 38)

   memset(menu,'F',4);
   memset(display,'F',4);
   memset(PC,'F',4);
   memset(format,'B',4);
   memset(portCFG,0,4);


   cursor =            1;
   user_state=       'O';              // Initialize User interface flag,

   tm_rd(&thistime);                   // read the real time clock
                                       // Make sure clock is within range
   if((thistime.tm_year <100 || thistime.tm_year > 147) && processor=='R')
      {
      thistime.tm_hour = 23;
      thistime.tm_min = 59;
      thistime.tm_sec = 50;
      thistime.tm_mon = 1;
      thistime.tm_mday = 19;
      thistime.tm_year = 105;          // Equates to 2004
      thistime.tm_wday =4;             // ignored for Rabbit..computed by RTC
      tm_wr(&thistime);
      }

   // ===========set default baud rates for other serial ports
//	commented out 4/27 if ( globals.baud[0]!= 0)initPort(0,globals.baud[0]);          // Initialize Port 0 serial port
      if ( globals.baud[1]!= 0)initPort(1,globals.baud[1]);          // Initialize Port 1 serial port
      if(processor=='R')                                             // If processor is a Rabbit, initilize the other ports
         {
         if (globals.baud[2] != 0) initPort(2,globals.baud[2]);      // Initialize Port 2 serial port
         if (globals.baud[3] != 0) initPort(3,globals.baud[3]);      // Initialize Port 3 serial port
         }

   //============Increment our reboot counter ===============
      rebootCounter++;

   //============Setup TimerB interrupt service routine =====
      count=globals.ISRTimer-2;                    // Plan on early Interrupt to run Init Macro
      SetVectIntern(0x0B, checkMacro);             // Set up interrupt for timerB
      WrPortI(TBCR, &TBCRShadow, 0x09);            // clock timer B with (use 0x09 perclk/16) or (0x01 perclk/16)
                                                   // set interrupt level to 1
      WrPortI(TBL1R, NULL, 0x00);
      WrPortI(TBM1R, NULL, 0x00);                  // set initial match
//    WrPortI(TBCSR, &TBCSRShadow, 0x03);          // enable timer B and B1 match interrupts

   //=============== Init A2D ====================================
      initADC();

    //======= Overwrite SPI control..improve A2D performance ====
      MSdelay(100);                             // Let Macro 38 start before completing User Init

      WrPortI(0x24,NULL,0x84);         // Set up port A for output
      WrPortI(0x40,NULL,0x40);         // Enable Periphial power on version 1.4 motherboards
      WrPortI(0x55,NULL,0x55);         // Setup PortC for normal Serial port use (4 channels)

      WrPortI(PADR,NULL,0xc0);                  // Raise A7 to enable GPS receiver (reset)

}  // Initialize Device

//===================== Read real-time clock ==================================
xmem  int tm_rd(struct tm *time)
{
unsigned long t0;
t0 = read_rtc();
mktm(time, t0);
return 1;
}

//=============================Initialize Protected Variables========================
//
//   - These variables are stored in battery backed RAM
//   - They are initialized only when first programmed, or when requested by command
//   - Send command 255 to reset protected variables
//===================================================================================

xmem void initProt()
{
int n;

   first_time=    'F';           // Indicate this code has been executed before,


//=======Modem Configuration parameters=========================================
//=== Initialize Stored SOH Data ==============================================

   rebootCounter =   0;

//=== Init Flash with defaults if not yet initialized =======================

   readUserBlock(Lbuf,(int)&PTFlashBasePtr->initialized, 1);
   if (Lbuf[0] != 'T')
      {
      loadDefaults();
      writeToFlash();
      }
}
//=============== Initialize Analog Converter on RM3400 =============
xmem void initADC()
{
   char data[2];
   brdInit();
}

//==================== load defaults into globals ====================
xmem void loadDefaults(void)
{
   int n;

// ============= set all values to zero to save code space.
//               Uncomment the variable that you want to initialize to non-zero
   memset(globals.RTU_ID,0,4);         // ID used for RTU functions (most significant BYTE)

//== Memory allocation =======================================================
   globals.TX_SIZE=            32;     // Transmit buffer size (number of commands)
   globals.TX_LEN=            256;     // max command length
   globals.NUM_RECORDS=       256;     // Data Storage Memory allocation (number of records)
   globals.RECORD_SIZE=       32;      // record size (Sectors)

//=== Default Serial Port configurations ======================================
   globals.baud[0]=           57600;   // 57600 default baud rate
   globals.baud[1]=           57600;   // 57600 default baud rate
   globals.baud[2]=           38400;      // 1200  default baud rate
   globals.baud[3]=           57600;      // 1200  default baud rate

   globals.schedule=           'T';    //'F' Disables schedule functions

   globals.ISRTimer=      180;         // Was 144- Counter used by Interrupt Service Routine (default 18 Mhz Rabbit)
   globals.RetryTime=       2;         // Seconds until discarding TX buffered data
   globals.discard=         3;         // Number of attempts to deliver command before discarding (used by iGate)

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

///////////////////////////////////////////////////////////
// DS1 led on protoboard is controlled by port G bit 6
// turns on if state = 0
// turns off if state = 1
///////////////////////////////////////////////////////////
xmem void DS1led(int state)
{
   if (state == ON)
   {
#asm
   ld    a,(PFDRShadow)       ;use shadow register to keep other bit values
   res   DS1,a                ;clear bit 6 only
   ioi   ld (PFDR),a          ;write data to port
   ld    (PFDRShadow),a       ;update shadow register
#endasm
   }
   else
   {
#asm
   ld    a,(PFDRShadow)       ;use shadow register to keep other bit values
   set   DS1,a                ;set bit 6 only
   ioi   ld (PFDR),a          ;write data to port
   ld    (PFDRShadow),a       ;update shadow register
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
   ld    a,(PFDRShadow)       ;use shadow register to keep other bit values
   res   DS2,a                ;clear bit 6 only
   ioi   ld (PFDR),a          ;write data to port
   ld    (PFDRShadow),a       ;update shadow register
#endasm
   }
   else
   {
#asm
   ld    a,(PFDRShadow)       ;use shadow register to keep other bit values
   set   DS2,a                ;set bit 6 only
   ioi   ld (PFDR),a          ;write data to port
   ld    (PFDRShadow),a       ;update shadow register
#endasm
   }
}


xmem void ACS_Test()
{
   char ACS_Cmd_Buf[30];
   int i,j;

   int Enter1, Enter2;

   Enter1=0x0d; Enter2=0x0a;
}

// GNC Rev 1.2
// Updated on 4/13/07 - Aditi/Cassandra
// updates : Emergency landing works for constant GNCJetrpm of 43000;
//           Altimeter in ; Avionics will give command to GNC depending on Altimeter reading to go in Emer_Land
//           Transformations for IMU

xmem void GNC_init()
{
   rowmax=91;
   nfiretotal=0;
   Ad_time=0.0;
// Initiate variables to zero
   facsx = facsy = facsz = facsphi = facspsi = facstheta = fxdes = fydes
   = fzdes = fphides = fpsides = fthetades = 0.0;
   xmeas = ymeas = zmeas = xdmeas = ydmeas = zdmeas = xddmeas = yddmeas = zddmeas = 0.0;
   phimeas = thetameas = psimeas = phidmeas = psidmeas = thetadmeas = 0.0;
   GNCJetrpm=0.0;
   facs[0]=facs[1]=facs[2]=facs[3]=facs[4]=facs[5]=facs[6]=facs[7]=facs[8]=facs[9]=facs[10]=facs[11]=0;
//GNC3
   X_force = Y_force = Z_force = Phi_force = Theta_force = Psi_force=0;
   vx_des= vy_des= vz_des= vx_deso= vy_deso= vz_deso= xdeso= ydeso= zdeso=0;
   vphi_des= vtheta_des= vpsi_des= vphi_deso= vtheta_deso= vpsi_deso= phideso= thetadeso= psideso=0;
   vx_err= vy_err= vz_err= vphi_err= vtheta_err= vpsi_err=0;

   xdes = ydes = zdes = phides = thetades = psides = fxdes = fydes =
      fzdes = fphides = fpsides = fthetades = 0;
   facsx = facsy = facsz = fth = facsphi = facspsi = facstheta = facsthruster = 0;
   x_integral = y_integral = z_integral = xerroro = yerroro = zerroro = 0;
   x_meas_integral = y_meas_integral = z_meas_integral = 0;
   x_des_integral = y_des_integral = z_des_integral = 0;
   xerrormeas = yerrormeas = zerrormeas = xerr = yerr = zerr = phierr =
      psierr = thetaerr = 0;
   kp_x = kp_y = kp_z = kd_vx = kd_vy = kd_vz = ki_x = ki_y = ki_z = 0;
   kp_phi = kp_theta = kp_psi = kd_phi = kd_theta = kd_psi = 0;
   totaltime = 0;

   phiband = psiband = thetaband = 0;
   ixo = ixf = iyo = iyf = izo = izf =alpha = 0;
   xmeaso = ymeaso = zmeaso = phimeaso = psimeaso = thetameaso = 0;
   xdmeaso = ydmeaso = zdmeaso = 0;
   x_err_integral = y_err_integral = z_err_integral = 0;
   phidmeaso = thetadmeaso = psidmeaso = 0;
   fthprev = maxrate = engrate = mfuelburn = fthmax = 0;
   mass = mcoo = mdry = mfuel = mcooburn = tremain = 0;
   tlandsub = 0;

   // gains that work !!

   kp_x = 5.55757650012304;   //kp_x=1.00000000000000;
   kd_vx = 14.94332827735925; //kd_vx=6.40312423743284;
   ki_x = -1.00000000000005;  //INTEGRAL CONTROL
   kp_y = 5.55757650012280;
   kd_vy = 14.94332827735925;
   ki_y = -0.99999999999999;  //INTEGRAL CONTROL
   kp_z = 3.52327742832395;
   kd_vz = 8.42410520865447;
   ki_z = -0.4;      //ws =-0.53746;    //INTEGRAL CONTROL  was =-0.70710678118655
   kp_phi = 1.00000000000000;
   kd_phi = 8.32204800152180;
   kp_theta = 1.00000000000000;
   kd_theta = 8.29162686783999;
   kp_psi = 1.00000000000001;
   kd_psi = 11.37725548000127;
   amp = 10.0;    //group amplifier gain  WAS 10
   //ampr = 10.0;
   ampz = 1000.0;    //z amplifier gain  WAS 100

// constants depending on input profiles

#ifdef profile0
 totaltime = 188;
 m = 95;
#endif

#ifdef profile2
 totaltime = 62;
 m = 62;
#endif

#ifdef profile4
 totaltime = 64;
 m = 64;
#endif

#ifdef profile5
 totaltime = 182;
 m = 182;
#endif


//Computational constants
   xband = 20;       //x, y band in N   WAS 100
   yband = 20;
   phiband = 8;//phi, theta, psi band in N WAS 2
   thetaband = 8; // set for firing thrusters at 0.1 rad/sec
   psiband = 8;

   zdist_cp_to_cg = .039;     //distance from cp to cg on z axis in m, distance when fueled
   dcgzoffset = 0.019;  //distance between the CG and the center of the lateral ACS thruster in m (when fueled)

//Interpolator initial conditions-need to find a way work this out
   //pron = 2;
   m = 0;
   pta = T[0];
   ptb = T[1];

   ixo = X[m];
   iyo = Y[m];
   izo = Z[m];
   ixf = X[m+1];
   iyf = Y[m+1];
   izf = Z[m+1];
}

int GNC3(int AbrtOrder)
{
   char send_Buffer[70];
   int j;

/*Rmv*/ //  DS1led(ON);
// sendMSG(Data_Port, "\n\rGNC3", 6);
// Get the measured values
   xddmeas = sensor_filtrd[0];//sensor_filtrd[0]*0.001;
   yddmeas = sensor_filtrd[1]*0.001;
   zddmeas = sensor_filtrd[2]*0.001;

   phidmeas = sensor_filtrd[3]*0.001;
   thetadmeas = sensor_filtrd[4]*0.001;
   psidmeas = sensor_filtrd[5];//*0.001;

// Find Velocity and Displacement
   xdmeas = xdmeas + xddmeas*dT;
   ydmeas = ydmeas + yddmeas*dT;
   zdmeas = zdmeas + zddmeas*dT;

   xmeas = xmeaso + xdmeas*dT + xddmeas*dTdT2;
   ymeas = ymeaso + ydmeas*dT + yddmeas*dTdT2;
   zmeas = zmeaso + zdmeas*dT + zddmeas*dTdT2;

// Calculate integrals of measured
   x_meas_integral = x_meas_integral + xmeaso*dT + xdmeaso*dTdT2 + xddmeas*dTdTdT6;
   y_meas_integral = y_meas_integral + ymeaso*dT + ydmeaso*dTdT2 + yddmeas*dTdTdT6;
   z_meas_integral = z_meas_integral + zmeaso*dT + zdmeaso*dTdT2 + zddmeas*dTdTdT6;

// Find the Angles
   phimeas = phimeas + phidmeas*dT;
   thetameas = thetameas + thetadmeas*dT;
   psimeas = psimeas + psidmeas*dT;

// Calculating Interpolated desired values

   Ad_time = Ad_time + dT;
   if (m+2<= rowmax)
   {
      if (Ad_time >= ptb)
      {
         m = m + 1;
         pta = T[m];      //ptb;
         ixo = X[m];      //ixf;
         iyo = Y[m];      //iyf;
         izo = Z[m];      //izf;

         ptb = T[m+1];
         ixf = X[m+1];
         iyf = Y[m+1];
         izf = Z[m+1];
      }
      alpha = (ixf-ixo) / (ptb - pta);
      xdes = ixo + alpha * (ixf - ixo);
      ydes = iyo + alpha * (iyf - iyo);
      zdes = izo + alpha * (izf - izo);
   }

// Calculate errors in the 6-dof
   xerr=xdes-xmeas;
   yerr=ydes-ymeas;
   zerr=zdes-zmeas;
   phierr=phides-phimeas;
   thetaerr=thetades-thetameas;
   psierr=psides-psimeas;

// Calculate derivative of desired    note: dT_2 is 2/dT
   vx_des = dT_2*(xdes-xdeso) - vx_deso;
   vy_des = dT_2*(ydes-ydeso) - vy_deso;
   vz_des = dT_2*(zdes-zdeso) - vz_deso;
   vphi_des = dT_2*(phides-phideso) - phideso;
   vtheta_des = dT_2*(thetades-thetadeso) - thetadeso;
   vpsi_des = dT_2*(psides-psideso) - psideso;

// Calculate derivative of errors
   vx_err = vx_des - xdmeas;
   vy_err = vy_des - ydmeas;
   vz_err = vz_des - zdmeas;
   vphi_err = vphi_des - phidmeas;
   vtheta_err = vtheta_des - thetadmeas;
   vpsi_err = vpsi_des - psidmeas;

// Calculate integrals of desired
   x_des_integral = x_des_integral + xdeso*dT + vx_deso*dTdT2 + dT3*(xdes-xdeso-vx_deso*dT);
   y_des_integral = y_des_integral + ydeso*dT + vy_deso*dTdT2 + dT3*(ydes-ydeso-vy_deso*dT);
   z_des_integral = z_des_integral + zdeso*dT + vz_deso*dTdT2 + dT3*(zdes-zdeso-vz_deso*dT);

// Calculate integral of errors
   x_err_integral = x_des_integral - x_meas_integral;
   y_err_integral = y_des_integral - y_meas_integral;
   z_err_integral = z_des_integral - z_meas_integral;

// Save these values for later
   xmeaso=xmeas;  xdmeaso=xdmeas;      xdeso=xdes;
   ymeaso=ymeas;  ydmeaso=ydmeas;      ydeso=ydes;
   zmeaso=zmeas;  zdmeaso=zdmeas;      zdeso=zdes;
   phimeaso=phimeas; phidmeaso=phidmeas;     phideso=phides;
   thetameaso=thetameas;   thetadmeaso=thetadmeas;    thetadeso=thetades;
   psimeaso=psimeas; psidmeaso=psidmeas;     psideso=psides;

//Apply PID gains
   X_force = kp_x * xerr + ki_x * x_err_integral + kd_vx * vx_err;
   Y_force = kp_y * yerr + ki_y * y_err_integral + kd_vy * vy_err;
   Z_force = kp_z * zerr + ki_z * z_err_integral + kd_vz * vz_err;
   Phi_force = kp_phi * phierr + kd_phi * vphi_err;
   Theta_force = kp_theta * thetaerr + kd_theta * vtheta_err;
   Psi_force = kp_psi * psierr + kd_psi * vpsi_err;

// Calculate force variables in the 6-dof
   fxdes = amp * X_force;
   fydes = amp * Y_force;
   fzdes = ampz * Z_force;
   fphides = amp * Phi_force;
   fthetades = amp * Theta_force;
   fpsides = amp * Psi_force;

//jtb dump   time, xmeas, ymeas, psimeas
/* sprintf(send_Buffer, "%.2f,%.3f,%.3f\n\r!", Ad_time, psidmeas, psimeas);
   j=0;
   while (send_Buffer[j]!='!')
   {
      sendByte(Data_Port, send_Buffer[j]);
      j++;
   }  */
 /*   sprintf(send_Buffer, "%.3f,%.3f,%.3f!", yddmeas, ydmeas, ymeas);
   j=0;
   while (send_Buffer[j]!='!')
   {
      sendByte(Data_Port, send_Buffer[j]);
      j++;
   }
   sprintf(send_Buffer, ",%.3f,%.3f,%.3f,\n\r!", zddmeas, zdmeas, zmeas);
   j=0;
   while (send_Buffer[j]!='!')
   {
      sendByte(Data_Port, send_Buffer[j]);
      j++;
   } */

/*Rmv*/  // DS1led(OFF);
TelemetryCount++;
   return 0;
}

int GNC2(int AbortRcvd)
{
   char send_Buffer[30];
   //Command thrusters  with bang bang CORRECT FOR TIME ON!!!

/*Rmv*///   DS1led(ON);
   //turn OFF thrusters if position ok
      if (fxdes <= xband)
         { if (fxdes >= -xband)
           {
            facsx = 0;  //for logging only
            facs[3] = 0;
            facs[11] = 0;
            facs[5] = 0;
            facs[9] = 0;
         }};
      if (fydes <= yband)
         { if (fydes >= -yband)
          {
            facsy = 0;  //for logging only
            facs[2] = 0;
            facs[6] = 0;
            facs[0] = 0;
            facs[8] = 0;
         }};
      if (fphides < phiband)
         { if (fphides > -phiband)
          {
            facsphi = 0;   //for logging only
            facs[10] = 0;
            facs[4] = 0;
         }};
      if (fthetades < thetaband)
         { if (fthetades > -thetaband)
          {
            facstheta = 0; //for logging only
            facs[1] = 0;
            facs[7] = 0;
         }};
      if (fpsides < psiband)
         { if (fpsides > -psiband)
          {
            facspsi = 0;   //for logging only
            facs[2] = 0;
            facs[5] = 0;
            facs[8] = 0;
            facs[11] = 0;
            facs[0] = 0;
            facs[3] = 0;
            facs[6] = 0;
            facs[9] = 0;
         }};

      // turn ON thrusters if position not ok
      if (fxdes >= xband)
         {
            facsx = facsthruster * 2;  //for logging only
            facs[5] = 1;
            facs[9] = 1;
            facs[3] = 0;
            facs[11] = 0;
            nfiretotal = nfiretotal + 2;
         };
      if (fxdes <= -xband)
         {
            facsx = -facsthruster * 2; //for logging only
            facs[3] = 1;
            facs[11] = 1;
            facs[5] = 0;
            facs[9] = 0;
            nfiretotal = nfiretotal + 2;
         };
   //facsy=fydes;
      if (fydes >= yband)
         {
            facsy = facsthruster * 2;  //for logging only
            facs[0] = 0;
            facs[8] = 0;
            facs[2] = 1;
            facs[6] = 1;
            nfiretotal = nfiretotal + 2;
         };
      if (fydes <= -yband)
         {
            facsy = -facsthruster * 2; //for logging only
            facs[2] = 0;
            facs[6] = 0;
            facs[0] = 1;
            facs[8] = 1;
            nfiretotal = nfiretotal + 2;
         };

   //facsphi=fphides;
      if (fphides >= phiband)
         {
            facsphi = facsthruster; //for logging only
            facs[4] = 1;
            facs[10] = 0;
            nfiretotal = nfiretotal + 1;
         };
      if (fphides <= -phiband)
         {
            facsphi = -facsthruster;   //for logging only
            facs[10] = 1;
            facs[4] = 0;
            nfiretotal = nfiretotal + 1;
         };

   //facstheta=fthetades;
      if (fthetades >= thetaband)
         {
            facstheta = facsthruster;  //for logging only
            facs[7] = 0;
            facs[1] = 1;
            nfiretotal = nfiretotal + 1;
         };
      if (fthetades <= -thetaband)
         {
            facstheta = -facsthruster; //for logging only
            facs[1] = 0;
            facs[7] = 1;
            nfiretotal = nfiretotal + 1;
         };

   //facspsi=fpsides;
      if (fpsides >= psiband)
         {
            facspsi = facsthruster * 4;   //for logging only
            facs[0] = 0;
            facs[3] = 0;
            facs[6] = 0;
            facs[9] = 0;
            facs[2] = 1;
            facs[5] = 1;
            facs[8] = 1;
            facs[11] = 1;
            nfiretotal = nfiretotal + 4;
         };
      if (fpsides <= -psiband)
         {
            facspsi = -facsthruster * 2;  //for logging only
            facs[2] = 0;
            facs[5] = 0;
            facs[8] = 0;
            facs[11] = 0;
            facs[0] = 1;
            facs[3] = 1;
            facs[6] = 1;
            facs[9] = 1;
            nfiretotal = nfiretotal + 4;
         };

   facsz = (facs[1] + facs[4] + facs[7] + facs[10]) * facsthruster;  //for logging only

   nfire = 0;     //ADJUST TO EQUAL NUMBER OF ACS FIRED

   //Command engine

   if (fzdes < 0.0)
      fth = fthprev + (fthprev-fzdes); //not rate limited yet, removed fthstepper to augment integrator
   if (fzdes > 0.0)
      fth = fthprev + (fthprev+fzdes);
   if (fzdes = 0.0)
      fth = fthprev;
   if (fth >= fthmax)
      fth = fthmax;
   if (fth <= 0.0)
      fth = 5.08;
   if (AbortCmdRcvd = 1)
      fth = 1107.30075;      // set GNCrpm to 43000 - corr. fth is as said.
   if (thetameas >= 0.2)
      fth = 1107.30075;      // set GNCrpm to 43000 - in case stall angle reached   - angle at 12 degress
   if (phimeas >= 0.2)
      fth = 1107.30075;      // set GNCrpm to 43000 - in case stall angle reached   - angle at 12 degrees

   fthprev = fth;     //step param

  GNCJetrpm=30317*(log(fth))-49293;
//GNCJetrpm=100000;
   //mfuelburn = ((0.00004) * fth + 0.002) / Ad_time;        //Real engine model for burned fuel in kg/s, ADJUST FOR RATE!!!
   mfuelburn = (1446*fth + 15624.002)/dT ;// mfuelburn = (1000*60*(0.0241*fth  + 0.2604) + 0.002)/dT ; // from the flow of mass/min Vs Thrust & rpm data - here time factor is 50 ms
   mfuel = mfuel - mfuelburn;

   //Reality   REMOVE BEFORE FLIGHT???

   if (mfuel <= 0.0)
         {
          mfuel = 0;
          //fth = 0.0;
          GNCJetrpm=0.0;
         };

   mcooburn = nfire * 0.00084646;   //ADJUST FOR AMOUNT OF ACS GAS USED PER FIRING
   mcoo = mcoo - mcooburn;
   if (mcoo <= 0.0)
      {
       mcoo = 0;
       facs[9] = 0;
       facs[5] = 0;
       facs[11] = 0;
       facs[3] = 0;
       facs[0] = 0;
       facs[8] = 0;
       facs[2] = 0;
       facs[6] = 0;
       facs[4] = 0;
       facs[10] = 0;
       facs[7] = 0;
       facs[1] = 0;
       GNCJetrpm = 0.0;
      };
                                              /*temporarily removed
      // For emergency landing
      tremain = (mfuel/ ((0.0241 * fthmax + 0.2604))- dT );    //time of flight remaining based on fuel left
      tlandsub = (-2 * zmeas) / (vmaxland + zd);               //sub calc for time required to land
      if (tlandsub > 0)
         {
           tland = sqrt(tlandsub);                             //time required to land in s
           fthmin = mass * (grav + (vmaxland - zd) / tland);   //min fth required to land and max landing velocity (in N)
          }
       else
          {
           tland = 0;
           fthmin = 0;
          };
      if (tremain < (totaltime - Ad_time))                     // if TOF is less than time left to land depending on fuel
          AbortCmdRcvd = 1;

      if (AbortCmdRcvd == 1)
        {
         xdes = xmeas;
         ydes = ymeas;
         //xdes = xfreeze;
         //ydes = yfreeze;
         phides = 0;
         thetades = 0;
         psides = 0;
         GNCJetrpm = 43000 ;         // reduce the rpm by 10%
        };

      if (AbortCmdRcvd == 0)
      {
         xfreeze = xmeas;
         yfreeze = ymeas;
      };
                                            */
   // Adjust MOI/dist CP to CG   CORRECT CG TO CHANGE WITH FUEL BURNED!!!

   mass = mdry + mcoo + mfuel;   //total mass

   // CORRECT MOI TO CHANGE WITH FUEL BURNED!!!
   //Ix=((1/12)*mass*((3*((0.381)^2))+((0.357)^2)))+(20*((0.357/2)^2));
   //Iy=((1/12)*mass*((3*((0.381)^2))+((0.357)^2)))+(20*((0.357/2)^2));
   //Iz=(.5)*mass*((.381)^2);

// Ix = 13.00286;
// Iy = 12.90658;
// Iz = 24.46819;

   //////>>>>>  REMOVE BEFORE FLIGHT  <<<<<//////

   //sprintf(send_Buffer, "[GNCa=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]", facs[11], facs[10], facs[9], facs[8], facs[7], facs[6], facs[5], facs[4], facs[3], facs[2], facs[1], facs[0]);
   //sendMSG(Data_Port, send_Buffer, 30);

   //sprintf(send_Buffer, "[GNCr=%f]    ", GNCJetrpm);
   //sendMSG(Data_Port, send_Buffer, 12);

/*Rmv*/  // DS1led(OFF);
   return 0;
};
//////  END FLIGHT LOOP  //////

xmem int InitAvionics()
{
   int i, j;
//   char Status_Buffer[60];
   //Initialize Varaibles
   TelemetryCount=1;
   GNCLoopCount=0;

   //Initialize the ACS System
   facs[0]=0;
   facs[1]=0;
   facs[2]=0;
   facs[3]=0;
   facs[4]=0;
   facs[5]=0;
   facs[6]=0;
   facs[7]=0;
   facs[8]=0;
   facs[9]=0;
   facs[10]=0;
   facs[11]=0;
   ACS_Cmd();

   //Initialize the Jet Engine
   /*sendMSG(Jet_Port, "1,TCO,1", 7);
   sendByte(Jet_Port, 0x0a);
   sendByte(Jet_Port, 0x0d);*/	//Commented out because jet connections are not existent

   GNC_init();

   //Enabling Mission - will stay 1 until GNC says so or we shutdown
   In_Flight=1;

   // Putting initializations for the SPI function
   for (j=0; j<32766; j++); //let ADIS go through reset sequence
   {
   }
   // PE4,5,6=>phi,theta,psi  PE7=>accel chip selects
   BitWrPortI(PEDR, &PEDRShadow, 1, 4);	// prewrite chip select high
   BitWrPortI (PEDDR, &PBDDRShadow, 1, 4);  //set PE7 to output
   BitWrPortI(PEDR, &PEDRShadow, 1, 5);	// prewrite chip select high
   BitWrPortI (PEDDR, &PBDDRShadow, 1, 5);  //set PE7 to output
   BitWrPortI(PEDR, &PEDRShadow, 1, 6);	// prewrite chip select high
   BitWrPortI (PEDDR, &PBDDRShadow, 1, 6);  //set PE7 to output
   BitWrPortI(PEDR, &PEDRShadow, 1, 7);	// prewrite chip select high
   BitWrPortI (PEDDR, &PBDDRShadow, 1, 7);  //set PE7 to output

   SPIinit();
// for accel
   BitWrPortI(PEDR, &PEDRShadow, 0, 7);	// chip select low
 	SPIWrite( "\x20\x83", 2); // write ctrl reg 1
   BitWrPortI(PEDR, &PEDRShadow, 1, 7);	// chip select high
      for (j=0; j<100; j++){}
 	BitWrPortI(PEDR, &PEDRShadow, 0, 7);	// chip select low
 	SPIWrite( "\x21\x40", 2); // write ctrl reg 2
   BitWrPortI(PEDR, &PEDRShadow, 1, 7);	// chip select high
      for (j=0; j<100; j++){}

// for gyro
//write sensitivity and filter taps, in that order
   BitWrPortI(PEDR, &PEDRShadow, 0, 6);	// chip select low
   SPIWrite( "\xb9\x01", 2); // write data at x39, 80 deg/sec
      for (j=0; j<1; j++ ){}
   BitWrPortI(PEDR, &PEDRShadow, 1, 6);	// chip select high
      for (j=0; j<10; j++){}
   BitWrPortI(PEDR, &PEDRShadow, 0, 6);	// chip select low
   SPIWrite( "\xb8\x0f", 2); // write data at x38, filter taps 16 (x04) (min), new filter taps (x0f)
      for (j=0; j<1; j++ ){}
   BitWrPortI(PEDR, &PEDRShadow, 1, 6);	// chip select high

      // write an autonull and DAC LATCH command, requires 50ms to complete
   BitWrPortI(PEDR, &PEDRShadow, 0, 6);	// chip select low
   // should only need to write to 3e
   SPIWrite( "\xbf\x00", 2); // write data at x3f, 00 don't care
      for (j=0; j<1; j++ ){}
   BitWrPortI(PEDR, &PEDRShadow, 1, 6);	// chip select high
      for (j=0; j<10; j++){}
   BitWrPortI(PEDR, &PEDRShadow, 0, 6);	// chip select low
   SPIWrite( "\xbe\x05", 2); // write data at x3e, set lsb and 4
      for (j=0; j<1; j++ ){}
   BitWrPortI(PEDR, &PEDRShadow, 1, 6);	// chip select high
   	// waste 50 ms
   	for (j=0; j<32500; j++){
      	for (i=0; i<10; i++){}
      }

//For Phi
sendByte(Data_Port, 'H');
   BitWrPortI(PEDR, &PEDRShadow, 0, 4);	// chip select low
   SPIWrite( "\xb9\x01", 2); // write data at x39, 80 deg/sec
      for (j=0; j<1; j++ ){}
   BitWrPortI(PEDR, &PEDRShadow, 1, 4);	// chip select high
      for (j=0; j<10; j++){}
   BitWrPortI(PEDR, &PEDRShadow, 0, 4);	// chip select low
   SPIWrite( "\xb8\x0f", 2); // write data at x38, filter taps 16 (x04) (min), new filter taps (x0f)
      for (j=0; j<1; j++ ){}
   BitWrPortI(PEDR, &PEDRShadow, 1, 4);	// chip select high

      // write an autonull and DAC LATCH command, requires 50ms to complete
   BitWrPortI(PEDR, &PEDRShadow, 0, 4);	// chip select low
   // should only need to write to 3e
   SPIWrite( "\xbf\x00", 2); // write data at x3f, 00 don't care
      for (j=0; j<1; j++ ){}
   BitWrPortI(PEDR, &PEDRShadow, 1, 4);	// chip select high
      for (j=0; j<10; j++){}
   BitWrPortI(PEDR, &PEDRShadow, 0, 4);	// chip select low
   SPIWrite( "\xbe\x05", 2); // write data at x3e, set lsb and 4
      for (j=0; j<1; j++ ){}
   BitWrPortI(PEDR, &PEDRShadow, 1, 4);	// chip select high
   	// waste 50 ms
   	for (j=0; j<32500; j++){
      	for (i=0; i<10; i++){}
      }
//End Phi
//For Theta
sendByte(Data_Port, 'T');
   BitWrPortI(PEDR, &PEDRShadow, 0, 5);	// chip select low
   SPIWrite( "\xb9\x01", 2); // write data at x39, 80 deg/sec
      for (j=0; j<1; j++ ){}
   BitWrPortI(PEDR, &PEDRShadow, 1, 5);	// chip select high
      for (j=0; j<10; j++){}
   BitWrPortI(PEDR, &PEDRShadow, 0, 5);	// chip select low
   SPIWrite( "\xb8\x0f", 2); // write data at x38, filter taps 16 (x04) (min), new filter taps (x0f)
      for (j=0; j<1; j++ ){}
   BitWrPortI(PEDR, &PEDRShadow, 1, 5);	// chip select high

      // write an autonull and DAC LATCH command, requires 50ms to complete
   BitWrPortI(PEDR, &PEDRShadow, 0, 5);	// chip select low
   // should only ne`ed to write to 3e
   SPIWrite( "\xbf\x00", 2); // write data at x3f, 00 don't care
      for (j=0; j<1; j++ ){}
   BitWrPortI(PEDR, &PEDRShadow, 1, 5);	// chip select high
      for (j=0; j<10; j++){}
   BitWrPortI(PEDR, &PEDRShadow, 0, 5);	// chip select low
   SPIWrite( "\xbe\x05", 2); // write data at x3e, set lsb and 4
      for (j=0; j<1; j++ ){}
   BitWrPortI(PEDR, &PEDRShadow, 1, 5);	// chip select high
   	// waste 50 ms
   	for (j=0; j<32500; j++){
      	for (i=0; i<10; i++){}
      }
//End Theta
// End SPI initialization

// Lets heat up the gyro
/*for (i=0; i<4000; i++)
   {
      ADC2();
      MSdelay(5);
   } */
// Setting sensor_offsets
for (i=0; i<=20; i++)
   {
      ADC(i);
      MSdelay(5);
   }
   DataFilter(21);

SPIaccelADC();
sensor_filtrd[0] = adc_sample_x;
sensor_filtrd[5] = adc_sample_gyro;
  for (i=0; i<=5; i++)      //put the initial filtered values as the offsets
  {
      sensor_offsets[i]=sensor_filtrd[i];
  }
  sensor_offsets[6]= 0.0;

// Initialize the Data Queue with values
   for (i=0; i<FilterDepth; i++)
   {
      SPIaccelADC();
   }
//Initialize the Timer Interrupt
   globals.ISRTimer = 180;
   WrPortI(TBCSR, &TBCSRShadow, 0x03);          // enable timer B and B1 match interrupts

   return 1;
}

void Jet_Send()
{
   char buffer[50];
   char *Jet_Cmd, *Jet_Addr;
   int i;
   int Enter1, Enter2;
   char Jet_Params[10];

/*Rmv*/  // DS1led(ON);
   i=0;
// sendMSG(Data_Port, "\n\rJet_Send", 10);

   if (GNCJetrpm>114000)   GNCJetrpm=114000;
   else if (GNCJetrpm<33000)  GNCJetrpm=33000;
/* sendMSG(Jet_Port, "1,WRP,", 6);
   sprintf(buffer, "%.0f@", Jet_Params);
   while (buffer[i]!='@')
   {
      sendByte(Jet_Port,buffer[i]);
      sendByte(Data_Port,buffer[i]);
      i=i+1;
   }
   sendMSG(Jet_Port, "\n\r", 2);
/*Rmv*/  // DS1led(OFF);
   return;
}

void ADC(int iteration_Number)
{
   unsigned long rawdata;
   unsigned int cmd, channel;
// sendMSG(Data_Port, "\n\rADC", 5);

/*Rmv*/ //  DS1led(ON);
   for(channel = STARTCHAN; channel <= ENDCHAN; channel++)
   {
      cmd = 0x80|(GAINSET*16+(channel|0x08));   //convert channel and gain to ADS7870 format in a direct mode
      rawdata = (long) anaInDriver(cmd, 12);    // execute low level A/D driver
      sensor_read[channel][iteration_Number] = (rawdata *0.001); // convert the averaged samples to a voltage

      if (sensor_read[channel][iteration_Number]>10 | sensor_read[channel][iteration_Number]<0.1) sensor_read[channel][iteration_Number]=1.6;//sensor_offsets[channel];

   }
/*Rmv*/ //  DS1led(OFF);
}

xmem void IMU_Status_Report()
{
   char Status_Buf[30];
   int i, j, channel;
   sendMSG(Data_Port, "\n\rIMU_Status", 12);

   ADC(0);

   for (i=0; i<=ENDCHAN; i++)
   {
      sprintf(Status_Buf, "\n\r%f!", sensor_read[i][0]);
      //if(i ==6){printf("\npsi\t%f", sensor_read[i][0]);}				//********************************
	  //if(i ==5){printf("\ntheta\t%f", sensor_filtrd[i]);}
	  //if(i ==4){printf("\nphi\t%f", sensor_filtrd[i]);}
      j=0;
      while (Status_Buf[j]!='!')
      {
         sendByte(Data_Port, Status_Buf[j]);
         j++;
      }
   }
   DataFilter(1);
   for (i=0; i<=ENDCHAN; i++)
   {
      sprintf(Status_Buf, "\n\r%f!", sensor_filtrd[i]);
	  if(i ==6){printf("\npsi\t%f", sensor_filtrd[i]);}				//********************************
	  //if(i ==5){printf("\ntheta\t%f", sensor_filtrd[i]);}
	  //if(i ==4){printf("\nphi\t%f", sensor_filtrd[i]);}
      j=0;
      while (Status_Buf[j]!='!')
      {
         sendByte(Data_Port, Status_Buf[j]);
         j++;
      }
   }
   ConvertSensorVals();
}

void ACS_Cmd()
{
   char ACS_buffer[21], ACS_datach[3];
   int ACS_data[3], i;
   int Enter1, Enter2;

/*Rmv*/  // DS1led(ON);

   Enter1=0x0d; Enter2=0x0a;

// sendMSG(Data_Port, "\n\rACS_cmd", 9);

   for (i=0; i<12; i++)
   {
/*rmv*/     facs[i]=i%2;

      if ((facs[i]==1)||(facs[i]==0))
      {
//       sendByte(Data_Port, (facs[i]+48));
      }
      else
      {
         facs[i]=0;
//         sendByte(Data_Port, 'Z');
      }
//    sendMSG(Data_Port, ACS_buffer, 7);
   }
// Test to see if opposite valves fire at same time, set to 0 if true
   if (facs[ZPLUSX]==facs[ZMINUSX])
   {
      facs[ZPLUSX]=facs[ZMINUSX]=0;
   }
   if (facs[ZPLUSY]==facs[ZMINUSY])
   {
      facs[ZPLUSY]=facs[ZMINUSY]=0;
   }
   if (facs[0]==facs[2])
   {
      facs[0]=facs[2]=0;
   }
   if (facs[3]==facs[5])
   {
      facs[3]=facs[5]=0;
   }
   if (facs[6]==facs[8])
   {
      facs[6]=facs[8]=0;
   }
   if (facs[9]==facs[11])
   {
      facs[9]=facs[11]=0;
   }

   i=0;
   ACS_data[0]=((facs[3]<<3)&0x0f)|((facs[2]<<2)&0x0f)|((facs[1]<<1)&0x0f)|(facs[0]&0x0f);
   ACS_data[1]=((facs[7]<<3)&0x0f)|((facs[6]<<2)&0x0f)|((facs[5]<<1)&0x0f)|(facs[4]&0x0f);
   ACS_data[2]=((facs[11]<<3)&0x0f)|((facs[10]<<2)&0x0f)|((facs[9]<<1)&0x0f)|(facs[8]&0x0f);

/*   if ((ACS_data[0]<10) & (ACS_data[1]<10) & (ACS_data[2]<10))
   {
      sprintf(ACS_buffer,"%d%d%d!", ACS_data[2], ACS_data[1], ACS_data[0]);
   }
   else
   {
      if (ACS_data[0]<10)
      {
         ACS_datach[0]=(char)(48+ACS_data[0]);
      }
      else
      {
         ACS_datach[0]=(char)(55+ACS_data[0]);
      }
      if (ACS_data[1]<10)
      {
         ACS_datach[1]=(char)(48+ACS_data[1]);
      }
      else
      {
         ACS_datach[1]=(char)(55+ACS_data[1]);
      }
      if (ACS_data[2]<10)
      {
         ACS_datach[2]=(char)(48+ACS_data[2]);
      }
      else
      {
         ACS_datach[2]=(char)(55+ACS_data[2]);
      }
      sprintf(ACS_buffer,"%c%c%c!", ACS_datach[2], ACS_datach[1], ACS_datach[0]);
   }

   sendMSG(Data_Port, "@01WVS02010", 11);
   while (ACS_buffer[i]!='!')
   {
      sendByte(ACS_Port,ACS_buffer[i]);
      sendByte(Data_Port,ACS_buffer[i]);
      i=i+1;
   }
   sendMSG(Data_Port, "00*\n\r", 5);
/*Rmv*/  // DS1led(OFF);
}

void ConvertSensorVals()      //Works on Filtered data
                              //Caters for Ratio Errors, constant Offsets and Unit Conversion
{
   int i, j;
   char Status_Buf[30];
/*Rmv*/  // DS1led(ON);
// sendMSG(Data_Port, "\n\rCnvrtSnsr", 11);
// sendMSG(Data_Port, "  ", 2);

   for (i=0; i<=ENDCHAN; i++)
   {
      sensor_filtrd[i]=(sensor_filtrd[i]-sensor_offsets[i])*sensor_UnitConvert[i];
/*    sprintf(Status_Buf, "%.3f, %.3f,!", sensor_filtrd[i], sensor_UnitConvert[i]);
      j=0;
      while (Status_Buf[j]!='!')
      {
         sendByte(Data_Port, Status_Buf[j]);
         j++;
      }    */
//    sendMSG(Data_Port, Status_Buf, 7);
   }
/*Rmv*/ //  DS1led(OFF);
}

void DataFilter(int NumofSamples)
{
   int i, j, k;
   int sensor_err_flag;
   sensor_err_flag=0;
/*Rmv*///   DS1led(ON);
//   char Status_Buf[30];
// sendMSG(Data_Port, "\n\rDataFilter", 12);
   i=0;
   for (i=0; i<=ENDCHAN; i++)
   {
      sensor_filtrd[i]=0;
      for (j=0; j<NumofSamples; j++)
      {
         if (sensor_read[i][j]==5) sensor_err_flag=1;
         sensor_filtrd[i]=sensor_read[i][j]+sensor_filtrd[i];
      }
      sensor_filtrd[i] = sensor_filtrd[i] / NumofSamples;
      if (sensor_err_flag!=1)
      {
         sensor_filtrd_prev[i]=sensor_filtrd[i];
      }
      else
      {
         sensor_filtrd[i]=sensor_filtrd_prev[i];
         sensor_err_flag=0;
      }
   }
/*Rmv*/  // DS1led(OFF);
}
xmem void SendTelemetry()
{
// sendMSG(Data_Port, "\n\rTelemetry", 11);
// sendMSG(Comms_Port, "\\d Telemetry\n\r", 14);
}

xmem void IdleShutDown()
{
//Go to Idle mode for JetEngine
   WrPortI(TBCSR, &TBCSRShadow, 0x00);          // disable timer B and B1 match interrupts
   sendMSG(Data_Port, "\n\rIdleShutDown", 14);
   GNCJetrpm=33000;   //Idle;
   Jet_Send();
   MSdelay(30000);

   In_Flight=0;        // Calls System Shutdown
   return;
}

xmem void SystemShutDown()
{
//ShutDown JetEngine
   WrPortI(TBCSR, &TBCSRShadow, 0x00);          // disable timer B and B1 match interrupts
   sendMSG(Data_Port, "\n\rSystemShutDown", 16);
   sendMSG(Jet_Port, ShutDownTurbine, 7);
   sendByte(Jet_Port, 0x0a);

//ShutDown ACS System
   facs[0]=0;
   facs[1]=0;
   facs[2]=0;
   facs[3]=0;
   facs[4]=0;
   facs[5]=0;
   facs[6]=0;
   facs[7]=0;
   facs[8]=0;
   facs[9]=0;
   facs[10]=0;
   facs[11]=0;
   ACS_Cmd();
   MSdelay(500);
}

xmem void Jet_Test()
{
   sendMSG(Data_Port, "\n\rJet_Test_Start", 16);
   MSdelay(2000);
   sendMSG(Jet_Port, "1,TCO,1", 7);
   sendByte(Jet_Port, 0x0a);
   sendByte(Jet_Port, 0x0d);
   sendMSG(Data_Port, "\n\rStart_Turbine", 15);
   MSdelay(30000);
   MSdelay(30000);

   GNCJetrpm=53000;  //25%=52000;
   if (serEgetc()!='T')
   {
      Jet_Send();
      MSdelay(10000);
      GNCJetrpm=72500;  //50%
      if (serEgetc()!='T')
      {
         Jet_Send();
         MSdelay(10000);
         GNCJetrpm=92250;   //75%
         if (serEgetc()!='T')
         {
            Jet_Send();
            MSdelay(10000);
            GNCJetrpm=96200;  //80%
            if (serEgetc()!='T')
            {
               Jet_Send();
               MSdelay(10000);
               GNCJetrpm=100000;   //85%
               if (serEgetc()!='T')
               {
                  Jet_Send();
                  MSdelay(10000);
                  GNCJetrpm=104000;   //90%
                  if (serEgetc()!='T')
                  {
                     Jet_Send();
                     MSdelay(10000);
                     GNCJetrpm=108000;  //90%
                     if (serEgetc()!='T')
                     {
                        Jet_Send();
                        MSdelay(10000);
                        GNCJetrpm=33000;   //Idle;
                        Jet_Send();
                        MSdelay(27000);
                     }
                  }
               }
            }
         }
      }
   }
   GNCJetrpm=33000;   //Idle;
   Jet_Send();
   MSdelay(3000);
   sendMSG(Jet_Port, "1,TCO,0", 7);
   sendByte(Jet_Port, 0x0a);
   sendByte(Jet_Port, 0x0d);
   sendMSG(Data_Port, "\n\rShut_Turbine", 14);
   sendMSG(Data_Port, "\n\rJet_Test_End", 14);
   return;
}

xmem void ACS_Test2(int IterationPerThruster)
{
   int i, j, k;
   unsigned int count;
   char count_buffer[15];
   count=0;

//ShutDown ACS System
   facs[0]=0;
   facs[1]=0;
   facs[2]=0;
   facs[3]=0;
   facs[4]=0;
   facs[5]=0;
   facs[6]=0;
   facs[7]=0;
   facs[8]=0;
   facs[9]=0;
   facs[10]=0;
   facs[11]=0;
   ACS_Cmd();

   MSdelay(1000);

   for (i=0;i<12;i++)                           //One Thruster at a time
   {
      for (j=0; j<IterationPerThruster; j++)    //Do it IterationPerThruster times
      {
         if (serEgetc()=='T') return;
         *(facs+i)=1;
         ACS_Cmd();
         MSdelay(30);
         *(facs+i)=0;
         ACS_Cmd();
         count++;
         sprintf(count_buffer, "\n\r%d     ", count);
         sendMSG(Data_Port, count_buffer, 7);
         MSdelay(1000);
      }
   }
   return;
}
xmem int get_CMD(int Port_Num)
{
   int flag_cmd, flag;
   char temp, tempch;

   flag_cmd=0;
   flag=0;

   while (!flag)
   {
//    temp=getByte(Data_Port, &tempch);
      temp=getByte(Port_Num, &tempch);
      if (temp>0)
      {
         if(flag_space!=1)
         {
            if (tempch=='\\')
            {
               if (flag_slash!=1)
               {
                  flag_slash=1;  sendByte(Data_Port, '#');
               }
               else
               {
                  flag_slash=0;  sendByte(Data_Port, '|');
               }
            }
            else if(tempch=='d')
            {
               if (flag_d!=1)
               {
                  if (flag_slash==1)
                  {
                     flag_d=1;   sendByte(Data_Port, 'D');
                  }
               }
               else
               {
                  flag_d=0;   sendByte(Data_Port, 'd');
               }
            }
            else if(tempch==' ')
            {
               if(flag_space!=1)
               {
                  if (flag_d==1)
                     if (flag_slash==1)
                     {  flag_space=1;}//  sendByte(Data_Port, '+');  }
               }
               else
               {
                  flag_space=0;  sendByte(Data_Port, '-');
               }
            }
         }
         else
         {
            flag_space=0;  flag_d=0;   flag_slash=0;
            cmd_BYTE=tempch;
            flag_cmd=1;
            sendByte(Data_Port, 'Q');
         }
      }
      else
         flag=1;
   }

   return flag_cmd;
}
int InterruptXmsec()
{
   int check_End;
   char serial_Inp;
   int temp;
// serial_Inp=serAgetc();  //check for commands
//DS1led(ON);
// sendMSG(Data_Port, "\n\rX", 3);
   serial_Inp=serEgetc();  //check for commands
   temp=get_CMD(Comms_Port);
   if(temp==1)
   {
//      sendByte(ACS_Port, cmd_BYTE);
      serial_Inp=cmd_BYTE;
   }
   if (serial_Inp=='A') AbortCmdRcvd=1;
   if (serial_Inp=='C') IdleShutDown();
   if (serial_Inp=='D') In_Flight=0;

   if (GNCLoopCount<2)
   {
      ACS_Cmd();
      Jet_Send();
      ADC(GNCLoopCount-1);
   }
   else if (GNCLoopCount==2)
   {
      //ShutDown ACS System
      facs[0]=0;
      facs[1]=0;
      facs[2]=0;
      facs[3]=0;
      facs[4]=0;
      facs[5]=0;
      facs[6]=0;
      facs[7]=0;
      facs[8]=0;
      facs[9]=0;
      facs[10]=0;
      facs[11]=0;
      ACS_Cmd();
      ADC(GNCLoopCount-1);
      DataFilter(NumOfReadingsPerChannel);
   }
   else if (GNCLoopCount==MaxGNCLoopCount)
   {
      ADC(GNCLoopCount-1);

      ConvertSensorVals();
   }
   else
   {
      ADC(GNCLoopCount-1);
   }

   if (GNCLoopCount>MaxGNCLoopCount)
   {
//    check_End = GNC(AbortCmdRcvd);
      check_End = GNC3(AbortCmdRcvd);
      GNC2(AbortCmdRcvd);
      GNCLoopCount=0;
      TelemetryCount++;
   }
   else if (TelemetryCount>MaxTelemetryCount)
   {
      SendTelemetry();
      TelemetryCount=1;
   }

   GNCLoopCount++;

   if (check_End==0x0e) In_Flight=0;
//DS1led(OFF);
   return 0;
}

void ADC2()
{
   unsigned long rawdata;
   char Status_Buf[21];
   unsigned int cmd, channel, iteration_Number, j;

/*Rmv*/ //  DS1led(ON);
   RobinPosition=RobinPosition%FilterDepth;
   iteration_Number=RobinPosition;
// sendMSG(Data_Port, "\n\rADC2", 6);
//   sendByte(Data_Port, (iteration_Number+49));
   RobinPosition++;
//   sendMSG(Data_Port, "  ", 2);
   for(channel = STARTCHAN; channel <= ENDCHAN; channel++)
   {
      cmd = 0x80|(GAINSET*16+(channel|0x08));   //convert channel and gain to ADS7870 format in a direct mode
      rawdata = (long) anaInDriver(cmd, 12);    // execute low level A/D driver
      sensor_read[channel][iteration_Number] = (rawdata);// *0.001); // convert the averaged samples to a voltage

      if (channel==STARTCHAN)
      {
      	sprintf(Status_Buf, "%.3f\n\r!", sensor_read[channel][iteration_Number]);
      	j=0;
      	while (Status_Buf[j]!='!')
      	{
         	sendByte(Data_Port, Status_Buf[j]);
         	j++;
      	}
      }
      if (sensor_read[channel][iteration_Number]>10 | sensor_read[channel][iteration_Number]<0.1) sensor_read[channel][iteration_Number]=sensor_offsets[channel];
   }
/*Rmv*/ //  DS1led(OFF);

}
void SPIaccelADC()
{
   char adc_lower[2];
   char adc_upper[2];
   char adc_reading[2];
   int i,j,k,q;
   long int adc_sample1_x, adc_sample_gyro1;
   char Status_Buf[30];
//DS1led(ON);
   i=0;
	adc_sample_x=adc_sample_gyro=0;
	for(k=0; k<10; k++)
	{
      /* for x */
      BitWrPortI(PEDR, &PEDRShadow, 0, 7);	// chip select low
      SPIWrRd( "\xa8\x00", adc_lower, 2); //read x low output
      BitWrPortI(PEDR, &PEDRShadow, 1, 7);	// chip select high

      BitWrPortI(PEDR, &PEDRShadow, 0, 7);	// chip select low
      SPIWrRd( "\xa9\x00", adc_upper, 2); //read x high output
      BitWrPortI(PEDR, &PEDRShadow, 1, 7);	// chip select high
      adc_sample1_x = (adc_upper[1] <<8) + (adc_lower[1]);
      adc_sample_x = adc_sample_x +  adc_sample1_x;
      /* end x */

      /* gyro */
   	// rate output
		BitWrPortI(PEDR, &PEDRShadow, 0, 6);	// chip select low
      SPIWrite( "\x05\xaa", 2); //read gyro data
      for( j=0; j<1; j++ ){}
      BitWrPortI(PEDR, &PEDRShadow, 1, 6);	// chip select high
         for (j=0; j<100; j++)
   	{
   	}
      BitWrPortI(PEDR, &PEDRShadow, 0, 6);	// chip select low
		SPIRead(adc_reading, 2);
		BitWrPortI(PEDR, &PEDRShadow, 1, 6);	// chip select high
      //sign extend from the highest good data bit (bit 13)
      if( adc_reading[0] & 0x20 ){
      		adc_reading[0] = adc_reading[0] | 0xc0;}
         else{
         	adc_reading[0] = adc_reading[0] & 0x7f;}
		adc_sample_gyro1 = (adc_reading[0] <<8 ) + (adc_reading[1]);
      adc_sample_gyro = adc_sample_gyro + adc_sample_gyro1;
         for (j=0; j<100; j++)
   	{
   	}
      /* end gyro */

      for (j=0; j<200; j++){}
      i = i++ & 0xff;
	}

   adc_sample_x = adc_sample_x/k;
   adc_sample_gyro = adc_sample_gyro/k;
//   sprintf(Status_Buf, "%d,%f,%f\n\r!", k, adc_sample_gyro, adc_sample_x);
	sprintf(Status_Buf, "%d,%f\n\r!", adc_reading, adc_sample_gyro);
   q=0;
   while (Status_Buf[q]!='!')
   {
    sendByte(Data_Port, Status_Buf[q]);
    q++;
   }
//DS1led(OFF);
}

void ConvertSpi()      //Works on Filtered data
{
      sensor_filtrd[0]=(adc_sample_x-sensor_offsets[0])*0.00981;
      sensor_filtrd[5]=(adc_sample_gyro-sensor_offsets[5]);
}

int InterruptX2()
{
   int check_End;
   char serial_Inp;
   int temp;

   DS2led(ON);

   serial_Inp=serEgetc();  //check for commands
   temp=get_CMD(Comms_Port);

   if (serial_Inp=='A') AbortCmdRcvd=1;
   if (serial_Inp=='C') IdleShutDown();
   if (serial_Inp=='D') In_Flight=0;
      ACS_Cmd();
      SPIaccelADC();
		ConvertSpi();
  //    DataFilter(FilterDepth);
  //    ConvertSensorVals();
      check_End=GNC3(AbortCmdRcvd);
      GNC2(AbortCmdRcvd);
      Jet_Send();

   if (check_End==0x0e) In_Flight=0;
   DS2led(OFF);
}