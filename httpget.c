



#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// XDCtools Header files
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* TI-RTOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Idle.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>
#include <ti/net/http/httpcli.h>
 #include <ti/drivers/I2C.h>
#include "Board.h"

#include <sys/socket.h>
#include <arpa/inet.h>


#define SOCKETTEST_IP     "192.168.2.6"
#define TASKSTACKSIZE     4096
#define OUTGOING_PORT     5011
#define INCOMING_PORT     5030

extern Mailbox_Handle mailbox0;
extern Mailbox_Handle mailbox1;
extern Mailbox_Handle semaphore0;
extern Task_Handle timer0;


char tem[60] =""; // string for communication taskTemperature to taskFxn
char final[60] =""; //string for send to server
uint16_t temperatureserver;
Void timerHWI01(UArg arg1)
{
    Semaphore_post(semaphore0);
}

/////////////// settings and task for the pressure sensor ///////////////
Task_Struct task1Struct;
Char task1Stack[TASKSTACKSIZE];
I2C_Handle      i2c1;
I2C_Params      i2cParams1;
I2C_Transaction i2cTransaction1;
uint16_t        tempval;
uint8_t         txBuffer[4];
uint8_t         rxBuffer[30];

short AC1, AC2, AC3, B1, B2, MB, MC, MD;    // calibration variables
unsigned short AC4, AC5, AC6;               // calibration variables
long UT, UP;    //uncompensated temperature and pressure
float B3, B4, B6, B7, X1t, X1p, X2t, X2p, X3p, B5t, B5p, Altitude;

void BMP180_getPressureCalibration(void)
{
    txBuffer[0] = 0xAA;  // calibration adress
    i2cTransaction1.slaveAddress = Board_BMP180_ADDR; //0x77, sensor ID
    i2cTransaction1.writeBuf = txBuffer;
    i2cTransaction1.writeCount = 1;
    i2cTransaction1.readBuf = rxBuffer;
    i2cTransaction1.readCount = 22;

    if (I2C_transfer(i2c1, &i2cTransaction1)) {

        AC1 = rxBuffer[0]<<8 | rxBuffer[1];
        AC2 = rxBuffer[2]<<8 | rxBuffer[3];
        AC3 = rxBuffer[4]<<8 | rxBuffer[5];
        AC4 = rxBuffer[6]<<8 | rxBuffer[7];
        AC5 = rxBuffer[8]<<8 | rxBuffer[9];
        AC6 = rxBuffer[10]<<8 | rxBuffer[11];
        B1 = rxBuffer[12]<<8 | rxBuffer[13];
        B2 = rxBuffer[14]<<8 | rxBuffer[15];
        MB = rxBuffer[16]<<8 | rxBuffer[17];
        MC = rxBuffer[18]<<8 | rxBuffer[19];
        MD = rxBuffer[20]<<8 | rxBuffer[21];
        }
}

void BMP180_startTemperatureAcquisition(void)
{
    txBuffer[0] = 0xf4;                                 // control register
    txBuffer[1] = 0x2e;                                 // temperature conversion command
    i2cTransaction1.slaveAddress = Board_BMP180_ADDR;    // 0x77
    i2cTransaction1.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction1.writeCount = 2;                      // two bytes will be sent
    i2cTransaction1.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction1.readCount = 0;                       // we are expecting 2 bytes

    I2C_transfer(i2c1, &i2cTransaction1);

}

void BMP180_startPressureAcquisition(void)
{
    txBuffer[0] = 0xf4;                                 // control register
    txBuffer[1] = 0x34;                                 // pressure conversion command
    i2cTransaction1.slaveAddress = Board_BMP180_ADDR;    // 0x77
    i2cTransaction1.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction1.writeCount = 2;                      // two bytes will be sent
    i2cTransaction1.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction1.readCount = 0;                       // we are expecting 2 bytes

    I2C_transfer(i2c1, &i2cTransaction1);

}

float BMP180_getTemperature(void)
{
    float temp;

    txBuffer[0] = 0xf6;                                 // temperature data register
    i2cTransaction1.slaveAddress = Board_BMP180_ADDR;    // 0x77
    i2cTransaction1.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction1.writeCount = 1;                      // two bytes will be sent
    i2cTransaction1.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction1.readCount = 2;                       // we are expecting 2 bytes

    I2C_transfer(i2c1, &i2cTransaction1);

    UT = rxBuffer[0]<<8 | rxBuffer[1];  //UT = raw temperature data

    //compute temperature
    X1t = ((UT - AC6) * AC5) >> 15;
    X2t = (MC << 11) / (X1t + MD);
    B5t = X1t + X2t;
    temp = ((B5t + 8) / 16) / 10;

    return temp;
}

float BMP180_getPressure(void)
{
    float pressure;

    txBuffer[0] = 0xf6;                                 // temperature register
    i2cTransaction1.slaveAddress = Board_BMP180_ADDR;    // 0x77
    i2cTransaction1.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction1.writeCount = 1;                      // two bytes will be sent
    i2cTransaction1.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction1.readCount = 2;                       // we are expecting 2 bytes

    I2C_transfer(i2c1, &i2cTransaction1);

    UP = rxBuffer[0]<<8 | rxBuffer[1];  //UT = raw pressure data

    //compute pressure
    B6 = B5t - 4000;
    X1p = (B2 * (B6 * B6 / 4096)) / 2048;
    X2p = AC2 * B6 / 2048;
    X3p = X1p = X2p;
    B3 = ((((long)AC1 * 4 + X3p)) + 2) / 4;
    X1p = AC3 * B6 / 8192;
    X2p = (B1 * (B6 * B6 / 4096)) / 65536;
    X3p = ((X1p + X2p) + 2) / 4;
    B4 = AC4 * (unsigned long)(X3p + 32768) / 32768;
    B7 = ((unsigned long)UP - B3) * (50000);
    if (B7 < 0x80000000) {
        pressure = (B7 * 2) / B4;
    }
    else {
        pressure = (B7 / B4) * 2;
    }
    X1p = (pressure / 256) * (pressure / 256);
    X1p = (X1p * 3038) / 65536;
    X2p = (-7357 * pressure) / 65536;
    pressure = pressure + (X1p + X2p + 3791) / 16;

    return pressure;
}

void initializeI2C()
{
    // Create I2C interface for sensor usage
    I2C_Params_init(&i2cParams1);
    i2cParams1.bitRate = I2C_400kHz;


    i2c1 = I2C_open(Board_I2C0, &i2cParams1);  // actually I2C7
    if (i2c1 == NULL) {
        // error initializing IIC
        //System_printf("I2C not Initialized!\n");
    }
    //System_printf("I2C Initialized!\n");
}


Void taskFxn(UArg arg0, UArg arg1)
{
    while(1){
    float temp, press, alt;

    Mailbox_pend(mailbox0, &tempval, BIOS_WAIT_FOREVER);

    initializeI2C(); // initialize I2C interface
    BMP180_getPressureCalibration();  // get pressure calibration data
    BMP180_startTemperatureAcquisition();  // start temperature acquisition
    Task_sleep(5);// wait for 5 mseconds for the acquisition
    temp = BMP180_getTemperature(); // get the uncompensated temperature value
    BMP180_startPressureAcquisition();// start pressure acquisition
    System_flush();
    Task_sleep(5);// wait for 5 mseconds for the acquisition
    press = BMP180_getPressure(); // get the uncompensated pressure value

    System_printf(" Pressure: %d\n  ",(int)press);
    System_flush();

    sprintf(tem," temperature: %d (c) pressure: %f (pascal)\n",tempval,press);

    I2C_close(i2c1);

    Mailbox_post(mailbox1, &tem, BIOS_NO_WAIT);
    }
}


////////////////////// task for the temperature sensor ////////////////////////////////
Void taskTemperature(UArg arg0, UArg arg1){
   while(1){
       Semaphore_pend(semaphore0, BIOS_WAIT_FOREVER);
       uint16_t temperature;
       uint8_t txBuffer1[6];
       uint8_t rxBuffer1[6];
       I2C_Handle i2c;
       I2C_Params i2cParams;
       I2C_Transaction i2cTransaction;

       I2C_Params_init(&i2cParams);
       i2cParams.bitRate = I2C_400kHz;
       i2c = I2C_open(Board_I2C_TMP, &i2cParams);
       if (i2c == NULL) {
           System_abort("Error Initializing I2C\n");
       }
       else {
           System_printf("I2C Initialized for tmp006\n");
       }

       //set tx
       txBuffer1[0] = 0x01; // temperature register
       i2cTransaction.slaveAddress = Board_TMP006_ADDR; // 0x41 , sensor ID
       i2cTransaction.writeBuf = txBuffer1; // write from txbuffer1
       i2cTransaction.writeCount = 1;
       i2cTransaction.readBuf = rxBuffer1; // read from rxbuffer1
       i2cTransaction.readCount = 2;

       if (I2C_transfer(i2c, &i2cTransaction)) {
           temperature = (rxBuffer1[0] << 6) | (rxBuffer1[1] >> 2);
           if (rxBuffer1[0] & 0x80) { // if the number is negative
               temperature |= 0xF000;
           }
           temperature /= 32;
           temperatureserver = temperature;
           System_printf("Sample : %d (C)\n",  temperature);

       }
       else {
           System_printf("I2C Bus fault\n");
       }
       System_flush();
       I2C_close(i2c);
       Mailbox_post(mailbox0, &temperature, BIOS_NO_WAIT); //communication taskTemperature to taskFxn
   }
}


//////////////////////client - server///////////////////////////////////

bool sendData2Server(char *serverIP, int serverPort, char *data, int size)
{
    int sockfd, connStat, numSend;
    bool retval=false;
    struct sockaddr_in serverAddr;

    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == -1) {
        System_printf("Socket not created");
        close(sockfd);
        return false;
    }

    memset(&serverAddr, 0, sizeof(serverAddr));  // clear serverAddr structure
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);     // convert port # to network order
    inet_pton(AF_INET, serverIP, &(serverAddr.sin_addr));

    connStat = connect(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if(connStat < 0) {
        System_printf("sendData2Server::Error while connecting to server\n");
        System_flush();
    }
    else {
        numSend = send(sockfd, data, size, 0);       // send data to the server
        if(numSend < 0) {
            System_printf("sendData2Server::Error while sending data to server\n");
            System_flush();
        }
        else {
            retval = true;      // successfully sent the data string
        }
    }
    System_flush();
    close(sockfd);
    return retval;
}

Void clientSocketTask(UArg arg0, UArg arg1)
{
    while(1) {
        Mailbox_pend(mailbox1, &final, BIOS_WAIT_FOREVER);

        // connect to SocketTest program on the system with given IP/port

        if(sendData2Server(SOCKETTEST_IP, OUTGOING_PORT, final, strlen(final))) {
            System_printf("clientSocketTask:: Temperature is sent to the server\n");
            System_flush();
        }
    }
}


bool createTasks(void)
{
    static Task_Handle  taskHandle2 ;
    Task_Params taskParams;
    Error_Block eb;

    Error_init(&eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskHandle2 = Task_create((Task_FuncPtr)clientSocketTask, &taskParams, &eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task1Stack;
    Task_construct(&task1Struct, (Task_FuncPtr)taskFxn, &taskParams, NULL);


    if ( taskHandle2 == NULL ) {
        printError(" Failed to create \n", -1);
        return false;
    }

    return true;
}

//  This function is called when IP Addr is added or deleted
//
void netIPAddrHook(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
    // Create a HTTP task when the IP address is added
    if (fAdd) {
        createTasks();
    }
}

void printError(char *errString, int code)
{
    System_printf("Error! code = %d, desc = %s\n", code, errString);
    BIOS_exit(code);
}
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initEMAC();
    Board_initI2C();
    /* Turn on user LED */

    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the HTTP GET example\nSystem provider is set to "
            "SysMin. Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();


    /* Start BIOS */
    BIOS_start();

    return (0);
}
