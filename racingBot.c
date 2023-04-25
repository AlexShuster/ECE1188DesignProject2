/*
 * racingBot.c
 *
 *  Created on: Apr 24, 2023
 *      Author: jsb19
 */

#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "pidController.h"
#include "distanceDriver.h"
#include "odometry.h"
#include "Bump.h"
#include "UART0.h"
#include "driverlib.h"
#include "simplelink.h"
#include "sl_common.h"
#include "MQTTClient.h"

#define SSID_NAME       "Pixel_2632"       /* Access point name to connect to. */
#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2     /* Security type of the Access piont */
#define PASSKEY         "password"   /* Password in case of secure AP */
#define PASSKEY_LEN     pal_Strlen(PASSKEY)  /* Password length in case of secure AP */

/*
 * MQTT server and topic properties that shall be modified per application
 */
#define MQTT_BROKER_SERVER  "broker.hivemq.com"
#define SUBSCRIBE_TOPIC "JoeyRobotNet/command"
#define PUBLISH_TOPIC "JoeyRobotNet"
#define PUBLISH_TOPIC_LEFT_PWM "JoeyRobotNet/leftPWM"
#define PUBLISH_TOPIC_RIGHT_PWM "JoeyRobotNet/rightPWM"
#define PUBLISH_TOPIC_BUMP "JoeyRobotNet/bumpSensor"
#define PUBLISH_TOPIC_X "JoeyRobotNet/x"
#define PUBLISH_TOPIC_Y "JoeyRobotNet/y"
#define PUBLISH_TOPIC_THETA "JoeyRobotNet/theta"
#define PUBLISH_TOPIC_L "JoeyRobotNet/left"
#define PUBLISH_TOPIC_R "JoeyRobotNet/right"
#define PUBLISH_TOPIC_C "JoeyRobotNet/center"

// MQTT message buffer size
#define BUFF_SIZE 32


#define APPLICATION_VERSION "1.0.0"

#define MCLK_FREQUENCY 48000000
#define PWM_PERIOD 255

#define SL_STOP_TIMEOUT        0xFF

#define SMALL_BUF           32
#define MAX_SEND_BUF_SIZE   512
#define MAX_SEND_RCV_SIZE   1024

/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

#define min(X,Y) ((X) < (Y) ? (X) : (Y))


/*
 * GLOBAL VARIABLES -- Start
 */
/* Button debounce state variables */
volatile unsigned int S1buttonDebounce = 0;
volatile unsigned int S2buttonDebounce = 0;
volatile int publishID = 0;

unsigned char macAddressVal[SL_MAC_ADDR_LEN];
unsigned char macAddressLen = SL_MAC_ADDR_LEN;

char macStr[18];        // Formatted MAC Address String
char uniqueID[9];       // Unique ID generated from TLV RAND NUM and MAC Address

Network n;
Client hMQTTClient;     // MQTT Client
int recentlyBumped = 0;

_u32  g_Status = 0;
struct{
    _u8 Recvbuff[MAX_SEND_RCV_SIZE];
    _u8 SendBuff[MAX_SEND_BUF_SIZE];

    _u8 HostName[SMALL_BUF];
    _u8 CityName[SMALL_BUF];

    _u32 DestinationIP;
    _i16 SockID;
}g_AppData;

static _i32 establishConnectionWithAP();
static _i32 configureSimpleLinkToDefaultState();
static _i32 initializeAppVariables();
static void displayBanner();
static void messageArrived(MessageData*);
static void generateUniqueID();
static void publishPWM(uint16_t leftPWM, uint16_t rightPWM);
void PORT4_IRQHandler(void);
static void publishBumpSensor(void);
static void publishOdometry(void);
static void publishDistance(uint32_t *distancesMeasured);


/*
 * STATIC FUNCTION DEFINITIONS -- End
 */


/*
 * ASYNCHRONOUS EVENT HANDLERS -- Start
 */

/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
        CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");

    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_Write(" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_Write(" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            CLI_Write(" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
        CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            /*
             * Information about the connected AP's IP, gateway, DNS etc
             * will be available in 'SlIpV4AcquiredAsync_t' - Applications
             * can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
             * <gateway_ip> = pEventData->gateway;
             *
             */
        }
        break;

        default:
        {
            CLI_Write(" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pHttpEvent - Contains the relevant event information
    \param[in]      pHttpResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
*/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    /*
     * This application doesn't work with HTTP server - Hence these
     * events are not handled here
     */
    CLI_Write(" [HTTP EVENT] Unexpected event \n\r");
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
    CLI_Write(" [GENERAL EVENT] \n\r");
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
        CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");

    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        {
            /*
            * TX Failed
            *
            * Information about the socket descriptor and status will be
            * available in 'SlSockEventData_t' - Applications can use it if
            * required
            *
            * SlSockEventData_t *pEventData = NULL;
            * pEventData = & pSock->EventData;
            */
            switch( pSock->EventData.status )
            {
                case SL_ECLOSE:
                    CLI_Write(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
                break;


                default:
                    CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
                break;
            }
        }
        break;

        default:
            CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
        break;
    }
}

int targetSpeed = 10000;
int leftSpeed = 0, rightSpeed = 0;
int stop = 1, crashed = 0;

//Port 4 interupt handler
void PORT4_IRQHandler(void)
{
    //Get which bumper was pressed
    uint8_t input = readP4();

    //Stop the motors and keep the state stopped
    setMotorSpeed(0,0);
    Clock_Delay1ms(1000);
    setMotorSpeed(-2000, -4000);
    Clock_Delay1ms(750);
    setMotorSpeed(0, 0);
    crashed = 100;
    //Reset the flag
    P4->IFG &= input;
}

void setCenterSpeed(uint32_t *distances)
{
    int newSpeed = targetSpeed;
    if(distances[1] < 500)
        newSpeed *= distances[1] / 500.0;
    if(distances[0] > 1125 && distances[2] > 1175 && distances[1] < 1500)
    {
        setMotorSpeed(3000, 0);
        Clock_Delay1ms(90);
        return;
    }
    if(distances[1] < 100)
    {
        setMotorSpeed(-2000, -4000);
        Clock_Delay1ms(750);
        setMotorSpeed(0, 0);
        return;
    }
    leftSpeed = newSpeed * sin((float)distances[2] * 90.0 / 1500.0 * 0.0174533);
    rightSpeed = newSpeed * sin((float)distances[0] * 90.0 / 1500.0 * 0.0174533);

    rightSpeed *= 1.085;
}

int main()
{
    uint32_t distances[3];
    int32_t x, y, theta;
    Clock_Init48MHz();
    motorPWMInit(15000, 0, 0);
    EnableInterrupts();
    initDistanceDriver();
    UART0_Init();
    initP4();
    Odometry_Init(0, 0, 0);

    _i32 retVal = -1;

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);
    stopWDT();
    initClk();
    retVal = configureSimpleLinkToDefaultState();
    retVal = sl_Start(0, 0, 0);
    retVal = establishConnectionWithAP();
    sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(unsigned char *)macAddressVal);
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            macAddressVal[0], macAddressVal[1], macAddressVal[2], macAddressVal[3], macAddressVal[4], macAddressVal[5]);
    // Generate 32bit unique ID from TLV Random Number and MAC Address
    generateUniqueID();

    int rc = 0;
    unsigned char buf[100];
    unsigned char readbuf[100];

    NewNetwork(&n);
    rc = ConnectNetwork(&n, MQTT_BROKER_SERVER, 1883);

    MQTTClient(&hMQTTClient, &n, 1000, buf, 100, readbuf, 100);
    MQTTPacket_connectData cdata = MQTTPacket_connectData_initializer;
    cdata.MQTTVersion = 3;
    cdata.clientID.cstring = uniqueID;
    rc = MQTTConnect(&hMQTTClient, &cdata);


    char goStop = 'S';
    while (goStop !='G' && goStop != 0xF8)
    {
        goStop = UART0_InChar();
    }

    int count = 0;

    while(1)
    {

        goStop = UART0_InChar();
        if (goStop == 'S' || goStop == 0x80)
        {
            // Stop Motors
            stop = 0;
        }

        getDistances(distances);
        setCenterSpeed(distances);
        UpdatePosition();
        Odometry_Get(&x, &y, &theta);
        setMotorSpeed(leftSpeed * stop, rightSpeed * stop);
        rc = MQTTYield(&hMQTTClient, 10);
        if (publishID) {
            int rc = 0;
            MQTTMessage msg;
            msg.dup = 0;
            msg.id = 0;
            msg.payload = uniqueID;
            msg.payloadlen = 8;
            msg.qos = QOS0;
            msg.retained = 0;
            rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC, &msg);
            publishID = 0;
        }
        crashed--;
        if(crashed < 0)
            crashed = 0;
        if(count % 300 == 0)
        {
            publishPWM(leftSpeed, rightSpeed);
            publishOdometry();
            publishDistance(distances);
            if(crashed > 0)
                publishBumpSensor();
        }
        else
            Clock_Delay1ms(15);
        count++;
    }

}

static void publishPWM(uint16_t leftPWM, uint16_t rightPWM)
{
    MQTTMessage msgLeft, msgRight;
    msgLeft.dup = 0;
    msgRight.dup = 0;
    msgLeft.id = 0;
    msgRight.id = 0;
    char outputLeft[25];
    sprintf(outputLeft, "%04d", (int)((float) leftPWM / 15000 * 1000));
    char outputRight[25];
    sprintf(outputRight, "%04d", (int)((float) rightPWM / 15000 * 1000));
    msgLeft.payload = outputLeft;
    msgRight.payload = outputRight;
    msgLeft.payloadlen = 4;
    msgRight.payloadlen = 4;
    msgLeft.qos = QOS0;
    msgRight.qos = QOS0;
    msgLeft.retained = 0;
    msgRight.retained = 0;
    MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_LEFT_PWM, &msgLeft);
    MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_RIGHT_PWM, &msgRight);
}

static void publishBumpSensor(void)
{
    MQTTMessage msgBump;
    msgBump.dup = 0;
    msgBump.id = 0;
    char output[5];
    sprintf(output, "Crashed!");
    msgBump.payload = output;
    msgBump.payloadlen = 8;
    msgBump.qos = QOS0;
    msgBump.retained = 0;
    MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_BUMP, &msgBump);
}

static void publishOdometry(void)
{
    int32_t x, y, theta;
    Odometry_Get(&x, &y ,&theta);
    MQTTMessage msgX, msgY, msgTheta;
    msgX.dup = 0;
    msgX.id = 0;
    msgY.dup = 0;
    msgY.id = 0;
    msgTheta.dup = 0;
    msgTheta.id = 0;
    char outputX[20];
    char outputY[20];
    char outputTheta[20];
    sprintf(outputX, "%20d", x);
    sprintf(outputY, "%20d", y);
    sprintf(outputTheta, "%20d", theta);
    msgX.payload = outputX;
    msgX.payloadlen = 20;
    msgX.qos = QOS0;
    msgX.retained = 0;
    msgY.payload = outputY;
    msgY.payloadlen = 20;
    msgY.qos = QOS0;
    msgY.retained = 0;
    msgTheta.payload = outputTheta;
    msgTheta.payloadlen = 20;
    msgTheta.qos = QOS0;
    msgTheta.retained = 0;
    MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_X, &msgX);
    MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_Y, &msgY);
    MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_THETA, &msgTheta);
}

static void publishDistance(uint32_t *distancesMeasured)
{
    MQTTMessage msgL, msgR, msgC;
    msgL.dup = 0;
    msgL.id = 0;
    msgR.dup = 0;
    msgR.id = 0;
    msgC.dup = 0;
    msgC.id = 0;
    char outputX[1];
    char outputY[1];
    char outputTheta[1];
    sprintf(outputX, "%d", (distancesMeasured[0] < 500));
    sprintf(outputY, "%d", (distancesMeasured[2] < 500));
    sprintf(outputTheta, "%d", (distancesMeasured[0] < 500));
    msgL.payload = outputX;
    msgL.payloadlen = 1;
    msgL.qos = QOS0;
    msgL.retained = 0;
    msgR.payload = outputY;
    msgR.payloadlen = 1;
    msgR.qos = QOS0;
    msgR.retained = 0;
    msgC.payload = outputTheta;
    msgC.payloadlen = 1;
    msgC.qos = QOS0;
    msgC.retained = 0;
    MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_L, &msgL);
    MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_R, &msgR);
    MQTTPublish(&hMQTTClient, PUBLISH_TOPIC_C, &msgC);
}

static _i32 configureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8           val = 1;
    _u8           configOpt = 0;
    _u8           configLen = 0;
    _u8           power = 0;

    _i32          retVal = -1;
    _i32          mode = -1;

    mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(mode);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
        }

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
        {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }

    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    return retVal; /* Success */
}

/*!
    \brief Connecting to a WLAN Access point

    This function connects to the required AP (SSID_NAME).
    The function will return once we are connected and have acquired IP address

    \param[in]  None

    \return     0 on success, negative error-code on error

    \note

    \warning    If the WLAN connection fails or we don't acquire an IP address,
                We will be stuck in this function forever.
*/
static _i32 establishConnectionWithAP()
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = PASSKEY;
    secParams.KeyLen = PASSKEY_LEN;
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

/*!
    \brief This function initializes the application variables

    \param[in]  None

    \return     0 on success, negative error-code on error
*/
static _i32 initializeAppVariables()
{
    g_Status = 0;
    pal_Memset(&g_AppData, 0, sizeof(g_AppData));

    return SUCCESS;
}

static void generateUniqueID() {
    CRC32_setSeed(TLV->RANDOM_NUM_1, CRC32_MODE);
    CRC32_set32BitData(TLV->RANDOM_NUM_2);
    CRC32_set32BitData(TLV->RANDOM_NUM_3);
    CRC32_set32BitData(TLV->RANDOM_NUM_4);
    int i;
    for (i = 0; i < 6; i++)
    CRC32_set8BitData(macAddressVal[i], CRC32_MODE);

    uint32_t crcResult = CRC32_getResult(CRC32_MODE);
    sprintf(uniqueID, "%06X", crcResult);
}

