/*  Helium LoRa detector
    Author Stephen Witty switty@level500.com
    Code based on WisBlock RAK4631 LoRa example - modified to fit application
    Started 12-8-22
    Description - Test for LoRa join, while joining blink green LED (IO1), if successful, solid green LED, if not solid White (IO2)
      Sleep for 3 minutes and repeat via MCU reset

    Used this example from Adafruit LittleFS to form persistent storage system
    https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/00d0801c1461ff60d563426ceffe87b8e3e919bd/libraries/InternalFileSytem/examples/Internal_ReadWrite/Internal_ReadWrite.ino
    
    V1 - 12-10-22 Initial release
    V2 - 1-17-23  - Added static storage of failed and good logins, added login time to good logins, sent all up to Helium console
    V3 - 1-20-23  - Making built in LED echo LED Green (IO1) status.  This allows ap to be used without external LEDs
*/

/**
 * @file LoRaWAN_OTAA_ABP.ino
 * @author rakwireless.com
 * @brief LoRaWan node example with OTAA/ABP registration
 * @version 0.1
 * @date 2020-08-21
 * 
 * @copyright Copyright (c) 2020
 */
#include <Arduino.h>
#include <LoRaWan-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
#include "keys.h" //Helium LoRa keys are in this include file
#include <Adafruit_SleepyDog.h>  //Watch dog include

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

#define VERSION "3"            //Version of application
#define BLINK_TIME 450         //Blink time for green LED joining network
#define RESET_TIME 180000      //Amount of time to wait until MCU reset (retry join operation) in milli-seconds (240000 is 4 minutes)
#define ENABLE_WATCH_DOG true  //Turn on watchdog?  May not get serial output
#define MAX_COUNT 5000000L      //Max count of login counts before reset to zero
#define FLASH_STORAGE "LoRa6"  //File name for flash storage to store login counts

// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60										  /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_1									  /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5							/*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3										  /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;					/* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_US915;    /* Region:US915*/
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;				  /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;							        /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                        lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler
                                       };


// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64                     /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 20000                        /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

File file(InternalFS);

char buf[100];  //Storage data for flash write conversions
unsigned long failed_logins=0, good_logins=0,store_time,login_time;

void setup()
{
  int cnt=0,pos=0,size;
  bool flag=false;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(WB_IO1,OUTPUT); //Green LED, blinking: trying to join LoRa, solid: joined LoRa
  pinMode(WB_IO2,OUTPUT); //White, LoRa join failed
  digitalWrite(WB_IO1,LOW);
  digitalWrite(WB_IO2,LOW);

  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 1000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }

  delay(1000);
  Serial.print("Helium LoRa Dector Version: ");
  Serial.println(VERSION);
  Serial.flush();

  if (ENABLE_WATCH_DOG) 
   {
     Serial.println("Turning on Watch-Dog"); Serial.flush();
     Watchdog.enable(8000); //Set watchdog timeout to 8 seconds (the maximum)
   }

  InternalFS.begin();
  
  file.open(FLASH_STORAGE,FILE_O_READ);
  if (file)
  {
    Serial.println("Flash storage file exists");
    size=file.read(buf,sizeof(buf));
    file.close();    
    Serial.print("Read storage file: ("); Serial.print(buf); Serial.println(")");
    Serial.print("Size of file read: "); Serial.println(size);
    for(int a=0;a<strlen(buf);a++) //Verify that file is all numeric or space
    { 
      if (buf[a]==' ') { cnt++; pos=a; }
      if (!(isDigit(buf[a])) && buf[a]!=' ') flag=true;
    }

    if (cnt==1 && strlen(buf)-1 > pos && pos!=0 && strlen(buf)<60 && flag==false) //check and make sure there is a single space and something after it and before it
    {
     buf[pos]=NULL; 
     good_logins=atoi(buf);
     failed_logins=atoi(&buf[pos+1]);
     Serial.print("Good logins from flash file: "); Serial.println(good_logins);
     Serial.print("Failed logins from flash: "); Serial.println(failed_logins);
        
     if (good_logins > MAX_COUNT || failed_logins > MAX_COUNT)
     {
      Serial.println("Resetting login counts, above max count");
      good_logins=0;
      failed_logins=0;
     }
    } 
     else 
     {
       Serial.println("Flash file corrupt, resetting to zero counts");
       good_logins=0;
       failed_logins=0;       
     }
  } //end of file opened
  else  //Storage file could not be opened
  {
    Serial.println("WARNING Could not open flash storage file");
    good_logins=0;
    failed_logins=0;
  }
  
  // Initialize LoRa chip.
  lora_rak4630_init();  

  if (ENABLE_WATCH_DOG) Watchdog.reset();

  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);

  // Initialize LoRaWan
  uint32_t err_code;
  err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  login_time=0;
  store_time=millis();
  // Start Join procedure
  lmh_join();

  if (ENABLE_WATCH_DOG) Watchdog.reset();  
}

bool lora_joined=false;       //Did MCU join LoRa?
bool lora_join_failed=false;  //Did MCU fail to join LoRa?
bool led_toggle=false;

void loop()
{
    while (lora_joined==false && lora_join_failed==false)  //MCU trying to join LoRa network
    {
      if (led_toggle==true) { digitalWrite(WB_IO1,HIGH); digitalWrite(LED_BUILTIN,HIGH); led_toggle=false; }
        else { digitalWrite(WB_IO1,LOW); digitalWrite(LED_BUILTIN,LOW); led_toggle=true; }

      delay(BLINK_TIME);
      if(ENABLE_WATCH_DOG) Watchdog.reset();  //Feed the dog, must feed ever so often or get a MCU reset
    }
    digitalWrite(WB_IO1,LOW);
    digitalWrite(LED_BUILTIN,LOW);

    if (lora_join_failed==true) { digitalWrite(WB_IO2,HIGH); failed_logins++; }
    if (lora_joined==true) 
    { 
      digitalWrite(WB_IO1,HIGH);
      digitalWrite(LED_BUILTIN,HIGH);

      good_logins++; 
      login_time=millis()-store_time;

       uint8_t data[50]; 

       memcpy(data,&good_logins,sizeof(good_logins));
       memcpy(&data[4],&failed_logins,sizeof(failed_logins));
       memcpy(&data[8],&login_time,sizeof(login_time));

       send_lora_frame(data,sizeof(good_logins)+sizeof(failed_logins)+sizeof(login_time));
    }

    Serial.print("Good logins: "); Serial.print(good_logins); Serial.print(" Failed logins: "); Serial.println(failed_logins);
    Serial.print("Login time: "); Serial.println(login_time);

    if (good_logins > MAX_COUNT || failed_logins > MAX_COUNT)
    {
      Serial.println("Resetting login counts");
      good_logins=0;
      failed_logins=0;
    }

    /*******************  Write information to flash storage *************************/
    Serial.println("Writing login counts to flash storage");
    if (file.open(FLASH_STORAGE,FILE_O_WRITE))
    {      
      file.seek(0);
      file.truncate();
      itoa(good_logins,buf,10); //Convert number to char[] for flash storage    
      strcat(buf," ");  
      itoa(failed_logins,&buf[strlen(buf)],10);
      Serial.print("Writing ("); Serial.print(buf); Serial.println(")");
      file.write(buf,strlen(buf));
      file.close();
      Serial.println("Flash write complete"); 
    }
    else Serial.println("ERROR: Failed to open file to write login counts");
    /********************************************************************************/
  
    //The file flash routines do something with the built in LED so after complete resetting output
    if (lora_join_failed==true) digitalWrite(LED_BUILTIN,LOW);
    if (lora_joined==true) digitalWrite(LED_BUILTIN,HIGH);

    time_t t=millis();
    while (millis()-t < RESET_TIME)
    {
      if (ENABLE_WATCH_DOG) Watchdog.reset();
      delay(3000);
    }

    //Reset MCU and start again via watch dog timeout
    Serial.println("Resetting MCU via watchdog"); Serial.flush();
    Watchdog.enable(8000);  //From Sleepydog example in IDE
    while(true); //Wait for watchdog
}

/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
  Serial.println("LoRa, Network joined");
  lora_joined=true;
}

/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  Serial.println("LoRa, Network join failed!");
  lora_join_failed=true;
}

/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[in] app_data  Pointer to rx data
 */
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
          app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
// Informs the server that switch has occurred ASAP
//  m_lora_app_data.buffsize = 0;
//  m_lora_app_data.port = gAppPort;
//  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void send_lora_frame(uint8_t data[], int size)
{
  if (lmh_join_status_get() != LMH_SET)  //Should not make it this far if no LoRa
  {
    Serial.println("ERROR no LoRa access");
    return;
  }
  
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);    //Zero out buffer for safety, probably not needed
  memcpy(m_lora_app_data.buffer,data,size);                         //Copy data into lora message buffer
  m_lora_app_data.port = gAppPort;                                  //Setting LoRa port
  m_lora_app_data.buffsize=size;                                    //Set data length of buffer

  Serial.print("Size of payload "); Serial.println(m_lora_app_data.buffsize);

  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS) Serial.println("LoRa send was successful");
    else Serial.println("LoRa send FAILED");
}


