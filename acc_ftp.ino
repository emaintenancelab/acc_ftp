/*
  curl 7058FEA4AE30.local/sample/stop
  
  curl esp32.local/sample/start
  curl esp32.local/sample/stop
*/

#include <SPI.h>
#include <WiFi.h>
#include "time.h"
#include "freertos/ringbuf.h"
#include <ESPmDNS.h>

#define LED_BUILTIN 2
#define BUT_BUILTIN 0

// SPI
#define BW_RATE        0x2C
#define POWER_CTL      0x2D // Power Control Register
#define DATA_FORMAT    0x31 // Data Format Register
#define DATAX0         0x32 // X-Axis Data 0
#define FIFO_CTL 0x38
#define ADXL345_3200HZ      0x0F

const char version[] = "build "  __DATE__ " " __TIME__;

int CS0 = 15;            // Chip select for the ADXL345
static const int spiClk = 10 * 1000 * 1000; // x MHz SPI communication speed
SPIClass * hspi = NULL;  // Uninitalised pointers to SPI objects

//char fifobuf[6 * 32];
////

#define CORE_ZERO 0
#define CORE_ONE 1

/*
  #define WIFI_NETWORK "DTwin"
  #define WIFI_PASSWORD "goodlife"

*/
#define WIFI_NETWORK "DT_Base"
#define WIFI_PASSWORD "morethan8"

#define DEVICE_NAME "ESP32"
#define WIFI_TIMEOUT 2000

//boolean debug = false;  // true = more messages
boolean debug = true;

uint64_t chipid;
char uniqId[16];

struct tm timeinfo;
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 2 * 3600; // +2 GMT
const int   daylightOffset_sec = 0;
char timebuf[16];

unsigned long previousMillis = 0;
unsigned long samplingMillis = 0;
size_t samplingState = 0;

// FTP
// change to your server
String mDnsHost = "b5180";
//IPAddress FTPserver;
IPAddress FTPserver( 192, 168, 43, 1 );

#define FTP_PORT 2121
#define FTPPKTSZ 1460
const char* ftp_user = "user";
const char* ftp_pass = "12345";

char outBuf[128];
char outCount;
char fileName[32];
char cmdBuf[32];
size_t cmdBufPtr = 0;

/*
  buffer should be more than 1450 to maximize data transmission speed
  ????should be multiple of 6*32 to ensure alignment through cycles
  6*32*100 bytes per second
  At start FTP may take time to init, buffer should have enough space to handle
  todo better solution
*/
#define FTPBUFSZ (FTPPKTSZ*30)
WiFiServer wifiServer(80);

RingbufHandle_t ringBufAcc;

////
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUT_BUILTIN, INPUT);
  Serial.begin(115200);
  delay(2000);
  if (debug) Serial.println(version);
  //>> ESP32 unique id
  chipid = ESP.getEfuseMac();
  snprintf(uniqId, 16, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  if (debug) Serial.printf("\nESP32 Chip Id: %s\n", uniqId);

  ringBufAcc = xRingbufferCreate(FTPBUFSZ, RINGBUF_TYPE_BYTEBUF);
  if (ringBufAcc == NULL) {
    Serial.printf("ring FTP buffer error\n");
  }
  connectWifi();

  //initmDNS();
  // wifiServer.begin();

  xTaskCreate(
    keepWiFiAliveT,
    "keepWiFiAlive",  // Task name
    10000,            // Stack size (bytes)
    NULL,             // Parameter
    1,                // Task priority
    NULL             // Task handle
  );

  /*
    xTaskCreate(
    updateTimeT,
    "updateTime",     // Task name
    10000,            // Stack size (bytes)
    NULL,             // Parameter
    1,                // Task priority
    NULL             // Task handle
    );
  */
  xTaskCreate(
    readAccT,
    "readAcc",     // Task name
    10000,            // Stack size (bytes)
    NULL,             // Parameter
    1,                // Task priority ################# high priority
    NULL             // Task handle
  );

  xTaskCreate(
    uploadDataT,
    "uploadData",     // Task name
    10000,            // Stack size (bytes)
    NULL,             // Parameter
    1,                // Task priority
    NULL             // Task handle
  );

  xTaskCreate(
    serverT,
    "server",     // Task name
    10000,            // Stack size (bytes)
    NULL,             // Parameter
    1,                // Task priority
    NULL             // Task handle
  );

}

void loop() {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  if (Serial.available()) {
    char inByte = Serial.read();
    if (inByte == 's') {
      samplingState = 1;
      samplingMillis = millis();
    }
    if (inByte == 'p') {
      samplingState = 2;
    }

    if (inByte == 't') {
      getLocalTime(&timeinfo);

      snprintf(timebuf, 16, "%02d%02d%02d_%02d%02d%02d",
               timeinfo.tm_year - 100, timeinfo.tm_mon + 1, timeinfo.tm_mday,
               timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      Serial.println(timebuf);
    }
    /*
      char inChar = Serial.read();
      if (inChar == '\n') { // process command
      cmdBuf[cmdBufPtr] = NULL;
      evalCmd(cmdBuf);
      }
      else {
      cmdBuf[cmdBufPtr++] = inChar;
      }
    */

  }
  /*
    if (currentMillis - samplingMillis >= 10 * 60 * 1000 && samplingState == 1) {
    samplingState = 2;
    Serial.print(currentMillis - samplingMillis);
    Serial.println("End of data");
    }
  */
  vTaskDelay(100 / portTICK_PERIOD_MS);
}
/*
  char cmds[][8] = {
  "help",
  "exit",
  };
  #define CMDCOUNT sizeof(cmds)/sizeof(cmds[0])

  size_t evalCmd(char* cmd) {
  int i = -1;
  // check if cmd is valid
  for (int i = 0; i < CMDCOUNT; i++) {
  if (strncmp(cmds[i], cmd, 32) == 0)
  break;
  }
  // run the command
  switch (i) {
  case 0: Serial.print("hello");
  break;
  default: Serial.println("Command Not Fount");
  }
  }
*/
/*
  Upload file
  filename UId_yymmdd_hhmmss
  IEExxx

*/
WiFiClient client;
WiFiClient dclient;

// FTP support functions

//----------------- FTP fail
void efail() {
  byte thisByte = 0;

  client.println(F("QUIT"));

  while (!client.available()) delay(1);

  while (client.available()) {
    thisByte = client.read();
    Serial.write(thisByte);
  }

  client.stop();
  Serial.println(F("Command disconnected"));
  Serial.println(F("SD closed"));
}  // efail

//-------------- FTP receive
byte eRcv() {
  byte respCode;
  byte thisByte;

  while (!client.available()) delay(1);

  respCode = client.peek();

  outCount = 0;

  while (client.available()) {
    thisByte = client.read();
    Serial.write(thisByte);

    if (outCount < 127) {
      outBuf[outCount] = thisByte;
      outCount++;
      outBuf[outCount] = 0;
    }
  }

  if (respCode >= '4') {
    efail();
    return 0;
  }
  return 1;
}  // eRcv()



void uploadDataT(void * parameter) {
  size_t tState = 0; // 0 entry, 1 running, 2 end
  size_t tStatel = 1;

  while (1) {
    if (tState != tStatel) {
      Serial.println(pcTaskGetTaskName(NULL));
      Serial.println(tState);
      tStatel = tState;
    }
    if (tState == 0) {
      if (samplingState == 1) { // data acq started
        tState = 1;
        Serial.println("A");
        /*
          if(! getLocalTime(&timeinfo)) {
          continue;
          }
          snprintf(fileName, 32, "%y%m%d_%H%M%S.txt",
          timeinfo.tm_year+1900, timeinfo.tm_mon, timeinfo.tm_mday,
          timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        */
        snprintf(fileName, 32, "%d.log", millis());
        Serial.println("B");
        // create file
        if (client.connect(FTPserver, FTP_PORT, 5000)) {
          Serial.println(F("Command connected"));
        } else {
          Serial.println(F("Command connection failed"));
          continue;//return 0;
        }

        if (!eRcv()) continue;//return 0;
        if (debug) Serial.println("Send USER");
        client.print(F("USER "));
        client.println(ftp_user);

        if (!eRcv()) continue;//return 0;
        if (debug) Serial.println("Send PASSWORD");
        client.print(F("PASS "));
        client.println(ftp_pass);


        if (!eRcv()) continue;//return 0;
        if (debug) Serial.println("Send SYST");
        client.println(F("SYST"));

        if (!eRcv()) continue;//return 0;
        if (debug) Serial.println("Send Type I");
        client.println(F("Type I"));

        if (!eRcv()) continue;//return 0;
        if (debug) Serial.println("Send PASV");
        client.println(F("PASV"));

        if (!eRcv()) continue;//return 0;

        char *tStr = strtok(outBuf, "(,");
        int array_pasv[6];
        for ( int i = 0; i < 6; i++) {
          tStr = strtok(NULL, "(,");
          array_pasv[i] = atoi(tStr);
          if (tStr == NULL) {
            Serial.println(F("Bad PASV Answer"));
          }
        }
        unsigned int hiPort, loPort;
        hiPort = array_pasv[4] << 8;
        loPort = array_pasv[5] & 255;

        if (debug) Serial.print(F("Data port: "));
        hiPort = hiPort | loPort;
        if (debug) Serial.println(hiPort);

        if (dclient.connect(FTPserver, hiPort)) {
          Serial.println(F("Data connected"));
        }
        else {
          Serial.println(F("Data connection failed"));
          client.stop();
          continue;//return 0;
        }

        if (debug) Serial.println("Send STOR filename");
        client.print(F("STOR "));
        client.println(fileName);

        if (!eRcv()) {
          dclient.stop();
          continue;//return 0;
        }
      }
    }
    if (tState == 1) { // uploading data
      while (FTPBUFSZ - xRingbufferGetCurFreeSize(ringBufAcc) >= FTPPKTSZ) {

        size_t item_size;
        char *item = (char *)xRingbufferReceiveUpTo(ringBufAcc, &item_size, 0, FTPPKTSZ);
        if (item != NULL) {
          dclient.write(item, FTPPKTSZ);
          vRingbufferReturnItem(ringBufAcc, (void *)item);
          if (debug) Serial.printf("C %d %d\n", xRingbufferGetCurFreeSize(ringBufAcc), (FTPBUFSZ - xRingbufferGetCurFreeSize(ringBufAcc)));
        }
      }

      // no more sampling now, data less than one packet left
      if ((samplingState == 2) && (FTPBUFSZ - xRingbufferGetCurFreeSize(ringBufAcc) < FTPPKTSZ) ) {

        tState = 0;
        Serial.println("D");
        // upload remaining data
        size_t dataLen = FTPBUFSZ - xRingbufferGetCurFreeSize(ringBufAcc);

        size_t item_size;
        char *item = (char *)xRingbufferReceiveUpTo(ringBufAcc, &item_size, 0, dataLen);
        if (item != NULL) {
          dclient.write(item, dataLen);
          vRingbufferReturnItem(ringBufAcc, (void *)item);
        }

        // close file
        dclient.stop();
        if (debug) Serial.println(F("Data disconnected"));

        if (!eRcv()) continue;//return 0;

        client.println(F("QUIT"));

        if (!eRcv()) continue;//return 0;

        client.stop();
        if (debug) Serial.println(F("Command disconnected"));
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


// Acc
const TickType_t x5ms = pdMS_TO_TICKS( 5 );



// accelerometer
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value) {
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  digitalWrite(CS0, LOW);           //Set Chip Select pin low to signal the beginning of an SPI packet.
  hspi->transfer(registerAddress);  //Transfer the register address over SPI.
  hspi->transfer(value);            //Transfer the desired register value over SPI.
  digitalWrite(CS0, HIGH);          //Set the Chip Select pin high to signal the end of an SPI packet.
  hspi->endTransaction();
}

//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values) {
  char address = 0x80 | registerAddress;  // Since we're performing a read operation, the most significant bit of the register address should be set.
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if (numBytes > 1) {
    address = address | 0x40;
  }
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  digitalWrite(CS0, LOW);  // Set the Chip select pin low to start an SPI packet.
  hspi->transfer(address);  //Transfer the starting register address that needs to be read.

  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for (int i = 0; i < numBytes; i++) {
    values[i] = hspi->transfer(0x00);
  }
  digitalWrite(CS0, HIGH);  //Set the Chips Select pin high to end the SPI packet.
  hspi->endTransaction();
}



void readAccT(void * parameter) {
#define BUFACCSZ 6*32
  char bufAcc[BUFACCSZ];
  //initialise hspi with default pins
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  pinMode(CS0, OUTPUT);    // Set up slave select (CS) pin as output
  digitalWrite(CS0, HIGH); // Before communication starts, the Chip Select pin needs to be set high

  hspi = new SPIClass(HSPI);
  hspi->begin();

  writeRegister(POWER_CTL, 0x00);
  writeRegister(BW_RATE, ADXL345_3200HZ);
  writeRegister(DATA_FORMAT, 0x00); // 10 bit 2g
  writeRegister(POWER_CTL, 0x08);   // Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(FIFO_CTL, 0B11011111); //11 FIFO,stream;0 trigger int1;11111 32samples


  Serial.println(pcTaskGetTaskName(NULL));
  // at 100Hz
  //   read 32 layers of FIFO
  //     each layer having 6 bytes
  while (1) {
    if (samplingState == 1) {
      // if we lose data, this should happen in interrupt
      // this should happen in less than one data read time of acc
      for (int i = 0; i < 32; i++) {
        readRegister (0x32, 6, bufAcc + (i * 6));
      }
      // if space not available
      if (xRingbufferGetCurFreeSize(ringBufAcc) < BUFACCSZ) {
        if (debug) Serial.println("B");
        continue;
      }
      // move to ring buffer
      UBaseType_t res = xRingbufferSend(ringBufAcc, (void *)bufAcc, BUFACCSZ, 0); // todo need check here
      if (res != pdTRUE) Serial.println(F("ringBufAcc data not copied"));

      //    for (int i = 0; i < 6; i++) {
      //      //if (i % 6 == 0) Serial.printf("\n");
      //      Serial.printf("%02X,", fifobuf[i]);
      //    }
      //    Serial.println();
    }
    // sleep for 1/10 second
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
////
void updateTimeT(void * parameter) {
  while (WiFi.status() != WL_CONNECTED) {} // wait for wifi
  Serial.println(pcTaskGetTaskName(NULL));
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  while (1) {
    if ( ! WiFi.isConnected()) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    if ( ! getLocalTime(&timeinfo)) {
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      Serial.println(F("Trying to obtain time"));
      continue;
    }
    vTaskDelay(60 * 60 * 1000 / portTICK_PERIOD_MS);
  }
}

void keepWiFiAliveT(void * parameter) {

  while (1) {
    if (WiFi.status() == WL_CONNECTED) {
      vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);
      continue;
    }

    WiFi.disconnect();
    Serial.println(pcTaskGetTaskName(NULL));
    Serial.println(F("Connecting"));

    if (WiFi.getMode() != WIFI_STA) {
      WiFi.mode(WIFI_STA);
    }
    //WiFi.setHostname(DEVICE_NAME);
    WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
    vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
    // If we couldn't connect within the timeout period, retry in x seconds.
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(F("[WIFI] FAILED"));
      vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
      continue;
    }
    Serial.println(WiFi.localIP());
  }
}

void serverT(void * parameter) {
  String cline, header;
  cline.reserve(256);
  header.reserve(256);

  wifiServer.begin();
  Serial.println(pcTaskGetTaskName(NULL));

  while (1) {
    WiFiClient cmd_client = wifiServer.available();
    if (cmd_client) {
      cline = "";
      while (cmd_client.connected()) {
        if (cmd_client.available()) {
          char c = cmd_client.read();
          Serial.write(c);
          header += c;
          if (c == '\n') {
            if (cline.length() == 0) {
              cmd_client.println("HTTP/1.1 200 OK");
              cmd_client.println("Content-type:text/html");
              cmd_client.println("Connection: close");
              cmd_client.println();

              if (header.indexOf("GET /sample/start") >= 0) {
                Serial.println("Sampling start");
                samplingState = 1;
                samplingMillis = millis();
              }
              else if (header.indexOf("GET /sample/stop") >= 0) {
                Serial.println("Sampling stop");
                samplingState = 2;
              }
              // Display the HTML web page
              cmd_client.println("<!DOCTYPE html><html>");
              cmd_client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              cmd_client.println("<link rel=\"icon\" href=\"data:,\">");
              // CSS to style the on/off buttons
              // Feel free to change the background-color and font-size attributes to fit your preferences
              cmd_client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
              cmd_client.println(".button { background-color: #00FF00; border: none; color: white; padding: 16px 40px;");
              cmd_client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
              cmd_client.println(".button2 {background-color: #FF0000;}</style></head>");

              // Web Page Heading
              cmd_client.println("<body><h1>Data server</h1>");

              // Sampling State
              // cmd_client.println("<p>Sampling State " + (samplingState) + "</p>");
              if (samplingState == 1) {
                cmd_client.println("<p><a href=\"/sample/stop\"><button class=\"button button2\">Stop</button></a></p>");
              } else {
                cmd_client.println("<p><a href=\"/sample/start\"><button class=\"button\">Start</button></a></p>");
              }
              cmd_client.println("</body></html>");
              // The HTTP response ends with another blank line
              cmd_client.println();

              break;
            }
            else {
              cline = "";
            }
          }
          else if (c != '\r') {
            cline += c;
          }
        }
        //delay(1);
      }
      header = "";
      cmd_client.stop();
      Serial.println("Cmd_Client disconnected");
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
////////////////
void initmDNS() {
  
  //if (!MDNS.begin(uniqId)) {
  if (!MDNS.begin(DEVICE_NAME)) {
    Serial.println("Error setting up MDNS responder!");
    signalError();
  }
  Serial.println("mDNS responder started");

  FTPserver = MDNS.queryHost(mDnsHost);
  while (FTPserver.toString() == "0.0.0.0") {
    Serial.println("Trying again to resolve mDNS");
    delay(250);
    FTPserver = MDNS.queryHost(mDnsHost);
  }
  Serial.print("FTP server: ");
  Serial.println(FTPserver.toString());
}
void connectWifi() {
  Serial.print("Connecting as wifi client to SSID: ");
  Serial.println(WIFI_NETWORK);

  // use in case of mode problem
  WiFi.disconnect();
  // switch to Station mode
  if (WiFi.getMode() != WIFI_STA) {
    WiFi.mode(WIFI_STA);
  }

  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);

  //if (debug ) WiFi.printDiag(Serial);

  // ... Give ESP 10 seconds to connect to station.
  // unsigned long startTime = millis();
  // while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
  while (WiFi.status() != WL_CONNECTED) {
    //blinker(500);
    delay(100);
    digitalWrite(LED_BUILTIN, 1 - digitalRead(LED_BUILTIN));
    Serial.print(".");
  }
  Serial.println("");
  // Check connection
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected; IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.print("WiFi connect failed to ssid: ");
    Serial.println(WIFI_NETWORK);
  }
}  // connectWiFi()

void signalError() {  // loop endless with LED blinking in case of error
  while (1) {
    digitalWrite(LED_BUILTIN, 1 - digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void blinker(int wait) {
  //digitalWrite(LED_BUILTIN, 1 - digitalRead(LED_BUILTIN));
  digitalWrite(LED_BUILTIN, HIGH);
  delay(wait);
  digitalWrite(LED_BUILTIN, LOW);
  delay(10);
}




/*
  Documentation links
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_additions.html

  ESP.restart();
  xEventGroupWaitBits
*/
