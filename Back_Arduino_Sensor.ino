// CAN bus libs
#include <SPI.h>
#include <mcp_can.h>

const byte _hall_sensor_pin = 3;

const byte _hall_sensor_to_front = 7;

const int SPI_CS_PIN = 9;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

// init CAN bus vars
unsigned char flagRecv = 0;
byte len = 0;
byte buf[8];
char str[20];


unsigned int caseCan;

int rpm=0;

void hallDetectedInterrupt() {
    
    digitalWrite(_hall_sensor_to_front, LOW);
    delay(15);
    digitalWrite(_hall_sensor_to_front, HIGH);

}

void setup()
{
    Serial.begin(115200);
    pinMode(_hall_sensor_to_front, OUTPUT);
    digitalWrite(_hall_sensor_to_front, HIGH);
    pinMode(_hall_sensor_pin, INPUT_PULLUP);
    // Interrupt is called when the hall sensor value rises
    attachInterrupt(digitalPinToInterrupt(_hall_sensor_pin), hallDetectedInterrupt, RISING);

    START_INIT:

    if(CAN_OK == CAN.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        //Serial.println("CAN BUS Shield init fail");
        //Serial.println("Init CAN BUS Shield again");
        delay(100);
        goto START_INIT;
    }

    attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt


    /*
     * set mask, set both the mask to 0x3ff
     */
    CAN.init_Mask(0, 0, 0x3ff);                         // there are 2 mask in mcp2515, you need to set both of them
    CAN.init_Mask(1, 0, 0x3ff);


    /*
     * set filter, we can receive id from 0x04 ~ 0x09
     */
    CAN.init_Filt(0, 0, 0x04);                          // there are 6 filter in mcp2515
    CAN.init_Filt(1, 0, 0x05);                          // there are 6 filter in mcp2515

    CAN.init_Filt(2, 0, 0x06);                          // there are 6 filter in mcp2515
    CAN.init_Filt(3, 0, 0x07);                          // there are 6 filter in mcp2515
    CAN.init_Filt(4, 0, 0x08);                          // there are 6 filter in mcp2515
    CAN.init_Filt(5, 0, 0x09);                          // there are 6 filter in mcp2515
    

}

void MCP2515_ISR()
{                                                                                                                      
    flagRecv = 1;
}

void loop()
{   
    /*if (rpm > 5000)
    {
       rpm=0;
       
    }
    rpm = rpm+30;
    Serial.println(rpm);
    delay(50);*/
    //Serial.println();
      
    // read and parse data from CAN Bus 
    /*********CAN.readMsgBuf(&len, buf);
    caseCan = CAN.getCanId();      
    Serial.println(caseCan);                                                     
    switch (caseCan) {
      case 1512:
      rpm = buf[2] + buf[3];
      Serial.println(caseCan);  
    /*switch (caseCan) {
        case 1520:
          // RPMs are the 7th and 8th byte from CAN ID 1520
          rpm.input = 256*buf[6]+buf[7];
          break;
      Serial.println(rpm);*/
      //break;

    delay(3);
    /*if(flagRecv)                   // check if get data
    {

        flagRecv = 0;                // clear flag
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        Serial.println("\r\n------------------------------------------------------------------");
        Serial.print("Get Data From id: ");
        Serial.println(CAN.getCanId());
        for(int i = 0; i<len; i++)    // print the data
        {
            Serial.print("0x");
            Serial.print(buf[i], HEX);
            Serial.print("\t");
        }
        Serial.println("HERE");
    */
  //}
  
}
