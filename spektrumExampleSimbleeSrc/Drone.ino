#include <SimbleeBLE.h>
#include "Adafruit_VL53L0X.h"

#define SPEKTRUM_DSMX_11 0xb2
#define SPEKTRUM_2048_CHANNEL_COUNT 12
#define SPEKTRUM_NEEDED_FRAME_INTERVAL     5000
#define SPEKTRUM_BAUDRATE 115200
#define MASK_2048_CHANID 0x7800 
#define MASK_2048_SXPOS 0x07FF 
#define SPEK_FRAME_SIZE 16

// Timer
unsigned long startMillis; 
unsigned long currentMillis;
const unsigned long period = 11;


// Spektrum system type values
uint8_t spekFrame[SPEK_FRAME_SIZE];
uint16_t spekChannelData[12];
uint8_t myByte = 0;
uint16_t bledata[6] = {0, 1024, 1024, 1024, 0, 0};
bool send = true;

// Telemetry
char telemetryData[20]={0};
char bufferpointer = 0;
char lastchar = 0;

// Sensor
Adafruit_VL53L0X tofSensor = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
uint16_t rangeMilliMeter = 0;
uint8_t rangeStatus = 0;

void setup() {
    override_uart_limit = true; 
    Serial.begin(SPEKTRUM_BAUDRATE,3,2);//baud rx tx
    startMillis = millis(); 
    SimbleeBLE.deviceName = "Drone";
    SimbleeBLE.begin();
    tofSensor.begin();
}


/*
 * main loop
 */
void loop() {
    currentMillis = millis(); 
    if (currentMillis - startMillis >= period)  {
        
        if(!send){
            send_Spektrum_frame();
            send = true;
        }
        tofSensor.rangingTest(&measure, false);
        send_tofSensor_data();
        
        startMillis = currentMillis; 
    }
    get_LTM_data();
}

/* 
 * send spektrum frame over the Uart 
 */
void send_Spektrum_frame(){
    // 16 Bytes
    Serial.write(SPEKTRUM_DSMX_11); //start 
    Serial.write((uint8_t)00);      // missed frame count
    
    for(uint16_t i = 0; i< 7;i++){ 
        uint16_t data =  (i << 11)+(0 & MASK_2048_SXPOS);// channel number
        if(i<6){//currently only 6 cannels are used
            data = (i << 11)+(bledata[i] & MASK_2048_SXPOS);// add Data
        }
        
        Serial.write((uint8_t)(data >> 8));//Send Upper Byte first
        Serial.write((uint8_t)(data));// Send Lower Byte
    }
}
/*
 * callack for BLE
 */
void SimbleeBLE_onReceive(char *dataBLE, int len){
    if(send){
        for(int i,e = 0;i<12 && i<len;i=i+2,e++){
            bledata[e]=(((uint16_t)dataBLE[i+1])<<8) + (uint16_t)dataBLE[i];
        } 
        send = false;
    }
}

/*
 * send the Sensor Data over ble
 */
void send_tofSensor_data()
{
    if (measure.RangeStatus != 4) {
        char data[5] = {'$','T','L', (char)(measure.RangeMilliMeter & 0x00FF), (char)(measure.RangeMilliMeter  >> 8)};
        SimbleeBLE.send(data, 5);
    } 
}

/* 
 * look if there are new data from LTM
 */
void get_LTM_data() 
{
    char data= 0;
    while(Serial.available()){ // read all available data
        data = Serial.read();
        if (data == 'T' && lastchar == '$'){ // new Frame
            SimbleeBLE.send(telemetryData,bufferpointer); //send old Frame
            bufferpointer=0;
            addData('$');
            addData('T');
        } else if(lastchar == '$'){
            addData(lastchar);
            addData(data);
        } else if(data != '$'){
            addData(data);
        } 
        lastchar = data;
    }
}

/*
 * add LTM data to the telemetryData buffer
 */
void addData( char data)
{
    if(bufferpointer<20){
        telemetryData[bufferpointer]= data;
        bufferpointer++;
    }
}
