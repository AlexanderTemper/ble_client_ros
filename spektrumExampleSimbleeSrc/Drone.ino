#include <SimbleeBLE.h>
#include "Adafruit_VL53L0X.h"

#define SPEKTRUM_DSMX_11 0xb2
#define SPEKTRUM_2048_CHANNEL_COUNT 12
#define SPEKTRUM_NEEDED_FRAME_INTERVAL     5000
#define SPEKTRUM_BAUDRATE 115200
#define MASK_2048_CHANID 0x7800 
#define MASK_2048_SXPOS 0x07FF 
#define SPEK_FRAME_SIZE 16

#define START_TIMER cpuUsedStart = micros();
#define STOP_TIMER cpuUsed = cpuUsed + (micros()-cpuUsedStart);

// Timer
unsigned long startMillis; 
unsigned long currentMillis;

unsigned long cpuUsed;
unsigned long cpuUsedStart;

unsigned long cpuTotalStart;


unsigned long statTimer;
const unsigned long period = 11*1000;


// Spektrum system type values
uint8_t spekFrame[SPEK_FRAME_SIZE];
uint16_t spekChannelData[12];
uint8_t myByte = 0;
uint16_t bledata[6] = {0, 1024, 1024, 1024, 0, 0};
bool send = true;
uint8_t spekFrameMissed = 0;
uint8_t spekFrameSend = 0;

// Telemetry
char telemetryData[20]={0};
char bufferpointer = 0;
char lastchar = 0;

// Sensor
Adafruit_VL53L0X tofSensor = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
uint16_t rangeMilliMeter = 0;
uint8_t rangeStatus = 0;
bool sendTOF = false;

void setup() {
    
    override_uart_limit = true; 
    Serial.begin(SPEKTRUM_BAUDRATE,3,2);//baud rx tx
    cpuTotalStart= statTimer = startMillis = micros(); 
    cpuUsed = 0;
    SimbleeBLE.deviceName = "Drone";
    SimbleeBLE.begin();
    tofSensor.begin();
}


/*
 * main loop
 */
void loop() {
    currentMillis = micros(); 
    
    START_TIMER
    if((currentMillis - startMillis >= 6000) && !sendTOF){
        if(!SimbleeBLE.radioActive){
            tofSensor.rangingTest(&measure, false);
            send_tofSensor_data();
            sendTOF=true;
        }
    }
    STOP_TIMER
    
    if (currentMillis - startMillis >= period)  {
        START_TIMER
            if(!send){
                send_Spektrum_frame();
                send = true;
                spekFrameSend ++;
            } else {
                spekFrameMissed ++;
            }
        
            sendTOF = false; 
            startMillis = currentMillis; 
        STOP_TIMER
            
        // End of Period send Timings
        send_statistik();
    }
    
    START_TIMER
        get_LTM_data();
    STOP_TIMER
}

void send_statistik(){
    // reset Timers
    unsigned long cpuTotal = micros() - cpuTotalStart;
    
    
    if(micros()-statTimer >= 1000000){
        char data[13] = {'$','T','Z', cpuUsed & 0xFF,(cpuUsed >>  8) & 0xFF,(cpuUsed >> 16) & 0xFF,(cpuUsed >> 24) & 0xFF, cpuTotal & 0xFF,(cpuTotal >>  8) & 0xFF,(cpuTotal >> 16) & 0xFF,(cpuTotal >> 24) & 0xFF,spekFrameMissed,spekFrameSend};
        SimbleeBLE.send(data, 13);
        statTimer = micros();
        spekFrameMissed = 0;
        spekFrameSend = 0;
    }
    
    cpuTotalStart = micros();
    cpuUsed = 0;
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


