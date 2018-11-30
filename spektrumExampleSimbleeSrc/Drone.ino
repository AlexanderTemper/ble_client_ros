#include <SimbleeBLE.h>
// pololu Libary https://github.com/pololu/vl53l0x-arduino
#include <Wire.h>
#include <VL53L0X.h>

#define SPEKTRUM_DSMX_11 0xb2
#define SPEKTRUM_BAUDRATE 115200
#define MASK_2048_SXPOS 0x07FF 
#define SPEK_FRAME_SIZE 16

#define START_TIMER cpuUsedStart = micros();
#define STOP_TIMER cpuUsed = cpuUsed + (micros()-cpuUsedStart);

// Timer using micros => overflow after 70 minutes
unsigned long startMicros; 
unsigned long currentMicros;

unsigned long cpuUsed;
unsigned long cpuUsedStart;

unsigned long cpuTotalStart;

unsigned long statTimer;
const unsigned long period = 11*1000;

// Spektrum system type values
uint8_t spekFrame[SPEK_FRAME_SIZE];
uint16_t spekChannelData[12];
uint16_t bledata[6] = {0, 1024, 1024, 1024, 0, 0};
bool spekFrameSent = true;
uint8_t spekFrameMissedCount = 0;
uint8_t spekFrameSentCount = 0;

// Telemetry
char telemetryInputBuffer[20]={0};
char telemetryInputBytes = 0;
char telemetryLastChar = 0;

char telemetryData[20]={0};
bool telemetrySent = true;
char telemetryBytes = 0;

// Sensor
VL53L0X sensor;
uint16_t rangeMilliMeter = 0;

void setup() {
    // Uart
    override_uart_limit = true; 
    Serial.begin(SPEKTRUM_BAUDRATE,3,2);//baud rx tx
    // Timers
    cpuTotalStart= statTimer = startMicros = micros(); 
    cpuUsed = 0;
    // tof Sensor
    Wire.begin();
    sensor.init();
    sensor.setTimeout(400);
    sensor.startContinuous();
    pinMode(4, INPUT); // Interupt pin (Active Low)
    // Simblee
    SimbleeBLE.deviceName = "Drone";
    SimbleeBLE.begin();
    // telemetry
    telemetryInputBytes = 0;
    telemetryLastChar = 0;
    telemetrySent = true;
    telemetryBytes = 0;
    // Spektrum
    spekFrameSent = true;
    spekFrameMissedCount = 0;
    spekFrameSentCount = 0;
}


/*
 * main loop
 */
void loop() {
    currentMicros = micros(); 
    
    
    if (currentMicros - startMicros >= period)  {
        START_TIMER
            if(!spekFrameSent){
                send_Spektrum_frame();
                spekFrameSent = true;
                spekFrameSentCount ++;
            } else {
                spekFrameMissedCount ++;
            }
            
            if(!SimbleeBLE.radioActive){ 
                send_tofSensor_data();
                send_LTM_data();
            }
            
            startMicros = currentMicros; // Resart Period
        STOP_TIMER
        
        send_statistik();
    }
    
    
    START_TIMER
        read_tofSensor();
        get_LTM_data();
    STOP_TIMER
    
}

/* update rangeMilliMeter if new data from the sensor is available */
void read_tofSensor(){
    if(digitalRead(4)==LOW){
        rangeMilliMeter = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
        sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    }
}

void send_statistik(){
    // reset Timers
    unsigned long cpuTotal = micros() - cpuTotalStart;
    
    if(micros()-statTimer >= 1000000){
        char data[13] = {'$','T','Z', cpuUsed & 0xFF,(cpuUsed >>  8) & 0xFF,(cpuUsed >> 16) & 0xFF,(cpuUsed >> 24) & 0xFF, cpuTotal & 0xFF,(cpuTotal >>  8) & 0xFF,(cpuTotal >> 16) & 0xFF,(cpuTotal >> 24) & 0xFF,spekFrameMissedCount,spekFrameSentCount};
        SimbleeBLE.send(data, 13);
        statTimer = micros();
        spekFrameMissedCount = 0;
        spekFrameSentCount = 0;
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
    if(spekFrameSent){
        for(int i,e = 0;i<12 && i<len;i=i+2,e++){
            bledata[e]=(((uint16_t)dataBLE[i+1])<<8) + (uint16_t)dataBLE[i];
        } 
        spekFrameSent = false;
    }
}

/*
 * send the Sensor Data over ble
 */
void send_tofSensor_data()
{
    char data[5] = {'$','T','L', (char)(rangeMilliMeter & 0x00FF), (char)(rangeMilliMeter  >> 8)};
    SimbleeBLE.send(data, 5);
}

/* 
 * Look if there are new LTM data available
 */
void get_LTM_data() 
{
    char data= 0;
    while(Serial.available()){ // read all available data
        data = Serial.read();
        if (data == 'T' && telemetryLastChar == '$'){ // new Frame
            memcpy(telemetryData, telemetryInputBuffer, 20); // Copy new Data
            telemetryBytes = telemetryInputBytes;
            telemetrySent = false;
            telemetryInputBytes=0;
            addData('$');
            addData('T');
        } else if(telemetryLastChar == '$'){
            addData(telemetryLastChar);
            addData(data);
        } else if(data != '$'){
            addData(data);
        } 
        telemetryLastChar = data;
    }
}

// send telemetry if new Data are available
void send_LTM_data(){
    if(!telemetrySent){
        SimbleeBLE.send(telemetryData,telemetryBytes);
        telemetrySent = true;
    }
}

/*
 * add LTM data to the telemetryInputBuffer buffer max 20 Bytes
 */
void addData( char data)
{
    if(telemetryInputBytes<20){
        telemetryInputBuffer[telemetryInputBytes]= data;
        telemetryInputBytes++;
    }
}

