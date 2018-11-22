#include <SimbleeBLE.h>
#define SPEKTRUM_DSMX_11 0xb2
#define SPEKTRUM_2048_CHANNEL_COUNT 12
#define SPEKTRUM_NEEDED_FRAME_INTERVAL     5000
#define SPEKTRUM_BAUDRATE 115200
#define MASK_2048_CHANID 0x7800 
#define MASK_2048_SXPOS 0x07FF 
#define SPEK_FRAME_SIZE 16

unsigned long startMillis;  
unsigned long currentMillis;
const unsigned long period = 11;

// Spektrum system type values

uint8_t spekFrame[SPEK_FRAME_SIZE];
uint16_t spekChannelData[12];
uint8_t myByte = 0;

uint16_t bledata[6] = {0, 1024, 1024, 1024, 0, 0};
bool send = true;

void setup() {
  override_uart_limit = true; 
  Serial.begin(SPEKTRUM_BAUDRATE,3,2);//baud rx tx
  startMillis = millis();  //initial start time 
  SimbleeBLE.deviceName = "Drone";
  SimbleeBLE.begin();
}



void loop() {
  currentMillis = millis(); 
  if (currentMillis - startMillis >= period)  {
    
    if(!send){     // Sende nur wenn Daten da sind
      Serial.write(SPEKTRUM_DSMX_11); //start 
      Serial.write((uint8_t)00);      // missed frame count
  
      for(uint16_t i = 0; i< 7;i++){ // mit den 2 Start bytes => 16 bytes
        uint16_t data =  (i << 11)+(0 & MASK_2048_SXPOS);
        if(i<6){
          data = (i << 11)+(bledata[i] & MASK_2048_SXPOS);
        }
        
        Serial.write((uint8_t)(data >> 8));//Send Upper Byte first
        Serial.write((uint8_t)(data));// Send Lower Byte
        send = true;
      }
    }
    startMillis = currentMillis; 
  }
  get_data();
}

void SimbleeBLE_onReceive(char *dataBLE, int len){
  if(send){
    for(int i,e = 0;i<12 && i<len;i=i+2,e++){
      bledata[e]=(((uint16_t)dataBLE[i+1])<<8) + (uint16_t)dataBLE[i];
    } 
    send = false;
  }
}

char telemetryData[20]={0};
char bufferpointer = 0;
char lastchar = 0;

void get_data() 
{
  char data= 0;
 while(Serial.available()){

    data = Serial.read();

    if(data == '$'){
      
    }
    else if (data == 'T' && lastchar == '$'){ // new Frame
        SimbleeBLE.send(telemetryData,bufferpointer); //send old Frame
        bufferpointer=0;
        addData('$');
        addData('T');
        
    } else if(lastchar == '$'){
      addData(lastchar);
      addData(data);
    } else {
      addData(data);
    }

    lastchar = data;
 }
}

void addData( char data){
  if(bufferpointer<20){
    telemetryData[bufferpointer]= data;
    bufferpointer++;
  }

}
