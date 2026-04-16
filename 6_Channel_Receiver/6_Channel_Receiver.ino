#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>      //無限收發用的
#include <Servo.h>     //伺服馬達用的

const uint64_t pipeIn = 0xABCDABCD71LL; //相同的接收發碼
RF24 radio(9, 10);

int ch_width_1 = 0;    //通道1到6
int ch_width_2 = 0;
int ch_width_3 = 0;
int ch_width_4 = 0;
int ch_width_5 = 0;
int ch_width_6 = 0;

Servo ch1;    //伺服馬達控制
Servo ch2;
Servo ch3;
Servo ch4;
Servo ch5;
Servo ch6;

struct Signal {   //一樣建立結構

byte throttle;    //油門
byte pitch;       //上下
byte roll;        //左右
byte yaw;         //翻滾
byte aux1;        //3段開關
byte aux2;        //3段開關

};

Signal data;


void ResetData()  //建立中位
{

data.throttle = 0;                            
data.roll = 127;
data.pitch = 127;
data.yaw = 127;
data.aux1 = 0;
data.aux2 = 0;

}

void setup()
{
  ch1.attach(2);    //設定類比腳位
  ch2.attach(3);
  ch3.attach(4);
  ch4.attach(5);
  ch5.attach(6);
  ch6.attach(7);

  ResetData();                                   //跟發射器是一樣的設定只是改成接收          
  radio.begin();
  radio.openReadingPipe(1,pipeIn);
  radio.setChannel(100);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);                         
  radio.setPALevel(RF24_PA_MAX);                       
  radio.startListening();                                  
}

unsigned long lastRecvTime = 0;                // 記錄最後一次收到無線資料的時間

void recvData()
{
while ( radio.available() ) {
radio.read(&data, sizeof(Signal));
lastRecvTime = millis();                                    // 更新最後接收到資料的時間
}
}

void loop()
{
recvData();
unsigned long now = millis();
if ( now - lastRecvTime > 1000 ) {
ResetData();                                                // 超過 1 秒未收到訊號，重置為安全的預設值
}

ch_width_1 = map(data.roll, 0, 255, 1000, 2000);             //轉化收到的數值到PWM訊號
ch_width_2 = map(data.pitch, 0, 255, 1000, 2000);
ch_width_3 = map(data.throttle, 0, 255, 1000, 2000);
ch_width_4 = map(data.yaw, 0, 255, 1000, 2000);
ch_width_5 = map(data.aux1, 0, 1, 1000, 2000);
ch_width_6 = map(data.aux2, 0, 1, 1000, 2000);

ch1.writeMicroseconds(ch_width_1);                          //把PWM訊號輸出到腳位
ch2.writeMicroseconds(ch_width_2);
ch3.writeMicroseconds(ch_width_3);
ch4.writeMicroseconds(ch_width_4);
ch5.writeMicroseconds(ch_width_5);
ch6.writeMicroseconds(ch_width_6);

}
