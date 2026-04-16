  #include <SPI.h>
  #include <nRF24L01.h>
  #include <RF24.h>             //匯入函式庫 特別是RF24是無限收發用的 要特別下載
  const uint64_t pipeOut = 0xABCDABCD71LL;//設定收發碼 接收發器藥一樣不然不能動
  RF24 radio(9, 10);     // 設定CE,CSN pin 無線模組的腳位 也只有這兩個腳位能亂改

  struct Signal { //建立結構裡面可以放整數、字元、字串之類的 可以想像成樣品的概念
  byte throttle; //油門
  byte pitch;    //起降
  byte roll;     //左右翻滾
  byte yaw;      //向左右飛
  byte aux1;    //3段開關
  byte aux2;    //3段開關

};
  Signal data; //創建名為data的signal結構 像是以signal為樣品建立data這個產品
  
  void ResetData() //建立初始值的函式
{
  data.throttle = 0;
  data.pitch = 127;
  data.roll = 127;
  data.yaw = 127;
  data.aux1 = 0;
  data.aux2 = 0;

}
  void setup()
{
//這整段就是在啟動無線通訊模組，收或發
//並設立頻道，設定資料流量，通訊範圍(強度)，呼叫初始值函式
  radio.begin();
  radio.openWritingPipe(pipeOut);
  radio.setChannel(100);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);    
  radio.setPALevel(RF24_PA_MAX);     
  radio.stopListening();             
  ResetData();

}
                                      
  int Border_Map(int val, int lower, int middle, int upper, bool reverse)
{// 將原始值轉到 0~255 的範圍，方便傳送。
  val = constrain(val, lower, upper);
  if ( val < middle )
  val = map(val, lower, middle, 0, 128);
  else
  val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}
  void loop() 
{
//最後執行 會接收搖桿的訊號值大小並輸出 
//最後一行就是打包送出資料，true或false是操控方向 就是伺服移動的方向變相反
  data.roll = Border_Map( analogRead(A3), 0, 512, 1023, true );        // CH1   
  data.pitch = Border_Map( analogRead(A0), 0, 512, 1023, true );       // CH2
  data.throttle = Border_Map( analogRead(A2),0, 340, 570, true );      
  // CH3 這個是馬達
  // data.throttle = Border_Map( analogRead(A2),0, 512, 1023, true );  
  // CH3 這是從最底下開始轉搖桿 但我這個是從中間開始
  data.yaw = Border_Map( analogRead(A1), 0, 512, 1023, false );        // CH4
  data.aux1 = digitalRead(0);                                          // CH5
  data.aux2 = digitalRead(3);                                          // CH6

  radio.write(&data, sizeof(Signal));
}
