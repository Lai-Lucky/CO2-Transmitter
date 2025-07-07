// 攀登：环境综合传感器（CO2） - OneNet 物联网接入
// 供电：12V
// 通信：RS485-TTL

 
#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h> // 添加 FreeRTOS 头文件
#include <freertos/task.h>     // 添加 FreeRTOS 任务管理头文件
#include <freertos/semphr.h>   // 添加 FreeRTOS 信号量头文件

#define M0 23 //M0控制引脚
#define M1 22 //M1控制引脚


/************** 函数声明 ***************/
uint16_t CRC16(const uint8_t *data, uint16_t length);
bool checkCRC(const uint8_t *data, uint16_t len);
void parseModbusData(const uint8_t *data, uint16_t len);
void sendSensorData(double data) ;
void ZigBeec_controller(int switchs);
void sensor_task_vtask(void *pv); //传感器任务

TaskHandle_t sensor_task_handle;  //传感器任务句柄

/************* 询问命令 *************/
const byte send_byte[1][8] = {
  {0x01,0x03,0x00,0x05,0x00,0x01,0x94,0x0B} // 二氧化碳
};

/************ OneNet平台的属性标识符 ************/
const char* sensor_names[] = {"CO2"/*二氧化碳*/ };

/******** 变量 ********/
byte temp[32]; // 传感器返回数据
int asr = 0;  // 传感器轮询索引

/************* 程序初始化 *************/
void setup() {
  pinMode(22,OUTPUT);
  pinMode(23,OUTPUT);

  Serial.begin(115200);
  Serial2.begin(9600);

  xTaskCreate(sensor_task_vtask,"sensor_task_vtask",4096,NULL,1,&sensor_task_handle);//创建土壤传感器任务
  
}

/************* 主循环 *************/
void loop() {
  
}

/************* CRC 计算 *************/
uint16_t CRC16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) 
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) 
    {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

/************* CRC 校验 *************/
bool checkCRC(const uint8_t *data, uint16_t len) {
  if (len < 5) return false; // 确保至少有地址、功能码、数据、CRC
  uint16_t computedCRC = CRC16(data, len - 2);
  uint16_t receivedCRC = (data[len - 1] << 8) | data[len - 2]; // 注意顺序
  return computedCRC == receivedCRC;
}


/************* 解析数据并上传 *************/
void parseModbusData(const uint8_t *data, uint16_t len) {

  if (data[1] == 0x03) 
  {
    uint16_t dataLength = data[2]; // 数据字节数

    if (dataLength == 2) 
    {  // 7字节
      uint16_t regValue = (data[3] << 8) | data[4];
      int32_t value = regValue ;
      sendSensorData(value);
    } 
  }
}



/************JSON数据构建************/
void sendSensorData(double data) 
{
  vTaskDelay(pdMS_TO_TICKS(100));
  JsonDocument doc;
  doc["id"] = String(millis());  // 使用时间戳作为唯一ID
  doc["version"] = "1.0";
  doc["params"][sensor_names[asr]]["value"]= data;

  String payload;
  serializeJson(doc, payload);

   //ZigBee模块全双工
  ZigBeec_controller(1);
  vTaskDelay(pdMS_TO_TICKS(200));

  Serial.println(payload.c_str());
  vTaskDelay(pdMS_TO_TICKS(1000));

  //ZigBee模块休眠
  ZigBeec_controller(0);
  vTaskDelay(pdMS_TO_TICKS(200));

  vTaskDelay(pdMS_TO_TICKS(100));

}

/********************* 传感器任务 *********************/
void sensor_task_vtask(void *pv){

  while(1)
  {
    // 发送请求
    Serial2.write(send_byte[asr], 8);

    vTaskDelay(pdMS_TO_TICKS(200));

    // 读取响应数据
    if (Serial2.available()) 
    {
      int len = Serial2.available();
      Serial2.readBytes(temp, len);
      parseModbusData(temp, len);

     if (checkCRC(temp, len)) 
      {
        asr = 0;
      } 

      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}


/************ZigBee模块控制************/
void ZigBeec_controller(int switchs){
  if(switchs==1)
  {
    digitalWrite(M1,LOW);//M1
    digitalWrite(M0,HIGH);//M0
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  else
  {
    digitalWrite(M1,HIGH);//M1
    digitalWrite(M0,HIGH);//M0
    vTaskDelay(pdMS_TO_TICKS(500));
  }

}
