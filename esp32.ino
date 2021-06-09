byte temperature;
byte humidity;
float humidityH;
float vva;
float temperatureC;
const char* ssid     = "Bang";   //Wi-Fi帳號
const char* password = "yike1234";   //Wi-Fi密碼
String myToken = "ldpIzR1aT4NxrblD3sqQhUqS2kQRsFpzbMnrh09z9uU";
uint64_t reg_b;
int pinDHT11 = 13;
#include <MQUnifiedsensor.h>
#define Board                   ("ESP-32")
#define Pin                     (2)
#define Type                    ("MQ-7")
#define Voltage_Resolution      (3.3)
#define ADC_Bit_Resolution      (12)
#define RatioMQ3CleanAir        (60)
MQUnifiedsensor MQ7(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <SimpleDHT.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "soc/sens_reg.h"

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
SimpleDHT11 dht11(pinDHT11);

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //關閉電壓不穩時重啟電源設定
  reg_b = READ_PERI_REG(SENS_SAR_READ_CTRL2_REG);
  Serial.begin(115200);
  pinMode(15,OUTPUT);
  pinMode(14,OUTPUT);
  MQ7.setRegressionMethod(1);
  MQ7.setA(521853); MQ7.setB(-3.821);
  /*
    Exponential regression:
  Gas    | a      | b
  LPG    | 44771  | -3.245
  CH4    | 2*10^31| 19.01
  CO     | 521853 | -3.821
  Alcohol| 0.3934 | -1.504
  Benzene| 4.8387 | -2.68
  Hexane | 7585.3 | -2.849
  */
  MQ7.init(); 
  Serial.print("MQ7校正中");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ7.update();
    calcR0 += MQ7.calibrate(RatioMQ3CleanAir);
    Serial.print(".");
  }
  MQ7.setR0(calcR0/10);
  Serial.println("MQ7校正完成");
  
  if(isinf(calcR0)) {Serial.println("MQ7短路,請檢查線路是否接好"); while(1);}
  if(calcR0 == 0){Serial.println("MQ7短路,請檢查線路是否接好"); while(1);}
  MQ7.serialDebug(true);
  //wifi-------------------------------------------------------------------------------------------------
  WiFi.mode(WIFI_STA);
  Serial.println("");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);  
  long int StartTime=millis();
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    if ((StartTime+10000) < millis()) break;
  } 

  Serial.println("");
  Serial.println("STAIP address: ");
  Serial.println(WiFi.localIP());
    
  Serial.println("");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reset");
    delay(1000);
    ESP.restart();  //若未連上Wi-Fi則重啟
  }
camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);  // XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
//測試傳送影像至Line Notify，一小時最多上傳50張照片------------------------------------------------------------------------
  sendCapturedImage2LineNotify(myToken);
  Serial.println();
}


void loop() {
  WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);
  SET_PERI_REG_MASK(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DATA_INV);
  temperature = 0;
  humidity = 0;
  MQ7.update();
  MQ7.readSensor();
  MQ7.serialDebug();
  delay(500);
  vva = float(MQ7.readSensor());
  int err = SimpleDHTErrSuccess;
  if ((err = dht11.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("DHT-11錯誤:"); Serial.println(err);delay(1000);
    return;
  }
  Serial.print("Sample OK: ");
  temperatureC=temperature;
  humidityH=humidity;
  Serial.print(temperatureC); Serial.print(" *C, "); 
  Serial.print(humidityH); Serial.println(" H");  
  delay(1000);
  if (vva>=10 or temperatureC>=50) {
    digitalWrite(15, HIGH);
    sendCapturedImage2LineNotify(myToken);  //不顯示傳送結果
    //Serial.println(sendCapturedImage2LineNotify(myToken));  //顯示傳送結果
    delay(30000);
  }
  delay(1000); 
}
String sendCapturedImage2LineNotify(String Token)
{
  String getAll="", getBody = "";
  
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("相機擷取失敗");
    delay(1000);
    ESP.restart();
    return "相機擷取失敗";
  }
   
  WiFiClientSecure client_tcp;

  Serial.println("正在連接:notify-api.line.me");
  
  if (client_tcp.connect("notify-api.line.me", 443)) 
  {
    digitalWrite(14, HIGH);
    Serial.println("連接成功");
    String stringvva = String(vva);
    String temc= String(temperatureC);
    String humh= String(humidityH);
    String message = "目前一氧化碳:" + stringvva + "ppm，溫度:" + temc + "*C，濕度:"+ humh + "%" + "已超過危險值!!";
    String head = "--Taiwan\r\nContent-Disposition: form-data; name=\"message\"; \r\n\r\n" + message + "\r\n--Taiwan\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--Taiwan--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;
  
    client_tcp.println("POST /api/notify HTTP/1.1");
    client_tcp.println("Connection: close"); 
    client_tcp.println("Host: notify-api.line.me");
    client_tcp.println("Authorization: Bearer " + Token);
    client_tcp.println("Content-Length: " + String(totalLen));
    client_tcp.println("Content-Type: multipart/form-data; boundary=Taiwan");
    client_tcp.println();
    client_tcp.print(head);
    
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0;n<fbLen;n=n+1024) {
      if (n+1024<fbLen) {
        client_tcp.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client_tcp.write(fbBuf, remainder);
      }
    }  
    
    client_tcp.print(tail);
    esp_camera_fb_return(fb);
    
    int waitTime = 10000;
    long startTime = millis();
    boolean state = false;
    
    while ((startTime + waitTime) > millis())
    {
      Serial.print(".");
      delay(100);      
      while (client_tcp.available()) 
      {
          char c = client_tcp.read();
          if (state==true) getBody += String(c);        
          if (c == '\n') 
          {
            if (getAll.length()==0) state=true; 
            getAll = "";
          } 
          else if (c != '\r')
            getAll += String(c);
          startTime = millis();
       }
       if (getBody.length()>0) break;
    }
    client_tcp.stop();
    Serial.println(getBody);
    digitalWrite(14,LOW);
  }
  else {
    getBody="連接失敗:notify-api.line.me";
    Serial.println("連接失敗:notify-api.line.me");
  }
  return getBody;
}
