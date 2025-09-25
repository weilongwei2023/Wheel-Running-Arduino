
#include <SPI.h>                                                          // SPI通讯
#include "RTClib.h"                                                       // 包含 RTClib 库
#include <mcp2515.h>                                                      // MCP2515

// 微动开关 --------------------------------------------------------------------
#define debounceDelay                 10                                  // 保持时间毫秒【可调】
#define buttonNumber                  2                                   // 按钮个数

int buttonPin[buttonNumber] = {22, 23};                                   // 设置按键引脚（A-22  B-23）
bool buttonFlag[buttonNumber] = {HIGH, HIGH};                             // 按键状态
bool lastButtonFlag[buttonNumber] = {HIGH, HIGH};                         // 上一次按键
unsigned long lastDebounceTime[buttonNumber] = {0, 0};                    // 按键按下去的初始时间

// 光幕传感器 -------------------------------------------------------------------
#define lightCurtainPin               24                                  // 光电传感器引脚

// DS3231 ---------------------------------------------------------------------
RTC_DS3231 rtc;                                                           // rtc对象

// MCP2515 --------------------------------------------------------------------
struct can_frame canMsg;                                                  // 结构体
#define mcpCsPin                      53                                  // mcp2515 CS引脚
MCP2515 mcp2515(mcpCsPin);                                                // 实例化对象
float currentAngle_T = 0;                                                 // 角度
float angularSpeed_T = 0;                                                 // 角速度 °/s
int revolutions_T = 0;                                                    // 圈数
float temperature_T = 0;                                                  // 温度℃

// MOS管控制 -------------------------------------------------------------------
#define mosCount                      5                                   // mos管数量【可调】
int mosPin[mosCount] = {                                                  // 设置MOS管控制引脚
  2, 3, 4, 5, 6               // A01-2   A02-3   B01-4   B02-5   电磁阀-6
};

#define solenoidValveON               LOW                                 // 打开是锁住
#define solenoidValveOFF              !solenoidValveON                    // 关闭是活动

// 屏幕 ------------------------------------------------------------------------
#define showSerial                    Serial1                             // 屏幕通讯

// 并口通道发送 -----------------------------------------------------------------
int sendHighPin[6] = {                                                    // 发送高电平信号引脚
  26, 27, 28, 29, 30, 31  
};
#define sendHighKeepTime              5                                   // 发送高电平【可调 ms】

// 存储数据 --------------------------------------------------------------------
#define SendBaudRate                  9600                                // 发送波特率
#define sendSerial                    Serial2                             // 发送存储数据串口

// 系统参数 --------------------------------------------------------------------
// 关于开关A部分 ---------------------
#define A_SwitchLight                 (5 * 1000UL)                        // A开关运行灯亮时间【可调 毫秒】
unsigned long A_SwitchRunTime = 0;                                        // A开关运行时间
int ratPressesNumber = 0;                                                 // 老鼠按压次数
bool A_SwitchRunState = false;                                            // A侧运行状态

#define A_judgeCount                  5                                   // 判断运行次数【可调 次数】
int A_TempRunRecordCount = 0;                                             // 记录运行次数

// 关于光电传感器控制 -----------------
#define getLightInterval              20                                  // 获取光电间隔状态【可调 毫秒】
unsigned long getLightTime = 0;                                           // 记录获取光电时间
#define lightCurtainTime              (5 * 1000UL)                        // 光幕运行开灯时间【可调 毫秒】
unsigned long recordLightCurtainTime = 0;                                 // 记录光幕时间
bool oldLightState = false;                                               // 上一次运行状态
int lightCurtainNumber = 0;                                               // 状态变化次数

// 屏幕按压部分 ----------------------
int A_SidePressCount = 0;                                                 // A侧按压次数
int B_SidePressCount = 0;                                                 // B侧按压次数
int modelSetRunData[3] = {                                                // 模式设置运行参数【可调 次数】
  1, 3, 5
};
int setCurrentNumber = modelSetRunData[0];                                // 默认参数（记录判定次数）
int correspondingMode = 0;                                                // 记录对应模式0 1 2 3

// 关于开关B部分 ---------------------
int ratPressesNumber_B = 0;                                               // 老鼠按压次数

uint8_t systemState = 0x00;                                               // 系统运行状态
bool initialRecordStatus = true;                                          // B侧按钮启动初始状态
int B_TempRunRecordCount = 0;                                             // B侧临时运行记录按压次数

#define judgeTestTime                 (30 * 1000UL)                       // 记录规定时间内判断运行时间【可调 毫秒】
#define runOneRoundTime               (20 * 1000UL)                       // 记录运行一轮时间【可调 毫秒 启动后一轮的运行时长】
unsigned long recordRunTime = 0;                                          // 记录运行时间

#define B01RunTime                    (5 * 1000UL)                        // BO1信号灯运行时间【可调 毫秒】
#define B02RunTime                    (8 * 1000UL)                        // BO2信号灯运行时间【可调 毫秒】

const float pi = 3.14159265359;                                           // 计算参数
const float wheelDiameter = 13;                                           // 轮子直径，单位为厘米【可调 轮子的直径】

String currentMouseName = "0001";                                         // 当前小鼠名字
float currentRunDistance = 0;                                             // 当前运行距离

#define dataUpdateInterval            100                                 // 数据更新间隔时间【可调 毫秒】
unsigned long dataUpdateTime = 0;                                         // 数据更新时间
float originalDistanceTravelled = 0;                                      // 已运行距离
bool recordStorageStatus = false;                                         // 记录存储状态

#define judgeMode4Time                (30 * 1000UL)                       // 判断模式4下回合判断间隔时间【可调 毫秒】
int currentModeRunIndex = 0;                                              // 当前模式运行下标 也是对应回合
#define mode4Quantity                 26                                  // 模式4下指令数量
int modeRunRound[mode4Quantity] = {                                       // 不同回合下不同按压条件
  1, 2, 4, 6, 9,
  12, 15, 20, 25, 32,
  40, 50, 62, 77, 95,
  118, 145, 178, 219, 268,
  328, 402, 492, 603, 737, 901
};

#define canGetMaxCount                5
int recordCanCount = 0;

// 增加判断次数运行设置 -------------------
int speedCount[2] = {0,0};
int setSpeedRunCount[2] = {5, 5};
bool parallelStatus = true;

void setup() {
  Serial.begin(9600);                                                     // 串口波特率9600
  // 初始化发送 -------------------------
  sendSerial.begin(SendBaudRate);                                         // 发送波特率
  // 发送高电平 -------------------------
  for(int i = 0; i < 6; i++){                                             // for循环设置
    pinMode(sendHighPin[i], OUTPUT);                                      // 针脚输出
    digitalWrite(sendHighPin[i], LOW);                                    // 初始化关闭
  }
  // MOS管控制 -------------------------
  for (int i = 0; i < mosCount; i++) {                                    // for循环设置
    pinMode(mosPin[i], OUTPUT);                                           // 针脚输出
    digitalWrite(mosPin[i], LOW);                                         // 初始化关闭
  }
  // 电磁阀 ----------------------------
  digitalWrite(mosPin[4], solenoidValveON);                               // 初始化锁住
  // 运行按钮 --------------------------
  for (int i = 0; i < buttonNumber; i++) {                                // for循环按钮
    pinMode(buttonPin[i], INPUT_PULLUP);                                  // 设置按键模式为上拉输入
  }
  // 光幕光电 --------------------------
  pinMode(lightCurtainPin, INPUT);                                        // 设置为输入
  // DS3231 ---------------------------
  if (!rtc.begin()) {                                                     // 初始化
    Serial.println("Couldn't find RTC");                                  // 串口信息
  }
  // 屏幕初始化 -------------------------
  showSerial.begin(9600);                                                 // 串口通讯
  // MCP2515 --------------------------
  mcp2515.reset();                                                        // mcp2515初始化
  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);                              // 设置通讯参数
  mcp2515.setNormalMode();                                                // 设置模式
  // 初始化屏幕设置 ---------------------
  sendHMIData("page 0");                                                  // 初始化页面
  sendHMIData("page 0");                                                  // 初始化页面
  // A侧按压
  sendHMIData("v0.txt=\"" + String(ratPressesNumber) + "\"");
  sendHMIData("v1.txt=\"" + String(A_SidePressCount) + "\"");
  sendHMIData("v2.txt=\"" + String(ratPressesNumber + A_SidePressCount) + "\"");
  // 光电
  sendHMIData("v3.txt=\"" + String(lightCurtainNumber) + "\"");
  // 运行参数
  sendHMIData("v4.txt=\"" + String(revolutions_T, 2) + "\"");
  sendHMIData("v5.txt=\"" + String(angularSpeed_T, 2) + "\"");
  // 名称
  sendHMIData("v7.txt=\"" + currentMouseName + "\"");
  // 时间
  sendHMIData("v8.txt=\"" + dateTime() + "\"");
  // B侧按压
  sendHMIData("v10.txt=\"" + String(ratPressesNumber_B) + "\"");
  sendHMIData("v11.txt=\"" + String(B_SidePressCount) + "\"");
  sendHMIData("v12.txt=\"" + String(ratPressesNumber_B + B_SidePressCount) + "\"");
  
  sendHMIData("v9.bco=63488");
  // 打印提示 --------------------------
  Serial.println("Started successfully!");                                // 启动提示
}

void loop() {
  // 获取串口数据 --------------------------------------
  comdataGet();                                                           // 串口更新DS3231时间

  // 获取微动开关状态 -----------------------------------
  getbutton();                                                            // 获取微动开关状态

  // 获取光电传感器状态 ---------------------------------
  if (millis() - getLightTime >= getLightInterval) {                      // 判断间隔时间是否满足
    getLightTime = millis();                                              // 记录阶段时间
    int reading = digitalRead(lightCurtainPin);                           // 获取光幕状态
    if (reading == HIGH) {                                                // 判断获取光幕状态是否符合
      if (++A_TempRunRecordCount >= A_judgeCount) {                       // 判断运行次数是否达到
        A_TempRunRecordCount = A_judgeCount;                              // 重置状态
        recordLightCurtainTime = millis();                                // 记录当前阶段时间
        // 对应操作 --------------------------
        if (oldLightState == false) {                                     // 判断上一次状态
          oldLightState = true;                                           // 状态变化
          digitalWrite(mosPin[1], HIGH);                                  // 打开A02灯
          lightCurtainNumber = lightCurtainNumber + 1;                    // 次数累加
          // 屏幕更新 ------------------------
          sendHMIData("v3.txt=\"" + String(lightCurtainNumber) + "\"");

          // 发送并口状态 ------------
          sendHighState(1);
        }
        // Serial.println(1);
      }
    } else {
      oldLightState = false;                                              // 状态更改
      A_TempRunRecordCount = 0;
      // Serial.println(0);
    }
  }

  // 获取屏幕指令数据 -----------------------------------
  if (showSerial.available() > 0) {                                       // 检测串口是否有数据
    String receiveData = "";                                              // 创建临时变量
    while (showSerial.available() > 0) {                                  // 如果串口有数据 循环读取
      receiveData += char(showSerial.read());                             // 读出串数据 累加存储
      delay(2);                                                           // 延迟对应时间
    }
    Serial.print("Screen Data: ");                                        // 打印提示
    Serial.println(receiveData);                                          // 打印提示
    // 判断指令状态 ------------------
    if (receiveData[0] == 'A') {                  // 判断A侧是否触发
      A_SidePressCount = A_SidePressCount + 1;                            // 累加A侧按压次数
      // 对应操作 --------------------
      digitalWrite(mosPin[0], HIGH);                                      // 打开A01灯
      A_SwitchRunState = true;                                            // A侧运行状态
      A_SwitchRunTime = millis();                                         // 记录阶段时间
      // 屏幕更新 --------------------
      sendHMIData("v1.txt=\"" + String(A_SidePressCount) + "\"");
      sendHMIData("v2.txt=\"" + String(ratPressesNumber + A_SidePressCount) + "\"");

    }
    else if (receiveData[0] == 'B') {             // 判断B侧是否触发
      B_TempRunRecordCount = 9999;
      recordRunTime = millis();
      Serial.println("B侧人为触发运行");

      // 模式累加次数 ------------------------------------------------------
      if(correspondingMode < 3){                                          // 判断模式小于3
        B_SidePressCount = B_SidePressCount + setCurrentNumber;           // 累积次数
      } else {
        B_SidePressCount = B_SidePressCount + modeRunRound[currentModeRunIndex];            // 更新累积次数
      }
      sendHMIData("v11.txt=\"" + String(B_SidePressCount) + "\"");        // 更新数据
      sendHMIData("v12.txt=\"" + String(ratPressesNumber_B + B_SidePressCount) + "\"");     // 更新累积次数
    }
    else if (receiveData[0] == 'M') {             // 判断当前模式
      int tempVal = receiveData.substring(1, 2).toInt() - 1;              // 获取下标值
      if (tempVal < 3) {                                                  // 如果小于
        setCurrentNumber = modelSetRunData[tempVal];                      // 获取设定次数
        Serial.print("Current Set Data: ");                               // 打印提示
        Serial.println(setCurrentNumber);                                 // 打印提示
      } else {                                    // 记录模式4初次
        setCurrentNumber = 3;                                             // 切换记录模式
      }
      correspondingMode = tempVal;                                        // 记录模式

      // 重置不同模式下已按压的次数 -----------------
      B_TempRunRecordCount = 0;                                           // 重置按压次数
      currentModeRunIndex = 0;                                            // 重置按压回合
    }
    else if (receiveData[0] == 'N') {             // 判断修改名字
      if (receiveData[receiveData.length() - 1] == '|') {                 // 判断最后一个字符是否是额外无用
        currentMouseName = receiveData.substring(receiveData.indexOf(",") + 1, receiveData.length() - 1);
      } else {                                                            // 无额外符号
        currentMouseName = receiveData.substring(receiveData.indexOf(",") + 1, receiveData.length());
      }
      Serial.print("Mouse Name: ");                                       // 打印提示
      Serial.println(currentMouseName);                                   // 打印提示
      delay(300);
      
      // 屏幕刷新更新 ----------------
      delay(300);
      sendHMIData("v0.txt=\"" + String(ratPressesNumber) + "\"");
      sendHMIData("v1.txt=\"" + String(A_SidePressCount) + "\"");
      sendHMIData("v2.txt=\"" + String(ratPressesNumber + A_SidePressCount) + "\"");
      sendHMIData("v3.txt=\"" + String(lightCurtainNumber) + "\"");

      sendHMIData("v10.txt=\"" + String(ratPressesNumber_B) + "\"");
      sendHMIData("v11.txt=\"" + String(B_SidePressCount) + "\"");        // 更新数据
      sendHMIData("v12.txt=\"" + String(ratPressesNumber_B + B_SidePressCount) + "\"");     // 更新累积次数

      // 重置不同模式下已按压的次数 -----------------
      setCurrentNumber = modelSetRunData[0];
      B_TempRunRecordCount = 0;                                           // 重置按压次数
      currentModeRunIndex = 0;                                            // 重置按压回合
    }
  }

  // 判断A侧微动开光运行状态 -----------------------------
  if (A_SwitchRunState == true && millis() - A_SwitchRunTime >= A_SwitchLight) {          // 判断间隔时间是否满足
    A_SwitchRunTime = millis();                                           // 记录阶段时间
    digitalWrite(mosPin[0], LOW);                                         // 关闭A01灯
    A_SwitchRunState = false;                                             // 运行状态改变
  }

  // 判断光幕传感器运行状态 ------------------------------
  if (oldLightState == false && millis() - recordLightCurtainTime >= lightCurtainTime) {  // 判断间隔时间是否满足
    recordLightCurtainTime = millis();                                    // 记录阶段时间
    digitalWrite(mosPin[1], LOW);                                         // 关闭A02灯
  }

  // B开关按压控制功能 ----------------------------------
  switch (systemState) {
    case 0x00: {                    // 判断按钮状态
        // 区分模式 ---------------------
        if (correspondingMode < 3) {        // 模式 1 2 3
          if (millis() - recordRunTime <= judgeTestTime) {                // 判断检测时间内是否达到次数开始一轮
            if (B_TempRunRecordCount >= setCurrentNumber) {               // 判断次数是否达到
              systemState = 0x01;                                         // 状态改变
              recordRunTime = millis();                                   // 记录当前时间
              Serial.println("B侧一轮启动 模式 " + String(correspondingMode + 1));   // 打印提示
              // 对应操作 ------------------
              digitalWrite(mosPin[4], solenoidValveOFF);                  // 电磁锁关闭
              digitalWrite(mosPin[2], HIGH);                              // 打开B01灯
              digitalWrite(mosPin[3], HIGH);                              // 打开B02灯
              sendHMIData("v9.bco=2032");                                 // 屏幕更新
            }
          } else {
            if (B_TempRunRecordCount != 0) {                              // 如果在规定时间内次数没有达到则重置
              B_TempRunRecordCount = 0;                                   // 次数归0
              Serial.println("B侧规定时间内运行失败");                        // 打印提示
            }
          }
        }
        else {                              // 模式 4
          if(millis() - recordRunTime <= judgeMode4Time){                 // 模式4判断间隔时间
            if(B_TempRunRecordCount >= modeRunRound[currentModeRunIndex]){    // 判断次数是否达到
              systemState = 0x01;                                         // 状态改变
              recordRunTime = millis();                                   // 记录当前时间
              
              if(++currentModeRunIndex >= mode4Quantity){
                currentModeRunIndex = 0;
                Serial.println("模式4运行，回合重置");
              }
              Serial.println("模式4运行开始，回合：" + String(currentModeRunIndex));  // 打印提示

              // 对应操作 ------------------
              digitalWrite(mosPin[4], solenoidValveOFF);                  // 电磁锁关闭
              digitalWrite(mosPin[2], HIGH);                              // 打开B01灯
              digitalWrite(mosPin[3], HIGH);                              // 打开B02灯
              sendHMIData("v9.bco=2032");                                 // 屏幕更新
            }
          } 
          else {
            if(currentModeRunIndex != 0){                                 // 如果是运行状态下
              Serial.println("模式4运行结束，回合次数最高运行达到 " + String(currentModeRunIndex));
              currentModeRunIndex = 0;                                    // 重置当前回合
              B_TempRunRecordCount = 0;                                   // 重置次数
            } else {
              recordRunTime = millis();                                   // 更新获取当前时间
            }
          }
        }
      } break;
    case 0x01: {                    // 运行检测存储
        // 获取编码器数据 -------------------------------
        if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {          // 判断状态是否有发送
          if (canMsg.data[1] == 0x55) {                                                           // 角度 角速度 转数
            currentAngle_T = ((canMsg.data[3] << 8) | canMsg.data[2]) * 360.0 / 32768.0;          // 角度
            angularSpeed_T = ((canMsg.data[5] << 8) | canMsg.data[4]) * 360.0 / 32768.0 / 0.1;    // 角速度
            revolutions_T = (canMsg.data[7] << 8) | canMsg.data[6];                               // 转数

            // 打印提示 -------------------
            if (recordStorageStatus == false) {                           // 判断记录状态是否关闭
              if(++recordCanCount >= canGetMaxCount){
                recordStorageStatus = true;                                 // 状态反转
                Serial.println();                                           // 打印提示
                Serial.println("开始运行 " + dateTime());                    // 打印提示
                
                //originalDistanceTravelled = pi * wheelDiameter * abs(revolutions_T) + (currentAngle_T / 360.0 * wheelDiameter);  // 获取当前已记录的参数距离
                originalDistanceTravelled = currentAngle_T;
                Serial.println("初始化已运行距离: " + String(fabs(originalDistanceTravelled)));
                
                // 发送存储数据 ----------------
                if(correspondingMode < 3){
                  sendSerial.println("开始记录模式" + String(correspondingMode + 1) + "," + dateTime());
                } else {
                  sendSerial.println("开始记录模式" + String(correspondingMode + 1) + "," + " 回合" + String(currentModeRunIndex) + "," + dateTime());
                }
              }
            }
          } 
          else if (canMsg.data[1] == 0x56) {                                                    // 特殊数据指令会有温度
            temperature_T = ((canMsg.data[3] << 8) | canMsg.data[2]) / 100.0;                     // 温度
          }
        }

        // 判断 B01信号 点亮时间是否达到 -----------------
        if (millis() - recordRunTime >= B01RunTime) {                     // 判断关灯时间是否达到
          digitalWrite(mosPin[2], LOW);                                   // 关闭B01灯
        }

        // 判断 B02信号 点亮时间是否达到 -----------------
        if (millis() - recordRunTime >= B02RunTime) {                     // 判断关灯时间是否达到
          digitalWrite(mosPin[3], LOW);                                   // 关闭B02灯
        }

        // 判断当前一轮是否运行完成 ----------------------
        if (millis() - recordRunTime >= runOneRoundTime) {                // 判断当前一轮运行时间是否达到
          systemState = 0x00;                                             // 回到检测状态
          digitalWrite(mosPin[4], solenoidValveON);                       // 电磁锁打开
          // 参数重置 -----------------------
          B_TempRunRecordCount = 0;                                       // B侧按压次数归0
          currentRunDistance = 0;                                         // 重置记录距离
          recordStorageStatus = false;                                    // 重置

          recordCanCount = 0;
          
          Serial.println("一轮检测运行完成");                               // 打印提示
          sendHMIData("v9.bco=63488");                                    // 屏幕更新

          // 判断模式4 ---------------
          if(correspondingMode < 3){
            sendSerial.println("模式记录运行结束");
          } 
          else {
            Serial.println("模式4运行结束，回合：" + String(currentModeRunIndex));      // 打印提示
            sendSerial.println("模式4运行结束,回合" + String(currentModeRunIndex));
            recordRunTime = millis();
          }
          sendSerial.println();

          // 发送并口状态 ------------
          parallelStatus = true;
          sendHighState(5);
        }

        // 数据记录 屏幕数据更新 -------------------------
        if (recordStorageStatus == true && millis() - dataUpdateTime >= dataUpdateInterval) {            // 判断间隔时间是否满足
          dataUpdateTime = millis();                                      // 记录阶段时间
          // float temporaryDistance = pi * wheelDiameter * abs(revolutions_T) + (currentAngle_T / 360.0 * wheelDiameter);   // 获取当前距离cm
          float temporaryDistance = currentAngle_T;
          float temporaryAngleSpeed = fabs((angularSpeed_T / 360.0) * pi * wheelDiameter);                                // 换算速度 cm/秒
          
          //currentRunDistance = currentRunDistance + (fabs(temporaryDistance) - fabs(originalDistanceTravelled));          // 求距离差 累加距离差和
          //originalDistanceTravelled = fabs(temporaryDistance);                                                            // 更新数据
          if(temporaryDistance - originalDistanceTravelled > 300){
            currentRunDistance += fabs(360 - temporaryDistance);
            currentRunDistance += fabs(0 - originalDistanceTravelled);
          } else if(temporaryDistance - originalDistanceTravelled < -300){
            currentRunDistance += fabs(temporaryDistance);
            currentRunDistance += fabs(360 - originalDistanceTravelled);
          } else if(fabs(temporaryDistance - originalDistanceTravelled) > 0.1){
            currentRunDistance += fabs(temporaryDistance - originalDistanceTravelled);
          }
          originalDistanceTravelled = temporaryDistance;

          // 打印提示
          // Serial.println(String(revolutions_T) + "    " + String(temporaryDistance) + "    " + String(currentRunDistance));             // 打印提示
          // 计算圆的总周长
          float circumference = pi * wheelDiameter;
       
          // 计算滚动的总距离
          float totalDistance = (currentRunDistance / 360.0) * circumference;
          
          Serial.println(dateTime() + "," + currentMouseName + "," + temporaryAngleSpeed + "," + String(totalDistance, 2));        // 数据打印存储
          sendSerial.println(dateTime() + "," + currentMouseName + "," + temporaryAngleSpeed + "," + String(totalDistance, 2));    // 数据打印存储
          
          // 屏幕更新 -------------
          sendHMIData("v4.txt=\"" + String(totalDistance, 2) + "\"");
          
          sendHMIData("v5.txt=\"" + String(temporaryAngleSpeed, 2) + "\"");
          sendHMIData("v8.txt=\"" + dateTime() + "\"");

          if(temporaryAngleSpeed > 0.2){
            if(++speedCount[0] >= setSpeedRunCount[0]){
              speedCount[0] = setSpeedRunCount[0];
              speedCount[1] = 0;

              // 发送并口状态 ------------
              if(parallelStatus == true){
                parallelStatus = false;
                sendHighState(3);
              }
            }

          } else {
            if(++speedCount[1] >= setSpeedRunCount[1]){
              speedCount[0] = 0;
              speedCount[1] = setSpeedRunCount[1];

              // 发送并口状态 ------------
              if(parallelStatus == false){
                parallelStatus = true;
                sendHighState(4);
              }
            }
          }
        }
      
        
      
      } break;
    default: break;
  }

  // loop回括号
}

// 获取微动开关运行状态 -------------------------------------------------------------
void getbutton() {
  for (int i = 0; i < buttonNumber; i++) {                                // for循环设置
    int reading = digitalRead(buttonPin[i]);                              // 起始状态为1（高电平）
    if (reading != lastButtonFlag[i]) {                                   // 当状态发生改变，给时间赋值
      lastDebounceTime[i] = millis();                                     // 并记录时间
    }
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {               // 判断时间是否满足
      if (reading != buttonFlag[i]) {                                     // 当状态发生改变
        buttonFlag[i] = reading;                                          // 赋值给buttonFlag
        if (buttonFlag[i] == LOW) {                                       // 判断按钮是否按下
          if (i == 0) {
            ratPressesNumber = ratPressesNumber + 1;                      // 按压次数自增
            // 对应操作 --------------------
            digitalWrite(mosPin[0], HIGH);                                // 打开A01灯
            Serial.println("A侧微动开关按下  " + String(ratPressesNumber)); // 打印提示
            // 屏幕更新 --------------------
            sendHMIData("v0.txt=\"" + String(ratPressesNumber) + "\"");
            sendHMIData("v2.txt=\"" + String(A_SidePressCount + ratPressesNumber) + "\"");

            // 发送并口状态 ------------
            sendHighState(0);
          } 
          else if (i == 1) {
            // 发送并口状态 ------------
            sendHighState(2);

            
            if (B_TempRunRecordCount <= 0) {                              // 判断如果为小于等于0
              recordRunTime = millis();                                   // 记录阶段时间
              if(correspondingMode < 3){
                Serial.println("B侧首次触发运行");
              }
            }
            B_TempRunRecordCount = B_TempRunRecordCount + 1;              // 累加记录运行次数
            Serial.println("B侧微动开关按下  " + String(B_TempRunRecordCount));

            // B侧开关按压变化 ---------------
            ratPressesNumber_B++;
            sendHMIData("v10.txt=\"" + String(ratPressesNumber_B) + "\"");
            sendHMIData("v12.txt=\"" + String(ratPressesNumber_B + B_SidePressCount) + "\"");
          }
        } 
        else {
          if (i == 0) {
            A_SwitchRunState = true;
            A_SwitchRunTime = millis();                                   // 记录阶段时间
            // Serial.println("A侧微动开关松开  " + String(ratPressesNumber)); // 打印提示
          } 
          else if (i == 1) {

          }
        }
      }
    }
    lastButtonFlag[i] = reading;                                          // 状态赋值
  }
}

// 串口屏发送函数 ------------------------------------------------------------------
void sendHMIData(String _data) {                                          // 发送数据到屏幕端
  unsigned char hexEND[3] = {0XFF, 0XFF, 0XFF};                           // 串口屏发送结束符
  showSerial.print(_data);                                                // 发送数据
  showSerial.write(hexEND, 3);                                            // 发送结尾数据
}

// DS3231 ------------------------------------------------------------------------
void comdataGet() {                                                       // 更新 按照如下格式
  while (Serial.available() > 0) {                                        // 2024,10,25,11,30,0,
    uint16_t year = Serial.parseInt();
    uint8_t month = Serial.parseInt();
    uint8_t day   = Serial.parseInt();
    uint8_t hour  = Serial.parseInt();
    uint8_t min   = Serial.parseInt();
    uint8_t sec   = Serial.parseInt();
    Serial.println(String(F("设置的RTC时间为：")) + year + "/" + month + "/" + day + " " + hour + ":" + min + ":" + sec);
    rtc.adjust(DateTime(year, month, day, hour, min, sec));
    while (Serial.available() > 0) {
      Serial.read();                                                      // 清空串口
      delay(2);
    }
    sendHMIData("v8.txt=\"" + dateTime() + "\"");                         // 更新时间
  }
}
String dateTime() {                                                       // 字符串 可串口打印 年月日时分秒
  char tim[22] = "0";                                                     // 日期储存字符串
  DateTime now = rtc.now();                                               // 建立时间对象
  // 格式化日期为指定的字符串格式
  snprintf(tim, sizeof(tim), "%02d-%02d-%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  //  Serial.println(tim);
  return tim;
}
String getStorageTime() {
  char tim[22] = "0";                                                     // 日期储存字符串
  DateTime now = rtc.now();                                               // 建立时间对象
  // 格式化日期为指定的字符串格式
  snprintf(tim, sizeof(tim), "%02d-%02d-%02d %02d-%02d-%02d",
           now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  //  Serial.println(tim);
  return tim;
}

// 发送高电平引脚 -------------------------------------------------------------------
void sendHighState(int index){                                            // 传入引脚下标设置运行
  digitalWrite(sendHighPin[index], HIGH);                                 // 高电平
  delay(sendHighKeepTime);                                                // 保持时间
  digitalWrite(sendHighPin[index], LOW);                                  // 低电平
}






void test() {}
