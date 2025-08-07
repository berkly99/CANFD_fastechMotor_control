#include <ModbusRTUSlave.h>

#define MODBUS_SERIAL Serial2
#define MODBUS_BAUD 115200
#define MODBUS_CONFIG SERIAL_8N1
#define MODBUS_UNIT_ID 1

const int16_t inputPins[] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37};
const int16_t outputPins[] = {38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51};
const int16_t numInputs = sizeof(inputPins) / sizeof(inputPins[0]);
const int16_t numOutputs = sizeof(outputPins) / sizeof(outputPins[0]);

ModbusRTUSlave modbus(MODBUS_SERIAL);

bool discreteInputs[numInputs];
bool previousInputs[numInputs];
bool coils[numOutputs];

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 2000; // 상태 출력 간격

void setup() {
    // 핀 모드 설정
    for (int i = 0; i < numInputs; i++) {
        pinMode(inputPins[i], INPUT_PULLUP);
        previousInputs[i] = !digitalRead(inputPins[i]); // 초기 상태 저장
    }
    for (int i = 0; i < numOutputs; i++) {
        pinMode(outputPins[i], OUTPUT);
        digitalWrite(outputPins[i], LOW); // 초기 상태를 LOW로 설정0
    }

    // Modbus 설정
    modbus.configureDiscreteInputs(discreteInputs, numInputs);
    modbus.configureCoils(coils, numOutputs);

    // begin 함수에 필요한 모든 인자 제공
    MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG);
    modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);

    Serial1.begin(115200); // Serial1 초기화
    Serial1.println("Modbus RTU Slave started");
}

void loop() {
    // 입력 상태 읽기 및 변화 감지
    for (int i = 0; i < numInputs; i++) {
        bool currentState = !digitalRead(inputPins[i]);
        discreteInputs[i] = currentState;

        if (currentState != previousInputs[i]) {
            previousInputs[i] = currentState;
        }
    }

    // Modbus 요청 처리
    modbus.poll();

    // 출력 상태 업데이트
    for (int i = 0; i < numOutputs; i++) {
        digitalWrite(outputPins[i], coils[i]);
    }

    // 상태 출력
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= printInterval) {
        printStatus();
        lastPrintTime = currentTime;
    }
}

void printStatus() {
/*
    Serial1.println("Input Status:");
    for (int i = 0; i < numInputs; i++) {
        Serial1.print("Input ");
        Serial1.print(inputPins[i]);
        Serial1.print(": ");
        Serial1.println(discreteInputs[i] ? "Activated" : "Deactivated");
    }
    
    Serial1.println("Output Status:");
    for (int i = 0; i < numOutputs; i++) {
        Serial1.print("Output ");
        Serial1.print(outputPins[i]);
        Serial1.print(": ");
        Serial1.println(coils[i] ? "ON" : "OFF");
    }
*/
    
    Serial1.println("--------------------");
}
