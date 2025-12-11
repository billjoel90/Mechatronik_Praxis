#include "bluetooth.h"

// Wir nutzen Serial1 (Pins 19 RX, 18 TX) mit 9600 Baud (Standard für HC-05/06)
BluetoothManager bt(&Serial1, 9600);

BluetoothManager::BluetoothManager(HardwareSerial* port, long baud) {
    this->serialPort = port;
    this->baudRate = baud;
}

void BluetoothManager::init() {
    serialPort->begin(baudRate);
    // Kurze Wartezeit
    delay(100);
    serialPort->println("BT READY - Linienfolger v1.0");
}

bool BluetoothManager::isAvailable() {
    return serialPort->available() > 0;
}

char BluetoothManager::readCommand() {
    if (serialPort->available() > 0) {
        char c = serialPort->read();
        
        // Optional: Puffer leeren wie in deiner main.cpp, 
        // falls du nur einzelne Zeichen als Befehle willst
        while(serialPort->available() > 0) {
            serialPort->read();
        }
        return c;
    }
    return 0; // Kein Befehl
}

void BluetoothManager::sendMessage(String msg) {
    serialPort->println(msg);
}