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
    serialPort->println("======================================");
    serialPort->println("  LINIENFOLGER - Bluetooth verbunden!");
    serialPort->println("======================================");
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

void BluetoothManager::sendMenu() {
    serialPort->println("\n=== LINIENFOLGER STEUERUNG ===");
    serialPort->println("HAUPTBEFEHLE:");
    serialPort->println("  c - Kalibrierung starten");
    serialPort->println("  s - Start (Linienfolger)");
    serialPort->println("  x - Stopp");
    serialPort->println("  h - Hilfe anzeigen");
    serialPort->println("\nDEBUG & INFO:");
    serialPort->println("  d - Debug-Modus (Sensorwerte)");
    serialPort->println("  k - Kreuzungs-Test");
    serialPort->println("  t - Analog-Pin Test");
    serialPort->println("  p - PID Live-Test");
    serialPort->println("  i - System-Status");
    serialPort->println("  m - Motor-Status");
    serialPort->println("\nMOTOR-STEUERUNG:");
    serialPort->println("  e - Motoren aktivieren");
    serialPort->println("  r - Motoren deaktivieren");
    serialPort->println("  l - Links abbiegen (Test)");
    serialPort->println("  g - Rechts abbiegen (Test)");
    serialPort->println("  f - Vorwaerts fahren (Test)");
    serialPort->println("\nMICROSTEPPING:");
    serialPort->println("  1 - Full Step (1/1)");
    serialPort->println("  2 - Half Step (1/2)");
    serialPort->println("  4 - Quarter Step (1/4)");
    serialPort->println("  8 - Eighth Step (1/8) [Standard]");
    serialPort->println("  6 - Sixteenth Step (1/16)");
    serialPort->println("===============================\n");
}
