#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>

class BluetoothManager {
private:
    HardwareSerial* serialPort; // Zeiger auf die Serial-Schnittstelle (z.B. &Serial1)
    long baudRate;

public:
    // Konstruktor: Erwartet die Serial-Schnittstelle (z.B. Serial1)
    BluetoothManager(HardwareSerial* port, long baud);

    void init();
    char readCommand();         // Liest ein Zeichen, wenn verfügbar
    void sendMessage(String msg); // Sendet Text an Handy zurück
    bool isAvailable();         // Prüft, ob Daten da sind
    void sendMenu();            // Sendet Menü über Bluetooth
};

// Globales Objekt verfügbar machen
extern BluetoothManager bt;

#endif
