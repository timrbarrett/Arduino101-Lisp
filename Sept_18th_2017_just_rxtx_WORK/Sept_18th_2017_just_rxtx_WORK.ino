/*
    Author: Tim Barrett
    Date: 19th Sept 2017

    Purpose: Arduino code to send and receive 20 characters at a time
      to from a source of bytes. I used nRF UART v2.0 android app

       whenever the received text is exactly 4 characters long
       the free memory of the Arduino is transmitted in response

       this is stage one of getting a minimal lisp interpreter workng over BLE

       note: if an attempt is made to transmit every time loop occurs happens,
       the Arduino hangs after less than a second.

*/
#include <CurieBLE.h>
#include <MemoryFree.h>

const char* localName = "Lisp";
int BLE_MAX_LENGTH = 20;

BLEService uartService = BLEService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLEDescriptor uartNameDescriptor = BLEDescriptor("2901", localName);

BLECharacteristic rxCharacteristic = BLECharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, BLE_MAX_LENGTH);
BLEDescriptor rxNameDescriptor = BLEDescriptor("2901", "RX - (Write)");
BLECharacteristic txCharacteristic = BLECharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLEIndicate, BLE_MAX_LENGTH);
BLEDescriptor txNameDescriptor = BLEDescriptor("2901", "TX - (Indicate)");

void rxCharacteristicWritten(BLECentral & central, BLECharacteristic & characteristic);
boolean outputMemoryOnce = false;

//  BLECentral central = blePeripheral.central();

/* end ble */

void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  BLE.begin();

  /* Set a local name for the BLE device
      This name will appear in advertising packets
      and can be used by remote devices to identify this BLE device
      The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName(localName);

  BLE.setAdvertisedService(uartService);
  //BLE.setAdvertisedServiceUuid(uartService.uuid());      // add the service UUID

  uartService.addCharacteristic(txCharacteristic);           // add the transmit characteristic
  uartService.addCharacteristic(rxCharacteristic);           // the the receive characteristic

  /*could set tx and rx Characteritic value */
  BLE.addService(uartService);              // Add the uart service

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  rxCharacteristic.setEventHandler(BLEWritten, rxCharacteristicWritten);

  BLE.advertise();

}

void loop() {

  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {

    while (central.connected()) {

      BLE.poll();

      if (outputMemoryOnce == true) {
        outputMemoryOnce = false;

        /*
          Returns the number of bytes free in the heap,
          i.e. the number of bytes free to be allocated using malloc().
        */
        transmit("Free memory: " + String(freeMemory()));

        // other types of memory selectable...
        /*
            Returns the number of bytes free in the stack,
            i.e. the space between the current stack frame and the end of the stack area.
            This function will return a negative number in the event of a stack overflow,
            representing the size of the overflow; for example,
            a return value of -20 means that the current stack frame is 20 bytes
            past the end of the stack area.
        */
        transmit("Free Stack: " + String(freeStack()));

        /*
            Returns the number of bytes free in both the stack and the heap.
            If a stack overflow has occurred,
            only the number of bytes free in the heap is returned.
        */
        transmit("Free Heap: " + String(freeHeap()));
      }

    }
  }
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void rxCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {

  // central wrote new value to characteristic
  Serial.println();
  Serial.print("Characteristic event, written: ");

  if (characteristic.value()) {

    int characteristiclength = characteristic.valueLength();
    Serial.println(characteristic.valueLength());

    if (characteristic.valueLength() == 4)
      outputMemoryOnce = true;

    for ( int idx = 0 ; idx < characteristiclength ; ++idx ) {
      Serial.print( (char)characteristic[ idx ] );
    }

    Serial.println();
  }
}

void transmit(String message) {

  unsigned int transmittedUntilByte = 0;

  while ((message.length() - transmittedUntilByte) > BLE_MAX_LENGTH) {
    // message needs sending in smaller chunks

    int str_len = BLE_MAX_LENGTH;
    char char_array[str_len];
    message.substring(transmittedUntilByte, BLE_MAX_LENGTH).toCharArray(char_array, str_len);
    txCharacteristic.setValue((unsigned char*)char_array, str_len);

    Serial.print("compound from:");
    Serial.print(transmittedUntilByte);
    Serial.print(" to:");
    //Serial.print(transmittedUntilByte);
    Serial.print(BLE_MAX_LENGTH);
    Serial.print(" message sent:");
    BLE.poll();
    transmittedUntilByte += BLE_MAX_LENGTH;
  }

  int str_len = message.length() - transmittedUntilByte;
  char char_array[str_len];
  message.substring(transmittedUntilByte, transmittedUntilByte + str_len).toCharArray(char_array, str_len);
  txCharacteristic.setValue((unsigned char*)char_array, str_len);

  Serial.print("from:");
  Serial.print(transmittedUntilByte);
  Serial.print(" to:");
  Serial.print((transmittedUntilByte + str_len));

}

