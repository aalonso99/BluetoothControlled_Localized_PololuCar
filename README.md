# BluetoothControlled_Localized_PololuCar

Code to remotely control a Pololu 32U4 car. A LiDAR and a particle filter based algorithm are used to localize the car inside a predefined map. The user commands sent via Bluetooth are read by the Pololu through an ESP32 microcontroller. The LiDAR reads are sent to the main server to run the localization algorithm. "Memoria.pdf" contains a summary (in Spanish) of the whole project and the role of each file.
