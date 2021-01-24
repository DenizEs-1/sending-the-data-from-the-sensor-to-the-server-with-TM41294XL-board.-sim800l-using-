#QUIPMENT USED IN THE PROJECT:

TM4C1294XL Evaluation Kit - Sensor Hub Booster Pack(for BMP180 Sensor and TMP006 Sensor) - Sim800l and LM2596 Voltage Regulator

THE BUILDING PHASE OF THE PROJECT

First, we place our sensor hub in the required place on our board. Then we open an empty ccs project in the code composer studio. You can examine the codes I will explain from the httpget.c file I added. I created the taskTemperature task to get data from the TMP006 sensor, while doing this, I opened the httpget.cfg file with XCGONF and created a task in the interface there. You must do it this. I wrote the necessary initialize commands and made my temperature data available. Here I changed my TMP006 ID from  0x040 to 0x41. I did this in the board.h file. Then I added and adjusted the necessary calibration, acquisition and taskFxn for my BMP180 sensor. What you need to do here is to define your BMP180 ID (Board_BMP180_ADDR) as 0x77 in the board.h file.Then I set up a socket and send it to send my data to the tcp server. You can observe this server on the appropriate port with a program like hercules. I use 5011 port in my code.

SYNCHRONIZATION
