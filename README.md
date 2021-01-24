#QUIPMENT USED IN THE PROJECT:

TM4C1294XL Evaluation Kit - Sensor Hub Booster Pack(for BMP180 Sensor and TMP006 Sensor) - Sim800l and LM2596 Voltage Regulator

THE BUILDING PHASE OF THE PROJECT

First, we place our sensor hub in the required place on our board. Then we open an empty ccs project in the code composer studio. You can examine the codes I will explain from the httpget.c file I added. I created the taskTemperature task to get data from the TMP006 sensor, while doing this, I opened the httpget.cfg file with XCGONF and created a task in the interface there. You must do it this. I wrote the necessary initialize commands and made my temperature data available. Here I changed my TMP006 ID from  0x040 to 0x41. I did this in the board.h file. Then I added and adjusted the necessary calibration, acquisition and taskFxn for my BMP180 sensor. What you need to do here is to define your BMP180 ID (Board_BMP180_ADDR) as 0x77 in the board.h file.Then I set up a socket and send it to send my data to the tcp server. You can observe this server on the appropriate port with a program like hercules. I use 5011 port in my code.

SYNCHRONIZATION

Of course, while these tasks are running, there must be aorder. Otherwise, I would either get an error or it would produce unwanted results. I made this layout with semaphore and mailbox structures. First, I sent a semaphore post every 5 seconds to the temperature task using hwi. So my system will run every 5 seconds. Then I sent the temperature data I received from here to the pressure task via mailbox, I made it work and also sent the temperature data too. !!!! The important thing here is to cut the i2c connection before sending the data. If we do not disconnect the pressure sensor cannot be connected to the i2c and an error will occur. I combine the data I receive in the pressure task with temperature efficiency and send it to the client task via mailbox. I am both running the client task and receiving the data that I will send to the server. The system continues in this order. If my Ethernet cable is not connected, I cannot send data to the server. Actually, in this case, I need to connect with gprs and send the data, but I could not do this.

Note: To open the TI-RTOS interface: While in the program, you need to right click on your cfg file and say open with XGCONF. You will click on system overview on the next screen. Click the TI-RTOS Kernel and you can easily create the structures you want here.

My problem with sim800l module: The reason I couldn't progress in the SIM800l module was that I couldn't get OK feedback when I sende "AT". SIM800l module works between 3.7 and 4.2 volts I provide this voltage with use voltage regulator. I inserted the TX pin to my board's pa0 pin and the RX pin to my board's pa1 pin. I gathered the whole system on the common ground. Still the problem was not resolved.
