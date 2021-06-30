# 2D-Location-Detection

This project has 2 different C Programs. 

The Transmitter Device is an STM32F3 with an eRIC Transciever, an LCD Display, and a Circuit designed to transmit an Ultrasonic Sound Signal.
The Transmitter Program will send out 2 enables one for the RF Signal and one for the Ultrasonic Sound Signal.

The Reciever Device is an STM32F3 with an eRIC Transciever, and LCD Display, and a Circuit designed to recieve an Ultrasonic Sound Signal.
The Reciever Program will start a clock when the RF Signal is recieved and will stop the clock when an Ultrasonic Sound Signal is recieved.
Based on the time difference the Reciever program will calculate a distance from the Transmitter device.
The Ultrasonic Sound signal trip is emulated using a buttonpress on the STM Board, but can easily be modified to check any GPIO enable.
