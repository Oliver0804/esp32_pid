This project is a temperature control system that uses an Arduino board to monitor the temperature using the ClosedCube SHT31D digital humidity and temperature sensor. The system includes a PID controller to regulate the temperature, with the setpoint temperature defined by the user.

The program uses the PID_v1_bc.h library to implement the PID controller, and the SevSeg library to display the temperature on a seven-segment display. The temperature readings are also outputted to the serial monitor for graphing.

The system controls a relay connected to GPIO 19 on the Arduino board to turn on and off a heating or cooling element, depending on the output of the PID controller. The program also includes a safety feature that turns off the heating or cooling element if the temperature exceeds a defined threshold.

Overall, this temperature control system is a useful tool for a wide range of applications, such as homebrewing, sous-vide cooking, and other temperature-sensitive tasks.