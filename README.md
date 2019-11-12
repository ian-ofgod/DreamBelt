# DreamBelt
Arduino MKR1010 implementation of an ActiGraph

Note:
Fell free to contribute!
The whole project was born during a 24h hacklaton so the code may not be as good as possile
This code should be an example of a generic use of MKR1010 to send data coming
from various sensor to a server via GET request. 

The system is composed by: 
- Arduino MKR1010
- Grove 9DoF sensor
- Grove DHT21 sensor
- Generic piezoelectric pulse sensor

The arduino takes all the information from the sensors and send it out to a mysql server via a GET http request
The signals can be filtered to obtain a more accurate chart, via the Filter.h library

Imported libraries:
- Grove sensors
- WiFiNINA
- MLP (for Filter.h)
- Adafruit pulsensor libraries playground



