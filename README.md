# STM32F103C8T6USBOscilloscope
STM32F103C8T6 USB dual channel oscilloscope with Pulse Generator, DDS Function Generator

<img src="STM32F103USBscope.png">

This displays an oscilloscope screen on the WEB page of the PC which is conected to the STM32F103C8T6 board with a USB cable.
The settings are controled on the WEB page.

Specifications:
<li>Dual input channel</li>
<li>Input voltage range 0 to 3.3V</li>
<li>12 bit ADC 5.14 Msps single channel, 2.57 Msps dual channel</li>
<li>Measures minimum and maximum values</li>
<li>Measures frequency</li>
<li>Sampling rate selection</li>
<li>Built in Pulse Generator</li>
<li>Built in DDS Function Generator</li>
<br>
<p>
Develop environment is:<br>
Arduino IDE 1.8.19<br>
STM32F1xx/GD32F1xx boards by stm32duino version 2022.9.26<br>
(additional URL: http://dan.drown.org/stm32duino/package_STM32duino_index.json )<br>
CPU speed 72MHz<br>
</p>

Libraries:<br>
arduinoFFT by Enrique Condes 2.0.0<br>

Schematics:<br>
<img src="STM32USBOscillo.png">

Usage:<br>
Upload the scketch from Arduino IDE with Upload Method: "STM32duino bootloader".<br>
Open the HTML file "STM32F103scope.html" on the Windows PC and select the communication port at upper left area of the web page.

Description is here, although it is written in Japanese language:<br>
http://harahore.g2.xrea.com/STM32/STM32USBOscillo.html
