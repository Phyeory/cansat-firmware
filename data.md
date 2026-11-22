Setup

	•	Fixed the HAL config file so the project actually compiles (before, it didn't).
	•	Told the compiler to print decimal numbers, which it was silently dropping.

Wiring sensors to the chip

	•	Connected every pin to its job. Before, no pins were linked to I2C, SPI, UART, the timer or the ADC, so nothing could talk.
	•	Moved the battery and ozone readings to ADC2, because the battery pin only exists there.
	•	Slowed the sensor bus to 100 kHz so the CO₂ sensor can keep up.
	•	Fixed a servo setup loop that froze the chip at startup.
	•	Added a startup scan that lists which sensors answered.

Sensors

	•	Light sensor: settings now match its maths (it was using the wrong gain and timing).
	•	UV sensor: fixed the UV index, which was about 24 times too low.
	•	CO₂ sensor: added the short waits it needs, plus a wait for it to boot up.
	•	IMU: replaced the BMI270 with an LSM6DSV on SPI1 (CS PB0).

Radio

	•	Switched to the E22-900T22D transparent UART link (USART3). The old RFM96W/SPI radio belonged to a different module.
	•	Set power to 14 dBm for 868.125 MHz, the UK legal limit.
	•	Sending no longer freezes the sensors while a packet goes out (the E22 write is bounded by a timeout).
	•	The E22 module and the ground E22-900T22U are configured identically (868.125 MHz, 62.5 kbps).

GPS

	•	It now catches every byte using an interrupt. Before, most of the data was lost.
	•	Fixed the parser, which broke when there was no GPS fix.
	•	Bad or corrupted sentences are now thrown away.
	•	Still UART: the SAM-M8Q has no SPI pins (UART/DDC only), so the requested SPI2 link needs a different GNSS module.

Main program

	•	The CanSat no longer stops if a sensor fails. It keeps sending GPS so you can find it.
	•	Packets now include a packet number (to spot lost packets) and a health code (to show which sensors work).
	•	The SD log now gets a header row.

Ground station

	•	Fixed station.py crashing on the first packet.
	•	It now shows lost packets, signal strength and sensor health.

New files

	•	Removed lora_receiver.ino: the E22-900T22U USB module replaces the Pico/RFM96W bridge.
	•	WIRING.md: a PCB checklist and first power-up guide.