# Study document - issue #1

## AD7606 interface protocol (SPI)
* PAR/SER/BYTE SEL (Pin 6) pulled to high
* RD/SCLK (Pin 12): Serial Clock Input
* CS (Pin 13): SPI Slave Select
* DB7/DOUTA (Pin 24, channel V1-V4) and DB8/DOUTB (Pin 25, channel V6-V8): Serial Data Output
* Pin 22-16, 31-27, 32, 33 tied to ground
## AD7606 Sampling Control Signal
* RANGE (Pin8): Input Range Selection (+-10V or +-5V)
* CONVST A, CONVST B (Pin 9,10): simultaneous sampling, oversampling is off, BUSY goes high on rising edge of CONVSTx Signals

## Interface Signal Diagram