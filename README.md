# ModbusRtuSlave

Modbus RTU slave device.
Released for MDR1986BE4 microcontroller (ARM Cortex M0).

## Functions
The following functions have been implemented:
Read Coil Status (0x01),
Read Discrete Inputs (0x02),
Read Holding Registers (0x03),
Read Input Registers (0x04),
Force Single Coil (0x05),
Preset Single Register (0x06),
Force Multiple Coils (0x0F),
Preset Multiple Registers (0x0A);

## Reg Map
16 discrete outputs are routed to the PORTA, pins 0..15;
8 discrete inputs are connected to the PORTB, pins 2..9;
32 Holding Registers in RAM;
32 Input Registers in RAM;

The register map and protocol settings are described in the regmap.pdf document.
