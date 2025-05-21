


# Controlling the device
It uses Modbus RTU over USB CDC (serial port emulation).  You can use whatever baud rate you wish, as CDC doesn't really care.

* Discrete inputs:
    * 0..256 are the buttons.  Each 7 bits represents one fixture. There are 24 fixtures, meaning the maximum address you can refer to is 167.
* Coils:
    * 0..255 are relays, which will be reflected to downstream modbus.  Each 32 addresses represent 1 device, so address 33 is address 1 on device 2
    * 256..319 are DALI on/off - On will recall last active level, 0 will 
* Handling Registers:
    * 0..255 are the bindings for the switches.  The top two bits indicate type (0 = Relay, 1 = DALI, 3 = NONE).  The remaining 14 indicate address
    * The remainder are banks of 64 for each of the DALI settings.  The register number within the bank indicates the DALI address.
        * BANK 0 (Address 256..319) - MSB = Status, LSB = level.  Note that status is ignored upon write, as it is read-only. 
        * BANK 1 (320..383) - Max Level, Min Level
        * BANK 2 (384..448) - Extended Fade Level, Fade Level and Rate
        * BANK 3 (448..511) - Power Failure Level, Power on Level
        * BANK 4 (512..575) - Group Membership.
* Input Registers are unused.

Attempts to read values outside of this range will return a modbus illegal address error. 