# modulo_io_expander
AtTiny817 based IO expander with I2C to WS2812 bridge for modulo keyboards

1. open ???.cproj file in Microchip Studio 7.0
2. build: Build >> Build ???
3. firmware: Debug/???.hex
4. flash by pyupdi: pyupdi -d tiny817 -c COM4 -f ???.hex
