#include <stdio.h>
#include <bmp280.h>

/**
 * NOTES:
 * 
 * <Datasheet> : https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
 * 
 * <Pressure resolution> : .16 Pa (aprox. 0.0133 m)
 * <Temperature resolution> : 0.01 °C
 * <MAX:. sampling freq> : 157 Hz (6.37 ms)
 * <Filter options> : up to 5 bandwidths
 * 
 * @details VDD is one pin and VDDIO is another
 * @warning Max. supply voltage (VDD and VDDIO): 3.6 V
 * 
 * @attention Time to communicate after VDD: 2 ms
 * 
 * @details Builtin power reset: resets logic circuitry and register values
 * @warning Do not hold any interface pin (SDI, SDO, SCK, CSB) at a logical high level
 * when VDDIO is power reset
 * 
 * <Noise>
 * 
 * - No filtering
 *      Pressure: 1.3 Pa
 *      Metters: .11 m (11 cm)
 * 
 * - Filtering
 *      Pressure: .2 Pa
 *      Metters: .017 m (1.7 cm)
 * 
 * <Measurements>
 *  - Pressure
 *      # The resolution bit is stored in the XLSB data register 0xF9
 * 
 *      # Enabling/Disabling its measurements and oversamplings is done through
 *      osrs_p[2:0] bits in control register 0xF4
 * 
 *      # Ultra high resolution (UHR)
 *          Pressure oversampling: x16
 *          Pressure resolution: 20 bit / .16 Pa
 *          Temperature oversampling: x2
 * 
 *       ----------------------------------------------------------------------------------------------------
 *      |        Setting         |     Oversampling    |     Resolution     | Recommended temp. oversampling |
 *      |------------------------|---------------------|--------------------|--------------------------------|
 *      |No measurement          |(auto) set to 0x80000|          -         |            As needed           |
 *      |------------------------|---------------------|--------------------|--------------------------------|
 *      |Ultra low power         |         x1          |  16 bit / 2.62 Pa  |                x1              |
 *      |------------------------|---------------------|--------------------|--------------------------------|
 *      |Low power               |         x2          |  17 bit / 1.31 Pa  |                x1              |
 *      |------------------------|---------------------|--------------------|--------------------------------|
 *      |Standard resolution     |         x4          |  18 bit / .66 Pa   |                x1              |
 *      |------------------------|---------------------|--------------------|--------------------------------|
 *      |High resolution         |         x8          |  19 bit / .33 Pa   |                x1              |
 *      |------------------------|---------------------|--------------------|--------------------------------|
 *      |Ultra high resolution   |         x16         |  20 bit / .16 Pa   |                x2              |
 *      |------------------------|---------------------|--------------------|--------------------------------|
 * 
 *  - Temperature
 *      # The resolution bit is stored in the XLSB data register 0xFC
 * 
 *      # Enabling/Disabling its measurements and oversamplings is done through
 *      osrs_t[2:0] bits in control register 0xF4
 * 
 *       -----------------------------------------------------------
 *      |      Mode      |     Oversampling    |     Resolution     |
 *      |----------------|---------------------|--------------------|
 *      |      000       |(auto) set to 0x80000|          -         |
 *      |----------------|---------------------|--------------------|
 *      |      001       |         x1          |  16 bit / .005 °C  |
 *      |----------------|---------------------|--------------------|
 *      |      010       |         x2          |  17 bit / .0025 °C |
 *      |----------------|---------------------|--------------------|
 *      |      011       |         x4          |  18 bit / .0012 °C |
 *      |----------------|---------------------|--------------------|
 *      |      100       |         x8          |  19 bit / .0006 °C |
 *      |----------------|---------------------|--------------------|
 *      |  101,110,111   |         x16         |  20 bit / .0003 °C |
 *      |----------------|---------------------|--------------------|
 * 
 * @attention Its recommended to base the values of the temperature register
 * osrs_t on the pressure register osrs_p
 * @attention Temperature oversampling above x2 is possible, but wont significantly
 * improve pressure accuracy any further. Pressure measurements depend more on raw
 * pressure rather than raw temperature
 * 
 * <Filters>
 *  - IIR: 
 *                            data_filtered_prev * ( coeff. - 1 ) + raw_data
 *      data_filtered_curr = ------------------------------------------------
 *                                              coeff.
 * 
 *      It can be configured using the filter[2:0] bits in control register 0xF5
 *      with the following options
 * 
 *       -----------------------------------------
 *      | Filter coeff. | Samples to reach >= 75% |
 *      |               |    of step response     |
 *      |---------------|-------------------------|
 *      |   Filter off  |            1            |
 *      |---------------|-------------------------|
 *      |       2       |            2            |
 *      |---------------|-------------------------|
 *      |       4       |            5            |
 *      |---------------|-------------------------|
 *      |       8       |            11           |
 *      |---------------|-------------------------|
 *      |       16      |            22           |
 *      |---------------|-------------------------|
 * 
 *     @attention When writting to the register filter, the filter is reset
 *     <FILTER: SETTINGS: (Recommended)>
 *          - Oversampling: Ultra high resoution
 *          - osrs_p: x16
 *          - osrs_t: x2
 *          - IIR filter coeff.: 16
 *          - ODR (Hz): 26.3
 *          - RMS Noise (cm): 1.6
 * 
 * <Noise>
 *  - At ultra high resolution and IIR coeff. of 16, the RMS noise in pressure
 *  is about .2 Pa and .002 °C for the temperature RMS noise  
 * 
 * <Power modes>
 *  They can be selected using the mode[1:0] bits in control register 0xF4
 *  
 *   -----------------------
 *  | mode[1:0] |   Mode    |
 *  |-----------|-----------|
 *  |    00     |   Sleep   |
 *  |-----------|-----------|
 *  |  01, 10   |   Forced  |
 *  |-----------|-----------|
 *  |    11     |   Normal  |
 *  |-----------|-----------|
 * 
 *  @attention Normal mode is recommended when using the IIR filter, and useful
 *  for applications in which short-term disturbances should be filtered
 * 
 *  The time between measurements can defined as follows
 * 
 *   -------------------------------
 *  | t_sb[1:0] |   t_standby[ms]   |
 *  |-----------|-------------------|
 *  |    000    |        .5         |
 *  |-----------|-------------------|
 *  |    001    |       62.5        |
 *  |-----------|-------------------|
 *  |    010    |       125         |
 *  |-----------|-------------------|
 *  |    011    |       250         |
 *  |-----------|-------------------|
 *  |    100    |       500         |
 *  |-----------|-------------------|
 *  |    110    |       1000        |
 *  |-----------|-------------------|
 *  |    111    |       4000        |
 *  |-----------|-------------------|
 * 
 *  The MCU runs at 10 ms so given the fact that .5 ms is far less from that and
 *  62.5 is too slow, therefore we have two options
 * 
 *  Option 1.
 *      Perform measurements in Normal mode with a sampling time of .5 ms and
 *      take unnecesarilly more measurements than nedded
 * 
 *  Option 2.
 *      Perform measurements in Forced mode with the same sampling time compared
 *      to the MCU. In ultra high resolution, the maximum measurement time in ms
 *      is 43.2 so i believe one could run the sensor at a lower sampling time with
 *      no problem
 * 
 *  <Data readout>
 *      @attention In order to prevent a possible mix-up of bytes belonging to different
 *      measurements and reduce interface traffic, its strongly recommended to
 *      use a burst read instead of addressing each register individually
 *      
 *      @attention The data readout burst must read from 0xF7 to 0xFC. The result should
 *      be read in an unsigned 20-bit format both for pressure and temperature
 * 
 *      @attention After reading the unsigned temperature and pressure values, the actual
 *      ones need to be calculated using the compensation parameters stored in the device
 *      (See COMPENSATION: procedure)
 * 
 *      @b Data_register_shadowing The end of the burst read is marked by the recognition
 *      of a stop condition in I2C. After the end of a burst read, all usre data registers
 *      are updated at once
 *       
 * 
 *      @b Fixed_compensation_parameters They are stored into the non volatile memory
 *      (NVM) of the device. Each compensation word is a 16-bit signed or unsigned
 *      integer value stored in 2 complement. Therefore due to the fact that the
 *      memory is organized into 8-bit registers, two words must always be combined
 *      in order to represent the compensation word
 *      
 *       ----------------------------------------------
 *      | Register Address |  Register  |     Data     |
 *      |     LSB/MSB      |  content   |     type     |
 *      |------------------|------------|--------------|
 *      |    0x88/0x89     |   dig_T1   |unsigned short|
 *      |------------------|------------|--------------|
 *      |    0x8A/0x8B     |   dig_T2   | signed short |
 *      |------------------|------------|--------------|
 *      |    0x8C/0x8D     |   dig_T3   | signed short |
 *      |------------------|------------|--------------|
 *      |    0x8E/0x8F     |   dig_P1   |unsigned short|
 *      |------------------|------------|--------------|
 *      |    0x90/0x91     |   dig_P2   | signed short |
 *      |------------------|------------|--------------|
 *      |    0x92/0x93     |   dig_P3   | signed short |
 *      |------------------|------------|--------------|
 *      |    0x94/0x85     |   dig_P4   | signed short |
 *      |------------------|------------|--------------|
 *      |    0x96/0x97     |   dig_P5   | signed short |
 *      |------------------|------------|--------------|
 *      |    0x98/0x99     |   dig_P6   | signed short |
 *      |------------------|------------|--------------|
 *      |    0x9A/0x9B     |   dig_P7   | signed short |
 *      |------------------|------------|--------------|
 *      |    0x9C/0x9D     |   dig_P8   | signed short |
 *      |------------------|------------|--------------|
 *      |    0x9E/0x9F     |   dig_P9   | signed short |
 *      |------------------|------------|--------------|
 *      |    0xA0/0xA1     |  reserved  |   reserved   |
 *      |------------------|------------|--------------|
 *      
 *      
 */     



