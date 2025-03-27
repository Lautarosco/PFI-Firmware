#include <stdio.h>
#include <com.h>
#include <bmp280.h>
#include <registers.h>
#include <string.h>
#include <esp_log.h>

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
 *      |No measurement          |       Skipped       |          -         |            As needed           |
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
 *      |      000       |       Skipped       |          -         |
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
 *      to the MCU. In ultra high resolution, the maximum measurement time is 43.2 ms
 *      so i believe one could run the sensor at a lower sampling time without any problem
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
 *  <Compensation formula>
 *  ...
 *   ...
 *   ...
 *   
 *  <Memory map>
 *       Registers width: 8 bits
 *         
 *         ------------------------------------------------
 *        |   Register name   |   Address   | Reset state  |
 *        |-------------------|-------------|--------------|
 *        |     temp_xlsb     |     0xFC    |    0x00      |
 *        |-------------------|-------------|--------------|
 *        |      temp_lsb     |     0xFB    |    0x00      |
 *        |-------------------|-------------|--------------|
 *        |      temp_msb     |     0xFA    |    0x80      |
 *        |-------------------|-------------|--------------|
 *        |     press_xlsb    |     0xF9    |    0x00      |
 *        |-------------------|-------------|--------------|
 *        |     press_lsb     |     0xF8    |    0x00      |
 *        |-------------------|-------------|--------------|
 *        |     press_msb     |     0xF7    |    0x80      |
 *        |-------------------|-------------|--------------|
 *        |      config       |     0xF5    |    0x00      |
 *        |-------------------|-------------|--------------|
 *        |     ctrl_meas     |     0xF4    |    0x00      |
 *        |-------------------|-------------|--------------|
 *        |      status       |     0xF3    |    0x00      |
 *        |-------------------|-------------|--------------|
 *        |       reset       |     0xE0    |    0x00      |
 *        |-------------------|-------------|--------------|
 *        |        id         |     0xD0    |    0x58      |
 *        |-------------------|-------------|--------------|
 *        | calib25...calib00 | 0xA1...0x88 |  individual  |
 *        |-------------------|-------------|--------------|
 * 
 *      @b Registers_description
 *          @c id: (0xD0) : chip identification number chip_id[7:0] = 0x58
 * 
 *          @c reset: (0xE0) : if 0xB6 is written to the register, the device is
 *          autimatically reset. @attention Writing other values has no effect
 *          
 *          @c status: (0xF3) : it contains two bits which indicate the status of the device
 * 
 *               -----------------------------------------------------------------
 *              |    Bits    |       Name       |           Description           |
 *              |------------|------------------|---------------------------------|
 *              |     3      |    measuring[0]  |               1*                |
 *              |------------|------------------|---------------------------------|
 *              |  4, 3, 2   |    im_update[0]  |               2*                |
 *              |------------|------------------|---------------------------------|
 * 
 *              1* : Automatically set to 1 whenever a conversion is running and
 *              back to 0 when the results have been transferred to the data registers
 * 
 *              2* : Automatically set to 1 when the NVM data are being copied
 *              to image registers and back to 0 when the coppying is done
 * 
 *          @c ctrl_meas (0xF4)
 *              
 *               -----------------------------------------------------------------
 *              |    Bits    |       Name       |           Description           |
 *              |------------|------------------|---------------------------------|
 *              |  7, 6, 5   |    osrs_t[2:0]   |Controls oversampling of temp.   |
 *              |------------|------------------|---------------------------------|
 *              |  4, 3, 2   |    osrs_p[2:0]   |Controls oversampling of press.  |
 *              |------------|------------------|---------------------------------|
 *              |    1, 0    |     mode[1:0]    |Controls the power mode of device|
 *              |------------|------------------|---------------------------------|
 * 
 * 
 *          @c config (0xF5)
 * 
 *               -----------------------------------------------------------------
 *              |    Bits    |       Name       |           Description           |
 *              |------------|------------------|---------------------------------|
 *              |  7, 6, 5   |     t_sb[2:0]    |Controls inactive duration       |
 *              |------------|------------------|---------------------------------|
 *              |  4, 3, 2   |    filter[2:0]   |Controls time const. of IIR      |
 *              |------------|------------------|---------------------------------|
 *              |      0     |    spi32_en[0]   |Enables 3-wire SPI when set to 1 |
 *              |------------|------------------|---------------------------------|
 * 
 *          @c press_(msb/lsb/xlsb) (0xF7...0xF9)
 * 
 *               -----------------------------------------------------------------
 *              |    Bits    |       Name       |           Description           |
 *              |------------|------------------|---------------------------------|
 *              |    0xF7    |  press_msb[7:0]  |MSB of raw pressure data         |
 *              |------------|------------------|---------------------------------|
 *              |    0xF8    |  press_lsb[7:0]  |LSB of raw pressure data         |
 *              |------------|------------------|---------------------------------|
 *              | 0xF9 bits  |                  |                                 |
 *              | (7,6,5,4)  | press_xlsb[3:0]  |XLSB of raw pressure data        |
 *              |------------|------------------|---------------------------------|
 * 
 *          @c temp_(msb/lsb/xlsb) (0xFA...0xFC)
 * 
 *               -----------------------------------------------------------------
 *              |    Bits    |       Name       |           Description           |
 *              |------------|------------------|---------------------------------|
 *              |    0xFA    |  temp_msb[7:0]   |MSB of raw temperature data      |
 *              |------------|------------------|---------------------------------|
 *              |    0xFB    |  temp_lsb[7:0]   |LSB of raw temperature data      |
 *              |------------|------------------|---------------------------------|
 *              | 0xFC bits  |                  |                                 |
 *              | (7,6,5,4)  | press_xlsb[3:0]  |XLSB of raw temperature data     |
 *              |------------|------------------|---------------------------------|
 * 
 *  <Digital interfaces>
 *      Supports SPI and I2C though we're using the latter. For this one, it supports
 *      the standard, fast and high speeds modes.
 * 
 *      @e Single_byte_write
 *      @e Multiple_byte_write (using paris of register addresses and register data)
 *      @e Single_byte_read
 *      @e Multiple_byte_read (using a single register address which is auto-incremented)
 *
 *      @b Interface_selection is done automatically based on CSB status. @attention If
 *      CSB is connected to VDDIO, then I2C interface is active
 * 
 *      @b Serial_address : 111011x => The 6 MSB bits are fiex while the last one is
 *      changeable by SDO value and can be changed during operation. Connecting SDO
 *      to GND results in slave address 1110110 (0x76), meanwhile if connected to
 *      VDDIO results in slave address 1110111 (0x77). @attention SDO pin cannot be
 *      left floating, if so device address will be undefined
 * 
 *      @c I2C interface uses the following pins
 *          @e SCK : Serial Clock (SCL)
 *          @e SDI : data (SDA)
 *              @attention SDI is bi-directional with open drain to GND, therefore
 *              it must be externally connected to VDDIO via a pull up resistor
 *          @e SDO : Slave address LSB (GND = 0, VDDIO = 1)
 * 
 *          @b I2C_Write : Send the slave address in write mode (RW = 0)
 * 
 *               ---------------------------------
 *              |       Slave adddress      |  RW |
 *              |---|---|---|---|---|---|---|-----|
 *              | 1 | 1 | 1 | 0 | 1 | 1 | x |  0  |
 *              |---|---|---|---|---|---|---|-----|
 * 
 *              @attention x is determined by SDO pin (See @b Serial_address)
 * 
 *              Once thats sent, master sends paris of registers addressses and
 *              register data as follows
 *              
 *              1st pair.
 *                  Register address (0xA0) : 10100000
 *                  Register data - address 0xA0 : 8-bit
 *              
 *              2nd pair.
 *                  Register address (0xA1) : 10100001
 *                  Register data - address 0xA1 : 8-bit
 * 
 *          @b I2C_Read : In order to read registers, first the register address
 *          must be sent in write mode. Then, either a stop or a repeated start
 *          condition must be generated
 *              After this, the slave is addressed in read mode (RW = 1) 1111011x1
 */


static const char * bmp280_tag = "BMP280";


/* #################### PROTOTYPES #################### */
/**
 * @brief Initialize an Bmp280 object
 * 
 * @param bmp: Pointer to Bmp280 object
 * @param addr: Address of bmp280 sensor
 * @param sda: I2C SDA data
 * @param scl: I2C SCL clock
 * 
 * @return none
 */
static esp_err_t bmp280_Init(bmp280_t * bmp, int addr, int sda, int scl);


/* #################### STRUCTS #################### */
/*
struct __i2c_cfg {
    int sda;
    int scl;
};
*/


/* #################### DEFINITIONS #################### */
bmp280_t * Bmp280(void) {
    bmp280_t * bmp = (bmp280_t *) malloc(sizeof(bmp280_t));

    memset(bmp, 0, sizeof(bmp280_t));

    bmp->init = bmp280_Init;

    return bmp;
}

static esp_err_t bmp280_Init(bmp280_t * bmp, int addr, int sda, int scl) {
    ESP_LOGI(bmp280_tag, "Initializing Bmp280 object...");

    bmp->addr = addr;
    bmp->i2c.sda = sda;
    bmp->i2c.scl = scl;

    if(i2c_init(bmp->i2c.sda, bmp->i2c.scl) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "Failed to initialized I2C interface");
        return ESP_FAIL;
    }

    sensors_addr_t sensors[] = {
        {
            .name = "bmp280",
            .addr = bmp->addr,
            .__found = false
        }
    };

    i2c_scan(sensors, sizeof(sensors) / (sizeof(sensors[0])));
    for (int i = 0; i < ((sizeof(sensors)) / (sizeof(sensors[0]))); i++) {
        if(!sensors[i].__found) {
            ESP_LOGE(bmp280_tag, "%s in line %d --> %s not found in I2C bus", __func__, __LINE__, sensors[i].name);
        }
    }
    

    if(i2c_read_byte(bmp->addr, BMP280_ID_REG, &(bmp->id), 1) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "Failed to read bytes from <0x%X>", BMP280_ID_REG);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp280_tag, "Chip id is <0x%X>", bmp->id);

    /**
     * REGISTER: <config> 0xF5
     *  set t_sb bits (7, 6, 5) to 000 (default value), but in forced mode => hence it has no action
     *  set filter bits (4, 3, 2) to 100 ? CHECK:
     *  set spi3w_en to 0 (default value => disabled)
     */

    if(i2c_write_byte(bmp->addr, BMP280_CONFIG_REG, (BMP280_TSB << 5) | (BMP280_FILTER_16 << 2) | BMP280_SPI32_DIS) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "Failed to write bytes to <0x%X>", BMP280_CONFIG_REG);
        return ESP_FAIL;
    }
    

    /**
     * REGISTER: <ctrl_meas> 0xF4
     *  set osrs_t bits (7, 6, 5) to 010 (x2 just to improve a little pressure measurements)
     *  set osrs_p bits (4, 3, 2) to 101 (Ultra high resolution) CHECK:
     *  set mode bits (1, 0) to 01 (Forced mode) => @attention always set force mode again before taking a new measurement (See @b Power_modes) 
     */

    if(i2c_write_byte(bmp->addr, BMP280_CTRL_MEAS_REG, (BMP280_OSRS_T_2 << 5) | (BMP280_OSRS_P_16 << 2) | (BMP280_FORCED_MODE)) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "Failed to write bytes to <0x%X>", BMP280_CTRL_MEAS_REG);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp280_tag, "Bmp280 object successfully initialized.");

    return ESP_OK;
}
