#include <application_layer/compensation/bmp390_compensation.h>

#include <hardware_layer/bmp390_hw_layer.h>

double bmp390_compensate_temp_celsius(uint32_t adc_temp, bmp390_calib_data_t calib_data) {
    /* Temporary variables used for compensation */
    double partial_data1 = 0.0;
    double partial_data2 = 0.0;

    /* BOSCH algorithm */
    partial_data1 = ((double) adc_temp) - calib_data.par_t1;
    partial_data2 = partial_data1 * calib_data.par_t2;

    return partial_data2 + (partial_data1 * partial_data1) * calib_data.par_t3;
}

double bmp390_compensate_press_pascal(uint32_t adc_press, double comp_temp, bmp390_calib_data_t calib_data) {
    /* Temporary variables used for compensation */
    double partial_data1 = 0.0;
    double partial_data2 = 0.0;
    double partial_data3 = 0.0;
    double partial_data4 = 0.0;

    double partial_out1 = 0.0;
    double partial_out2 = 0.0;

    /* BOSCH algorithm */
    partial_data1 = calib_data.par_p6 * comp_temp;
    partial_data2 = calib_data.par_p7 * (comp_temp * comp_temp);
    partial_data3 = calib_data.par_p8 * (comp_temp * comp_temp * comp_temp);
    partial_out1 = calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib_data.par_p2 * comp_temp;
    partial_data2 = calib_data.par_p3 * (comp_temp * comp_temp);
    partial_data3 = calib_data.par_p4 * (comp_temp * comp_temp * comp_temp);
    partial_out2 = ((double) adc_press) * (calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = ((double) adc_press) * ((double) adc_press);
    partial_data2 = calib_data.par_p9 + calib_data.par_p10 * comp_temp;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + (((double) adc_press) * ((double) adc_press) * ((double) adc_press)) * calib_data.par_p11;
    
    return partial_out1 + partial_out2 + partial_data4;
}

double bmp390_compensate_press_hectopascal(uint32_t adc_press, double comp_temp, bmp390_calib_data_t calib_data) {
    /* Temporary variables used for compensation */
    double partial_data1 = 0.0;
    double partial_data2 = 0.0;
    double partial_data3 = 0.0;
    double partial_data4 = 0.0;

    double partial_out1 = 0.0;
    double partial_out2 = 0.0;

    /* BOSCH algorithm */
    partial_data1 = calib_data.par_p6 * comp_temp;
    partial_data2 = calib_data.par_p7 * (comp_temp * comp_temp);
    partial_data3 = calib_data.par_p8 * (comp_temp * comp_temp * comp_temp);
    partial_out1 = calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib_data.par_p2 * comp_temp;
    partial_data2 = calib_data.par_p3 * (comp_temp * comp_temp);
    partial_data3 = calib_data.par_p4 * (comp_temp * comp_temp * comp_temp);
    partial_out2 = ((double) adc_press) * (calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = ((double) adc_press) * ((double) adc_press);
    partial_data2 = calib_data.par_p9 + calib_data.par_p10 * comp_temp;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + (((double) adc_press) * ((double) adc_press) * ((double) adc_press)) * calib_data.par_p11;
    
    return (partial_out1 + partial_out2 + partial_data4) / 100.0;
}
