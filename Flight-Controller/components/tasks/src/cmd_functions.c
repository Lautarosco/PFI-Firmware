#include <cmd_functions.h>
#include <esp_log.h>
#include <ctype.h>
#include <string.h>
#include <drone.h>


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Check if Bluetooth command for PID index is correct
 * @param index: Desired index
 * @param n_obj: Total of Pid objects
 * @param func: Function caller
 * @param line: Line from called
 * @retval true if index is OK - false if it's not
 */
static bool PID_INDEX_CHECK( int index, int n_obj, const char * func, int line ) {

    /* Default returning value */
    bool ret = false;

    /* If index is a string */
    if( isalpha( ( unsigned char ) index ) ) {

        ESP_LOGE( "TASK3", "Type error: index must be int. See function %s in line %d", func, line );
    }

    else if( index == -1 ) {

        ESP_LOGE( "TASK3", "Index error ( function %s, line %d ): STATE NOT FOUND ( check 'state_specs' variable from tasks.c source file ).", func, line );
    }

    /* If index is greater that total of Pid objects or less than 0 */
    else if( ( index >= n_obj ) || ( index < 0 ) ) {

        ESP_LOGE( "TASK3", "Index error: index out of range. See function %s in line %d", func, line );
    }

    /* Index is OK */
    else {

        ret = true;
    }

    /* Return answer */
    return ret;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief State's name and index
 */
typedef struct stateSpecs {

    const char * name;
    int index;

} stateSpecs_t;

/* State's matrix */
static stateSpecs_t state_specs[] = {

    { .name = "z",       .index = Z },
    { .name = "roll",    .index = ROLL },
    { .name = "pitch",   .index = PITCH },
    { .name = "yaw",     .index = YAW },
    { .name = "roll_d",  .index = ROLL_D },
    { .name = "pitch_d", .index = PITCH_D },
    { .name = "yaw_d",   .index = YAW_D },
};

/**
 * @brief Retrieve state's label
 * @param stateIndex: state index ( See state_t enum defined in controllers_structs.h header file )
 * @retval State's label
 */
static const char * GetStateName( int stateIndex ) {

    for( int i = 0; i < ( ( sizeof( state_specs ) ) / ( sizeof( state_specs[ 0 ] ) ) ); i++ ) {

        if( state_specs[ i ].index == stateIndex ) {
            
            return state_specs[ i ].name;
        }
    }

    return "STATE NOT FOUND";
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Get state's index ( states enum ) given it's name
 * @param stateName: State's name
 * @retval State's index
 */
static int GetStateIndex( const char * stateName ) {

    for( int i = 0; i < ( ( sizeof( state_specs ) ) / ( sizeof( state_specs[ 0 ] ) ) ); i++ ) {

        if( !strcmp( state_specs[ i ].name, stateName ) ) {
            
            return state_specs[ i ].index;
        }
    }

    return -1;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Update general variables such as pid gains or filers coefficient.
 */
typedef struct vars_update {

    /* Name of variable => MUST be same as received from cmd */
    const char * name;

    /* Address of variable */
    void * addr;

} vars_update_t;


void VarsUpdateCmdFunc(drone_t * drone, char * arr[4]) {
    vars_update_t general_vars[] = {
        {.name = "ema_roll",  .addr = &(drone->attributes.config.IIR_coeff_roll_dot)},
        {.name = "ema_pitch", .addr = &(drone->attributes.config.IIR_coeff_pitch_dot)},
        {.name = "ema_yaw",   .addr = &(drone->attributes.config.IIR_coeff_yaw_dot)},

        {.name = "roll/P",    .addr = &(drone->attributes.components.controllers[ROLL].gain.kp)},
        {.name = "roll/I",    .addr = &(drone->attributes.components.controllers[ROLL].gain.ki)},
        {.name = "roll/D",    .addr = &(drone->attributes.components.controllers[ROLL].gain.kd)},
        {.name = "roll/D_Alpha",    .addr = &(drone->attributes.components.controllers[ROLL].derivative_lpf.alpha)},
        {.name = "roll/KB",    .addr = &(drone->attributes.components.controllers[ROLL].gain.kb)},


        {.name = "roll_d/P",  .addr = &(drone->attributes.components.controllers[ROLL_D].gain.kp)},
        {.name = "roll_d/I",  .addr = &(drone->attributes.components.controllers[ROLL_D].gain.ki)},
        {.name = "roll_d/D",  .addr = &(drone->attributes.components.controllers[ROLL_D].gain.kd)},
        {.name = "roll_d/D_Alpha",    .addr = &(drone->attributes.components.controllers[ROLL_D].derivative_lpf.alpha)},  // TODO: hacerlo para todos
        {.name = "roll_d/KB", .addr = &(drone->attributes.components.controllers[ROLL_D].gain.kb)},


        {.name = "pitch/P",   .addr = &(drone->attributes.components.controllers[PITCH].gain.kp)},
        {.name = "pitch/I",   .addr = &(drone->attributes.components.controllers[PITCH].gain.ki)},
        {.name = "pitch/D",   .addr = &(drone->attributes.components.controllers[PITCH].gain.kd)},
        {.name = "pitch/KB",  .addr = &(drone->attributes.components.controllers[PITCH].gain.kb)},


        {.name = "pitch_d/P", .addr = &(drone->attributes.components.controllers[PITCH_D].gain.kp)},
        {.name = "pitch_d/I", .addr = &(drone->attributes.components.controllers[PITCH_D].gain.ki)},
        {.name = "pitch_d/D", .addr = &(drone->attributes.components.controllers[PITCH_D].gain.kd)},
        {.name = "pitch_d/KB",.addr = &(drone->attributes.components.controllers[PITCH_D].gain.kb)},


        {.name = "yaw/P",     .addr = &(drone->attributes.components.controllers[YAW].gain.kp)},
        {.name = "yaw/I",     .addr = &(drone->attributes.components.controllers[YAW].gain.ki)},
        {.name = "yaw/D",     .addr = &(drone->attributes.components.controllers[YAW].gain.kd)},
        {.name = "yaw/KB",    .addr = &(drone->attributes.components.controllers[YAW].gain.kb)},


        {.name = "yaw_d/P",   .addr = &(drone->attributes.components.controllers[YAW_D].gain.kp)},
        {.name = "yaw_d/I",   .addr = &(drone->attributes.components.controllers[YAW_D].gain.ki)},
        {.name = "yaw_d/D",   .addr = &(drone->attributes.components.controllers[YAW_D].gain.kd)},
        {.name = "yaw_d/KB",    .addr = &(drone->attributes.components.controllers[YAW_D].gain.kb)},


        {.name = "z/P",       .addr = &(drone->attributes.components.controllers[Z].gain.kp)},
        {.name = "z/I",       .addr = &(drone->attributes.components.controllers[Z].gain.ki)},
        {.name = "z/D",       .addr = &(drone->attributes.components.controllers[Z].gain.kd)},
        {.name = "z/KB",       .addr = &(drone->attributes.components.controllers[Z].gain.kb)},

        {.name = "misc/0",    .addr = &(drone->attributes.global_variables.misc_floats[0])},        // amplitude
        {.name = "misc/1",    .addr = &(drone->attributes.global_variables.misc_floats[1])},        // period
        {.name = "misc/2",    .addr = &(drone->attributes.global_variables.misc_floats[2])},        // period

        {.name = NULL,        .addr = NULL}
    };

    bool found = false;
    for(int i = 0; general_vars[i].name != NULL; i++) {
        
        if(!strcmp(arr[ARG2], general_vars[i].name)) {
            *( float * ) general_vars[i].addr = (float) atof(arr[ARG3]);
            found = true;
            return;
        }
    }

    if(!found) {

        ESP_LOGW("TASK3", "%s in line %d --> ValueError: '%s' not found", __func__, __LINE__, arr[ARG2]);
    }
}


void SpUpdateCmdFunc( drone_t * drone, char * arr[ 4 ] ) {

    /* Get index ( states enum ) of received state */
    int index = GetStateIndex( arr[ ARG1 ] );
 
    /* Check if received state is valid */
    if( PID_INDEX_CHECK( index, sizeof( drone->attributes.components.controllers ) / ( sizeof( drone->attributes.components.controllers[ 0 ] ) ), __func__, __LINE__ ) ) {

        typedef struct sp_index {
            int state_index;
            float *ptr;
        } sp_index_t;

        sp_index_t sp_arr[] = {
            {.state_index = Z,       .ptr = &(drone->attributes.sp.z)},
            {.state_index = ROLL,    .ptr = &(drone->attributes.sp.roll)},
            {.state_index = ROLL_D,  .ptr = &(drone->attributes.sp.roll_dot)},
            {.state_index = PITCH,   .ptr = &(drone->attributes.sp.pitch)},
            {.state_index = PITCH_D, .ptr = &(drone->attributes.sp.pitch_dot)},
            {.state_index = YAW,     .ptr = &(drone->attributes.sp.yaw)},
            {.state_index = YAW_D,   .ptr = &(drone->attributes.sp.yaw_dot)},
            {.state_index = 0,       .ptr = NULL},
        };

        bool found = false;

        for (int i = 0; sp_arr[i].ptr != NULL; i++)
        {
            if (sp_arr[i].state_index == index) {
                *(sp_arr[i].ptr) = atof(arr[ARG2]);
                found = true;
                break;
            }
        }
                

        if( !found ) {

            ESP_LOGE( "TASK3", "Drone's <%s> state not found. See func %s, in line %d", arr[ARG1], __func__, __LINE__ );
        }
    }
}


void PidGainsCmdFunc( drone_t * drone, char * arr[ 4 ] ) {

    /* Get index ( states enum ) of received state */
    int index = GetStateIndex( arr[ ARG1 ] );

    /* Check if received state is valid */
    if( PID_INDEX_CHECK( index, sizeof( drone->attributes.components.controllers ) / ( sizeof( drone->attributes.components.controllers[ 0 ] ) ), __func__, __LINE__ ) ) {

        vars_update_t vars_arr[] = {

            { .name = "p",   .addr = &( drone->attributes.components.controllers[ index ].gain.kp ) },
            { .name = "i",   .addr = &( drone->attributes.components.controllers[ index ].gain.ki ) },
            { .name = "d",   .addr = &( drone->attributes.components.controllers[ index ].gain.kd ) },
            { .name = "b",   .addr = &( drone->attributes.components.controllers[ index ].gain.kb ) },
            { .name = "D_alpha", .addr = &( drone->attributes.components.controllers[ index ].derivative_lpf.alpha ) },
            { .name = NULL,  .addr = NULL },
        
        };

        bool found = false;

        for( int i = 0; vars_arr[ i ].name != NULL; i++ ) {

            if( !strcmp( arr[ ARG2 ], vars_arr[ i ].name ) ) {

                *( float * ) vars_arr[ i ].addr = ( float ) atof( arr[ ARG3 ] );
                found = true;
            }
        }

        if( !found ) {

            ESP_LOGW( "TASK3", "Drone's variable name was not found.\n[ Details ] See func %s, in line %d", __func__, __LINE__ );
        }
    }
}

void NvsStoreCmdFunc( drone_t * drone, char * arr[ 4 ]) {
    drone->methods.save_to_nvs(drone);
}

/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Update Pid actions with received command
 */
typedef struct pid_action_function {

    /* Action name */
    const char * action_name;

    /* Pointer to Pid Class set action method */
    void ( * pid_setterFunc )( pid_controller_t * pid_controller, ControllerFunction * actionFunc );

    /* Pointer to controller action function */
    float ( * actionFunc )( pid_controller_t * pid_controller, float error );

} pid_action_function_t;

static pid_action_function_t pid_actions_array[] = {

    { .action_name = "P_Basic",    .pid_setterFunc = &PidSetActionP, .actionFunc = P_Basic },
    { .action_name = "I_Basic",    .pid_setterFunc = &PidSetActionI, .actionFunc = I_Basic },
    { .action_name = "I_Clamping", .pid_setterFunc = &PidSetActionI, .actionFunc = I_Clamping },
    { .action_name = "I_BackCalc", .pid_setterFunc = &PidSetActionI, .actionFunc = I_BackCalc },
    { .action_name = "D_Basic",    .pid_setterFunc = &PidSetActionD, .actionFunc = D_Basic },
    { .action_name = "D_LPF",      .pid_setterFunc = &PidSetActionD, .actionFunc = D_LPF },
};

void PidActionsCmdFunc( drone_t * drone, char * arr[ 4 ] ) {

    /* Get index ( states enum ) of received state */
    int index = GetStateIndex( arr[ ARG1 ] );

    bool found = false;

    /* Check if received state is valid */
    if( PID_INDEX_CHECK( index, sizeof( drone->attributes.components.controllers ) / ( sizeof( drone->attributes.components.controllers[ 0 ] ) ), __func__, __LINE__ ) ) {

        for( int i = 0; i < ( ( sizeof( pid_actions_array ) ) / ( sizeof( pid_actions_array[ 0 ] ) ) ); i++ ) {

            if( !strcmp( pid_actions_array[ i ].action_name, arr[ ARG2 ] ) ){

                pid_actions_array[ i ].pid_setterFunc( &(drone->attributes.components.controllers[ index ]), pid_actions_array[ i ].actionFunc );
                found = true;
                printf("PID Action %s changed to %s\n", arr[ARG1], arr[ARG2]);
            }
        }

        /* Check if action was found */
        if( !found ) {

            ESP_LOGW( "TASK3", "PID action name was not found.\n[ Details ] See func %s, in line %d", __func__, __LINE__ );
        }
    }
}

const char * TXCMD_TAG= "[TX_CMD_FUNC]";  // TODO: standarize TAGs in this task

void TxCmdFunc(drone_t * drone, char * arr[ 4 ]) {

    // Reset serial_data flag
    drone->attributes.global_variables.serial_data.state = false;

    /* Declared in drone.c source file */
    extern tx_buttons_t * GlobalTxButtons;

    typedef struct tx_btns {
            char * btn_name;
            bool * tx_btn_ptr;
        } tx_btns_t;

        tx_btns_t tx_btns_arr[] = {
            {.btn_name = "cross",    .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.cross)},
            {.btn_name = "triangle", .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.triangle)},
            {.btn_name = "square",   .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.square)},
            {.btn_name = "circle",   .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.circle)},
            {.btn_name = "up",       .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.up)},
            {.btn_name = "down",     .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.down)},
            {.btn_name = "left",     .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.left)},
            {.btn_name = "right",    .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.right)},
            {.btn_name = "l1",       .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.l1)},
            {.btn_name = "l2",       .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.l2)},
            {.btn_name = "r1",       .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.r1)},
            {.btn_name = "r2",       .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.r2)},
            {.btn_name = "start",    .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.start)},
            {.btn_name = "reset",    .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.ps)}
        };

        bool found = false;
        /* If any button was pressed */
        if( !strcmp( arr[ARG2], "press" ) ) {
            for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++)
            {
                if(!strcmp(arr[ARG1], tx_btns_arr[i].btn_name)) {
                    (*tx_btns_arr[i].tx_btn_ptr) = true;
                    // printf("<%s> button was pressed\n", tx_btns_arr[i].btn_name);
                    found = true;
                    break;
                }
            }
        } else if (!strcmp(arr[ARG2], "release")) {
            for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++)
            {
                if(!strcmp(arr[ARG1], tx_btns_arr[i].btn_name)) {
                    (*tx_btns_arr[i].tx_btn_ptr) = false;
                    // printf("<%s> button was released\n", tx_btns_arr[i].btn_name);
                    found = true;
                    break;
                }
            }
        } else {
            ESP_LOGE(TXCMD_TAG, "Action must be press/release. See function %s in line %d.", __func__, __LINE__);
            return;
        }

        if (!found) {
            printf("Buttons must be: <");
            for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++) {
                printf("%s/", tx_btns_arr[i].btn_name);
            }
            printf(">\n");
        }

}