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

    bool found = false;
    for(int i = 0; drone->attributes.flash_params_arr[i].name != NULL; i++) {
        if(!strcmp(arr[ARG2], drone->attributes.flash_params_arr[i].name)) {
            switch (drone->attributes.flash_params_arr[i].type) {
                case PARAM_TYPE_FLOAT:
                    *(float *)drone->attributes.flash_params_arr[i].ptr = (float)atof(arr[ARG3]);
                    break;
                case PARAM_TYPE_INT:
                    *(int *)drone->attributes.flash_params_arr[i].ptr = atoi(arr[ARG3]);
                    break;
                case PARAM_TYPE_STRING:
                    strncpy((char *)drone->attributes.flash_params_arr[i].ptr, arr[ARG3], drone->attributes.flash_params_arr[i].size - 1);
                    ((char *)drone->attributes.flash_params_arr[i].ptr)[drone->attributes.flash_params_arr[i].size - 1] = '\0';
                    break;
                default:
                    // Handle unknown type
                    break;
            }
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