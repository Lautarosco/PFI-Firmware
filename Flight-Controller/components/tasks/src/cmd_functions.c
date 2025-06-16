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
        {.name = "ema_roll",  .addr = &(drone->attributes.global_variables.ema_filter_roll)},
        {.name = "ema_pitch", .addr = &(drone->attributes.global_variables.ema_filter_pitch)},
        {.name = "ema_yaw",   .addr = &(drone->attributes.global_variables.ema_filter_yaw)},
        {.name = NULL,        .addr = NULL}
    };

    bool found = false;
    for(int i = 0; general_vars[i].name != NULL; i++) {
        if(!strcmp(arr[VAR_INDEX], general_vars[i].name)) {
            *( float * ) general_vars[i].addr = (float) atof(arr[VALUE_INDEX]);
            found = true;
        }
    }

    if(!found) {

        ESP_LOGW("TASK3", "%s in line %d --> ValueError: '%s' not found", __func__, __LINE__, arr[VAR_INDEX]);
    }
}


void SpUpdateCmdFunc( drone_t * obj, char * arr[ 4 ] ) {

    /* Get index ( states enum ) of received state */
    int index = GetStateIndex( arr[ STATE_INDEX ] );

    /* Check if received state is valid */
    if( PID_INDEX_CHECK( index, sizeof( obj->attributes.components.controllers ) / ( sizeof( obj->attributes.components.controllers[ 0 ] ) ), __func__, __LINE__ ) ) {

        typedef struct sp_index {
            int state_index;
            float *ptr;
        } sp_index_t;

        sp_index_t sp_arr[] = {
            {.state_index = Z,       .ptr = &(obj->attributes.sp.z)},
            {.state_index = ROLL,    .ptr = &(obj->attributes.sp.roll)},
            {.state_index = ROLL_D,  .ptr = &(obj->attributes.sp.roll_dot)},
            {.state_index = PITCH,   .ptr = &(obj->attributes.sp.pitch)},
            {.state_index = PITCH_D, .ptr = &(obj->attributes.sp.pitch_dot)},
            {.state_index = YAW,     .ptr = &(obj->attributes.sp.yaw)},
            {.state_index = YAW_D,   .ptr = &(obj->attributes.sp.yaw_dot)},
            {.state_index = 0,       .ptr = NULL},
        };

        bool found = false;

        /* Get state index */
        int ret = GetStateIndex(arr[STATE_INDEX]);

        /* If valid state */
        if (ret != -1) {
            for (int i = 0; sp_arr[i].ptr != NULL; i++)
            {
                if (sp_arr[i].state_index == ret) {
                    *(sp_arr[i].ptr) = atof(arr[VALUE_INDEX]);
                    found = true;
                }
                break;
            }
                
        }

        if( !found ) {

            ESP_LOGE( "TASK3", "Drone's <%s> state not found. See func %s, in line %d", arr[STATE_INDEX], __func__, __LINE__ );
        }
    }
}


void PidGainsCmdFunc( drone_t * obj, char * arr[ 4 ] ) {

    /* Get index ( states enum ) of received state */
    int index = GetStateIndex( arr[ STATE_INDEX ] );

    /* Check if received state is valid */
    if( PID_INDEX_CHECK( index, sizeof( obj->attributes.components.controllers ) / ( sizeof( obj->attributes.components.controllers[ 0 ] ) ), __func__, __LINE__ ) ) {

        vars_update_t vars_arr[] = {

            { .name = "p",   .addr = &( obj->attributes.components.controllers[ index ]->gain.kp ) },
            { .name = "i",   .addr = &( obj->attributes.components.controllers[ index ]->gain.ki ) },
            { .name = "d",   .addr = &( obj->attributes.components.controllers[ index ]->gain.kd ) },
            { .name = "b",   .addr = &( obj->attributes.components.controllers[ index ]->gain.kb ) },
            { .name = "tau", .addr = &( obj->attributes.components.controllers[ index ]->derivative_lpf.tau_s ) },
            { .name = NULL,  .addr = NULL },
        
        };

        bool found = false;

        for( int i = 0; vars_arr[ i ].name != NULL; i++ ) {

            if( !strcmp( arr[ VAR_INDEX ], vars_arr[ i ].name ) ) {

                *( float * ) vars_arr[ i ].addr = ( float ) atof( arr[ VALUE_INDEX ] );
                found = true;
            }
        }

        if( !found ) {

            ESP_LOGW( "TASK3", "Drone's variable name was not found.\n[ Details ] See func %s, in line %d", __func__, __LINE__ );
        }
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Update Pid actions with received command
 */
typedef struct pid_action_function {

    /* Action name */
    const char * action_name;

    /* Pointer to Pid Class set action method */
    void ( * pid_setterFunc )( pid_controller_t * obj, ControllerFunction * actionFunc );

    /* Pointer to controller action function */
    float ( * actionFunc )( pid_controller_t * obj, float error );

} pid_action_function_t;

static pid_action_function_t pid_actions_array[] = {

    { .action_name = "P_Basic",    .pid_setterFunc = &PidSetActionP, .actionFunc = P_Basic },
    { .action_name = "I_Basic",    .pid_setterFunc = &PidSetActionI, .actionFunc = I_Basic },
    { .action_name = "I_Clamping", .pid_setterFunc = &PidSetActionI, .actionFunc = I_Clamping },
    { .action_name = "I_BackCalc", .pid_setterFunc = &PidSetActionI, .actionFunc = I_BackCalc },
    { .action_name = "D_Basic",    .pid_setterFunc = &PidSetActionD, .actionFunc = D_Basic },
    { .action_name = "D_LPF",      .pid_setterFunc = &PidSetActionD, .actionFunc = D_LPF },
};

void PidActionsCmdFunc( drone_t * obj, char * arr[ 4 ] ) {

    /* Get index ( states enum ) of received state */
    int index = GetStateIndex( arr[ STATE_INDEX ] );

    bool found = false;

    /* Check if received state is valid */
    if( PID_INDEX_CHECK( index, sizeof( obj->attributes.components.controllers ) / ( sizeof( obj->attributes.components.controllers[ 0 ] ) ), __func__, __LINE__ ) ) {

        for( int i = 0; i < ( ( sizeof( pid_actions_array ) ) / ( sizeof( pid_actions_array[ 0 ] ) ) ); i++ ) {

            if( !strcmp( pid_actions_array[ i ].action_name, arr[ VALUE_INDEX ] ) ){

                pid_actions_array[ i ].pid_setterFunc( obj->attributes.components.controllers[ index ], pid_actions_array[ i ].actionFunc );
                found = true;
            }
        }

        /* Check if action was found */
        if( !found ) {

            ESP_LOGW( "TASK3", "PID action name was not found.\n[ Details ] See func %s, in line %d", __func__, __LINE__ );
        }
    }
}
