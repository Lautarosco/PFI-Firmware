#include "button_helper.h"

bool pressed(drone_t* drone, sm_event_t event) {
    switch(event) {
        case EV_UP:
            return drone->attributes.buttons.current.up && !drone->attributes.buttons.previous.up;
        case EV_DOWN:
            return drone->attributes.buttons.current.down && !drone->attributes.buttons.previous.down;
        case EV_LEFT:
            return drone->attributes.buttons.current.left && !drone->attributes.buttons.previous.left;
        case EV_RIGHT:
            return drone->attributes.buttons.current.right && !drone->attributes.buttons.previous.right;
        case EV_SQUARE:
            return drone->attributes.buttons.current.square && !drone->attributes.buttons.previous.square;
        case EV_CROSS:
            return drone->attributes.buttons.current.cross && !drone->attributes.buttons.previous.cross;
        case EV_TRIANGLE:
            return drone->attributes.buttons.current.triangle && !drone->attributes.buttons.previous.triangle;
        case EV_CIRCLE:
            return drone->attributes.buttons.current.circle && !drone->attributes.buttons.previous.circle;
        case EV_PS:
            return drone->attributes.buttons.current.ps && !drone->attributes.buttons.previous.ps;
        case EV_START:
            return drone->attributes.buttons.current.start && !drone->attributes.buttons.previous.start;
        case EV_R1:
            return drone->attributes.buttons.current.r1 && !drone->attributes.buttons.previous.r1;
        case EV_R2:
            return drone->attributes.buttons.current.r2 && !drone->attributes.buttons.previous.r2;
        case EV_L1:
            return drone->attributes.buttons.current.l1 && !drone->attributes.buttons.previous.l1;
        case EV_L2:
            return drone->attributes.buttons.current.l2 && !drone->attributes.buttons.previous.l2;
        default:
            return false;
    }
}
bool released(drone_t* drone, sm_event_t event) {
    switch(event) {
        case EV_UP:
            return !drone->attributes.buttons.current.up && drone->attributes.buttons.previous.up;
        case EV_DOWN:
            return !drone->attributes.buttons.current.down && drone->attributes.buttons.previous.down;
        case EV_LEFT:
            return !drone->attributes.buttons.current.left && drone->attributes.buttons.previous.left;
        case EV_RIGHT:
            return !drone->attributes.buttons.current.right && drone->attributes.buttons.previous.right;
        case EV_SQUARE:
            return !drone->attributes.buttons.current.square && drone->attributes.buttons.previous.square;
        case EV_CROSS:
            return !drone->attributes.buttons.current.cross && drone->attributes.buttons.previous.cross;
        case EV_TRIANGLE:
            return !drone->attributes.buttons.current.triangle && drone->attributes.buttons.previous.triangle;
        case EV_CIRCLE:
            return !drone->attributes.buttons.current.circle && drone->attributes.buttons.previous.circle;
        case EV_PS:
            return !drone->attributes.buttons.current.ps && drone->attributes.buttons.previous.ps;
        case EV_START:
            return !drone->attributes.buttons.current.start && drone->attributes.buttons.previous.start;
        case EV_R1:
            return !drone->attributes.buttons.current.r1 && drone->attributes.buttons.previous.r1;
        case EV_R2:
            return !drone->attributes.buttons.current.r2 && drone->attributes.buttons.previous.r2;
        case EV_L1:
            return !drone->attributes.buttons.current.l1 && drone->attributes.buttons.previous.l1;
        case EV_L2:
            return !drone->attributes.buttons.current.l2 && drone->attributes.buttons.previous.l2;
        default:
            return false;
    }
}
