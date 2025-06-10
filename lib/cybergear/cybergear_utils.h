#ifndef CYBERGEAR_UTILS_h
#define CYBERGEAR_UTILS_H

#include "cybergear.h"

void cybergear_print_faults(cybergear_fault_t *faults);
void cybergear_print_status(cybergear_status_t *status);
char state_as_char(cybergear_state_e state);

#endif