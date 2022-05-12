#ifndef IR_DETECTION_H
#define IR_DETECTION_H

#include <hal.h>

void detection_init(void);
void update_obstacle_array(bool *obstacle);
bool is_obstacle(void);

#endif
