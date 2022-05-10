#ifndef IR_DETECTION_H
#define IR_DETECTION_H

#include <hal.h>

int detection_init(void);
bool is_obstacle(void);
void update_obstacle_array(bool *obstacle);

#endif
