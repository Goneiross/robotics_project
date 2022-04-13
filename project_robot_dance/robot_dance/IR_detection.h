#ifndef IR_DETECTION_H
#define IR_DETECTION_H

int detection_init(void);
void update_obstacle_array(bool *obstacle);
bool is_obstacle();

#endif