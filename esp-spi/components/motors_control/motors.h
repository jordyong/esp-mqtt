#ifndef MOTORS_H
#define MOTORS_H

#define MOVESTOP 0
#define MOVEFRONT 1
#define MOVEBACK 2
#define MOVELEFT 3
#define MOVERIGHT 4

#ifdef __cplusplus
extern "C" {
#endif

// from 0 to 1024 ||25% duty cycle (256/1024) || 50%
// duty cycle (512/1024) 75% duty cycle (768/1024) ||
// 100% duty cycle (1024/1024)
void setDutyCycle(int M1, int M2);
void setDirection(int direction);

void motorsInit();

#ifdef __cplusplus
}
#endif
#endif
