#ifndef MACROS_H
#define MACROS_H

/* Macro that is used to get rid of 'unused variable' compile messages */
#define NOT_USED(_x_) (void)(_x_)

#define MIN(a, b)                       ((a) < (b) ? (a) : (b))
#define MAX(a, b)                       ((a) > (b) ? (a) : (b))
#define CLAMP(val, min_val, max_val)    MAX(MIN(val, max_val), min_val)
#define SQUARE_OF(a)  (a * a)

#endif