#ifndef MACROS_H
#define MACROS_H

/* Macro that is used to get rid of 'unused variable' messages */
#define NOT_USED(_x_) (void)(_x_)

#define MIN(a, b)     ((a) < (b) ? (a) : (b))
#define MAX(a, b)     ((a) > (b) ? (a) : (b))

/* x to the power of x*/
#define SQUARE_OF(a)  (a * a)

#endif