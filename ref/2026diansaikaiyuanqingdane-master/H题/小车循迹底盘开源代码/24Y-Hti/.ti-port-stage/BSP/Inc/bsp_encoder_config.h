#ifndef BSP_ENCODER_CONFIG_H
#define BSP_ENCODER_CONFIG_H

/* MG310: 13 encoder lines, x4 quadrature, 20.4:1 gearbox. */
#define BSP_ENCODER_COUNTS_PER_WHEEL_REV 1061L

/* Normalize both counters so vehicle-forward rotation is positive. */
#define BSP_ENCODER_LEFT_DIRECTION_SIGN  (1L)
#define BSP_ENCODER_RIGHT_DIRECTION_SIGN (-1L)

#endif /* BSP_ENCODER_CONFIG_H */
