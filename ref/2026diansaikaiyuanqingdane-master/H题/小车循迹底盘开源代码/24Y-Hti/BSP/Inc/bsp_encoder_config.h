#ifndef BSP_ENCODER_CONFIG_H
#define BSP_ENCODER_CONFIG_H

/* Measured complete wheel revolution, after quadrature decoding. */
#define BSP_ENCODER_COUNTS_PER_WHEEL_REV 1320L

/* Normalize both counters so vehicle-forward rotation is positive. */
#define BSP_ENCODER_LEFT_DIRECTION_SIGN  (-1L)
#define BSP_ENCODER_RIGHT_DIRECTION_SIGN (1L)

#endif /* BSP_ENCODER_CONFIG_H */
