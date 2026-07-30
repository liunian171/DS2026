#ifndef ZDT_STEPPER_H
#define ZDT_STEPPER_H

#include <stdbool.h>
#include <stdint.h>

bool ZDT_Stepper_Init(uint8_t address, uint32_t pulses_per_revolution);
void ZDT_Stepper_Process(void);

bool ZDT_Stepper_Enable(bool enable);
bool ZDT_Stepper_Stop(void);
bool ZDT_Stepper_MoveAbsolutePulses(int32_t position_pulses,
                                    uint16_t speed_rpm,
                                    uint8_t acceleration);
bool ZDT_Stepper_MoveRelativePulses(int32_t delta_pulses,
                                    uint16_t speed_rpm,
                                    uint8_t acceleration);
bool ZDT_Stepper_SetAngleMdeg(int32_t angle_mdeg,
                              uint16_t speed_rpm,
                              uint8_t acceleration);
bool ZDT_Stepper_RequestPosition(void);

int32_t ZDT_Stepper_GetPositionPulses(void);
int32_t ZDT_Stepper_GetAngleMdeg(void);

extern volatile uint32_t g_zdt_stepper_initialized;
extern volatile uint32_t g_zdt_stepper_tx_command_count;
extern volatile uint32_t g_zdt_stepper_rx_frame_count;
extern volatile uint32_t g_zdt_stepper_rx_error_count;
extern volatile uint32_t g_zdt_stepper_position_valid;
extern volatile uint32_t g_zdt_stepper_last_function;
extern volatile uint32_t g_zdt_stepper_last_status;
extern volatile int32_t g_zdt_stepper_position_raw;
extern volatile int32_t g_zdt_stepper_position_pulses;
extern volatile int32_t g_zdt_stepper_angle_mdeg;

#endif /* ZDT_STEPPER_H */
