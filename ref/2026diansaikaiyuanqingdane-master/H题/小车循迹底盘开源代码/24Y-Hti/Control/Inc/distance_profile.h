#ifndef DISTANCE_PROFILE_H
#define DISTANCE_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t target_distance_um;
    int32_t maximum_speed_mm_s;
    int32_t minimum_speed_mm_s;
    int32_t slowdown_distance_um;
    int32_t tolerance_um;
    bool finished;
} DistanceProfile;

void DistanceProfile_Init(DistanceProfile *profile,
                          int32_t maximum_speed_mm_s,
                          int32_t minimum_speed_mm_s,
                          int32_t slowdown_distance_mm,
                          int32_t tolerance_mm);
void DistanceProfile_Start(DistanceProfile *profile,
                           int32_t current_distance_um,
                           int32_t relative_distance_mm);
int32_t DistanceProfile_Update(DistanceProfile *profile,
                               int32_t current_distance_um);
int32_t DistanceProfile_GetRemainingUm(const DistanceProfile *profile,
                                       int32_t current_distance_um);
bool DistanceProfile_IsFinished(const DistanceProfile *profile);

#endif /* DISTANCE_PROFILE_H */
