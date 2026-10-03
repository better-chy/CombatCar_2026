#ifndef __DANGER_MODEL_H
#define __DANGER_MODEL_H

#include <stdint.h>

#define DANGER_MODEL_FEATURE_COUNT 109U
#define DANGER_MODEL_CLASS_COUNT 2U

/* Label ids:
 * 0: safe_full_speed
 * 1: slow_after_50pct
 */

typedef enum
{
    DANGER_MODEL_LABEL_0 = 0,
    DANGER_MODEL_LABEL_1 = 1
} DangerModelLabel;

uint8_t DangerModel_Predict(const int16_t features[DANGER_MODEL_FEATURE_COUNT]);

#endif
