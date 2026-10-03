#ifndef __STAGE_MODEL_H
#define __STAGE_MODEL_H

#include <stdint.h>

#define STAGE_MODEL_FEATURE_COUNT 109U
#define STAGE_MODEL_CLASS_COUNT 8U
#define STAGE_MODEL_INPUT_PROFILE 2U

/* Label ids:
 * 0: 台上正常
 * 1: 台下平行区域
 * 2: 台下角在车左前
 * 3: 台下角在车右前
 * 4: 台下角在车左后
 * 5: 台下角在车右后
 * 6: 台下四角未对齐
 * 9: 台下平行且可直接倒车上台
 */

typedef enum
{
    STAGE_MODEL_LABEL_0 = 0,
    STAGE_MODEL_LABEL_1 = 1,
    STAGE_MODEL_LABEL_2 = 2,
    STAGE_MODEL_LABEL_3 = 3,
    STAGE_MODEL_LABEL_4 = 4,
    STAGE_MODEL_LABEL_5 = 5,
    STAGE_MODEL_LABEL_6 = 6,
    STAGE_MODEL_LABEL_9 = 9
} StageModelLabel;

uint8_t StageModel_Predict(const int16_t features[STAGE_MODEL_FEATURE_COUNT]);

#endif
