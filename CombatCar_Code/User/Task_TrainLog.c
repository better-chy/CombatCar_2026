#include "Task_TrainLog.h"

#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "Task_Stage.h"
#include "Track_Enemy.h"
#include "usart.h"

/* 串口接收缓存 */
#define TRAINLOG_RX_DMA_ADDR             0x30000400U /* USART6 训练命令 DMA 缓冲地址。 */
#define TRAINLOG_RX_BUF_LEN                    64U   /* 上位机命令接收缓冲长度。 */

/* 单次采集节奏 */
#define TRAINLOG_CAPTURE_FRAME_COUNT            5U   /* cap 命令一次返回的训练帧数量。 */
#define TRAINLOG_CAPTURE_SETTLE_MS            150U   /* 收到 cap 后等待传感器稳定的时间。 */
#define TRAINLOG_CAPTURE_INTERVAL_MS           30U   /* 单次采集中相邻帧的间隔。 */

/* 连续采集节奏 */
#define TRAINLOG_STREAM_INTERVAL_MS           200U   /* start 连续采集时每帧间隔，200ms 即 1s 5 帧。 */
#define TRAINLOG_STREAM_HOST_TIMEOUT_MS      3000U   /* 连续采集下位机超过该时间收不到 keepalive 就自动停止。 */

/* 训练帧数据包协议 */
#define TRAINLOG_PACKET_MAGIC_0              0xA5U   /* 训练数据包帧头第 1 字节。 */
#define TRAINLOG_PACKET_MAGIC_1              0x5AU   /* 训练数据包帧头第 2 字节。 */
#define TRAINLOG_PACKET_VERSION              0x01U   /* 训练数据包协议版本。 */
#define TRAINLOG_PACKET_TYPE_FRAME           0x31U   /* 台上台下训练帧数据包类型。 */
#define TRAINLOG_PACKET_TYPE_ENEMY_SEGMENTS  0x41U   /* 索敌线段 debug 数据包类型。 */
#define TRAINLOG_PACKET_TYPE_ENEMY_HIT       0x42U   /* 索敌瞬时命中事件包；上位机可选择是否冻结响应。 */
#define TRAINLOG_PACKET_PAYLOAD_LEN          (8U + (2U * (TASKSTAGE_FRONT_SECTOR_POINT_COUNT + TASKSTAGE_RIGHT_SECTOR_POINT_COUNT + TASKSTAGE_LEFT_SECTOR_POINT_COUNT + LASER_RANGE_SENSOR_COUNT))) /* 训练帧 payload 长度。 */
#define TRAINLOG_PACKET_MAX_LEN              (6U + TRAINLOG_PACKET_PAYLOAD_LEN + 2U) /* 训练帧完整包最大长度。 */
#define TRAINLOG_UART_TX_TIMEOUT_MS          250U    /* 串口发送单包超时。 */

/* 索敌线段 debug 流 */
#define TRAINLOG_SEGMENT_STREAM_INTERVAL_MS  200U    /* 索敌线段 debug 连续发送间隔；包含 500 个原始点，115200 波特率下需要放慢。 */
#define TRAINLOG_ENEMY_RAW_SCAN_BYTES        (2U * LIDAR_SCAN_POINT_COUNT) /* 一圈原始 LiDAR 距离数据字节数。 */
#define TRAINLOG_ENEMY_SEGMENT_PAYLOAD_LEN   (32U + (26U * TRACK_ENEMY_DEBUG_MAX_SEGMENTS) + TRAINLOG_ENEMY_RAW_SCAN_BYTES) /* 索敌线段 payload 长度。 */
#define TRAINLOG_ENEMY_SEGMENT_PACKET_MAX_LEN (6U + TRAINLOG_ENEMY_SEGMENT_PAYLOAD_LEN + 2U) /* 索敌线段完整包最大长度。 */

/* 上位机命令类型 */
#define TRAINLOG_CMD_NONE                      0U    /* 当前没有待处理命令。 */
#define TRAINLOG_CMD_CAPTURE                   1U    /* 单次采集命令。 */
#define TRAINLOG_CMD_STREAM_START              2U    /* 开始连续采集命令。 */
#define TRAINLOG_CMD_STREAM_STOP               3U    /* 停止连续采集命令。 */
#define TRAINLOG_CMD_STREAM_KEEPALIVE          4U    /* 连续采集保活命令。 */
#define TRAINLOG_CMD_SEGMENT_START             5U    /* 开始索敌线段 debug 流。 */
#define TRAINLOG_CMD_SEGMENT_STOP              6U    /* 停止索敌线段 debug 流。 */
#define TRAINLOG_CMD_SEGMENT_EVENT_ENABLE      7U    /* 开关索敌瞬时命中事件包。 */

static TaskHandle_t s_trainlog_task_handle = NULL;
static uint8_t *const s_trainlog_rx_buf = (uint8_t *)TRAINLOG_RX_DMA_ADDR;
static volatile uint8_t s_command_type = TRAINLOG_CMD_NONE;
static volatile uint8_t s_command_label = 0U;
static uint8_t s_stream_active = 0U;
static uint8_t s_stream_label = 0U;
static uint8_t s_stream_frame_index = 0U;
static uint32_t s_stream_last_host_tick = 0U;
static uint8_t s_segment_stream_active = 0U;
static volatile uint8_t s_segment_event_enabled = 0U;
static uint32_t s_segment_stream_last_host_tick = 0U;
static uint32_t s_segment_stream_last_send_tick = 0U;
static volatile uint8_t s_segment_send_requested = 0U;
static volatile uint8_t s_segment_hit_send_requested = 0U;
static uint8_t s_enemy_segment_packet[TRAINLOG_ENEMY_SEGMENT_PACKET_MAX_LEN]; /* 索敌 debug 包发送缓存，避免大包占用任务栈。 */
static uint16_t s_enemy_scan_snapshot[LIDAR_SCAN_POINT_COUNT]; /* 索敌 debug 原始距离快照，避免大数组占用任务栈。 */

static void TaskTrainLog_StartRx(void);
static uint8_t TaskTrainLog_ParseCommand(const uint8_t *data, uint16_t len, uint8_t *command_out, uint8_t *label_out);
static uint8_t TaskTrainLog_StartsWith(const uint8_t *data, uint16_t len, const char *prefix);
static uint8_t TaskTrainLog_ParseLabelAfterComma(const uint8_t *data, uint16_t len, uint8_t *label_out);
static void TaskTrainLog_SendCapture(uint8_t label);
static void TaskTrainLog_SendFrame(uint8_t frame_index, uint8_t total_count, uint8_t label);
static void TaskTrainLog_SendEnemySegments(uint8_t packet_type);
static void TaskTrainLog_PutU16(uint8_t *buf, uint16_t *offset, uint16_t value);
static void TaskTrainLog_PutU32(uint8_t *buf, uint16_t *offset, uint32_t value);
static uint16_t TaskTrainLog_Crc16(const uint8_t *data, uint16_t len);

void Task_TrainLog_Run(void *argument)
{
    (void)argument;
    s_trainlog_task_handle = xTaskGetCurrentTaskHandle();
    TaskTrainLog_StartRx();

    for (;;)
    {
        uint8_t command;
        uint8_t label;
        uint8_t segment_send_requested;
        uint8_t segment_hit_send_requested;
        uint32_t now_tick;
        uint32_t wait_ms = 100U;

        if (s_stream_active != 0U && s_segment_stream_active == 0U)
        {
            wait_ms = TRAINLOG_STREAM_INTERVAL_MS;
        }

        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms));
        now_tick = HAL_GetTick();

        taskENTER_CRITICAL();
        command = s_command_type;
        label = s_command_label;
        segment_send_requested = s_segment_send_requested;
        segment_hit_send_requested = s_segment_hit_send_requested;
        s_command_type = TRAINLOG_CMD_NONE;
        s_segment_send_requested = 0U;
        s_segment_hit_send_requested = 0U;
        taskEXIT_CRITICAL();

        if (command == TRAINLOG_CMD_CAPTURE)
        {
            osDelay(TRAINLOG_CAPTURE_SETTLE_MS);
            TaskTrainLog_SendCapture(label);
            continue;
        }

        if (command == TRAINLOG_CMD_STREAM_START)
        {
            s_stream_active = 1U;
            s_segment_stream_active = 0U;
            s_stream_label = label;
            s_stream_frame_index = 0U;
            s_stream_last_host_tick = now_tick;
        }
        else if (command == TRAINLOG_CMD_STREAM_STOP)
        {
            s_stream_active = 0U;
            s_segment_stream_active = 0U;
            s_segment_event_enabled = 0U;
        }
        else if (command == TRAINLOG_CMD_STREAM_KEEPALIVE)
        {
            s_stream_last_host_tick = now_tick;
            s_segment_stream_last_host_tick = now_tick;
        }
        else if (command == TRAINLOG_CMD_SEGMENT_START)
        {
            s_stream_active = 0U;
            s_segment_stream_active = 1U;
            s_segment_event_enabled = 1U;
            s_segment_stream_last_host_tick = now_tick;
            s_segment_stream_last_send_tick = 0U;
        }
        else if (command == TRAINLOG_CMD_SEGMENT_STOP)
        {
            s_segment_stream_active = 0U;
            s_segment_event_enabled = 0U;
        }
        else if (command == TRAINLOG_CMD_SEGMENT_EVENT_ENABLE)
        {
            s_segment_event_enabled = (label != 0U) ? 1U : 0U;
        }

        if (s_stream_active != 0U)
        {
            if ((now_tick - s_stream_last_host_tick) > TRAINLOG_STREAM_HOST_TIMEOUT_MS)
            {
                s_stream_active = 0U;
                continue;
            }

            TaskTrainLog_SendFrame(s_stream_frame_index++, 0U, s_stream_label);
        }

        if (s_segment_stream_active != 0U)
        {
            if ((now_tick - s_segment_stream_last_host_tick) > TRAINLOG_STREAM_HOST_TIMEOUT_MS)
            {
                s_segment_stream_active = 0U;
                s_segment_event_enabled = 0U;
                continue;
            }

            if (s_segment_event_enabled != 0U && segment_hit_send_requested != 0U)
            {
                TaskTrainLog_SendEnemySegments(TRAINLOG_PACKET_TYPE_ENEMY_HIT);
            }

            if ((segment_send_requested != 0U) ||
                (s_segment_stream_last_send_tick == 0U) ||
                ((now_tick - s_segment_stream_last_send_tick) >= TRAINLOG_SEGMENT_STREAM_INTERVAL_MS))
            {
                s_segment_stream_last_send_tick = now_tick;
                TaskTrainLog_SendEnemySegments(TRAINLOG_PACKET_TYPE_ENEMY_SEGMENTS);
            }
        }
    }
}

void TaskTrainLog_NotifyEnemyHit(void)
{
    if (s_segment_event_enabled == 0U)
    {
        return;
    }

    s_segment_hit_send_requested = 1U;
    if (s_trainlog_task_handle != NULL)
    {
        (void)xTaskNotifyGive(s_trainlog_task_handle);
    }
}

void TaskTrainLog_NotifyEnemyFound(void)
{
    s_segment_send_requested = 1U;
    if (s_trainlog_task_handle != NULL)
    {
        (void)xTaskNotifyGive(s_trainlog_task_handle);
    }
}

void TaskTrainLog_HandleRxEventFromISR(UART_HandleTypeDef *huart, uint16_t size)
{
    uint8_t label = 0U;
    uint8_t command = TRAINLOG_CMD_NONE;

    if (huart == NULL || huart->Instance != USART6)
    {
        return;
    }

    if (size > TRAINLOG_RX_BUF_LEN)
    {
        size = TRAINLOG_RX_BUF_LEN;
    }

    if (TaskTrainLog_ParseCommand(s_trainlog_rx_buf, size, &command, &label) != 0U)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        s_command_label = label;
        s_command_type = command;

        if (s_trainlog_task_handle != NULL)
        {
            vTaskNotifyGiveFromISR(s_trainlog_task_handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    TaskTrainLog_StartRx();
}

void TaskTrainLog_HandleError(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != USART6)
    {
        return;
    }

    (void)HAL_UART_DMAStop(huart);
    TaskTrainLog_StartRx();
}

static void TaskTrainLog_StartRx(void)
{
    HAL_StatusTypeDef status;

    memset(s_trainlog_rx_buf, 0, TRAINLOG_RX_BUF_LEN);
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart6, s_trainlog_rx_buf, TRAINLOG_RX_BUF_LEN);
    if (status != HAL_OK)
    {
        (void)HAL_UART_Receive_DMA(&huart6, s_trainlog_rx_buf, TRAINLOG_RX_BUF_LEN);
    }

    if (huart6.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    }
}

static uint8_t TaskTrainLog_ParseCommand(const uint8_t *data, uint16_t len, uint8_t *command_out, uint8_t *label_out)
{
    if (data == NULL || len == 0U)
    {
        return 0U;
    }

    if (command_out != NULL)
    {
        *command_out = TRAINLOG_CMD_NONE;
    }
    if (label_out != NULL)
    {
        *label_out = 0U;
    }

    if (TaskTrainLog_StartsWith(data, len, "STOP") != 0U)
    {
        if (command_out != NULL)
        {
            *command_out = TRAINLOG_CMD_STREAM_STOP;
        }
        return 1U;
    }

    if (TaskTrainLog_StartsWith(data, len, "KEEP") != 0U ||
        TaskTrainLog_StartsWith(data, len, "PING") != 0U)
    {
        if (command_out != NULL)
        {
            *command_out = TRAINLOG_CMD_STREAM_KEEPALIVE;
        }
        return 1U;
    }

    if (TaskTrainLog_StartsWith(data, len, "SEGSTOP") != 0U)
    {
        if (command_out != NULL)
        {
            *command_out = TRAINLOG_CMD_SEGMENT_STOP;
        }
        return 1U;
    }

    if (TaskTrainLog_StartsWith(data, len, "SEGEVENT") != 0U ||
        TaskTrainLog_StartsWith(data, len, "EVENT") != 0U)
    {
        if (TaskTrainLog_ParseLabelAfterComma(data, len, label_out) == 0U)
        {
            return 0U;
        }
        if (command_out != NULL)
        {
            *command_out = TRAINLOG_CMD_SEGMENT_EVENT_ENABLE;
        }
        return 1U;
    }

    if (TaskTrainLog_StartsWith(data, len, "SEGSTART") != 0U ||
        TaskTrainLog_StartsWith(data, len, "SEGMENTS") != 0U ||
        TaskTrainLog_StartsWith(data, len, "SEG") != 0U)
    {
        if (command_out != NULL)
        {
            *command_out = TRAINLOG_CMD_SEGMENT_START;
        }
        return 1U;
    }

    if (TaskTrainLog_StartsWith(data, len, "CAP") != 0U)
    {
        if (TaskTrainLog_ParseLabelAfterComma(data, len, label_out) == 0U)
        {
            return 0U;
        }
        if (command_out != NULL)
        {
            *command_out = TRAINLOG_CMD_CAPTURE;
        }
        return 1U;
    }

    if (TaskTrainLog_StartsWith(data, len, "START") != 0U ||
        TaskTrainLog_StartsWith(data, len, "STREAM") != 0U ||
        TaskTrainLog_StartsWith(data, len, "RUN") != 0U)
    {
        if (TaskTrainLog_ParseLabelAfterComma(data, len, label_out) == 0U)
        {
            return 0U;
        }
        if (command_out != NULL)
        {
            *command_out = TRAINLOG_CMD_STREAM_START;
        }
        return 1U;
    }

    return 0U;
}

static uint8_t TaskTrainLog_StartsWith(const uint8_t *data, uint16_t len, const char *prefix)
{
    uint16_t i = 0U;

    if (data == NULL || prefix == NULL)
    {
        return 0U;
    }

    while (prefix[i] != '\0')
    {
        if (i >= len || data[i] != (uint8_t)prefix[i])
        {
            return 0U;
        }
        ++i;
    }

    return 1U;
}

static uint8_t TaskTrainLog_ParseLabelAfterComma(const uint8_t *data, uint16_t len, uint8_t *label_out)
{
    uint16_t i;
    uint16_t label = 0U;
    uint8_t has_digit = 0U;

    if (data == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < len; ++i)
    {
        if (data[i] == ',')
        {
            ++i;
            while (i < len && data[i] >= '0' && data[i] <= '9')
            {
                label = (uint16_t)((label * 10U) + (uint16_t)(data[i] - '0'));
                has_digit = 1U;
                ++i;
            }
            break;
        }
    }

    if (has_digit == 0U)
    {
        return 0U;
    }

    if (label > 255U)
    {
        label = 255U;
    }

    if (label_out != NULL)
    {
        *label_out = (uint8_t)label;
    }

    return 1U;
}

static void TaskTrainLog_SendCapture(uint8_t label)
{
    uint8_t i;

    for (i = 0U; i < TRAINLOG_CAPTURE_FRAME_COUNT; ++i)
    {
        TaskTrainLog_SendFrame(i, TRAINLOG_CAPTURE_FRAME_COUNT, label);
        osDelay(TRAINLOG_CAPTURE_INTERVAL_MS);
    }
}

static void TaskTrainLog_SendFrame(uint8_t frame_index, uint8_t total_count, uint8_t label)
{
    TaskStageTrainFrame frame;
    uint8_t packet[TRAINLOG_PACKET_MAX_LEN];
    uint16_t offset = 0U;
    uint16_t payload_len_offset;
    uint16_t payload_start;
    uint16_t payload_len;
    uint16_t crc;
    uint16_t i;

    TaskStage_CollectTrainFrame(&frame);

    packet[offset++] = TRAINLOG_PACKET_MAGIC_0;
    packet[offset++] = TRAINLOG_PACKET_MAGIC_1;
    packet[offset++] = TRAINLOG_PACKET_VERSION;
    packet[offset++] = TRAINLOG_PACKET_TYPE_FRAME;
    payload_len_offset = offset;
    offset += 2U;
    payload_start = offset;

    TaskTrainLog_PutU32(packet, &offset, frame.timestamp_ms);
    packet[offset++] = frame_index;
    packet[offset++] = total_count;
    packet[offset++] = label;
    packet[offset++] = 0U;

    for (i = 0U; i < TASKSTAGE_FRONT_SECTOR_POINT_COUNT; ++i)
    {
        TaskTrainLog_PutU16(packet, &offset, frame.lidar_front_mm[i]);
    }
    for (i = 0U; i < TASKSTAGE_RIGHT_SECTOR_POINT_COUNT; ++i)
    {
        TaskTrainLog_PutU16(packet, &offset, frame.lidar_right_mm[i]);
    }
    for (i = 0U; i < TASKSTAGE_LEFT_SECTOR_POINT_COUNT; ++i)
    {
        TaskTrainLog_PutU16(packet, &offset, frame.lidar_left_mm[i]);
    }
    for (i = 0U; i < LASER_RANGE_SENSOR_COUNT; ++i)
    {
        TaskTrainLog_PutU16(packet, &offset, frame.laser_mm[i]);
    }

    payload_len = (uint16_t)(offset - payload_start);
    packet[payload_len_offset] = (uint8_t)(payload_len & 0xFFU);
    packet[payload_len_offset + 1U] = (uint8_t)((payload_len >> 8U) & 0xFFU);

    crc = TaskTrainLog_Crc16(packet, offset);
    TaskTrainLog_PutU16(packet, &offset, crc);

    (void)HAL_UART_Transmit(&huart6, packet, offset, TRAINLOG_UART_TX_TIMEOUT_MS);
}

static void TaskTrainLog_SendEnemySegments(uint8_t packet_type)
{
    TrackEnemyDebugInfo debug;
    EnemyInfo tracker_enemy_info;
    EnemyInfo packet_enemy_info;
    uint8_t *packet = s_enemy_segment_packet;
    uint16_t offset = 0U;
    uint16_t payload_len_offset;
    uint16_t payload_start;
    uint16_t payload_len;
    uint16_t crc;
    uint16_t i;
    uint8_t segment_count;

    taskENTER_CRITICAL();
    memcpy(&debug, (const void *)&g_track_enemy_debug, sizeof(debug));
    tracker_enemy_info = g_enemy_tracker.enemy_info;
    memcpy(s_enemy_scan_snapshot, g_enemy_tracker.scan_distances, sizeof(s_enemy_scan_snapshot));
    taskEXIT_CRITICAL();

    segment_count = debug.segment_count;
    if (segment_count > TRACK_ENEMY_DEBUG_MAX_SEGMENTS)
    {
        segment_count = TRACK_ENEMY_DEBUG_MAX_SEGMENTS;
    }
    packet_enemy_info = tracker_enemy_info;
    if (packet_type == TRAINLOG_PACKET_TYPE_ENEMY_HIT)
    {
        packet_enemy_info = debug.enemy_info;
        debug.find_ok = debug.enemy_info.is_found;
    }

    packet[offset++] = TRAINLOG_PACKET_MAGIC_0;
    packet[offset++] = TRAINLOG_PACKET_MAGIC_1;
    packet[offset++] = TRAINLOG_PACKET_VERSION;
    packet[offset++] = packet_type;
    payload_len_offset = offset;
    offset += 2U;
    payload_start = offset;

    TaskTrainLog_PutU32(packet, &offset, HAL_GetTick());
    TaskTrainLog_PutU32(packet, &offset, debug.update_count);
    TaskTrainLog_PutU32(packet, &offset, debug.update_tick);
    packet[offset++] = debug.snapshot_ok;
    packet[offset++] = debug.find_ok;
    packet[offset++] = debug.last_reject_reason;
    packet[offset++] = segment_count;
    packet[offset++] = debug.segment_overflow;
    packet[offset++] = 0U;
    TaskTrainLog_PutU16(packet, &offset, debug.valid_point_count);
    TaskTrainLog_PutU16(packet, &offset, debug.masked_valid_point_count);
    packet[offset++] = packet_enemy_info.is_found;
    packet[offset++] = 0U;
    TaskTrainLog_PutU16(packet, &offset, (uint16_t)packet_enemy_info.angle_deg);
    TaskTrainLog_PutU16(packet, &offset, packet_enemy_info.distance_mm);
    TaskTrainLog_PutU16(packet, &offset, packet_enemy_info.center_index);
    TaskTrainLog_PutU16(packet, &offset, packet_enemy_info.point_count);

    for (i = 0U; i < segment_count; ++i)
    {
        const TrackEnemyDebugSegment *segment = &debug.segments[i];

        packet[offset++] = segment->accepted;
        packet[offset++] = segment->reject_reason;
        TaskTrainLog_PutU16(packet, &offset, segment->start_index);
        TaskTrainLog_PutU16(packet, &offset, segment->end_index);
        TaskTrainLog_PutU16(packet, &offset, segment->center_index);
        TaskTrainLog_PutU16(packet, &offset, (uint16_t)segment->center_angle_deg);
        TaskTrainLog_PutU16(packet, &offset, segment->min_distance_mm);
        TaskTrainLog_PutU16(packet, &offset, segment->point_count);
        TaskTrainLog_PutU16(packet, &offset, segment->min_points_required);
        TaskTrainLog_PutU16(packet, &offset, segment->width_mm);
        TaskTrainLog_PutU16(packet, &offset, segment->left_background_mm);
        TaskTrainLog_PutU16(packet, &offset, segment->right_background_mm);
        TaskTrainLog_PutU16(packet, &offset, segment->left_jump_mm);
        TaskTrainLog_PutU16(packet, &offset, segment->right_jump_mm);
    }

    for (i = 0U; i < LIDAR_SCAN_POINT_COUNT; ++i)
    {
        TaskTrainLog_PutU16(packet, &offset, s_enemy_scan_snapshot[i]);
    }

    payload_len = (uint16_t)(offset - payload_start);
    packet[payload_len_offset] = (uint8_t)(payload_len & 0xFFU);
    packet[payload_len_offset + 1U] = (uint8_t)((payload_len >> 8U) & 0xFFU);

    crc = TaskTrainLog_Crc16(packet, offset);
    TaskTrainLog_PutU16(packet, &offset, crc);

    (void)HAL_UART_Transmit(&huart6, packet, offset, TRAINLOG_UART_TX_TIMEOUT_MS);
}

static void TaskTrainLog_PutU16(uint8_t *buf, uint16_t *offset, uint16_t value)
{
    buf[(*offset)++] = (uint8_t)(value & 0xFFU);
    buf[(*offset)++] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void TaskTrainLog_PutU32(uint8_t *buf, uint16_t *offset, uint32_t value)
{
    buf[(*offset)++] = (uint8_t)(value & 0xFFU);
    buf[(*offset)++] = (uint8_t)((value >> 8U) & 0xFFU);
    buf[(*offset)++] = (uint8_t)((value >> 16U) & 0xFFU);
    buf[(*offset)++] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint16_t TaskTrainLog_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    if (data == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < len; ++i)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}
