/*
 * 蓝球识别诊断补丁
 *
 * 使用方法：
 * 1. 将此文件的内容添加到 ball_vision.c 的适当位置
 * 2. 启用 BALL_VISION_DEBUG 宏来开启详细日志
 * 3. 通过串口监视器观察诊断信息
 *
 * 在 ball_vision.c 的顶部（#include 之后）添加：
 * #define BALL_VISION_DEBUG 1
 */

#ifndef BALL_VISION_DEBUG
#define BALL_VISION_DEBUG 0
#endif

#if BALL_VISION_DEBUG
#define DEBUG_LOG(fmt, ...) ESP_LOGI(TAG, "[DEBUG] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_LOG(fmt, ...)
#endif

/*
 * 在 ball_vision_analyze() 函数中，在 "out->valid = true;" 之前添加：
 */

#if BALL_VISION_DEBUG
ESP_LOGI(TAG, "========== Ball Vision Diagnostics ==========");
ESP_LOGI(TAG, "Frame: %ux%u, stride=%zu", width, height, stride_bytes);
ESP_LOGI(TAG, "Table luma: %u (histogram peak)", table_luma);
ESP_LOGI(TAG, "Pixel classification: red=%u, blue=%u, pocket=%u",
         red_px, blue_px, pocket_px);

// 显示亮度分布
ESP_LOGI(TAG, "Luma histogram (16 bins):");
for (int i = 0; i < 16; i++) {
    ESP_LOGI(TAG, "  [%2d-%2d]: %u", i*16, (i+1)*16-1, out->luma_hist[i]);
}
#endif

/*
 * 在 pick_best_ball() 函数中，修改循环部分（约第366-392行）：
 */

#if BALL_VISION_DEBUG
DEBUG_LOG("Processing %u blue components", count);
#endif

for (unsigned i = 0; i < count; ++i) {
    const ball_vision_component_t *c = &components[i];

    const unsigned area = (unsigned)c->area;
    const unsigned min_area = class_id == BALL_VISION_CLASS_RED
        ? (unsigned)BALL_RED_MIN_AREA_PX : (unsigned)BALL_MIN_AREA_PX;
    const unsigned compactness_min = class_id == BALL_VISION_CLASS_RED
        ? (unsigned)BALL_RED_COMPACTNESS_MIN
        : (unsigned)BALL_COMPACTNESS_MIN;
    const int max_aspect = class_id == BALL_VISION_CLASS_RED
        ? (int)BALL_RED_MAX_ASPECT : (int)BALL_MAX_ASPECT;
    const int axis_min = class_id == BALL_VISION_CLASS_RED
        ? (int)BALL_RED_AXIS_RATIO_MIN_MILLE
        : (int)BALL_AXIS_RATIO_MIN_MILLE;

#if BALL_VISION_DEBUG
    const int w = c->x1 - c->x0 + 1;
    const int h = c->y1 - c->y0 + 1;
    const int compactness = (int)bbox_fill_percent(c);
    const int aspect = (w > h) ? (w * 100 / h) : (h * 100 / w);
    const int axis = (int)axis_ratio_mille(c);
    const int luma_range = c->luma_max - c->luma_min;
    const int cx = c->sum_x / c->area;
    const int cy = c->sum_y / c->area;

    DEBUG_LOG("Component %u: area=%u, bbox=(%d,%d)-(%d,%d) %dx%d, "
              "compactness=%d%%, aspect=%d, axis=%d, luma_range=%d, center=(%d,%d)",
              i, area, c->x0, c->y0, c->x1, c->y1, w, h,
              compactness, aspect, axis, luma_range, cx, cy);
#endif

    if (!shape_ok(c, class_id)) {
#if BALL_VISION_DEBUG
        if (area < min_area) {
            DEBUG_LOG("  → REJECTED: area %u < %u (min_area)", area, min_area);
        } else if (area > BALL_MAX_AREA_PX) {
            DEBUG_LOG("  → REJECTED: area %u > %u (max_area)", area, BALL_MAX_AREA_PX);
        } else if (compactness < compactness_min) {
            DEBUG_LOG("  → REJECTED: compactness %d%% < %u%%", compactness, compactness_min);
        } else if (aspect > max_aspect) {
            DEBUG_LOG("  → REJECTED: aspect %d > %d (max_aspect)", aspect, max_aspect);
        } else if (axis < axis_min) {
            DEBUG_LOG("  → REJECTED: axis ratio %d < %d (min_axis)", axis, axis_min);
        } else {
            DEBUG_LOG("  → REJECTED: unknown shape issue");
        }
#endif
        continue;
    }

    if (class_id == BALL_VISION_CLASS_BLUE) {
        const int cy = c->sum_y / c->area;
        if (cy > BALL_BLUE_MAX_CY_PX) {
#if BALL_VISION_DEBUG
            DEBUG_LOG("  → REJECTED: cy=%d > %d (BALL_BLUE_MAX_CY_PX)",
                      cy, BALL_BLUE_MAX_CY_PX);
#endif
            continue;
        }
        if (c->luma_max - c->luma_min < BALL_BLUE_GRADIENT_MIN) {
#if BALL_VISION_DEBUG
            DEBUG_LOG("  → REJECTED: luma_range=%d < %d (BALL_BLUE_GRADIENT_MIN)",
                      luma_range, BALL_BLUE_GRADIENT_MIN);
#endif
            continue;
        }
    }

#if BALL_VISION_DEBUG
    DEBUG_LOG("  → ACCEPTED: valid ball candidate");
#endif

    if ((unsigned)c->area > best_area) {
        best_area = (unsigned)c->area;
        best = (int)i;
    }
}

/*
 * 在 ball_vision_analyze() 函数的最后（返回之前），添加：
 */

#if BALL_VISION_DEBUG
ESP_LOGI(TAG, "=== Final Results ===");
ESP_LOGI(TAG, "Red ball: %s",
         out->balls[BALL_COLOR_RED].present ? "FOUND" : "NOT FOUND");
if (out->balls[BALL_COLOR_RED].present) {
    ESP_LOGI(TAG, "  cx=%d, cy=%d, radius=%d, area=%u",
             out->balls[BALL_COLOR_RED].cx,
             out->balls[BALL_COLOR_RED].cy,
             out->balls[BALL_COLOR_RED].radius,
             out->balls[BALL_COLOR_RED].area);
}
ESP_LOGI(TAG, "Blue ball: %s",
         out->balls[BALL_COLOR_BLUE].present ? "FOUND" : "NOT FOUND");
if (out->balls[BALL_COLOR_BLUE].present) {
    ESP_LOGI(TAG, "  cx=%d, cy=%d, radius=%d, area=%u",
             out->balls[BALL_COLOR_BLUE].cx,
             out->balls[BALL_COLOR_BLUE].cy,
             out->balls[BALL_COLOR_BLUE].radius,
             out->balls[BALL_COLOR_BLUE].area);
}
ESP_LOGI(TAG, "Pockets: %u found", out->pocket_count);
for (unsigned i = 0; i < out->pocket_count; i++) {
    ESP_LOGI(TAG, "  Pocket %u: cx=%d, cy=%d, area=%u",
             i, out->pockets[i].cx, out->pockets[i].cy, out->pockets[i].area);
}
ESP_LOGI(TAG, "==========================================");
#endif

/*
 * 使用说明：
 *
 * 1. 在 ball_vision.c 文件开头（所有 #include 之后）添加：
 *    #define BALL_VISION_DEBUG 1
 *
 * 2. 将上面的代码块插入到指定位置
 *
 * 3. 重新编译并烧录：
 *    idf.py build flash monitor
 *
 * 4. 观察串口输出，查找以下信息：
 *
 *    a) 如果 blue_px 非常小（<10）：
 *       → 像素分类问题，需要调整 BALL_BLUE_MIN_* 阈值
 *
 *    b) 如果 blue_px 正常但没有候选组件：
 *       → 连通区域问题，可能需要调整最小面积
 *
 *    c) 如果有候选组件但全部被 REJECTED：
 *       → 查看具体的拒绝原因：
 *          - "area < min_area": 降低 BALL_MIN_AREA_PX
 *          - "compactness < min": 降低 BALL_COMPACTNESS_MIN
 *          - "aspect > max_aspect": 提高 BALL_MAX_ASPECT
 *          - "axis < min_axis": 降低 BALL_AXIS_RATIO_MIN_MILLE
 *          - "cy > BALL_BLUE_MAX_CY_PX": 提高 BALL_BLUE_MAX_CY_PX
 *          - "luma_range < BALL_BLUE_GRADIENT_MIN": 降低 BALL_BLUE_GRADIENT_MIN
 *
 *    d) 如果有 ACCEPTED 候选但最终结果为 NOT FOUND：
 *       → 可能是选择了错误的候选（选择了面积最大的，但不是真正的球）
 *
 * 5. 根据诊断结果调整 board_config.h 中的相应参数
 *
 * 6. 完成调试后，将 BALL_VISION_DEBUG 改为 0 或删除定义
 */

/*
 * 示例输出：
 *
 * I (12345) ball_vision: ========== Ball Vision Diagnostics ==========
 * I (12346) ball_vision: Frame: 120x80, stride=360
 * I (12347) ball_vision: Table luma: 145 (histogram peak)
 * I (12348) ball_vision: Pixel classification: red=234, blue=0, pocket=156
 * I (12349) ball_vision: Luma histogram (16 bins):
 * I (12350) ball_vision:   [ 0-15]: 245
 * I (12351) ball_vision:   [16-31]: 1890
 * I (12352) ball_vision:   [32-47]: 4567
 * I (12353) ball_vision:   ...
 * I (12354) ball_vision: Processing 3 blue components
 * I (12355) ball_vision: [DEBUG] Component 0: area=5, bbox=(40,20)-(44,24) 5x5, compactness=20%, aspect=100, axis=850, luma_range=2, center=(42,22)
 * I (12356) ball_vision: [DEBUG]   → REJECTED: area 5 < 10 (min_area)
 * I (12357) ball_vision: [DEBUG] Component 1: area=12, bbox=(50,30)-(55,35) 6x6, compactness=33%, aspect=100, axis=900, luma_range=4, center=(52,32)
 * I (12358) ball_vision: [DEBUG]   → ACCEPTED: valid ball candidate
 * I (12359) ball_vision: [DEBUG] Component 2: area=8, bbox=(60,40)-(64,43) 5x4, compactness=40%, aspect=125, axis=700, luma_range=1, center=(62,41)
 * I (12360) ball_vision: [DEBUG]   → REJECTED: axis ratio 700 < 750 (min_axis)
 * I (12361) ball_vision: === Final Results ===
 * I (12362) ball_vision: Red ball: FOUND
 * I (12363) ball_vision:   cx=45, cy=48, radius=8, area=234
 * I (12364) ball_vision: Blue ball: FOUND
 * I (12365) ball_vision:   cx=52, cy=32, radius=3, area=12
 * I (12366) ball_vision: Pockets: 2 found
 * I (12367) ball_vision:   Pocket 0: cx=15, cy=8, area=245
 * I (12368) ball_vision:   Pocket 1: cx=105, cy=8, area=238
 * I (12369) ball_vision: ==========================================
 *
 * 从这个输出可以看出：
 * - 只有12个蓝色像素（blue_px=0 说明没有像素通过分类）
 * - 3个候选组件，但只有1个通过所有检查
 * - Component 0 因面积太小被拒绝
 * - Component 2 因主轴比太低被拒绝
 * - Component 1 最终被选为蓝球
 */
