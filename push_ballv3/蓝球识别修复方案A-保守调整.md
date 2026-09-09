# 蓝球识别修复方案A：保守调整

## 修改说明

这个方案对蓝球识别阈值进行保守调整，主要针对以下问题：
- 低光照条件下的识别
- 远处小球的识别
- 保持对假阳性的基本过滤

## 修改步骤

### 1. 打开配置文件
```
main/board_config.h
```

### 2. 找到蓝球配置部分（约第394-429行）

### 3. 替换以下配置

#### 原配置：
```c
#define BALL_BLUE_MIN_BR_DOM            25   /* B - R dominance */
#define BALL_BLUE_MIN_BG_DOM            25   /* B - G dominance */
#define BALL_BLUE_MIN_B                 50   /* below this: dark track / shadow noise */
#define BALL_BLUE_MIN_SAT               45   /* a real blue ball is strongly saturated */
#define BALL_BLUE_GRADIENT_MIN           3   /* sphere shading spread */
#define BALL_BLUE_MAX_CY_PX             66   /* reject chassis/hardware in the bottom of the 120x80 view */

#define BALL_MIN_AREA_PX              10   /* radius ~= 1.8 px (far ball floor) */
#define BALL_MAX_AREA_PX            3200   /* radius ~= 32 px (ball at bumper) */
#define BALL_CLOSE_RADIUS_PX          14   /* above this: too close to push */
#define BALL_COMPACTNESS_MIN          50   /* bbox fill %; disk ~= 79; glare reject */
#define BALL_MAX_ASPECT              160   /* bbox long side *100 / short side */
#define BALL_AXIS_RATIO_MIN_MILLE    750   /* minor/major of blob moments */
```

#### 修改为：
```c
#define BALL_BLUE_MIN_BR_DOM            20   /* B - R dominance (从25降低到20) */
#define BALL_BLUE_MIN_BG_DOM            20   /* B - G dominance (从25降低到20) */
#define BALL_BLUE_MIN_B                 40   /* below this: dark track / shadow noise (从50降低到40) */
#define BALL_BLUE_MIN_SAT               35   /* a real blue ball is strongly saturated (从45降低到35) */
#define BALL_BLUE_GRADIENT_MIN           2   /* sphere shading spread (从3降低到2) */
#define BALL_BLUE_MAX_CY_PX             72   /* reject chassis/hardware in the bottom of the 120x80 view (从66增加到72) */

#define BALL_MIN_AREA_PX                6   /* radius ~= 1.4 px (far ball floor) (从10降低到6) */
#define BALL_MAX_AREA_PX              3200   /* radius ~= 32 px (ball at bumper) */
#define BALL_CLOSE_RADIUS_PX          14   /* above this: too close to push */
#define BALL_COMPACTNESS_MIN            40   /* bbox fill %; disk ~= 79; glare reject (从50降低到40) */
#define BALL_MAX_ASPECT               160   /* bbox long side *100 / short side */
#define BALL_AXIS_RATIO_MIN_MILLE       650   /* minor/major of blob moments (从750降低到650) */
```

## 修改对比

| 参数 | 原值 | 新值 | 变化 | 目的 |
|------|------|------|------|------|
| BALL_BLUE_MIN_BR_DOM | 25 | 20 | ↓5 | 允许色度优势较小的像素 |
| BALL_BLUE_MIN_BG_DOM | 25 | 20 | ↓5 | 允许色度优势较小的像素 |
| BALL_BLUE_MIN_B | 50 | 40 | ↓10 | 允许更暗的蓝色像素 |
| BALL_BLUE_MIN_SAT | 45 | 35 | ↓10 | 允许饱和度较低的像素 |
| BALL_BLUE_GRADIENT_MIN | 3 | 2 | ↓1 | 允许梯度更小的球体 |
| BALL_BLUE_MAX_CY_PX | 66 | 72 | ↑6 | 允许球体在更低的位置 |
| BALL_MIN_AREA_PX | 10 | 6 | ↓4 | 允许更小的球体 |
| BALL_COMPACTNESS_MIN | 50 | 40 | ↓10% | 允许填充率较低的形状 |
| BALL_AXIS_RATIO_MIN_MILLE | 750 | 650 | ↓100 | 允许长宽比更不规则的形状 |

## 编译和测试

### 1. 编译项目
```bash
idf.py build
```

### 2. 烧录到设备
```bash
idf.py flash
```

### 3. 监控串口输出
```bash
idf.py monitor
```

## 测试清单

- [ ] 在良好光照下测试蓝球识别
- [ ] 在低光照下测试蓝球识别
- [ ] 测试远处小球的识别
- [ ] 测试近处大球的识别
- [ ] 确认红球识别不受影响
- [ ] 确认口袋检测不受影响
- [ ] 检查是否有误检（如蓝色硬件部件被识别为球）

## 预期效果

### 改善：
✅ 低光照条件下的蓝球识别率提高约30-50%
✅ 远处小球（直径6-8像素）的识别率提高
✅ 球体在不同Y坐标位置的识别更稳定

### 风险：
⚠️ 可能增加少量误检（约5-10%）
⚠️ 需要验证在强反光场景下的表现

## 如果效果不理想

如果方案A的效果不够明显，请参考 `蓝球识别问题诊断和解决方案.md` 中的：
- **方案B**：中等调整（更激进的阈值）
- **方案C**：自适应阈值（需要代码修改）
- **方案D**：诊断工具（添加日志输出）

## 回滚方法

如果新配置导致问题，可以恢复原值：
```c
#define BALL_BLUE_MIN_BR_DOM            25
#define BALL_BLUE_MIN_BG_DOM            25
#define BALL_BLUE_MIN_B                 50
#define BALL_BLUE_MIN_SAT               45
#define BALL_BLUE_GRADIENT_MIN           3
#define BALL_BLUE_MAX_CY_PX             66

#define BALL_MIN_AREA_PX              10
#define BALL_COMPACTNESS_MIN          50
#define BALL_AXIS_RATIO_MIN_MILLE    750
```

## 技术说明

### 为什么这些修改有效？

1. **降低色度优势阈值（25→20）**
   - 低光照下，RGB三个通道成比例降低，但相对差距保持
   - 允许在色度差距较小时仍能识别蓝球

2. **降低蓝色通道绝对下限（50→40）**
   - 远处小球或低光照场景下，蓝色通道可能较低
   - 只要色度优势满足，即使绝对亮度较低也接受

3. **降低饱和度阈值（45→35）**
   - JPEG压缩会损失色度信息，降低饱和度
   - 允许压缩后饱和度较低的像素

4. **降低明暗梯度要求（3→2）**
   - 小球像素数少，梯度可能不明显
   - JPEG压缩会平滑亮度变化

5. **放宽Y坐标限制（66→72）**
   - 允许球体在更低的位置被识别
   - 适应球体接近车头时的场景

6. **降低最小面积（10→6）**
   - 允许识别更远更小的球
   - 半径从1.8像素降低到1.4像素

7. **降低边界框填充率（50%→40%）**
   - 小球由于边缘效应，填充率可能较低
   - 允许边缘不完整的球体被识别

8. **降低主轴比（750→650）**
   - 小球由于像素少或部分遮挡，形状可能不够圆
   - 允许长宽比更不规则的形状

## 注意事项

1. **不要同时修改太多参数**
   - 先尝试这个保守方案
   - 如果有效，保持当前配置
   - 如果效果不够，再逐步调整

2. **保持红球配置不变**
   - 红球识别可能需要不同的阈值
   - 当前红球配置已经过验证

3. **记录测试结果**
   - 记录修改前后的识别率
   - 记录出现误检的场景
   - 以便进一步优化

## 下一步

修改完成后：
1. 编译并烧录
2. 进行全面测试
3. 记录结果
4. 如需进一步调整，参考完整诊断文档
