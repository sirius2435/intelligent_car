# 推球入洞（循迹终点后的任务）

小车完成循迹、压过终点条（`LINE_STOP_REASON_FINISH`）停车后，自动切换摄像头解码
网格并开始推球：在**白色背景板**上找到随机摆放的两个小球（红、蓝），先把**红球**推入
**左侧**底袋，再把**蓝球**推入**右侧**底袋（映射与顺序见
`BALL_TASK_*` 宏）。推失自动退避重试，双球完成或超时/重试耗尽后锁停。

## 原理

推球是 1 自由度的事——**只要车头的推线对**：球沿被撞击的方向滚，所以车必须站在
"球 → 袋"连线的延长线上（球在车与袋之间）直线推。相机装在车上且正前方＝车头，
于是图像空间里有一个非常干净的判据：

> **球心和袋心都落在画面中线上 ⇔ 车、球、袋三点共线，且车在球后。**

控制上把这条共线拆成两个近似解耦的调节器：

| 执行器 | 控制量 | 作用 |
|---|---|---|
| 旋转 turn | 袋心横偏 `kp*(pocket.x − 中线)` | 只有旋转能改变远处袋的横向位置 → 瞄准"推线方向" |
| 横移 lateral | 球心横偏 `−kp*(ball.x − 中线)` | 把车平移上"球→袋"线 → 站到球正后方 |

袋不在画面内时对**虚拟袋方位**（`POCKET_FALLBACK_*`）瞄准试探。对好后直推
（`PUSH`），只做小幅转角保持球心居中；球像素半径是**距离代理**（固定大小球，
越大越近），过大说明球贴到车头 → 后退（`BACKOFF`）拉开。

为什么不能"看到球就冲"：只对准球心只能保证车头指向球，不能保证球会滚向袋——
这正是双调节器要解决的。

## 运行流程

```
循迹 STOPPED (stop_reason=FINISH)
   ↓ 切换相机到 120×80 推球网格（scale 4）
CREEP_OFF_FINISH  缓速前探，直到看见球或前探步数用尽
   ↓（球不可见）
SCAN              原地扫腿（编码器计数标定，先右后左，最多 4 腿）
   ↓（找到当前颜色球）
ALIGN ⇄ BACKOFF   袋心对准中线 + 球心对准中线 200ms → 太近则后退重对
   ↓
PUSH              直线推 + 小幅转角保持；监测漂移/越顶/丢失/堵转
   ├ 成功：球心进入袋区(内缩 4px) 连续 3 帧，或球在袋内消失
   └ 失败：退避 BACKOUT → 重试(≤ BALL_TASK_RETRY_MAX) → 换球 / FAULT
EGRESS（第一球成功后倒车脱离桌边）→ 推第二球（蓝→右）→ DONE 锁停
全局 BALL_TASK_TIMEOUT_MS 或任意阶段堵转/找不到球 → FAULT_STOP 锁停
```

推球阶段避障完全关闭（`main.c` 按 run_phase 结构性不调用它），超声波的读数只给
LCD 用，否则它会把球和台面当成障碍触发避障。

## 标定清单（先用 Wi-Fi 页面调视觉，再上台跑）

1. **光照**：均匀、无直射反光（反光斑会被圆度/长宽比门控剔除，但越少越好）。
2. **画面参考**：浏览器连热点 `vison_car`/`12345678`，开 http://192.168.4.1/ 。
   推球阶段推流页仍在跑（MJPEG 为原始 480×320，叠加信息为循迹画框，推球用
   串口心跳日志 `push=...` 观察）。
3. **红球阈值** `BALL_RED_*`：红球应读出 R−G、R−B 明显优势。若台面暖色漏检，
   先抬 `MIN_RG_DOM / MIN_RB_DOM`，再抬 `MIN_R`。
4. **蓝球阈值** `BALL_BLUE_*`：与红球同一套思路——**纯色差优势**判定
   （`B−R ≥ MIN_BR_DOM` 且 `B−G ≥ MIN_BG_DOM`），不设亮度窗口，所以欠曝时三通道
   等比变暗、色差优势仍在，球不会被阈值杀掉。白色背景板与板上阴影都是
   **中性**的，原理上不可能冒充蓝球（这正是原白球判据最容易翻车的地方，也是
   换成蓝球后附带拿到的好处）。`MIN_B` / `MIN_SAT` 是**暗区噪声底**：黑巡线带、
   袋内阴影这类暗区的色度噪声相对更大，用这两个下限挡在球类之外。球还需有
   球面明暗梯度（`GRADIENT_MIN`，现场若球面读太平可降到 0）、圆度/长宽比/二阶矩
   轴比门控，以及 `MAX_CY_PX`（画面下部的车体金属、线束可能带蓝偏）。
5. **袋阈值** `POCKET_*`：底袋在相机里读作**暗色近中性**区域，判据是
   `luma ≤ POCKET_BLACK_MAX_LUMA` 且 `sat ≤ POCKET_BLACK_MAX_SAT`，只扫远端上带
   （`REGION_MAX_Y_PERCENT`），且连通域必须**贴到画面上边缘**
   （`POCKET_EDGE_TOUCH_ROWS_PIXELS`）——台面内部的地板标记、阴影、散落黑物都
   不会变成假袋；若物理边界是一整条连续黑带，`add_edge_zone_pocket()` 会按左右
   半幅切出两个目标区。**注意蓝球与底袋的颜色冲突**：分类顺序是 红 → 蓝 → 暗袋，
   蓝球优先，因此球的暗侧不会被袋吃掉；但反过来，若现场底袋的蓝漆在画面里真的
   发蓝（sat 超过 `POCKET_BLACK_MAX_SAT`），它就会落进蓝球类，此时只剩几何门控
   （长宽比/轴比/面积）在挡，必须恢复"蓝色色差袋"判据并让它优先于球分类。
6. **尺寸（重点）**：台上实测球在工作图里很小——约 4~6 px 直径（和伪红外
   采样块差不多大）。`BALL_MIN_AREA_PX=10`（半径≈1.8px）已按此设定下限，
   上限 3200 覆盖贴脸球；页面叠加的球圆会显示半径数字，可据此核对。**若
   检出率仍不够，把 `PUSH_CAMERA_DECODE_SCALE` 从 4 改成 2**：工作图变
   240×160、球翻倍到 ~12 px 直径，识别更稳，代价是解码帧率降到 ~2-3 fps
   （推球速度本就慢，可接受；真机试跑对比即可）。推球/横移速度先低后高。

## 台上调参顺序

1. 架空/低速台上：确认终点停车后能切到 `push=CREEP` 并前探。
2. 看心跳日志 `ball=[red=1 blue=0] pockets=2` 等字段确认识别稳定（含抖动帧）。
3. 只留一个球手动放到正前方远处：调 `ALIGN` 增益使车能原地对准（袋心+球心都居中）。
4. 短距（10 cm）直推测试：调 `PUSH_PUSH_FORWARD` 与转角保持，确认入袋判据触发。
5. 全流程：两球随机摆放跑完整任务；观察 miss 原因（日志 `state=PUSH`→`BACKOUT`
   说明推偏/丢失），对症调 `DRIFT_MAX_PX / BALL_LOST_FRAMES / PUSH_PUSH_MAX_COUNTS`
   或增益。

## 关键参数（main/board_config.h 尾部）

```c
#define PUSH_CAMERA_DECODE_SCALE    4    // 推球阶段解码网格
#define BALL_TASK_FIRST_COLOR       0    // 0=红球   BALL_COLOR_RED
#define BALL_TASK_FIRST_POCKET      0    // 0=左袋
#define BALL_TASK_SECOND_COLOR      1    // 1=蓝球 BALL_COLOR_BLUE
#define BALL_TASK_SECOND_POCKET     1    // 1=右袋
// BALL_RED_* / BALL_BLUE_* / BALL_MIN|MAX_AREA_PX / BALL_COMPACTNESS_MIN /
// BALL_MAX_ASPECT / POCKET_* / PUSH_* / BALL_TASK_RETRY_MAX /
// BALL_TASK_TIMEOUT_MS / BALL_VISION_FRESH_MAX_MS 见文件内注释
```

## 新模块

| 文件 | 作用 |
|---|---|
| `main/ball_vision.h/.c` | 颜色分类 + 连通域：红球/蓝球/暗色底袋，逻辑坐标输出，纯 C 可主机测试 |
| `main/ball_push.h/.c` | 推球状态机（见上表），输出 motion + drive_mode |
| `main/camera_vision.c` | LINE/PUSH 双模式逐帧解码，PUSH 模式内跑 ball_vision 并发布结果 |
| `main/line_follow.c` | 新增 `stop_reason` 区分终点/丢线/搜索失败 |
| `main/main.c` | run_phase 门控与交接 |
| `tests/ball_vision_test.c` | 合成图像检测单测 |
| `tests/ball_push_sequence_test.c` | 状态机场景回放单测 |
