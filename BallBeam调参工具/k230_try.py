"""K230D 钢珠识别 - BG约束霍夫圆（独立单文件版）。

================================================================
  启动要求：前60帧轨道内无钢珠（背景差分需要），霍夫圆即刻工作。
================================================================

  核心思路：
    背景差分精度高，但光照突变时可能短暂失效。
    霍夫圆天然抗光照，但圆心抖动大。
    → BG有效时，用BG位置约束霍夫圆：两者一致才信任霍夫圆，不一致则丢弃。
    → BG失效时，霍夫圆独立兜底。

  核心流程:
    1. GC2093 摄像头 640×480 灰度 @60fps
    2. ROI 裁剪轨道区域
    3. 通道A-背景差分: 差分 → 高斯 → 二值化 → blob → 加权质心精修
    4. 通道B-霍夫圆:  cv_lite 识别圆形轮廓 → 半径+Y+连续性筛选
    5. BG约束: BG有效时，霍夫圆须在BG位置±CROSS_CONSTRAINT_PX内才信任
    6. 融合: 两道一致→取均值; 仅BG→用它; 仅霍夫圆→用它(兜底)
    7. BallTracker 状态机平滑
    8. UART 输出相对轨道中心O的位置、帧号和识别状态

  用法: 复制到 K230D 的 /sdcard/main.py 即可开机自启动。
================================================================
"""

import time
import image
import cv_lite

from machine import UART, FPIOA
from media.sensor import *
from media.display import *
from media.media import *


# ======================================================================
#  第一部分：所有可调参数（按模块分组，常量名全大写，含说明）
# ======================================================================

# ----------------------------------------------------------------------
#  1. 图像采集
# ----------------------------------------------------------------------

CAMERA_WIDTH  = 640                  # 摄像头采集宽度 (px)
CAMERA_HEIGHT = 480                  # 摄像头采集高度 (px)
CAMERA_TARGET_FPS = 60               # 摄像头目标帧率

# ROI 区域：裁剪出轨道所在画面区域
#   格式 (x_左上角, y_左上角, width, height)，坐标基于 640x480 全图
#   x 越大越靠右, y 越大越靠下, 原点在画面左上角
ROI_X          = 30                  # ROI 左上角 X
ROI_Y          = 190             # ROI 左上角 Y（调此值上下移动ROI窗口）
ROI_WIDTH      = 610                # ROI 宽度 (px), 应尽量覆盖整根轨道
ROI_HEIGHT     = 50               # ROI 高度 (px), 包含轨道+钢珠即可, 不要太宽
ROI_LENGTH_CM  = 25.0                # ROI 宽度对应的物理轨道长度 (cm), 固定=摆杆长度

# ----------------------------------------------------------------------
#  2. UART 通信 (K230D → B板 的 RM A型板)
# ----------------------------------------------------------------------

UART_TX_PIN    = 11                  # K230D TX 引脚 (接 B板 RX)
UART_RX_PIN    = 12                  # K230D RX 引脚 (接 B板 TX, 备用)
UART_BAUDRATE  = 38400               # 波特率 (≤115200, 越低越可靠)
UART_ENABLE    = True                # 是否启用 UART 输出 (调试关: False)
USB_BRIDGE_MIRROR_ENABLE = True       # 电脑桥接: 同帧镜像到USB控制台

# ----------------------------------------------------------------------
#  3. IDE 显示 & 调试
# ----------------------------------------------------------------------

LOG_INTERVAL_MS       = 100          # 控制台打印位置信息间隔 (ms)
DISPLAY_EVERY_N_FRAMES = 2           # 每N帧更新一次 IDE 显示 (1=每帧, 值越大FPS越高)
IDE_JPEG_QUALITY      = 40           # IDE 远程显示 JPEG 压缩质量 (1-100, 越低越省带宽)
EXPOSURE_MANUAL_US    = 0            # 手动曝光时间 (μs), 0=自动曝光

# IDE 显示颜色 (灰度值 0=黑 255=白, 中间值为不同灰度)
DISPLAY_COLOR_HOUGH_CIRCLE   = 192   # 霍夫圆检测结果颜色 (亮灰=偏红在IDE中)
DISPLAY_COLOR_BG_RECT        = 64    # 背景差分检测结果颜色 (深灰=偏绿在IDE中)
DISPLAY_COLOR_ROI_BORDER     = 255   # ROI 边框颜色 (白色)
DISPLAY_COLOR_PROJ_LINE      = 128   # 辅助线颜色 (中灰)
DISPLAY_COLOR_INFO_BG        = 0     # 顶部信息栏背景 (黑色)
DISPLAY_COLOR_INFO_TEXT      = 255   # 顶部信息栏文字 (白色)

# ----------------------------------------------------------------------
#  4. BallTracker 状态机
# ----------------------------------------------------------------------

TRACK_CONFIRM_FRAMES      = 2        # 新球确认需要连续检测到的帧数
TRACK_CONFIRM_MAX_JUMP_PX = 30       # 确认阶段候选帧间最大允许跳动 (px)
TRACK_MAX_JUMP_PX         = 80       # 跟踪阶段单帧最大允许跳动 (px), 超出丢弃
TRACK_HOLD_FRAMES         = 3        # 漏检时保持上帧位置的帧数
TRACK_EMA_ALPHA           = 0.35     # EMA 指数平滑系数 (0=不平滑 1=全跟新值)

# ----------------------------------------------------------------------
#  5. 速度 & 加速度计算
# ----------------------------------------------------------------------

VELOCITY_EMA_ALPHA = 0.25            # 速度 EMA 平滑 (值越小越平滑, 噪声越低延迟越大)
ACCEL_EMA_ALPHA    = 0.15            # 加速度 EMA 平滑 (比速度更平滑, 加速度噪声更大)
VELOCITY_MIN_DT_MS = 5               # 最小时间间隔 (ms), dt太小会导致速度计算爆炸
VELOCITY_RESET_GAP_MS = 500          # 位置丢失超过此时间 → 速度/加速度清零

# ----------------------------------------------------------------------
#  6. UART 状态码 (发送给B板, 与 common.py 协议保持一致)
# ----------------------------------------------------------------------

UART_STATE_LOST   = 0                # 丢失: 无有效位置
UART_STATE_VERIFY = 1                # 确认中: 候选累积中
UART_STATE_TRACK  = 2                # 跟踪中: 位置有效且稳定
UART_STATE_HOLD   = 3                # 保持中: 漏检暂用上帧位置

# ======================================================================
#  通道A: 背景差分
# ======================================================================

# ----------------------------------------------------------------------
#  6. 背景采集
# ----------------------------------------------------------------------

BG_CAPTURE_AFTER_FRAMES = 60         # 启动后等多少帧再采集空轨道背景

# ----------------------------------------------------------------------
#  7. 差分 & 二值化
# ----------------------------------------------------------------------

BG_DIFF_THRESHOLD       = 50         # 差分图二值化阈值: 差值>此值→钢珠前景(255)

# ----------------------------------------------------------------------
#  8. blob 尺寸筛选 (钢珠在ROI中约 20~30px, 具体取决于ROI宽度)
# ----------------------------------------------------------------------

BG_BLOB_MIN_WIDTH       = 12         # blob 最小宽度 (px), 过滤噪点碎片
BG_BLOB_MAX_WIDTH       = 55         # blob 最大宽度 (px), 过滤大面积异物
BG_BLOB_MIN_HEIGHT      = 10         # blob 最小高度 (px)
BG_BLOB_MAX_HEIGHT      = 55         # blob 最大高度 (px)
BG_BLOB_MIN_ASPECT      = 0.35       # blob 最小宽高比 (过滤细长边缘噪声)
BG_BLOB_MIN_DENSITY     = 0.12       # blob 最小填充密度 (过滤稀疏反光斑)

# ----------------------------------------------------------------------
#  9. blob 查找参数
# ----------------------------------------------------------------------

BG_FINDBLOBS_PIXELS_THRESHOLD = 35   # 最少白色像素数 (过滤孤立噪点)
BG_FINDBLOBS_AREA_THRESHOLD   = 70   # 最少外接矩形面积 (px²)
BG_FINDBLOBS_MERGE_MARGIN     = 3    # 相邻blob合并距离 (px), 弥合碎片

# ----------------------------------------------------------------------
#  10. blob 评分权重 & 位置约束 (调这些值可改变候选blob的排序逻辑)
# ----------------------------------------------------------------------

BG_BLOB_SCORE_ASPECT_WEIGHT  = 80.0  # 宽高比权重 (越接近1分越高)
BG_BLOB_SCORE_DENSITY_WEIGHT = 30.0  # 密度权重 (越密实分越高, 上限0.80)
BG_BLOB_SCORE_AREA_WEIGHT    = 0.02  # 面积权重 (越大分越高)
BG_BLOB_MAX_JUMP_PX          = 80    # blob中心距参考位置>此值→直接丢弃
BG_BLOB_JUMP_PENALTY         = 0.25  # 位置偏离惩罚系数 (px→分数)

# ----------------------------------------------------------------------
#  11. 加权质心精修
# ----------------------------------------------------------------------

BG_CENTROID_SAMPLE_STEP = 2          # 遍历blob区域时的采样步长 (px)

# ======================================================================
#  通道B: 霍夫圆
# ======================================================================

# ----------------------------------------------------------------------
#  12. cv_lite 霍夫圆检测参数
# ----------------------------------------------------------------------

HOUGH_DP              = 1            # 累加器分辨率反比 (1=原图, 2=半分辨率)
HOUGH_MIN_DISTANCE    = 24           # 两圆心最小间距 (px), 避免同一球检出多圆
HOUGH_CANNY_HIGH      = 90           # Canny边缘检测高阈值 (越低检出越多圆)
HOUGH_ACCUMULATOR     = 20           # 累加器阈值 (调大→更严格, 调小→更敏感, 18~22为宜)

# ----------------------------------------------------------------------
#  13. 钢珠半径范围 (ROI约620px/25cm=24.8px/cm, 球直径1cm→约12.4px半径)
# ----------------------------------------------------------------------

HOUGH_BALL_MIN_RADIUS = 10           # 钢珠最小半径 (px), 约4mm
HOUGH_BALL_MAX_RADIUS = 22           # 钢珠最大半径 (px), 约9mm

# ----------------------------------------------------------------------
#  14. 轨道中心线 Y 约束 (过滤轨道上方/下方的误检圆)
# ----------------------------------------------------------------------

HOUGH_TRACK_CENTER_Y_LEFT  = 50      # ROI左端的轨道中心Y (px, 在ROI坐标系内)
HOUGH_TRACK_CENTER_Y_RIGHT = 50      # ROI右端的轨道中心Y (轨道水平则相等)
HOUGH_TRACK_Y_TOLERANCE    = 35      # 圆心距轨道中心线允差 (px), 太小漏检, 太大误检多

# ----------------------------------------------------------------------
#  15. 霍夫圆评分公式 (选最佳圆的排序逻辑)
# ----------------------------------------------------------------------

HOUGH_BASE_SCORE             = 100.0 # 基准分
HOUGH_RADIUS_PENALTY         = 3.0   # 半径偏离预期每1px扣分
HOUGH_Y_ERROR_PENALTY        = 1.5   # Y偏离轨道中心每1px扣分
HOUGH_JUMP_PENALTY           = 0.25  # X跳变每1px扣分 (同背景差分)
HOUGH_MAX_JUMP_PX            = 200    # 圆心跳变>此值→直接丢弃 (同背景差分)

# ======================================================================
#  BG约束霍夫圆
# ======================================================================

CROSS_CONSTRAINT_PX = 15             # BG有效时，霍夫圆须在BG位置±此值内才被信任
                                      # 超出则丢弃霍夫圆，只用BG (防止霍夫圆抖动干扰)

# ======================================================================
#  双通道融合 (BG有效时霍夫圆已被约束, 此函数只做简单逻辑)
# ======================================================================

def fuse_results(x_bg, x_hough):
    """融合两个通道结果。
    BG有效时霍夫圆已被交叉约束过, 这里只做:
      - 两通道都有 → 取均值
      - 仅一个有 → 用它
      - 都无 → None
    """
    if x_bg is not None and x_hough is not None:
        return (x_bg + x_hough) / 2.0
    elif x_bg is not None:
        return x_bg
    elif x_hough is not None:
        return x_hough
    else:
        return None

# ======================================================================
#  像素→厘米换算用 (内部计算, 不改)
# ======================================================================

ROI_CENTER_PX = ROI_WIDTH / 2.0      # ROI中心像素坐标 (零点)
PX_PER_CM     = ROI_WIDTH / ROI_LENGTH_CM  # 每厘米对应像素数


# ======================================================================
#  第二部分：BallTracker 状态机
# ======================================================================

class BallTracker:
    """钢珠位置跟踪状态机。

    四状态流转:
      LOST → VERIFY → TRACK → HOLD → LOST

    LOST (丢失):
      无有效位置。首次收到candidate_x → 进入VERIFY开始累积确认。

    VERIFY (确认中):
      检测到新候选，需要连续 TRACK_CONFIRM_FRAMES 帧检测到
      且相邻帧跳动 ≤ TRACK_CONFIRM_MAX_JUMP_PX → 取均值 → TRACK。
      跳动过大或中间漏检 → 重置累积计数。

    TRACK (跟踪中):
      正常跟踪，用 EMA 指数移动平均平滑位置。
      平滑公式: x_new = x_prev * (1-alpha) + candidate * alpha
      大幅跳变(>TRACK_MAX_JUMP_PX) → 丢弃该帧 → HOLD。

    HOLD (保持):
      短暂漏检时保持最后有效位置。连续HOLD超过TRACK_HOLD_FRAMES帧 → LOST。
    """

    def __init__(self):
        self.x = None                 # 当前平滑后的跟踪位置 (ROI px坐标)
        self.pending_x = None         # VERIFY阶段累积的候选位置均值
        self.pending_count = 0        # VERIFY阶段已累积的连续有效帧数
        self.missed_frames = TRACK_HOLD_FRAMES + 1  # 连续漏检帧数
        self.state = "LOST"           # 当前状态字: LOST/VERIFY/TRACK/HOLD

    def update(self, candidate_x):
        """输入原始检测坐标, 输出平滑跟踪坐标。

        Args:
          candidate_x: 检测器返回的球心 x (ROI px), None=漏检

        Returns:
          (tracked_x, state): 平滑后x坐标和当前状态
        """
        if candidate_x is not None:

            if self.x is None:
                # ---------- LOST / VERIFY: 候选累积确认 ----------
                if (
                    self.pending_x is None
                    or abs(candidate_x - self.pending_x)
                    > TRACK_CONFIRM_MAX_JUMP_PX
                ):
                    # 首次候选 或 跳动过大: 重置累积
                    self.pending_x = candidate_x
                    self.pending_count = 1
                else:
                    # 连续一致: 取均值
                    self.pending_x = (
                        self.pending_x + candidate_x
                    ) / 2.0
                    self.pending_count += 1

                if self.pending_count >= TRACK_CONFIRM_FRAMES:
                    # 确认成功 → 转TRACK
                    self.x = self.pending_x
                    self.pending_x = None
                    self.pending_count = 0
                    self.missed_frames = 0
                    self.state = "TRACK"
                else:
                    self.state = "VERIFY"

            else:
                # ---------- TRACK: EMA平滑 + 跳变检查 ----------
                if abs(candidate_x - self.x) <= TRACK_MAX_JUMP_PX:
                    # 正常跟踪: EMA平滑
                    self.x = (
                        self.x * (1.0 - TRACK_EMA_ALPHA)
                        + candidate_x * TRACK_EMA_ALPHA
                    )
                    self.missed_frames = 0
                    self.state = "TRACK"
                else:
                    # 跳变过大(可能是误检): 丢弃该帧, 走HOLD
                    candidate_x = None

        if candidate_x is None:
            # ---------- 漏检 / 被丢弃 ----------
            self.pending_x = None
            self.pending_count = 0

            if self.x is not None and self.missed_frames < TRACK_HOLD_FRAMES:
                # 短暂漏检: 保持上帧位置
                self.missed_frames += 1
                self.state = "HOLD"
            else:
                # 持续漏检: 丢失
                self.x = None
                self.missed_frames = TRACK_HOLD_FRAMES + 1
                self.state = "LOST"

        return self.x, self.state


# ======================================================================
#  第三部分：通道A - 背景差分检测器
# ======================================================================

class BackgroundDifferenceDetector:
    """空轨道背景差分检测器。

    算法步骤:
      1. 启动后前N帧等待曝光稳定 → 第N帧采集空轨道背景
      2. 每帧: 当前ROI - 背景 → abs → 高斯去噪 → 缓存灰度差分图
      3. 灰度差分图二值化(阈值BG_DIFF_THRESHOLD) → 闭运算弥合碎片
      4. find_blobs 找所有白色连通域
      5. 按尺寸/宽高比/密度/位置连续性筛选+评分 → 最佳blob
      6. 在blob区域内对灰度差分图做加权质心(替代几何中心)
         → 球心区域差分值最大=权重最高 → 质心自动锁定球心
         → 光线变化时blob边界偏移，但质心几乎不动

    属性:
      background       : 空轨道背景灰度图(仅采集一次)
      diff_image       : 灰度差分图缓存(二值化前, 存差分原始值)
      work_image       : 二值化工作图(用于blob查找)
      frame_count      : 帧计数器
      background_ready : 背景是否已采集
    """

    def __init__(self):
        self.roi_width        = 0
        self.roi_height       = 0
        self.frame_count      = 0
        self.background_ready = False
        self.background       = None  # image.Image: 空轨道背景
        self.diff_image       = None  # image.Image: 灰度差分缓存
        self.work_image       = None  # image.Image: 二值化工作图

    def setup(self, roi_width, roi_height):
        """分配3张图像缓冲区 (K230D内存有限, 仅分配必要的)。"""
        self.roi_width  = roi_width
        self.roi_height = roi_height
        self.background = image.Image(
            roi_width, roi_height, image.GRAYSCALE,
        )
        self.diff_image = image.Image(
            roi_width, roi_height, image.GRAYSCALE,
        )
        self.work_image = image.Image(
            roi_width, roi_height, image.GRAYSCALE,
        )
        print(
            "BG: 初始化完成, 请在%d帧内移走钢珠"
            % BG_CAPTURE_AFTER_FRAMES
        )

    # ----------------------------------------------------------------
    #  加权质心计算
    # ----------------------------------------------------------------
    def _weighted_centroid(self, blob):
        """在blob外接矩形内, 用灰度差分值计算像素值加权的x中心。

        原理: 球心与背景差异最大 → 差分值最高 → 质心权重最高。
        相比blob几何中心(受二值化阈值漂移影响), 加权质心几乎不随
        光照变化而偏移, 精度可达到亚像素级别。

        Args:
          blob: find_blobs返回的blob对象 (提供包围盒坐标)

        Returns:
          加权质心的x坐标 (float, ROI px)
        """
        x0 = max(0, blob.x())
        y0 = max(0, blob.y())
        x1 = min(x0 + blob.w(), self.roi_width)
        y1 = min(y0 + blob.h(), self.roi_height)

        sum_wx = 0.0   # Σ(x * 灰度差分值)
        sum_w  = 0.0   # Σ(灰度差分值)

        # 步长BG_CENTROID_SAMPLE_STEP跳过非关键像素以提速
        for y in range(y0, y1, BG_CENTROID_SAMPLE_STEP):
            for x in range(x0, x1, BG_CENTROID_SAMPLE_STEP):
                v = self.diff_image.get_pixel(x, y)
                if v > 0:                 # 仅计入有差分的像素
                    sum_wx += x * v
                    sum_w  += v

        if sum_w == 0:
            # 兜底: 差分图区域内全为0 → 回退到几何中心
            return blob.x() + blob.w() / 2.0

        return sum_wx / sum_w

    # ----------------------------------------------------------------
    #  blob 筛选 + 评分
    # ----------------------------------------------------------------
    def _find_best_blob(self, reference_x):
        """在二值化图(black&white)中找所有白色(255)连通域,
        按钢珠特征筛选并评分, 返回最佳候选。

        筛选顺序(性能考虑, 先证伪再评分):
          尺寸范围 → 宽高比 → 密度 → 位置连续性
        通过全部筛选后才计算综合评分。

        评分公式:
          score = ar*W_ASPECT + min(density,0.80)*W_DENSITY + pixels*W_AREA - pos_penalty

        Args:
          reference_x: 上一帧跟踪位置 (BallTracker.x, None=首次)

        Returns:
          最佳blob对象, 无匹配返回None
        """
        # 第一步: 查找所有白色连通域
        blobs = self.work_image.find_blobs(
            [(255, 255)],                      # 找白色(255)像素
            x_stride=1, y_stride=1,            # 逐像素扫描
            pixels_threshold=BG_FINDBLOBS_PIXELS_THRESHOLD,
            area_threshold=BG_FINDBLOBS_AREA_THRESHOLD,
            merge=True,                         # 合并相邻blob
            margin=BG_FINDBLOBS_MERGE_MARGIN,   # 合并距离
        )

        best_blob  = None
        best_score = -1.0

        for blob in blobs:
            w = blob.w()
            h = blob.h()

            # 尺寸筛选: 太大=异物/反光, 太小=噪点
            if w < BG_BLOB_MIN_WIDTH or w > BG_BLOB_MAX_WIDTH:
                continue
            if h < BG_BLOB_MIN_HEIGHT or h > BG_BLOB_MAX_HEIGHT:
                continue

            # 宽高比筛选: 钢珠接近1:1, 轨道边缘差分噪声是细长条
            aspect_ratio = min(w, h) / max(w, h)
            if aspect_ratio < BG_BLOB_MIN_ASPECT:
                continue

            # 密度筛选: 钢珠实心→高密度, 散斑噪声→低密度
            density = blob.pixels() / (w * h)
            if density < BG_BLOB_MIN_DENSITY:
                continue

            # 位置连续性检查
            center_x = blob.x() + w / 2.0
            position_penalty = 0.0

            if reference_x is not None:
                jump_x = abs(center_x - reference_x)
                if jump_x > BG_BLOB_MAX_JUMP_PX:
                    continue                          # 跳变太大→丢弃
                position_penalty = jump_x * BG_BLOB_JUMP_PENALTY

            # 综合评分 (值域: aspect~0.35~1.0, density~0.12~0.80, area~几百)
            score = (
                aspect_ratio * BG_BLOB_SCORE_ASPECT_WEIGHT
                + min(density, 0.80) * BG_BLOB_SCORE_DENSITY_WEIGHT
                + blob.pixels() * BG_BLOB_SCORE_AREA_WEIGHT
                - position_penalty
            )

            if score > best_score:
                best_score = score
                best_blob  = blob

        return best_blob

    # ----------------------------------------------------------------
    #  主处理入口 (每帧调用)
    # ----------------------------------------------------------------
    def process(self, raw_roi, reference_x):
        """对一帧 ROI 做背景差分检测。

        Args:
          raw_roi:     当前帧的 ROI 区域灰度图 (image.Image)
          reference_x: 上一帧跟踪位置 (BallTracker.x, None=首次)

        Returns:
          (candidate_x, blob_rect, work_image)
            candidate_x: 加权质心 x (float), None=无检测
            blob_rect:   (x,y,w,h), None=无检测
            work_image:  二值化图 (供IDE显示叠加用)
        """
        self.frame_count += 1

        # ---- 背景采集阶段: 前N帧等待, 第N帧采样 ----
        if not self.background_ready:
            if self.frame_count >= BG_CAPTURE_AFTER_FRAMES:
                # 在第N帧将当前画面复制为背景(此时轨道内应无球)
                self.background.draw_image(raw_roi, 0, 0)
                self.background_ready = True
                print("BG: 空轨道背景采集完成")
            # 背景就绪前不输出检测结果
            return None, None, None

        # ---- 检测阶段 ----

        # 1. 拷贝当前ROI → 与背景做绝对值差分 → 高斯模糊去噪
        self.work_image.draw_image(raw_roi, 0, 0)
        self.work_image.difference(self.background)
        self.work_image.gaussian(1)

        # 2. ★关键: 缓存灰度差分图 (二值化前) 供加权质心使用
        self.diff_image.draw_image(self.work_image, 0, 0)

        # 3. 二值化(阈值BG_DIFF_THRESHOLD) → 闭运算(弥补内部空洞)
        self.work_image.binary([(BG_DIFF_THRESHOLD, 255)])
        self.work_image.close(1)

        # 4. 二值化图上查找+筛选最佳blob
        blob = self._find_best_blob(reference_x)
        if blob is None:
            return None, None, None

        # 5. 在灰度差分图上计算加权质心 (替代blob几何中心)
        center_x = self._weighted_centroid(blob)
        return center_x, blob.rect(), self.work_image

    def status_text(self):
        """返回检测器状态文本 (显示在IDE控制台)。"""
        if self.background_ready:
            return "BG_OK"
        remaining = BG_CAPTURE_AFTER_FRAMES - self.frame_count
        return "BG:%d" % max(0, remaining)

    def deinit(self):
        """释放图像缓冲区。"""
        self.background = None
        self.diff_image = None
        self.work_image  = None


# ======================================================================
#  第四部分：通道B - 霍夫圆检测器
# ======================================================================

class HoughCircleDetector:
    """cv_lite 霍夫圆检测器。

    原理:
      cv_lite.grayscale_find_circles 对灰度图做霍夫变换,
      检测图像中的圆形轮廓。钢球在轨道凹槽内呈近圆形,
      直接通过几何特征识别。

    筛选逻辑:
      1. 半径在 [BALL_MIN_RADIUS, BALL_MAX_RADIUS] 范围内
      2. 圆心Y坐标须在轨道中心线 ±TRACK_Y_TOLERANCE 内
         (过滤轨道上方/下方的误检)
      3. 圆心X位置与上一帧的跳动 ≤HOUGH_MAX_JUMP_PX
      4. 综合评分: 半径匹配度 + Y偏差惩罚 + 位置连续性惩罚
    """

    def __init__(self):
        self.roi_width       = 0
        self.roi_height      = 0
        self.image_shape     = None   # [height, width] 供cv_lite调用
        self.candidate_count = 0      # 本帧检出候选圆数量
        self.best_radius     = 0      # 选中圆的半径(调试)

    def setup(self, roi_width, roi_height):
        """记录ROI尺寸, 准备image_shape供cv_lite使用。"""
        self.roi_width   = roi_width
        self.roi_height  = roi_height
        self.image_shape = [roi_height, roi_width]
        print(
            "HOUGH: 半径范围=%d~%dpx  Canny高阈值=%d  累加器阈值=%d"
            % (HOUGH_BALL_MIN_RADIUS, HOUGH_BALL_MAX_RADIUS,
               HOUGH_CANNY_HIGH, HOUGH_ACCUMULATOR)
        )

    def _track_center_y(self, x):
        """根据x坐标线性插值轨道中心Y。

        支持左右端不同值以补偿摄像头轻微倾斜。
        x=0  → TRACK_CENTER_Y_LEFT
        x=W-1 → TRACK_CENTER_Y_RIGHT
        """
        if self.roi_width <= 1:
            return HOUGH_TRACK_CENTER_Y_LEFT
        return int(
            HOUGH_TRACK_CENTER_Y_LEFT
            + (HOUGH_TRACK_CENTER_Y_RIGHT - HOUGH_TRACK_CENTER_Y_LEFT)
            * x / (self.roi_width - 1)
        )

    def _normalize_circles(self, raw_results):
        """统一cv_lite返回的圆结果为 [(x,y,r), ...] 列表。

        cv_lite在不同CanMV固件版本中返回格式不同:
          版本A: 嵌套列表 [(x1,y1,r1), (x2,y2,r2), ...]
          版本B: 一维数组 [x1,y1,r1, x2,y2,r2, ...]
        此函数自动识别格式并统一转换。
        """
        circles = []
        if raw_results is None or len(raw_results) == 0:
            return circles

        first = raw_results[0]

        if isinstance(first, (list, tuple)):
            # 版本A: 嵌套列表
            for item in raw_results:
                if len(item) >= 3:
                    circles.append((
                        int(item[0]), int(item[1]), int(item[2]),
                    ))
        else:
            # 版本B: 一维扁平数组, 每3个元素一组
            idx = 0
            while idx + 2 < len(raw_results):
                circles.append((
                    int(raw_results[idx]),
                    int(raw_results[idx + 1]),
                    int(raw_results[idx + 2]),
                ))
                idx += 3

        return circles

    def process(self, raw_roi, reference_x):
        """对一帧ROI做霍夫圆检测并筛选最佳圆。

        Args:
          raw_roi:     当前帧ROI灰度图
          reference_x: 上一帧跟踪位置

        Returns:
          (candidate_x, circle_data, None)
            candidate_x:  圆心x坐标 (float), None=无检测
            circle_data:  (cx, cy, r) 用于IDE绘制圆, None=无检测
        """
        # 调用cv_lite霍夫圆检测 (C代码, 相对耗时)
        raw_results = cv_lite.grayscale_find_circles(
            self.image_shape,          # [height, width]
            raw_roi.to_numpy_ref(),    # numpy数组引用
            HOUGH_DP,                  # 分辨率反比
            HOUGH_MIN_DISTANCE,        # 圆心最小间距
            HOUGH_CANNY_HIGH,          # Canny高阈值
            HOUGH_ACCUMULATOR,         # 累加器阈值
            HOUGH_BALL_MIN_RADIUS,     # 最小半径
            HOUGH_BALL_MAX_RADIUS,     # 最大半径
        )

        circles = self._normalize_circles(raw_results)
        self.candidate_count = len(circles)

        # 筛选最佳圆
        best       = None     # (cx, cy, r)
        best_score = -100000.0
        expected_r = (HOUGH_BALL_MIN_RADIUS + HOUGH_BALL_MAX_RADIUS) / 2.0

        for cx, cy, r in circles:
            # 半径范围 (理论上cv_lite已过滤, 此处二次确认)
            if r < HOUGH_BALL_MIN_RADIUS or r > HOUGH_BALL_MAX_RADIUS:
                continue

            # Y坐标须在轨道中心线附近
            track_y = self._track_center_y(cx)
            y_err   = abs(cy - track_y)
            if y_err > HOUGH_TRACK_Y_TOLERANCE:
                continue

            # 位置连续性
            pos_penalty = 0.0
            if reference_x is not None:
                jump = abs(cx - reference_x)
                if jump > HOUGH_MAX_JUMP_PX:
                    continue
                pos_penalty = jump * HOUGH_JUMP_PENALTY

            # 综合评分: 基准分 - 半径偏离 - Y偏离 - 位置跳变
            score = (
                HOUGH_BASE_SCORE
                - abs(r - expected_r) * HOUGH_RADIUS_PENALTY
                - y_err * HOUGH_Y_ERROR_PENALTY
                - pos_penalty
            )

            if score > best_score:
                best_score = score
                best       = (cx, cy, r)

        if best is None:
            self.best_radius = 0
            return None, None, None

        cx, cy, r = best
        self.best_radius = r
        return float(cx), (cx, cy, r), None

    def status_text(self):
        """返回检测器状态文本。C=候选圆数量, R=选中半径。"""
        return "C:%d R:%d" % (self.candidate_count, self.best_radius)

    def deinit(self):
        self.image_shape = None


# ======================================================================
#  第五部分：辅助函数
# ======================================================================

def ball_px_to_cm(center_x):
    """ROI像素坐标 → 物理厘米换算。

    以ROI中心为零点, 向右为正, 向左为负。

    Args:
      center_x: 球心在ROI中的x像素坐标

    Returns:
      距轨道中心距离 (cm), None=无效
    """
    if center_x is None:
        return None
    return (center_x - ROI_CENTER_PX) / PX_PER_CM


def uart_send(uart, frame_id, distance_cm, track_state):
    """通过UART向控制板发送钢珠相对轨道中心O的位置数据包。

    协议格式 (ASCII文本):
      B,<帧ID>,<位置mm>,<状态码>\n

    字段:
      - 帧ID:      自增帧号, 控制板可用于丢包检测
      - 位置mm:    相对轨道中心O，带符号整数，正=右/负=左；无数据时发0
      - 状态码:    0=丢失 1=确认中 2=跟踪中 3=保持中

    示例:
      B,523,12,2\n   → 帧523, 球在中心O右侧12mm, TRACK
      B,524,0,0\n    → 帧524, 球丢失
    """
    # 状态字 → 数值码
    if track_state == "VERIFY":
        sc = UART_STATE_VERIFY
    elif track_state == "TRACK":
        sc = UART_STATE_TRACK
    elif track_state == "HOLD":
        sc = UART_STATE_HOLD
    else:
        sc = UART_STATE_LOST

    # 厘米→毫米 (四舍五入)
    distance_mm = (
        0 if distance_cm is None
        else int(round(distance_cm * 10.0))
    )

    packet = "B,%d,%d,%d\n" % (frame_id, distance_mm, sc)
    if uart is not None:
        uart.write(packet.encode())
    if USB_BRIDGE_MIRROR_ENABLE:
        print(packet, end="")


def uart_init():
    """初始化K230D的UART2, 连接B板RM A型板。

    引脚映射:
      K230D Pin 11 (TX) → B板 RX
      K230D Pin 12 (RX) → B板 TX (备用)

    配置: 38400bps, 8数据位, 无校验, 1停止位
    """
    if not UART_ENABLE:
        return None

    try:
        fpioa = FPIOA()
        fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
        fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
        return UART(
            UART.UART2,
            baudrate=UART_BAUDRATE,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )
    except BaseException as err:
        print("UART 初始化失败 (调试模式可忽略):", err)
        return None


def roi_validate():
    """验证ROI参数是否在640×480范围内, 非法则抛异常。"""
    if (
        ROI_X < 0 or ROI_Y < 0
        or ROI_WIDTH <= 0 or ROI_HEIGHT <= 0
        or ROI_X + ROI_WIDTH > CAMERA_WIDTH
        or ROI_Y + ROI_HEIGHT > CAMERA_HEIGHT
    ):
        raise ValueError(
            "ROI 超出画面范围! 当前ROI=(%d,%d,%d,%d) 画面=(%d,%d)"
            % (ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT,
               CAMERA_WIDTH, CAMERA_HEIGHT)
        )


# ======================================================================
#  第六部分：主循环
# ======================================================================

def main():
    """运行双通道融合检测主循环。

    每帧流水线:
      1. sensor.snapshot()             采集640×480灰度图
      2. raw_roi.draw_image(frame, …)  裁剪ROI区域
      3. detector_bg.process()         通道A: 背景差分 → 加权质心
      4. detector_hough.process()      通道B: 霍夫圆检测 → 圆心
      5. fuse_results()                融合两通道结果
      6. tracker.update()             BallTracker平滑跟踪
      7. ball_px_to_cm()              像素→厘米
      8. uart_send()                  UART发给B板
      9. 每N帧IDE显示: 灰度ROI + 霍夫圆(亮灰) + BG框(深灰)
    """

    sensor        = None
    uart          = None
    display_ok    = False
    media_ok      = False

    try:
        roi_validate()

        # ============================================================
        #  摄像头初始化
        # ============================================================
        sensor = Sensor(
            width=CAMERA_WIDTH,
            height=CAMERA_HEIGHT,
            fps=CAMERA_TARGET_FPS,
        )
        sensor.reset()
        sensor.set_framesize(
            width=CAMERA_WIDTH, height=CAMERA_HEIGHT,
            chn=CAM_CHN_ID_0,
        )
        # 灰度模式: 钢珠检测无需颜色信息, 灰度节省带宽→更高FPS
        sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)

        # 手动曝光 (EXPOSURE_MANUAL_US=0时为自动曝光)
        if EXPOSURE_MANUAL_US > 0:
            try:
                sensor.auto_exposure(False)
            except:
                pass

        # ============================================================
        #  IDE 远程显示初始化
        # ============================================================
        Display.init(
            Display.VIRT,                    # 虚拟显示→CanMV IDE
            width=CAMERA_WIDTH,
            height=CAMERA_HEIGHT,
            fps=CAMERA_TARGET_FPS,
            to_ide=True,
            quality=IDE_JPEG_QUALITY,
        )
        display_ok = True

        # ============================================================
        #  媒体管理器启动
        # ============================================================
        MediaManager.init()
        media_ok = True
        sensor.run()

        if EXPOSURE_MANUAL_US > 0:
            try:
                sensor.exposure(EXPOSURE_MANUAL_US)
            except:
                pass

        try:
            sensor._set_chn_fps(chn=CAM_CHN_ID_0, fps=CAMERA_TARGET_FPS)
        except:
            pass

        # ============================================================
        #  双通道检测器 + 跟踪器初始化
        # ============================================================
        uart = uart_init()

        # ROI裁剪缓存
        raw_roi = image.Image(ROI_WIDTH, ROI_HEIGHT, image.GRAYSCALE)

        # 通道A: 背景差分
        detector_bg = BackgroundDifferenceDetector()
        detector_bg.setup(ROI_WIDTH, ROI_HEIGHT)

        # 通道B: 霍夫圆
        detector_hough = HoughCircleDetector()
        detector_hough.setup(ROI_WIDTH, ROI_HEIGHT)

        # IDE显示合成缓存: ROI灰度图 + BG框 + 霍夫圆
        display_buf = image.Image(ROI_WIDTH, ROI_HEIGHT, image.GRAYSCALE)

        # 跟踪器
        tracker = BallTracker()

        # 速度/加速度追踪变量
        prev_x_px     = None    # 上一帧位置 (ROI px)
        prev_v_px_s   = None    # 上一帧 EMA 速度 (px/s)
        prev_t_ms     = 0       # 上一帧时间戳 (ms)
        ema_v_px_s    = 0.0     # 当前 EMA 速度 (px/s)
        ema_a_px_s2   = 0.0     # 当前 EMA 加速度 (px/s²)
        lost_t_ms     = 0       # 丢失发生时刻 (ms), 用于超时清零
        vel_label     = "V:--"
        acc_label     = "A:--"
        bg_label      = "BG:--"
        hough_label   = "H:--"

        # ============================================================
        #  帧循环变量
        # ============================================================
        fps_counter   = 0              # 500ms内采集帧数
        frame_id      = 0              # 全局帧号(自增, 不发复位)
        display_tick  = 0              # 显示间隔计数器
        fps_period_ms = time.ticks_ms()# FPS统计周期起点
        log_period_ms = fps_period_ms  # 日志打印周期起点
        fps_label     = "FPS: 0.0"

        print("BEST v2.0 就绪  ROI=%s  %d×%dpx"
              % (str((ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT)),
                 ROI_WIDTH, ROI_HEIGHT))
        print("  通道A: 背景差分(加权质心)  通道B: 霍夫圆")
        print("  BG约束霍夫圆阈值: ±%dpx" % CROSS_CONSTRAINT_PX)

        # ============================================================
        #  主循环
        # ============================================================
        while True:
            # ---- 1. 采集一帧 ----
            frame = sensor.snapshot(chn=CAM_CHN_ID_0)
            fps_counter += 1
            frame_id    += 1
            display_tick += 1

            # ---- 2. 裁剪ROI ----
            raw_roi.draw_image(frame, -ROI_X, -ROI_Y)

            # ---- 3. 通道A: 背景差分 ----
            #   背景未就绪时返回None, 就绪后返回加权质心
            x_bg, bg_rect, _ = detector_bg.process(raw_roi, tracker.x)

            # ---- 4. 通道B: 霍夫圆 ----
            #   始终工作, 不依赖背景
            x_hough, hough_data, _ = detector_hough.process(
                raw_roi, tracker.x,
            )

            # ---- 4.5 BG约束霍夫圆 ----
            #   BG有效时: 霍夫圆必须在BG位置±CROSS_CONSTRAINT_PX内才信任
            #   超出则丢弃霍夫圆, 只留BG (防止霍夫抖动污染结果)
            if x_bg is not None and x_hough is not None:
                if abs(x_bg - x_hough) > CROSS_CONSTRAINT_PX:
                    x_hough = None
                    hough_data = None

            # 各通道独立物理坐标 (用于显示对比)
            bg_cm    = ball_px_to_cm(x_bg)
            hough_cm = ball_px_to_cm(x_hough)
            bg_label = "BG:--" if bg_cm is None else "BG:%+.2f" % bg_cm
            hough_label = "H:--" if hough_cm is None else "H:%+.2f" % hough_cm

            # ---- 5. 融合 ----
            candidate_x = fuse_results(x_bg, x_hough)

            # ---- 6. BallTracker 平滑跟踪 ----
            tracked_x, track_state = tracker.update(candidate_x)

            # ---- 7. 像素→厘米 ----
            distance_cm = ball_px_to_cm(tracked_x)
            dist_label  = (
                "DIST_X_CM: --"
                if distance_cm is None
                else "DIST_X_CM: %+.2f" % distance_cm
            )
            # ---- 7.5 速度 & 加速度计算 ----
            now_ms = time.ticks_ms()

            if tracked_x is not None and prev_x_px is not None:
                dt_s = time.ticks_diff(now_ms, prev_t_ms) / 1000.0
                if dt_s >= VELOCITY_MIN_DT_MS / 1000.0:
                    # 速度 = Δpos/Δt  (px/s)
                    v_raw = (tracked_x - prev_x_px) / dt_s

                    if prev_v_px_s is None:
                        # 首次有速度数据: 直接赋值
                        ema_v_px_s = v_raw
                    else:
                        # EMA 平滑速度
                        ema_v_px_s = (
                            ema_v_px_s * (1.0 - VELOCITY_EMA_ALPHA)
                            + v_raw * VELOCITY_EMA_ALPHA
                        )

                    if prev_v_px_s is not None:
                        # 加速度 = Δv/Δt  (px/s²)
                        a_raw = (ema_v_px_s - prev_v_px_s) / dt_s
                        ema_a_px_s2 = (
                            ema_a_px_s2 * (1.0 - ACCEL_EMA_ALPHA)
                            + a_raw * ACCEL_EMA_ALPHA
                        )

                    prev_v_px_s = ema_v_px_s

                prev_x_px = tracked_x
                prev_t_ms = now_ms

            elif tracked_x is not None and prev_x_px is None:
                # 首次获得位置
                prev_x_px = tracked_x
                prev_t_ms = now_ms
                prev_v_px_s = None
                ema_v_px_s  = 0.0
                ema_a_px_s2 = 0.0

            else:
                # 位置丢失: 检查是否超时清零
                if (prev_v_px_s is not None
                        and time.ticks_diff(now_ms, prev_t_ms) > VELOCITY_RESET_GAP_MS):
                    prev_x_px   = None
                    prev_v_px_s = None
                    ema_v_px_s  = 0.0
                    ema_a_px_s2 = 0.0
                    vel_label   = "V:--"
                    acc_label   = "A:--"

            # 速度/加速度换算到物理单位
            if prev_v_px_s is not None and track_state == "TRACK":
                velocity_cm_s  = ema_v_px_s / PX_PER_CM
                accel_cm_s2    = ema_a_px_s2 / PX_PER_CM
                vel_label      = "V:%+.1f" % velocity_cm_s
                acc_label      = "A:%+.1f" % accel_cm_s2
            else:
                velocity_cm_s  = 0.0
                accel_cm_s2    = 0.0

            # ---- 8. UART发送 ----
            try:
                uart_send(uart, frame_id, distance_cm, track_state)
            except BaseException as err:
                print("UART发送失败:", err)

            # ---- 9. FPS统计 (每500ms) ----
            now_ms     = time.ticks_ms()
            elapsed_ms = time.ticks_diff(now_ms, fps_period_ms)
            if elapsed_ms >= 500:
                fps_label   = "FPS: %.1f" % (
                    fps_counter * 1000.0 / elapsed_ms
                )
                fps_counter = 0
                fps_period_ms = now_ms

            # ---- 10. 控制台日志 (每LOG_INTERVAL_MS毫秒) ----
            if time.ticks_diff(now_ms, log_period_ms) >= LOG_INTERVAL_MS:
                print("%s  %s  %s  %s  %s  STATE:%s  %s  %s"
                      % (fps_label, dist_label, bg_label, hough_label,
                         vel_label, track_state,
                         detector_bg.status_text(),
                         detector_hough.status_text()))
                log_period_ms = now_ms

            # ---- 11. IDE显示 (每DISPLAY_EVERY_N_FRAMES帧) ----
            if display_tick >= DISPLAY_EVERY_N_FRAMES:
                display_tick = 0

                # 基图 = 当前ROI灰度图
                display_buf.draw_image(raw_roi, 0, 0)

                # 画通道B(霍夫圆): 外圈圆 + 圆心十字
                if hough_data is not None:
                    hx, hy, hr = hough_data
                    display_buf.draw_circle(
                        hx, hy, hr,
                        color=DISPLAY_COLOR_HOUGH_CIRCLE,
                        thickness=2,
                    )
                    display_buf.draw_cross(
                        hx, hy,
                        color=DISPLAY_COLOR_HOUGH_CIRCLE,
                        size=5, thickness=2,
                    )

                # 画通道A(背景差分): 矩形框 + 中心十字
                if bg_rect is not None and x_bg is not None:
                    display_buf.draw_rectangle(
                        bg_rect,
                        color=DISPLAY_COLOR_BG_RECT,
                        thickness=2,
                    )
                    bg_cross_y = bg_rect[1] + bg_rect[3] // 2
                    display_buf.draw_cross(
                        int(x_bg), bg_cross_y,
                        color=DISPLAY_COLOR_BG_RECT,
                        size=5, thickness=2,
                    )

                # 叠加合成图到原始帧
                frame.draw_image(display_buf, ROI_X, ROI_Y)

                # ROI白色边框
                frame.draw_rectangle(
                    ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT,
                    color=DISPLAY_COLOR_ROI_BORDER,
                    thickness=2,
                )

                # 顶部信息栏 (黑底白字)
                frame.draw_rectangle(
                    5, 5, 625, 42,
                    color=DISPLAY_COLOR_INFO_BG,
                    fill=True,
                )
                frame.draw_string_advanced(
                    12, 10, 18,
                    "%s  %s|%s  %s  %s  BEST"
                    % (fps_label, bg_label, hough_label, vel_label, track_state),
                    color=DISPLAY_COLOR_INFO_TEXT,
                )

                Display.show_image(frame)

            # 释放帧对象 (K230D内存小, 及时回收)
            del frame

    except KeyboardInterrupt:
        print("\n用户中断")
    except BaseException as err:
        import sys
        sys.print_exception(err)
    finally:
        # ---- 清理资源 (按逆序) ----
        try:
            detector_bg.deinit()
        except:
            pass
        try:
            detector_hough.deinit()
        except:
            pass

        if uart is not None:
            try:
                uart.deinit()
            except:
                pass
        if sensor is not None:
            try:
                sensor.stop()
            except:
                pass
        if display_ok:
            try:
                Display.deinit()
            except:
                pass

        time.sleep_ms(100)

        if media_ok:
            try:
                MediaManager.deinit()
            except:
                pass


main()
