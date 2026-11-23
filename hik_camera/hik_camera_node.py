#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Hikrobot (HIKROBOT) 工业相机 ROS 2 节点（基于 MVS SDK Python 封装）。

发布:
  /left_camera/image        sensor_msgs/Image        (bgr8)
  /left_camera/camera_info  sensor_msgs/CameraInfo

用法:
  ros2 run hik_camera hik_camera_node --ros-args -p exposure_us:=10000.0 -p fps:=15.0
"""

import os
import sys
import threading
import time
import json
from array import array

# MVS SDK 运行环境（MvCameraControl_class.py 依赖这两个环境/路径）
os.environ.setdefault("MVCAM_COMMON_RUNENV", "/opt/MVS/lib")
_MVS_PY = "/opt/MVS/Samples/64/Python/MvImport"
if _MVS_PY not in sys.path:
    sys.path.insert(0, _MVS_PY)

from ctypes import byref, cast, c_ubyte, memset, POINTER, sizeof  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.time import Time  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402

import CameraParams_header as camParams  # noqa: E402
import MvCameraControl_class as MvCamera  # noqa: E402
import PixelType_header as PixelType  # noqa: E402


class HikCameraNode(Node):
    def __init__(self):
        super().__init__("hik_camera")

        self.declare_parameter("serial", "")            # 空 = 选第一台
        self.declare_parameter("exposure_us", 10000.0)  # 曝光，微秒
        self.declare_parameter("gain_db", 0.0)          # 增益 dB，<=0 不设置
        self.declare_parameter("fps", 15.0)             # 目标帧率，<=0 不设置
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("image_topic", "/left_camera/image")
        self.declare_parameter("camera_info_topic", "/left_camera/camera_info")
        self.declare_parameter("qos", "sensor_data")    # sensor_data | reliable
        self.declare_parameter("width", 0)              # 0 = 相机原始宽度
        self.declare_parameter("height", 0)             # 0 = 相机原始高度
        # 相机内参（标定后填这里）
        self.declare_parameter("fx", 0.0)
        self.declare_parameter("fy", 0.0)
        self.declare_parameter("cx", 0.0)
        self.declare_parameter("cy", 0.0)
        self.declare_parameter("distortion", [0.0, 0.0, 0.0, 0.0, 0.0])
        # 触发设置（硬件同步用）
        self.declare_parameter("trigger_mode", 0)              # 0=连续采集(自由运行) 1=外部触发
        self.declare_parameter("trigger_source", "Line0")      # 外部触发信号源: Line0/Line1
        self.declare_parameter("trigger_polarity", "RisingEdge")  # 触发极性: RisingEdge/FallingEdge
        self.declare_parameter("time_offset", 0.0)  # 手动 trim(秒):自动对齐之后仍可选加的残差修正
        # ---- 时间基准自动对齐 ----
        # 相机只收到同步器的触发脉冲、没有时间码，绝对时间只能靠驱动自己造。
        # 因此"墙钟 - 雷达时钟"的差每次开机都不同，必须开机自测，不能写死。
        self.declare_parameter("time_offset_auto", True)        # True=开机自动测量基准差(推荐)
        self.declare_parameter("sync_ref_topic", "/livox/imu")  # 参考时钟源(与雷达同一硬件基准)
        self.declare_parameter("sync_ref_type", "auto")         # auto|imu|custom|pointcloud2
        self.declare_parameter("sync_ref_samples", 30)          # 取多少帧参考数据求中位数
        self.declare_parameter("sync_ref_timeout", 20.0)        # 等参考话题的秒数，超时退化为手动 time_offset
        self.declare_parameter("sync_warn_s", 0.2)              # 运行中相机-参考时钟偏差超此值告警(秒)

        gp = self.get_parameter
        self.serial = gp("serial").value
        self.exposure_us = float(gp("exposure_us").value)
        self.gain_db = float(gp("gain_db").value)
        self.fps = float(gp("fps").value)
        self.frame_id = gp("frame_id").value
        self.fx = float(gp("fx").value)
        self.fy = float(gp("fy").value)
        self.cx = float(gp("cx").value)
        self.cy = float(gp("cy").value)
        self.distortion = list(gp("distortion").value)
        self.width = int(gp("width").value)
        self.height = int(gp("height").value)
        self.trigger_mode = int(gp("trigger_mode").value)
        self.trigger_source = gp("trigger_source").value
        self.trigger_polarity = gp("trigger_polarity").value
        self.time_offset = float(gp("time_offset").value)
        self.time_offset_auto = bool(gp("time_offset_auto").value)
        self.sync_ref_topic = gp("sync_ref_topic").value
        self.sync_ref_type = gp("sync_ref_type").value
        self.sync_ref_samples = max(5, int(gp("sync_ref_samples").value))
        self.sync_ref_timeout = max(1.0, float(gp("sync_ref_timeout").value))
        self.sync_warn_s = float(gp("sync_warn_s").value)
        self._manual_trim_ns = int(self.time_offset * 1e9)

        qos = qos_profile_sensor_data if gp("qos").value == "sensor_data" else 10
        self.pub_img = self.create_publisher(Image, gp("image_topic").value, qos)
        self.pub_info = self.create_publisher(CameraInfo, gp("camera_info_topic").value, qos)

        self.cam = None
        self.dev_info = None
        self._running = False
        self._thread = None
        self._buf = None
        self._buf_size = 0
        # 硬件时间戳基准(用于把相机设备时间戳换算成 ROS 时间)
        # 注意: 不再持久化到文件。设备 tick 会随相机断电归零，持久化的锚点反而
        # 会让时间戳整体错位；现在每次启动都在首帧重建锚点，绝对时间由下面的
        # 自动基准差 _delta_ns 保证一致。
        self._dev_ts_ref = None
        self._ros_ts_ref = None
        # 自动基准差: 墙钟 - 参考时钟(雷达/IMU)。未就绪前不发布图像。
        self._delta_ns = None
        self._sync_ready = threading.Event()
        self._sync_samples = []
        self._sync_last_stamp = None   # 参考传感器最近一帧时间(秒)
        self._sync_t0 = time.monotonic()
        self._last_warn_t = 0.0
        if not self.time_offset_auto:
            self._delta_ns = 0
            self._sync_ready.set()
            self.get_logger().warn("time_offset_auto=false：使用手动 time_offset，"
                                   "相机与雷达的基准差需自己保证")
        else:
            self._init_sync_probe()

        self._open_camera()

        self._running = True
        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("hik_camera 已启动")

    # ------------------------------------------------------------------ #
    @staticmethod
    def _decode(arr):
        return bytes(arr).split(b"\x00")[0].decode(errors="ignore")

    # ------------------------------------------------------------------ #
    def _init_sync_probe(self):
        """订阅雷达/IMU 话题，测量"墙钟 - 该时钟基准"的固定差。

        背景：同步器只给相机发触发脉冲，不给时间码；相机的绝对时间只能由驱动
        用"首帧的系统墙钟 + 设备 tick"构造。所以相机时间戳与雷达时间差多少，
        完全取决于同步器这次上电后累计了多久——这个差每次开机都变，必须实测。
        """
        typ = self.sync_ref_type
        if typ == "auto":
            typ = "imu" if "imu" in self.sync_ref_topic.lower() else "custom"
        msg_cls = None
        try:
            if typ == "imu":
                from sensor_msgs.msg import Imu
                msg_cls = Imu
            elif typ == "custom":
                from livox_ros_driver2.msg import CustomMsg
                msg_cls = CustomMsg
            elif typ == "pointcloud2":
                from sensor_msgs.msg import PointCloud2
                msg_cls = PointCloud2
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"无法加载参考消息类型 {typ}: {e}")
        if msg_cls is None:
            self.get_logger().error(
                f"时间基准自校准不可用（参考类型 {typ}），退化为手动 time_offset"
            )
            self._delta_ns = 0
            self._sync_ready.set()
            return
        self.create_subscription(msg_cls, self.sync_ref_topic, self._on_sync_ref,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f"时间基准自校准：等待 {self.sync_ref_topic} 的 {self.sync_ref_samples} 帧…"
        )

    def _on_sync_ref(self, msg):
        """记录 (接收时刻墙钟 - 消息时间戳)，取中位数作为基准差。"""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            return
        self._sync_last_stamp = stamp
        if self._delta_ns is not None:
            return
        wall_ns = self.get_clock().now().nanoseconds
        self._sync_samples.append(wall_ns - int(stamp * 1e9))
        if len(self._sync_samples) >= self.sync_ref_samples:
            vals = sorted(self._sync_samples)
            med = vals[len(vals) // 2]
            spread_ms = (vals[-1] - vals[0]) * 1e-6
            self._delta_ns = med
            self._sync_ready.set()
            self.get_logger().info(
                f"时间基准已校定: 墙钟-雷达时钟 = {med * 1e-9:.6f}s "
                f"(抖动 ±{spread_ms / 2:.3f}ms, n={len(vals)})，相机时间戳已对齐雷达基准"
            )
            if spread_ms > 100.0:
                self.get_logger().warn(
                    f"基准差抖动 {spread_ms:.1f}ms 偏大，检查 {self.sync_ref_topic} 是否稳定"
                )

    def _dev_ts_to_ros(self, high, low):
        """把相机设备时间戳(64 位,单位:10纳秒/tick)换算成已对齐雷达基准的 ROS 时间。

        返回 None 表示基准还没就绪，调用方跳过这一帧（宁可不发，也不发错的时间）。
        """
        if not self._sync_ready.is_set():
            # 关键：等不到参考时钟时，绝不退化成"用墙钟当初基准"去发布——
            # 那样会发出一批差几亿秒的时间戳，Fast-LIVO2 静默不建图，很难查。
            # 这里宁可一帧都不发，并把原因持续打在日志里。
            waited = time.monotonic() - self._sync_t0
            if waited > self.sync_ref_timeout and time.monotonic() - self._last_warn_t > 10.0:
                self._last_warn_t = time.monotonic()
                self.get_logger().error(
                    f"已等 {waited:.0f}s 仍没收到 {self.sync_ref_topic}，暂停发布图像帧"
                    f"（不会发出错的时间戳）。检查雷达驱动/同步器是否在跑；"
                    f"若只想单独用相机，启动时加 -p time_offset_auto:=false"
                )
            return None

        dev = (int(high) << 32) | int(low)
        now = self.get_clock().now()
        offset_ns = int(self._delta_ns) + self._manual_trim_ns
        if self._dev_ts_ref is None:
            # 首帧：建立 (设备 tick, 墙钟) 锚点
            self._dev_ts_ref = dev
            self._ros_ts_ref = now
            self.get_logger().info(f"建立设备时间戳锚点 dev_ref={dev}")
            return Time(nanoseconds=now.nanoseconds - offset_ns).to_msg()

        elapsed_ns = (dev - self._dev_ts_ref) * 10   # 1 tick = 10ns(100MHz)
        total_ns = self._ros_ts_ref.nanoseconds + elapsed_ns - offset_ns
        # 防呆：tick 跳变(相机重上电/计数器重置)会让时间戳跳到过去或失真到离谱，
        # 一旦偏离参考时钟超过 10s 就重建锚点，而不是发布一个错的时间戳。
        sanity_limit_ns = 10 * 10**9
        if elapsed_ns < 0 or abs(total_ns - (now.nanoseconds - offset_ns)) > sanity_limit_ns:
            self.get_logger().warn(
                f"设备时间戳异常(elapsed={elapsed_ns * 1e-9:+.3f}s)，重建锚点")
            self._dev_ts_ref = dev
            self._ros_ts_ref = now
            total_ns = now.nanoseconds - offset_ns
        return Time(nanoseconds=total_ns).to_msg()

    def _check_sync(self, stamp_msg):
        """运行期抽查：相机时间戳应与参考时钟相差不到一个帧周期的量级。"""
        if not self.time_offset_auto or self._sync_last_stamp is None:
            return
        cam = stamp_msg.sec + stamp_msg.nanosec * 1e-9
        diff = cam - self._sync_last_stamp
        if abs(diff) > self.sync_warn_s:
            now = time.monotonic()
            if now - self._last_warn_t > 5.0:
                self._last_warn_t = now
                self.get_logger().warn(
                    f"相机与 {self.sync_ref_topic} 时间基准相差 {diff * 1e3:+.1f}ms "
                    f"(阈值 {self.sync_warn_s * 1e3:.0f}ms)，检查同步器/驱动"
                )


    def _open_camera(self):
        ret = MvCamera.MvCamera.MV_CC_Initialize()
        if ret != 0:
            raise RuntimeError(f"MV_CC_Initialize 失败: 0x{ret:x}")

        dev_list = camParams.MV_CC_DEVICE_INFO_LIST()
        tlayer = (camParams.MV_GIGE_DEVICE | camParams.MV_USB_DEVICE)
        ret = MvCamera.MvCamera.MV_CC_EnumDevices(tlayer, dev_list)
        if ret != 0:
            raise RuntimeError(f"MV_CC_EnumDevices 失败: 0x{ret:x}")
        if dev_list.nDeviceNum == 0:
            raise RuntimeError("没有找到相机（检查 USB 连接 / 是否被 MVS 占用）")

        chosen = -1
        for i in range(dev_list.nDeviceNum):
            info = cast(dev_list.pDeviceInfo[i], POINTER(camParams.MV_CC_DEVICE_INFO)).contents
            if info.nTLayerType == camParams.MV_USB_DEVICE:
                sn = self._decode(info.SpecialInfo.stUsb3VInfo.chSerialNumber)
                model = self._decode(info.SpecialInfo.stUsb3VInfo.chModelName)
            elif info.nTLayerType == camParams.MV_GIGE_DEVICE:
                sn = self._decode(info.SpecialInfo.stGigEInfo.chSerialNumber)
                model = self._decode(info.SpecialInfo.stGigEInfo.chModelName)
            else:
                continue
            self.get_logger().info(f"[{i}] {model} SN={sn}")
            if chosen < 0 and (self.serial == "" or sn == self.serial):
                chosen = i

        if chosen < 0:
            raise RuntimeError(f"没有匹配的相机（serial='{self.serial}'）")

        self.dev_info = cast(
            dev_list.pDeviceInfo[chosen], POINTER(camParams.MV_CC_DEVICE_INFO)
        ).contents

        self.cam = MvCamera.MvCamera()
        ret = self.cam.MV_CC_CreateHandle(self.dev_info)
        if ret != 0:
            raise RuntimeError(f"MV_CC_CreateHandle 失败: 0x{ret:x}")
        ret = self.cam.MV_CC_OpenDevice(camParams.MV_ACCESS_Exclusive, 0)
        if ret != 0:
            raise RuntimeError(f"MV_CC_OpenDevice 失败: 0x{ret:x}（是否被 MVS 客户端占用？）")

        # 设置分辨率（ROI），0 表示用相机默认
        if self.width > 0 and self.height > 0:
            r1 = self.cam.MV_CC_SetIntValue("Width", self.width)
            r2 = self.cam.MV_CC_SetIntValue("Height", self.height)
            self.get_logger().info(
                f"设置分辨率 {self.width}x{self.height} ret=0x{r1:x}/0x{r2:x}"
            )

        # 触发模式：0=连续采集(自由运行) 1=外部触发(硬件同步)
        if self.trigger_mode == 1:
            r1 = self.cam.MV_CC_SetEnumValueByString("TriggerMode", "On")
            r2 = self.cam.MV_CC_SetEnumValueByString("TriggerSource", self.trigger_source)
            r3 = self.cam.MV_CC_SetEnumValueByString("TriggerActivation", self.trigger_polarity)
            self.get_logger().info(
                f"外部触发: source={self.trigger_source} polarity={self.trigger_polarity} "
                f"ret=0x{r1:x}/0x{r2:x}/0x{r3:x}"
            )
        else:
            self.cam.MV_CC_SetEnumValue("TriggerMode", 0)
        if self.exposure_us > 0:
            r = self.cam.MV_CC_SetFloatValue("ExposureTime", self.exposure_us)
            self.get_logger().info(f"ExposureTime={self.exposure_us}us ret=0x{r:x}")
        if self.gain_db > 0:
            r = self.cam.MV_CC_SetFloatValue("Gain", self.gain_db)
            self.get_logger().info(f"Gain={self.gain_db} ret=0x{r:x}")
        if self.fps > 0:
            self.cam.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", True)
            r = self.cam.MV_CC_SetFloatValue("AcquisitionFrameRate", self.fps)
            self.get_logger().info(f"FPS={self.fps} ret=0x{r:x}")

        # Bayer 插值用快速算法，降低 CPU
        try:
            self.cam.MV_CC_SetBayerCvtQuality(0)
        except Exception:  # noqa: BLE001
            pass

        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise RuntimeError(f"MV_CC_StartGrabbing 失败: 0x{ret:x}")

    # ------------------------------------------------------------------ #
    def _grab_loop(self):
        frame = camParams.MV_FRAME_OUT()
        memset(byref(frame), 0, sizeof(frame))

        n_fail = 0
        n_pub = 0
        t0 = time.perf_counter()
        t_last = t0
        sum_get = sum_conv = sum_pub = 0.0
        last_fail_ret = 0

        while self._running and rclpy.ok():
            ta = time.perf_counter()
            ret = self.cam.MV_CC_GetImageBuffer(frame, 1000)
            tb = time.perf_counter()

            if ret != 0 or frame.pBufAddr is None:
                n_fail += 1
                last_fail_ret = ret
                if (tb - t_last) > 2.0:
                    self.get_logger().warn(
                        f"取帧失败 {n_fail} 次，最后错误码 0x{ret:x}"
                    )
                    n_fail = 0
                    t_last = tb
                continue

            try:
                info = frame.stFrameInfo
                w = int(info.nWidth)
                h = int(info.nHeight)
                size = w * h * 3

                conv = camParams.MV_CC_PIXEL_CONVERT_PARAM()
                memset(byref(conv), 0, sizeof(conv))
                conv.nWidth = w
                conv.nHeight = h
                conv.pSrcData = cast(frame.pBufAddr, POINTER(c_ubyte))
                conv.nSrcDataLen = int(info.nFrameLen)
                conv.enSrcPixelType = info.enPixelType
                conv.enDstPixelType = PixelType.PixelType_Gvsp_BGR8_Packed

                if size != self._buf_size:
                    self._buf = (c_ubyte * size)()
                    self._buf_size = size
                buf = self._buf
                conv.pDstBuffer = buf
                conv.nDstBufferSize = size

                ret2 = self.cam.MV_CC_ConvertPixelType(conv)
                tc = time.perf_counter()
                if ret2 == 0 and int(conv.nDstLen) == size:
                    stamp = self._dev_ts_to_ros(info.nDevTimeStampHigh, info.nDevTimeStampLow)
                    if stamp is None:
                        # 时间基准还没校定：这一帧丢弃，避免发布错的时间戳
                        continue
                    self._check_sync(stamp)

                    img = Image()
                    img.header.stamp = stamp
                    img.header.frame_id = self.frame_id
                    img.height = h
                    img.width = w
                    img.encoding = "bgr8"
                    img.is_bigendian = 0
                    img.step = w * 3
                    img.data = array("B", memoryview(buf).tobytes())
                    self.pub_img.publish(img)

                    ci = CameraInfo()
                    ci.header.stamp = stamp
                    ci.header.frame_id = self.frame_id
                    ci.width = w
                    ci.height = h
                    ci.distortion_model = "plumb_bob"
                    ci.d = self.distortion
                    ci.k = [self.fx, 0.0, self.cx,
                            0.0, self.fy, self.cy,
                            0.0, 0.0, 1.0]
                    ci.r = [1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0]
                    ci.p = [self.fx, 0.0, self.cx, 0.0,
                            0.0, self.fy, self.cy, 0.0,
                            0.0, 0.0, 1.0, 0.0]
                    self.pub_info.publish(ci)
                    td = time.perf_counter()

                    n_pub += 1
                    sum_get += (tb - ta) * 1000.0
                    sum_conv += (tc - tb) * 1000.0
                    sum_pub += (td - tc) * 1000.0
                else:
                    self.get_logger().warn(
                        f"像素转换失败 ret=0x{ret2:x} dstLen={int(conv.nDstLen)} expect={size}"
                    )
            finally:
                self.cam.MV_CC_FreeImageBuffer(frame)

            now = time.perf_counter()
            if now - t_last >= 2.0:
                dt = now - t_last
                if n_pub > 0:
                    self.get_logger().info(
                        f"帧率 {n_pub / dt:.1f} Hz | 取帧 {sum_get / n_pub:.1f}ms | "
                        f"转换 {sum_conv / n_pub:.1f}ms | 发布 {sum_pub / n_pub:.1f}ms"
                    )
                else:
                    self.get_logger().warn(
                        f"2 秒内没有发布任何帧（取帧失败 {n_fail} 次，最后错误 0x{last_fail_ret:x}）"
                    )
                n_pub = 0
                sum_get = sum_conv = sum_pub = 0.0
                t_last = now

    # ------------------------------------------------------------------ #
    def destroy_node(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.cam is not None:
            try:
                self.cam.MV_CC_StopGrabbing()
                self.cam.MV_CC_CloseDevice()
                self.cam.MV_CC_DestroyHandle()
            except Exception:  # noqa: BLE001
                pass
            self.cam = None
        try:
            MvCamera.MvCamera.MV_CC_Finalize()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HikCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
