import asyncio
import json
import logging
import platform
import numpy as np
import cv2
import pyrealsense2 as rs
import paho.mqtt.client as mqtt
import threading
import uuid
import queue
import os
import subprocess
import stat
from concurrent.futures import ThreadPoolExecutor
from aiortc.rtcrtpparameters import RTCRtpCodecCapability
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack,RTCRtpCodecParameters,RTCRtpSender, RTCIceCandidate
from aiortc.sdp import candidate_from_sdp, candidate_to_sdp
from aiortc.rtcconfiguration import RTCConfiguration, RTCIceServer
from av import VideoFrame
from aiortc.contrib.media import MediaRelay


# 设置空日志器（禁用日志输出，减少 I/O 与 CPU 开销）
class _NullLogger:
    def debug(self, *args, **kwargs):
        pass
    def info(self, *args, **kwargs):
        pass
    def warning(self, *args, **kwargs):
        pass
    def error(self, *args, **kwargs):
        pass
    def exception(self, *args, **kwargs):
        pass

logger = _NullLogger()

# 限制 OpenCV 线程数，避免过度占用 CPU
try:
    cv2.setNumThreads(1)
except Exception:
    pass

# 提升码率以减少马赛克（不改变分辨率与帧率）
TARGET_BITRATE_KBPS = 6000  # 6 Mbps for 1280x720@15fps
MIN_BITRATE_KBPS = 3000     # 3 Mbps 最低码率

# -------------------------------------------------------------------
# 1. RealSense D435 视频轨道类（性能优化版）
# -------------------------------------------------------------------
class RealSenseTrack(VideoStreamTrack):
    """
    一个从 Intel RealSense D435 相机捕获视频的视频轨道。
    优化版：使用线程池异步捕获、减少内存复制、优化编码器配置
    """
    def __init__(self):
        super().__init__()
        logger.info("Initializing RealSense D435 (Optimized)...")
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # 获取设备产品线，以确定是否为 D400 系列
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        
        # 找到彩色传感器
        color_sensor = None
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                color_sensor = s
                break
        if not color_sensor:
            logger.error("Could not find RGB Camera sensor.")
            exit()

        # 低延迟与小缓冲
        try:
            color_sensor.set_option(rs.option.frames_queue_size, 1)
        except Exception:
            pass
        try:
            color_sensor.set_option(rs.option.low_latency_mode, 1.0)
        except Exception:
            pass

        # 配置流的分辨率和帧率（保持 1280x720@15fps）
        self.width, self.height, self.fps = 1280, 720, 15
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        
        # 启动管道
        self.profile = self.pipeline.start(config)
        logger.info(f"RealSense D435 started at {self.width}x{self.height}@{self.fps}fps")
        
        # 创建线程池用于异步帧捕获（避免阻塞主事件循环）
        self.executor = ThreadPoolExecutor(max_workers=1)
        
        # 仅保留最新一帧，避免堆积
        self.frame_queue = queue.Queue(maxsize=1)
        self.running = True
        
        # 启动后台帧捕获线程
        self.capture_thread = threading.Thread(target=self._capture_frames, daemon=True)
        self.capture_thread.start()
        
        # 等待队列填充第一帧（避免初始的队列为空警告）
        import time
        time.sleep(0.2)  # 等待200ms让捕获线程填充队列

    def _capture_frames(self):
        """后台线程：持续捕获帧并放入队列"""
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                if color_frame:
                    # 保持底层数据有效，避免复制
                    color_frame.keep()
                    # 覆盖旧帧，仅保留最新一帧
                    if not self.frame_queue.empty():
                        try:
                            self.frame_queue.get_nowait()
                        except queue.Empty:
                            pass
                    try:
                        self.frame_queue.put_nowait(color_frame)
                    except queue.Full:
                        pass
            except Exception as e:
                if self.running:
                    logger.warning(f"Frame capture error: {e}")
                    
    async def recv(self):
        """
        这个方法由 aiortc 内部循环调用，以获取新的视频帧。
        优化版：从队列获取预捕获的帧，避免阻塞
        """
        # 尝试从队列获取帧
        color_frame = None
        retry_count = 0
        max_retries = 3
        
        while color_frame is None and retry_count < max_retries:
            try:
                # 从队列获取帧（带超时避免永久阻塞）
                color_frame = await asyncio.get_event_loop().run_in_executor(
                    None, self.frame_queue.get, True, 0.3
                )
            except queue.Empty:
                retry_count += 1
                if retry_count >= max_retries:
                    logger.warning("Frame queue empty after retries, skipping frame")
                    # 创建一个黑色的占位帧
                    img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                    frame = VideoFrame.from_ndarray(img, format="bgr24")
                    pts, time_base = await self.next_timestamp()
                    frame.pts = pts
                    frame.time_base = time_base
                    return frame
                await asyncio.sleep(0.01)  # 短暂等待

        # 将 RealSense 帧转换为 numpy 数组
        img = np.asanyarray(color_frame.get_data())
        # 将 numpy 数组转换为 av.VideoFrame
        frame = VideoFrame.from_ndarray(img, format="bgr24")
        
        # 设置时间戳，这对于同步非常重要
        pts, time_base = await self.next_timestamp()
        frame.pts = pts
        frame.time_base = time_base
        
        return frame

    def stop(self):
        """
        停止 RealSense 管道
        """
        logger.info("Stopping RealSense D435...")
        self.running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join(timeout=2.0)
        if hasattr(self, 'executor'):
            self.executor.shutdown(wait=False)
        try:
            self.pipeline.stop()
        except Exception:
            pass



# -------------------------------------------------------------------
# 2. MQTT WebRTC 信令处理
# -------------------------------------------------------------------
class WebRTCStreamer:
    def __init__(self):
        self.pcs = {}  # 存储客户端UUID到RTCPeerConnection的映射
        self.mqtt_client = None
        self.mqtt_broker = "118.31.73.84"  # MQTT Broker地址
        self.mqtt_port = 8083  # MQTT WebSocket端口
        
        # MQTT主题
        self.offer_topic = "webrtc/offer"
        self.answer_topic_prefix = "webrtc/answer/"
        self.ice_candidate_topic_prefix = "webrtc/ice_candidate/"
        self.system_topic='system'

        self.video_track = RealSenseTrack()
        # 多路观众共享同一路采集，避免重复前处理
        self.relay = MediaRelay()

        # 用于在主线程中处理MQTT消息的队列
        self.message_queue = queue.Queue()
        self.running = True
        
        # 启动MQTT客户端
        self.setup_mqtt()
        
    def setup_mqtt(self):
        """设置并连接MQTT客户端"""
        self.mqtt_client = mqtt.Client(transport="websockets")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            logger.info(f"Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
            
            # 在单独的线程中运行MQTT循环
            mqtt_thread = threading.Thread(target=self.mqtt_client.loop_forever)
            mqtt_thread.daemon = True
            mqtt_thread.start()
            
        except Exception as e:
            logger.error(f"Failed to connect to MQTT broker: {e}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            
            # 订阅Offer主题
            client.subscribe(self.offer_topic)
            
            # 订阅ICE候选主题（所有客户端）
            ice_sub_topic = f"{self.ice_candidate_topic_prefix}#"
            client.subscribe(ice_sub_topic)
            client.subscribe(self.system_topic)
            
        else:
            logger.error(f"❌ Failed to connect to MQTT broker, return code: {rc}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调"""
        logger.warning("MQTT client disconnected")

    def on_mqtt_message(self, client, userdata, msg):
        """MQTT消息回调 - 将消息放入队列，在主线程中处理"""
        try:
            # 将消息放入队列，在主线程中处理
            self.message_queue.put(msg)
        except Exception as e:
            logger.error(f"Error putting message in queue: {e}")

    async def process_messages(self):
        """在主线程中处理MQTT消息（优化版：减少空循环CPU占用）"""
        while self.running:
            try:
                # 使用阻塞获取但带超时，减少CPU空转
                msg = await asyncio.get_event_loop().run_in_executor(
                    None, self.message_queue.get, True, 0.5
                )
                await self.handle_mqtt_message(msg)
            except queue.Empty:
                # 超时后继续循环，不需要额外sleep
                continue
            except Exception as e:
                logger.error(f"Error processing message from queue: {e}")

    async def handle_mqtt_message(self, msg):
        """处理MQTT消息（优化版：减少日志输出）"""
        try:
            payload = json.loads(msg.payload.decode())
            topic = msg.topic
            
            # 只在必要时输出日志（减少I/O开销）
            if topic == self.offer_topic:
                logger.info(f"Received WebRTC offer on topic: {topic}")
                await self.handle_offer(payload)
            elif topic.startswith(self.ice_candidate_topic_prefix):
                # ICE候选日志降级为debug（减少频繁日志输出）
                client_uuid = topic.replace(self.ice_candidate_topic_prefix, "")
                logger.debug(f"Received ICE candidate for client: {client_uuid}")
                await self.handle_ice_candidate(client_uuid, payload)
            elif topic == self.system_topic:
                logger.info(f"Received system message: {payload.get('type', 'unknown')}")
                await self.handle_system_message(payload)
                
        except Exception as e:
            logger.error(f"Error processing MQTT message: {e}")

    async def handle_system_message(self, payload):
        try:
            script_path = None
            if payload.get("type") == "open_drive":
                script_path = "/home/nvidia/Desktop/driver.sh"
            elif payload.get("type") == "close_drive":
                script_path = "/home/nvidia/Desktop/driver_close.sh"
            elif payload.get("type") == "open_nav":
                script_path = "/home/nvidia/Desktop/localization.sh"
            elif payload.get("type") == "close_nav":
                script_path = "/home/nvidia/Desktop/localization_close.sh"
            else:
                pass
            if script_path and os.path.exists(script_path):
                subprocess.Popen(
                    ["gnome-terminal", "--title=Driver", "--", "bash", "-i", "-c",
                    f"exec nice -n 5 ionice -c2 -n7 {script_path}; exec bash"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True,     # 与本进程信号完全隔离
                    close_fds=True              # 不继承本进程的 sockets/FD
                )
        except Exception as e:
                pass


    def set_bitrate(self, sdp, bitrate_kbps):
        """在视频m=段落设置TIAS/AS码率，避免重复插入。"""
        lines = sdp.splitlines()
        video_m_line_index = -1
        # 找到视频媒体描述行 "m=video ..."
        for i, line in enumerate(lines):
            if line.startswith("m=video"):
                video_m_line_index = i
                break

        if video_m_line_index != -1:
            # 清理该m=段落紧随其后的旧b=行
            insert_index = video_m_line_index + 1
            while insert_index < len(lines) and lines[insert_index].startswith("b="):
                lines.pop(insert_index)
            # 插入TIAS(比特/秒)和AS(千比特/秒)
            tias_bps = bitrate_kbps * 1000
            lines.insert(insert_index, f"b=TIAS:{tias_bps}")
            lines.insert(insert_index + 1, f"b=AS:{bitrate_kbps}")
            logger.info(f"SDP set TIAS={tias_bps} bps, AS={bitrate_kbps} kbps")

        return "\r\n".join(lines)

    def set_h264_bitrate_hints(self, sdp, min_kbps, max_kbps):
        """为H264 payload附加 x-google-min/max-bitrate 提示（仅在answer中修改）。
        优化版：添加更快的编码器预设以减少CPU负担"""
        lines = sdp.splitlines()
        h264_pts = set()
        # 收集H264负载类型号
        for i, line in enumerate(lines):
            if line.startswith("a=rtpmap:") and "H264/90000" in line:
                try:
                    pt = line.split(":", 1)[1].split()[0]
                    h264_pts.add(pt)
                except Exception:
                    continue
        if not h264_pts:
            return sdp

        def update_fmtp(payload_type):
            prefix = f"a=fmtp:{payload_type} "
            for i, line in enumerate(lines):
                if line.startswith(prefix):
                    params = line[len(prefix):]
                    parts = [p.strip() for p in params.split(";") if p.strip()]
                    kv = {}
                    for p in parts:
                        if "=" in p:
                            k, v = p.split("=", 1)
                            kv[k.strip()] = v.strip()
                        else:
                            kv[p] = ""
                    # 添加码率提示
                    kv["x-google-max-bitrate"] = str(max_kbps)
                    kv["x-google-min-bitrate"] = str(min_kbps)
                    # 添加编码器速度提示（减少CPU负担）
                    kv["x-google-start-bitrate"] = str((min_kbps + max_kbps) // 2)
                    new_params = ";".join([f"{k}={v}" if v != "" else k for k, v in kv.items()])
                    lines[i] = prefix + new_params
                    return True
            return False

        for pt in h264_pts:
            if not update_fmtp(pt):
                # 若没有fmtp行，则新增一行
                lines.append(f"a=fmtp:{pt} x-google-min-bitrate={min_kbps};x-google-max-bitrate={max_kbps};x-google-start-bitrate={(min_kbps + max_kbps) // 2}")

        return "\r\n".join(lines)


    async def handle_offer(self, offer_data):
        """处理WebRTC Offer"""
        client_uuid = offer_data.get("uuid")
        sdp_offer = offer_data.get("sdp")
        offer_type = offer_data.get("type")
        
        if not all([client_uuid, sdp_offer, offer_type]):
            logger.error("Invalid offer data")
            return
        
        logger.info(f"Received WebRTC offer from client: {client_uuid}")
        
        # 创建新的PeerConnection（配置公网可用的STUN/TURN）
        configuration = RTCConfiguration(iceServers=[
            RTCIceServer(urls="stun:stun.l.google.com:19302"),
            RTCIceServer(urls="stun:stun1.l.google.com:19302"),
            RTCIceServer(urls="stun:stun.miwifi.com:3478"),
            RTCIceServer(urls="stun:stun.qq.com:3478"),
            RTCIceServer(urls="turn:openrelay.metered.ca:80", username="openrelayproject", credential="openrelayproject"),
            RTCIceServer(urls="turn:openrelay.metered.ca:443?transport=tcp", username="openrelayproject", credential="openrelayproject"),
            RTCIceServer(urls="turn:openrelay.metered.ca:443", username="openrelayproject", credential="openrelayproject"),
        ])
        pc = RTCPeerConnection(configuration=configuration)

        # ---------> 新增代码开始 <---------
        # 尝试筛选并优先使用 H264 编码器
        # 注意: 这部分代码比较 hacky，用于强制改变编解码器协商顺序
        try:
            transceiver = pc.addTransceiver("video", direction="sendonly")
            codecs = RTCRtpSender.getCapabilities("video").codecs
            h264_codecs = [codec for codec in codecs if codec.mimeType.lower() == "video/h264"]
            if h264_codecs:
                transceiver.setCodecPreferences(h264_codecs)
                logger.info("✅ Set H.264 as the preferred codec.")
            else:
                logger.warning("H.264 codec not found, using default.")
            # 将视频轨道绑定到发送端（通过 relay 订阅实现单源多发）
            transceiver.sender.replaceTrack(self.relay.subscribe(self.video_track))
        except Exception as e:
            logger.warning(f"Transceiver setup failed, falling back to addTrack: {e}")
            pc.addTrack(self.relay.subscribe(self.video_track))
        # ---------> 新增代码结束 <---------


        self.pcs[client_uuid] = pc
        
        # 设置连接状态变化回调
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            state = pc.connectionState
            logger.info(f"Connection state for {client_uuid}: {state}")
            if state == "failed" or state == "closed":
                await pc.close()
                if client_uuid in self.pcs:
                    del self.pcs[client_uuid]
                logger.info(f"Connection closed for client: {client_uuid}")

        # 处理ICE候选（优化版：减少日志输出）
        @pc.on("icecandidate")
        def on_icecandidate(candidate):
            if candidate:
                # ICE候选日志降级为debug（减少频繁日志输出）
                logger.debug(f"Sending ICE candidate to client: {client_uuid} -> sdpMid={candidate.sdpMid}, index={candidate.sdpMLineIndex}")
                candidate_data = {
                    "candidate": candidate_to_sdp(candidate),
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex,
                }
                # 发布ICE候选到客户端特定主题
                ice_topic = f"{self.ice_candidate_topic_prefix}{client_uuid}"
                self.mqtt_client.publish(ice_topic, json.dumps(candidate_data))

        # 视频轨道已在上面通过 transceiver 或 addTrack 绑定
        
        # 设置远程描述
        offer = RTCSessionDescription(sdp=sdp_offer, type=offer_type)
        await pc.setRemoteDescription(offer)
        
        # 创建 Answer，并在设置本地描述前修改 SDP 以提高比特率
        answer = await pc.createAnswer()
        modified_sdp = self.set_bitrate(answer.sdp, TARGET_BITRATE_KBPS)
        modified_sdp = self.set_h264_bitrate_hints(modified_sdp, MIN_BITRATE_KBPS, TARGET_BITRATE_KBPS)
        modified_answer = RTCSessionDescription(sdp=modified_sdp, type=answer.type)
        await pc.setLocalDescription(modified_answer)
        # ---------> 新增代码结束 <---------
        
        # 发送Answer给客户端
        answer_data = {
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        }
        answer_topic = f"{self.answer_topic_prefix}{client_uuid}"
        self.mqtt_client.publish(answer_topic, json.dumps(answer_data))
        logger.info(f"Answer sent to topic: {answer_topic}")

    async def handle_ice_candidate(self, client_uuid, candidate_data):
        """处理来自客户端的ICE候选（优化版：减少日志输出）"""
        if client_uuid not in self.pcs:
            logger.warning(f"Received ICE candidate for unknown client: {client_uuid}")
            return
        
        pc = self.pcs[client_uuid]
        candidate = candidate_data.get("candidate")
        sdp_mid = candidate_data.get("sdpMid")
        sdp_mline_index = candidate_data.get("sdpMLineIndex")
        
        if candidate and sdp_mid is not None and sdp_mline_index is not None:
            rtc_ice = candidate_from_sdp(candidate)
            rtc_ice.sdpMid = sdp_mid
            rtc_ice.sdpMLineIndex = sdp_mline_index
            await pc.addIceCandidate(rtc_ice)
            # ICE候选日志降级为debug（减少频繁日志输出）
            logger.debug(f"Added ICE candidate for client: {client_uuid} -> sdpMid={sdp_mid}, index={sdp_mline_index}")

    async def cleanup(self):
        """清理资源（优化版：正确停止视频轨道）"""
        self.running = False
        logger.info("Cleaning up WebRTC connections...")
        
        # 先停止视频轨道
        if hasattr(self, 'video_track'):
            self.video_track.stop()
            logger.info("Video track stopped")
        
        # 关闭所有peer connections
        for client_uuid, pc in self.pcs.items():
            await pc.close()
        self.pcs.clear()
        
        # 断开MQTT连接
        if self.mqtt_client:
            self.mqtt_client.disconnect()


# -------------------------------------------------------------------
# 3. 主程序入口
# -------------------------------------------------------------------

def main():
    """ROS2 entry point - 同步函数"""
    asyncio.run(main_async())

async def main_async():
    """异步主函数"""
    logger.info("Starting RealSense D435 WebRTC Streamer with MQTT...")
    
    # 创建WebRTC流媒体实例
    streamer = WebRTCStreamer()
    
    try:
        # 同时处理MQTT消息和保持程序运行
        await asyncio.gather(
            streamer.process_messages(),
            asyncio.sleep(3600)  # 运行1小时，或者直到被中断
        )
    except asyncio.CancelledError:
        logger.info("Main task cancelled")
    except KeyboardInterrupt:
        logger.info("Received interrupt signal, shutting down...")
    finally:
        await streamer.cleanup()
        logger.info("Streamer shutdown complete.")

if __name__ == "__main__":
    # 运行主程序
    main()