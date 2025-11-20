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
from aiortc.rtcrtpparameters import RTCRtpCodecCapability
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack,RTCRtpCodecParameters,RTCRtpSender, RTCIceCandidate
from aiortc.sdp import candidate_from_sdp, candidate_to_sdp
from aiortc.rtcconfiguration import RTCConfiguration, RTCIceServer
from av import VideoFrame

# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("realsense_webrtc")

# -------------------------------------------------------------------
# 1. RealSense D435 视频轨道类
# -------------------------------------------------------------------
class RealSenseTrack(VideoStreamTrack):
    """
    一个从 Intel RealSense D435 相机捕获视频的视频轨道。
    """
    def __init__(self):
        super().__init__()
        logger.info("Initializing RealSense D435...")
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

        # 配置流的分辨率和帧率
        self.width, self.height, self.fps = 1280, 720, 15
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        
        # 启动管道
        self.profile = self.pipeline.start(config)
        logger.info(f"RealSense D435 started at {self.width}x{self.height}@{self.fps}fps")

    async def recv(self):
        """
        这个方法由 aiortc 内部循环调用，以获取新的视频帧。
        """
        # 等待一对连贯的帧: 深度和颜色
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return await super().recv()

        # 将图像数据转换为 numpy 数组
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
        #self.pipeline.stop()



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
        

        self.video_track = RealSenseTrack()

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
            logger.info("✅ Successfully connected to MQTT broker")
            # 订阅Offer主题
            client.subscribe(self.offer_topic)
            logger.info(f"✅ Subscribed to offer topic: {self.offer_topic}")
            # 订阅ICE候选主题（所有客户端）
            ice_sub_topic = f"{self.ice_candidate_topic_prefix}#"
            client.subscribe(ice_sub_topic)
            logger.info(f"✅ Subscribed to ICE topic: {ice_sub_topic}")
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
        """在主线程中处理MQTT消息"""
        while self.running:
            try:
                # 非阻塞获取消息
                msg = self.message_queue.get_nowait()
                await self.handle_mqtt_message(msg)
            except queue.Empty:
                # 队列为空，等待一段时间再检查
                await asyncio.sleep(0.1)
            except Exception as e:
                logger.error(f"Error processing message from queue: {e}")

    async def handle_mqtt_message(self, msg):
        """处理MQTT消息"""
        try:
            payload = json.loads(msg.payload.decode())
            topic = msg.topic
            
            logger.info(f"Received MQTT message on topic: {topic}")
            
            if topic == self.offer_topic:
                # 处理WebRTC Offer
                await self.handle_offer(payload)
            elif topic.startswith(self.ice_candidate_topic_prefix):
                # 处理ICE候选
                client_uuid = topic.replace(self.ice_candidate_topic_prefix, "")
                await self.handle_ice_candidate(client_uuid, payload)
                
        except Exception as e:
            logger.error(f"Error processing MQTT message: {e}")


    def set_bitrate(self, sdp, bitrate_kbps):
        
        lines = sdp.splitlines()
        video_m_line_index = -1
        # 找到视频媒体描述行 "m=video ..."
        for i, line in enumerate(lines):
            if line.startswith("m=video"):
                video_m_line_index = i
                break
        
        if video_m_line_index != -1:
            # 在 "m=video ..." 行下面插入比特率控制行 "b=AS:xxxx"
            # AS 表示 Application Specific Maximum
            bitrate_line = f"b=AS:{bitrate_kbps}"
            lines.insert(video_m_line_index + 1, bitrate_line)
            logger.info(f"✅ SDP modified to request bitrate: {bitrate_kbps} kbps")
        
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
            # 将视频轨道绑定到发送端
            transceiver.sender.replaceTrack(self.video_track)
        except Exception as e:
            logger.warning(f"Transceiver setup failed, falling back to addTrack: {e}")
            pc.addTrack(self.video_track)
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

        # 处理ICE候选
        @pc.on("icecandidate")
        def on_icecandidate(candidate):
            if candidate:
                logger.info(f"Sending ICE candidate to client: {client_uuid} -> sdpMid={candidate.sdpMid}, index={candidate.sdpMLineIndex}")
                candidate_data = {
                    "candidate": candidate_to_sdp(candidate),
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex,
                }
                # 发布ICE候选到客户端特定主题
                ice_topic = f"{self.ice_candidate_topic_prefix}{client_uuid}"
                self.mqtt_client.publish(ice_topic, json.dumps(candidate_data))
                logger.info(f"ICE candidate sent to topic: {ice_topic}")

        # 视频轨道已在上面通过 transceiver 或 addTrack 绑定
        
        # 设置远程描述
        offer = RTCSessionDescription(sdp=sdp_offer, type=offer_type)
        await pc.setRemoteDescription(offer)
        
        # 创建并设置本地描述（Answer）
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        # 在发送前修改SDP以设置更高的比特率
        target_bitrate = 2000  # 设置目标比特率为 2000 kbps，您可以调整
        answer.sdp = self.set_bitrate(answer.sdp, target_bitrate)
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
        """处理来自客户端的ICE候选"""
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
            logger.info(f"Added ICE candidate for client: {client_uuid} -> sdpMid={sdp_mid}, index={sdp_mline_index}")

    async def cleanup(self):
        """清理资源"""
        self.running = False
        logger.info("Cleaning up WebRTC connections...")
        for client_uuid, pc in self.pcs.items():
            await pc.close()
        self.pcs.clear()
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