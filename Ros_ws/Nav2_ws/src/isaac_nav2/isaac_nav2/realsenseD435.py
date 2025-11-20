import asyncio
import json
import numpy as np
import cv2
import pyrealsense2 as rs
import paho.mqtt.client as mqtt
import threading
import queue
import os
import subprocess
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCRtpSender
from aiortc.sdp import candidate_from_sdp, candidate_to_sdp
from aiortc.rtcconfiguration import RTCConfiguration, RTCIceServer
from av import VideoFrame
from aiortc.contrib.media import MediaRelay

cv2.setNumThreads(1)

TARGET_BITRATE = 1500
MIN_BITRATE = 500

class RealSenseTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        device = config.resolve(rs.pipeline_wrapper(self.pipeline)).get_device()
        sensor = next((s for s in device.sensors if s.get_info(rs.camera_info.name) == 'RGB Camera'), None)
        if not sensor:
            exit()
        
        try: sensor.set_option(rs.option.frames_queue_size, 1)
        except: pass
        try: sensor.set_option(rs.option.low_latency_mode, 1.0)
        except: pass

        self.width, self.height, self.fps = 1280, 720, 15
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self.profile = self.pipeline.start(config)
        
        self.frame_queue = queue.Queue(maxsize=1)
        self.running = True
        threading.Thread(target=self._capture, daemon=True).start()
        
        import time
        time.sleep(0.15)

    def _capture(self):
        while self.running:
            try:
                frame = self.pipeline.wait_for_frames(1000).get_color_frame()
                if frame:
                    frame.keep()
                    try: self.frame_queue.get_nowait()
                    except: pass
                    try: self.frame_queue.put_nowait(frame)
                    except: pass
            except: pass
                    
    async def recv(self):
        frame = None
        for _ in range(3):
            try:
                frame = await asyncio.get_event_loop().run_in_executor(None, self.frame_queue.get, True, 0.25)
                break
            except queue.Empty:
                await asyncio.sleep(0.01)
        
        if not frame:
            img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        else:
            img = np.asanyarray(frame.get_data())
        
        vf = VideoFrame.from_ndarray(img, format="bgr24")
        pts, tb = await self.next_timestamp()
        vf.pts, vf.time_base = pts, tb
        return vf

    def stop(self):
        self.running = False
        try: self.pipeline.stop()
        except: pass

class WebRTCStreamer:
    def __init__(self):
        self.pcs = {}
        self.mqtt_broker, self.mqtt_port = "118.31.73.84", 8083
        self.video_track = RealSenseTrack()
        self.relay = MediaRelay()
        self.msg_queue = queue.Queue()
        self.running = True
        
        self.mqtt = mqtt.Client(transport="websockets")
        self.mqtt.on_connect = lambda c, u, f, rc: (c.subscribe("webrtc/offer"), c.subscribe("webrtc/ice_candidate/#"), c.subscribe("system")) if rc == 0 else None
        self.mqtt.on_message = lambda c, u, m: self.msg_queue.put(m)
        self.mqtt.on_disconnect = lambda c, u, rc: None
        
        try:
            self.mqtt.connect(self.mqtt_broker, self.mqtt_port, 60)
            threading.Thread(target=self.mqtt.loop_forever, daemon=True).start()
        except: pass

    async def process_messages(self):
        while self.running:
            try:
                msg = await asyncio.get_event_loop().run_in_executor(None, self.msg_queue.get, True, 0.5)
                await self._handle_msg(msg)
            except: pass

    async def _handle_msg(self, msg):
        try:
            data = json.loads(msg.payload)
            topic = msg.topic
            
            if topic == "webrtc/offer":
                await self._handle_offer(data)
            elif topic.startswith("webrtc/ice_candidate/"):
                await self._handle_ice(topic.replace("webrtc/ice_candidate/", ""), data)
            elif topic == "system":
                await self._handle_sys(data)
        except: pass

    async def _handle_sys(self, data):
        try:
            script_path = None
            msg_type = data.get("type")
            
            if msg_type == "open_drive":
                script_path = "/home/nvidia/Desktop/driver.sh"
            elif msg_type == "close_drive":
                script_path = "/home/nvidia/Desktop/driver_close.sh"
            elif msg_type == "open_nav":
                script_path = "/home/nvidia/Desktop/localization.sh"
            elif msg_type == "close_nav":
                script_path = "/home/nvidia/Desktop/localization_close.sh"
            
            if script_path and os.path.exists(script_path):
                subprocess.Popen(
                    ["gnome-terminal", "--title=Driver", "--", "bash", "-i", "-c",
                    f"exec nice -n 5 ionice -c2 -n7 {script_path}; exec bash"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True,
                    close_fds=True
                )
        except: pass

    def _set_bitrate(self, sdp, br):
        lines = sdp.splitlines()
        idx = next((i for i, l in enumerate(lines) if l.startswith("m=video")), -1)
        if idx != -1:
            i = idx + 1
            while i < len(lines) and lines[i].startswith("b="): lines.pop(i)
            lines.insert(i, f"b=TIAS:{br*1000}")
            lines.insert(i+1, f"b=AS:{br}")
        return "\r\n".join(lines)

    def _set_h264_hints(self, sdp, min_br, max_br):
        lines = sdp.splitlines()
        pts = {l.split(":")[1].split()[0] for l in lines if l.startswith("a=rtpmap:") and "H264" in l}
        
        for pt in pts:
            for i, l in enumerate(lines):
                if l.startswith(f"a=fmtp:{pt} "):
                    params = l.split(" ", 1)[1]
                    parts = [p.strip() for p in params.split(";") if p.strip()]
                    kv = {p.split("=")[0]: p.split("=")[1] if "=" in p else "" for p in parts}
                    kv["x-google-max-bitrate"] = str(max_br)
                    kv["x-google-min-bitrate"] = str(min_br)
                    kv["x-google-start-bitrate"] = str((min_br+max_br)//2)
                    lines[i] = f"a=fmtp:{pt} " + ";".join([f"{k}={v}" if v else k for k,v in kv.items()])
                    break
        return "\r\n".join(lines)

    async def _handle_offer(self, data):
        uuid, sdp, typ = data.get("uuid"), data.get("sdp"), data.get("type")
        if not all([uuid, sdp, typ]): return
        
        pc = RTCPeerConnection(RTCConfiguration(iceServers=[
            RTCIceServer(urls="stun:stun.l.google.com:19302"),
            RTCIceServer(urls="stun:stun.qq.com:3478"),
            RTCIceServer(urls="turn:openrelay.metered.ca:80", username="openrelayproject", credential="openrelayproject"),
        ]))

        try:
            t = pc.addTransceiver("video", direction="sendonly")
            h264 = [c for c in RTCRtpSender.getCapabilities("video").codecs if c.mimeType.lower() == "video/h264"]
            if h264: t.setCodecPreferences(h264)
            t.sender.replaceTrack(self.relay.subscribe(self.video_track))
        except:
            pc.addTrack(self.relay.subscribe(self.video_track))

        self.pcs[uuid] = pc
        
        @pc.on("connectionstatechange")
        async def _():
            if pc.connectionState in ["failed", "closed"]:
                await pc.close()
                self.pcs.pop(uuid, None)

        @pc.on("icecandidate")
        def _(c):
            if c: self.mqtt.publish(f"webrtc/ice_candidate/{uuid}", json.dumps({"candidate": candidate_to_sdp(c), "sdpMid": c.sdpMid, "sdpMLineIndex": c.sdpMLineIndex}))

        await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=typ))
        ans = await pc.createAnswer()
        sdp_mod = self._set_h264_hints(self._set_bitrate(ans.sdp, TARGET_BITRATE), MIN_BITRATE, TARGET_BITRATE)
        await pc.setLocalDescription(RTCSessionDescription(sdp=sdp_mod, type=ans.type))
        self.mqtt.publish(f"webrtc/answer/{uuid}", json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}))

    async def _handle_ice(self, uuid, data):
        if uuid not in self.pcs: return
        c, mid, idx = data.get("candidate"), data.get("sdpMid"), data.get("sdpMLineIndex")
        if all([c, mid is not None, idx is not None]):
            ice = candidate_from_sdp(c)
            ice.sdpMid, ice.sdpMLineIndex = mid, idx
            await self.pcs[uuid].addIceCandidate(ice)

    async def cleanup(self):
        self.running = False
        if hasattr(self, 'video_track'): self.video_track.stop()
        for pc in self.pcs.values(): await pc.close()
        self.pcs.clear()
        if self.mqtt: self.mqtt.disconnect()

async def main_async():
    s = WebRTCStreamer()
    try:
        await asyncio.gather(s.process_messages(), asyncio.sleep(7200))
    except: pass
    finally:
        await s.cleanup()

def main():
    asyncio.run(main_async())

if __name__ == "__main__":
    main()
