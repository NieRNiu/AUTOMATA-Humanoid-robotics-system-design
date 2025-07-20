#coding=utf-8

'''
shenlan tts bot
requires Python 3.6 or later

pip install asyncio
pip install websockets

'''

import asyncio
import websockets
import uuid
import json
import gzip
import copy, os
import pyaudio


import queue
import threading, time, signal
import api


config = {
    "api_url" : f"wss://openspeech.bytedance.com/api/v1/tts/ws_binary",
    "cluster" : "volcano_tts",
    "voice_type" : "BV001_streaming",
}

MESSAGE_TYPES = {11: "audio-only server response", 12: "frontend server response", 15: "error message from server"}
MESSAGE_TYPE_SPECIFIC_FLAGS = {0: "no sequence number", 1: "sequence number > 0",
                               2: "last message from server (seq < 0)", 3: "sequence number < 0"}
MESSAGE_SERIALIZATION_METHODS = {0: "no serialization", 1: "JSON", 15: "custom type"}
MESSAGE_COMPRESSIONS = {0: "no compression", 1: "gzip", 15: "custom compression method"}

# version: b0001 (4 bits)
# header size: b0001 (4 bits)
# message type: b0001 (Full client request) (4bits)
# message type specific flags: b0000 (none) (4bits)
# message serialization method: b0001 (JSON) (4 bits)
# message compression: b0001 (gzip) (4bits)
# reserved data: 0x00 (1 byte)
default_header = bytearray(b'\x11\x10\x11\x00')

class TTSClient(object):
    def __init__(self, appid:str, token:str):
        """"""
        self.appid = appid
        self.token = token

        self.api_url = config["api_url"]
        self.cluster = config["cluster"]
        self.voice_type = config["voice_type"]
        self.web_socket = None

        # 初始化
        self.audio = pyaudio.PyAudio()
        self.output_stream = self.audio.open(
            format=pyaudio.paInt16,
            channels = 1,
            rate = 24000,
            output = True,
            frames_per_buffer = 3200)
        self.audio_queue = queue.Queue()
        # start audio play thread
        self.is_playing = True
        self.interrupt_play = False
        signal.signal(signal.SIGINT, self._keyboard_signal)
        self.player_thread = threading.Thread(target=self._audio_player_thread)
        self.player_thread.daemon = True
        self.player_thread.start()

    def _keyboard_signal(self, sig, frame):
        print(f"receive keyboard Ctrl+C")
        self.is_playing = False
        self.interrupt_play = True

    def _audio_player_thread(self):
        """音频播放线程"""
        while self.is_playing:
            try:
                # 从队列获取音频数据
                audio_data = self.audio_queue.get(timeout=1.0)
                for start in range(0, len(audio_data) + 3200, 3200):
                    self.output_stream.write(audio_data[start:start + 3200])
            except queue.Empty:
                # 队列为空时等待一小段时间
                time.sleep(0.1)
            except Exception as e:
                print(f"音频播放错误: {e}")
                time.sleep(0.1)

    def clean_up(self):
        """打断并清空音频播放"""
        while not self.audio_queue.empty():
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                continue
        self.interrupt_play = True

    async def request_once(self, text : str) -> None:
        query_request_json = {
            "app": {
                "appid": self.appid,
                "token": self.token,
                "cluster": self.cluster,
            },
            "user": {
                "uid": "shenlan_tts_bot",
            },
            "audio": {
                "voice_type": self.voice_type,
                "encoding": "pcm",
                "speed_ratio": 1.0,
                "volume_ratio": 1.0,
                "pitch_ratio": 1.0,
                "emotion": "happy",
            },
            "request": {
                "reqid": str(uuid.uuid4()),
                "text": text,
                "text_type": "plain",
                "operation": "query"
            }
        }
        payload_bytes = str.encode(json.dumps(query_request_json))
        payload_bytes = gzip.compress(payload_bytes)
        full_client_request = bytearray(default_header)
        full_client_request.extend((len(payload_bytes)).to_bytes(4, 'big'))  # payload size(4 bytes)
        full_client_request.extend(payload_bytes)  # payload
        header = {"Authorization": f"Bearer; {self.token}"}
        async with websockets.connect(self.api_url, extra_headers=header, ping_interval=None) as ws:
            await ws.send(full_client_request)
            res = await ws.recv()
            self.parse_response(res)
        return None


    def parse_response(self, res) -> None:
        """接收并解析response"""
        protocol_version = res[0] >> 4
        header_size = res[0] & 0x0f
        message_type = res[1] >> 4
        message_type_specific_flags = res[1] & 0x0f
        serialization_method = res[2] >> 4
        message_compression = res[2] & 0x0f
        reserved = res[3]
        header_extensions = res[4:header_size * 4]
        payload = res[header_size * 4:]

        if header_size != 1:
            print(f"           Header extensions: {header_extensions}")
        if message_type == 0xb:  # audio-only server response
            if message_type_specific_flags == 0:  # no sequence number as ACK
                print("                Payload size: 0")
                return None
            else:
                sequence_number = int.from_bytes(payload[:4], "big", signed=True)
                payload_size = int.from_bytes(payload[4:8], "big", signed=False)
                payload = payload[8:]
                self.audio_queue.put(payload)
            if sequence_number < 0:
                return None
            else:
                return None
        elif message_type == 0xf:
            code = int.from_bytes(payload[:4], "big", signed=False)
            msg_size = int.from_bytes(payload[4:8], "big", signed=False)
            error_msg = payload[8:]
            if message_compression == 1:
                error_msg = gzip.decompress(error_msg)
            error_msg = str(error_msg, "utf-8")
            print(f"          Error message code: {code}")
            print(f"          Error message size: {msg_size} bytes")
            print(f"               Error message: {error_msg}")
            return None
        elif message_type == 0xc:
            msg_size = int.from_bytes(payload[:4], "big", signed=False)
            payload = payload[4:]
            if message_compression == 1:
                payload = gzip.decompress(payload)
            print(f"            Frontend message: {payload}")
        else:
            print("undefined message type!")
            return None


if __name__ == '__main__':
    tts_bot = TTSClient(
        appid=api.APP_ID,
        token=api.APP_TOKEN)

    for i in range(3):
        text = f"这是第{i+1}句话！"
        print(text)
        asyncio.run(tts_bot.request_once(text))  ## 流式待做

    time.sleep(1)