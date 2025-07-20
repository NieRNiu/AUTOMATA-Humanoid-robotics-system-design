# coding=utf-8

"""
requires Python 3.6 or later

pip install asyncio
pip install websockets
pip install pyaudio
"""

import asyncio
import base64
import gzip
import hmac
import json
import logging
import os
import uuid
import wave, time
from enum import Enum
from hashlib import sha256
from urllib.parse import urlparse
import time
import websockets
import pyaudio
import threading
import queue
from io import BytesIO
import api



PROTOCOL_VERSION = 0b0001
DEFAULT_HEADER_SIZE = 0b0001

PROTOCOL_VERSION_BITS = 4
HEADER_BITS = 4
MESSAGE_TYPE_BITS = 4
MESSAGE_TYPE_SPECIFIC_FLAGS_BITS = 4
MESSAGE_SERIALIZATION_BITS = 4
MESSAGE_COMPRESSION_BITS = 4
RESERVED_BITS = 8

# Message Type:
CLIENT_FULL_REQUEST = 0b0001
CLIENT_AUDIO_ONLY_REQUEST = 0b0010
SERVER_FULL_RESPONSE = 0b1001
SERVER_ACK = 0b1011
SERVER_ERROR_RESPONSE = 0b1111

# Message Type Specific Flags
NO_SEQUENCE = 0b0000  # no check sequence
POS_SEQUENCE = 0b0001
NEG_SEQUENCE = 0b0010
NEG_SEQUENCE_1 = 0b0011

# Message Serialization
NO_SERIALIZATION = 0b0000
JSON = 0b0001
THRIFT = 0b0011
CUSTOM_TYPE = 0b1111

# Message Compression
NO_COMPRESSION = 0b0000
GZIP = 0b0001
CUSTOM_COMPRESSION = 0b1111


def generate_header(
    version=PROTOCOL_VERSION,
    message_type=CLIENT_FULL_REQUEST,
    message_type_specific_flags=NO_SEQUENCE,
    serial_method=JSON,
    compression_type=GZIP,
    reserved_data=0x00,
    extension_header=bytes()
):
    """
    protocol_version(4 bits), header_size(4 bits),
    message_type(4 bits), message_type_specific_flags(4 bits)
    serialization_method(4 bits) message_compression(4 bits)
    reserved （8bits) 保留字段
    header_extensions 扩展头(大小等于 8 * 4 * (header_size - 1) )
    """
    header = bytearray()
    header_size = int(len(extension_header) / 4) + 1
    header.append((version << 4) | header_size)
    header.append((message_type << 4) | message_type_specific_flags)
    header.append((serial_method << 4) | compression_type)
    header.append(reserved_data)
    header.extend(extension_header)
    return header


def generate_full_default_header():
    return generate_header()


def generate_audio_default_header():
    return generate_header(
        message_type=CLIENT_AUDIO_ONLY_REQUEST
    )


def generate_last_audio_default_header():
    return generate_header(
        message_type=CLIENT_AUDIO_ONLY_REQUEST,
        message_type_specific_flags=NEG_SEQUENCE
    )

def parse_response(res):
    """
    protocol_version(4 bits), header_size(4 bits),
    message_type(4 bits), message_type_specific_flags(4 bits)
    serialization_method(4 bits) message_compression(4 bits)
    reserved （8bits) 保留字段
    header_extensions 扩展头(大小等于 8 * 4 * (header_size - 1) )
    payload 类似与http 请求体
    """
    protocol_version = res[0] >> 4
    header_size = res[0] & 0x0f
    message_type = res[1] >> 4
    message_type_specific_flags = res[1] & 0x0f
    serialization_method = res[2] >> 4
    message_compression = res[2] & 0x0f
    reserved = res[3]
    header_extensions = res[4:header_size * 4]
    payload = res[header_size * 4:]
    result = {}
    payload_msg = None
    payload_size = 0
    if message_type == SERVER_FULL_RESPONSE:
        payload_size = int.from_bytes(payload[:4], "big", signed=True)
        payload_msg = payload[4:]
    elif message_type == SERVER_ACK:
        seq = int.from_bytes(payload[:4], "big", signed=True)
        result['seq'] = seq
        if len(payload) >= 8:
            payload_size = int.from_bytes(payload[4:8], "big", signed=False)
            payload_msg = payload[8:]
    elif message_type == SERVER_ERROR_RESPONSE:
        code = int.from_bytes(payload[:4], "big", signed=False)
        result['code'] = code
        payload_size = int.from_bytes(payload[4:8], "big", signed=False)
        payload_msg = payload[8:]
    if payload_msg is None:
        return result
    if message_compression == GZIP:
        payload_msg = gzip.decompress(payload_msg)
    if serialization_method == JSON:
        payload_msg = json.loads(str(payload_msg, "utf-8"))
    elif serialization_method != NO_SERIALIZATION:
        payload_msg = str(payload_msg, "utf-8")
    result['payload_msg'] = payload_msg
    result['payload_size'] = payload_size
    return result


def read_wav_info(data: bytes = None) -> (int, int, int, int, int):
    with BytesIO(data) as _f:
        wave_fp = wave.open(_f, 'rb')
        nchannels, sampwidth, framerate, nframes = wave_fp.getparams()[:4]
        wave_bytes = wave_fp.readframes(nframes)
    return nchannels, sampwidth, framerate, nframes, len(wave_bytes)


class AudioType(Enum):
    LOCAL = 1  # 使用本地音频文件
    MICROPHONE = 2  # 使用麦克风实时采集

class ASRClient:
    def __init__(self, appid:str, token:str,  **kwargs):
        self.audio_type = AudioType.MICROPHONE
        self.cluster = kwargs.get("cluster", "volcengine_streaming_common")
        self.success_code = 1000  # success code, default is 1000
        self.seg_duration = int(kwargs.get("seg_duration", 15000))
        self.nbest = int(kwargs.get("nbest", 1))
        self.appid = appid
        self.token = token
        self.ws_url = kwargs.get("ws_url", "wss://openspeech.bytedance.com/api/v2/asr")
        self.uid = kwargs.get("uid", "streaming_asr_demo")
        self.workflow = kwargs.get("workflow", "audio_in,resample,partition,vad,fe,decode,itn,nlu_punctuate")
        self.show_language = kwargs.get("show_language", False)
        self.show_utterances = kwargs.get("show_utterances", True)
        self.result_type = kwargs.get("result_type", "single") # single / full
        self.format = kwargs.get("format", "pcm")  # 使用原始PCM格式
        self.rate = kwargs.get("sample_rate", 16000)  # 采样率
        self.language = kwargs.get("language", "zh-CN")
        self.bits = kwargs.get("bits", 16)  # 位深度
        self.channel = kwargs.get("channel", 1)  # 声道数
        self.codec = "raw"  # 原始PCM格式
        self.secret = kwargs.get("secret", "access_secret")
        self.auth_method = kwargs.get("auth_method", "token")
        self.chunk_size = int(kwargs.get("chunk_size", 1600))  # 音频块大小
        self.audio_queue = queue.Queue()  # 音频数据队列
        self.stop_event = threading.Event()  # 停止事件

        self.recording_enabled = True
        self.result_queue = queue.Queue()

    def get_result(self) -> (str, bool):
        return self.result_queue.get(timeout=1.0)

    def enable_recording(self, flag : bool):
        self.recording_enabled = flag

    # ... [其他方法保持不变] ...
    def construct_request(self, reqid):
        req = {
            'app': {
                'appid': self.appid,
                'cluster': self.cluster,
                'token': self.token,
            },
            'user': {
                'uid': self.uid
            },
            'request': {
                'reqid': reqid,
                'nbest': self.nbest,
                'workflow': self.workflow,
                'show_language': self.show_language,
                'show_utterances': self.show_utterances,
                'result_type': self.result_type,
                "sequence": 1
            },
            'audio': {
                'format': self.format,
                'rate': self.rate,
                'language': self.language,
                'bits': self.bits,
                'channel': self.channel,
                'codec': self.codec
            }
        }
        return req

    @staticmethod
    def slice_data(data: bytes, chunk_size: int) -> (list, bool):
        """
        slice data
        :param data: wav data
        :param chunk_size: the segment size in one request
        :return: segment data, last flag
        """
        data_len = len(data)
        offset = 0
        while offset + chunk_size < data_len:
            yield data[offset: offset + chunk_size], False
            offset += chunk_size
        else:
            yield data[offset: data_len], True

    def _real_processor(self, request_params: dict) -> dict:
        pass

    def token_auth(self):
        return {'Authorization': 'Bearer; {}'.format(self.token)}

    def signature_auth(self, data):
        header_dicts = {
            'Custom': 'auth_custom',
        }

        url_parse = urlparse(self.ws_url)
        input_str = 'GET {} HTTP/1.1\n'.format(url_parse.path)
        auth_headers = 'Custom'
        for header in auth_headers.split(','):
            input_str += '{}\n'.format(header_dicts[header])
        input_data = bytearray(input_str, 'utf-8')
        input_data += data
        mac = base64.urlsafe_b64encode(
            hmac.new(self.secret.encode('utf-8'), input_data, digestmod=sha256).digest())
        header_dicts['Authorization'] = 'HMAC256; access_token="{}"; mac="{}"; h="{}"'.format(self.token,
                                                                                              str(mac, 'utf-8'), auth_headers)
        return header_dicts

    async def segment_data_processor(self, wav_data: bytes, segment_size: int):
        reqid = str(uuid.uuid4())
        # 构建 full client request，并序列化压缩
        request_params = self.construct_request(reqid)
        payload_bytes = str.encode(json.dumps(request_params))
        payload_bytes = gzip.compress(payload_bytes)
        full_client_request = bytearray(generate_full_default_header())
        full_client_request.extend((len(payload_bytes)).to_bytes(4, 'big'))  # payload size(4 bytes)
        full_client_request.extend(payload_bytes)  # payload
        header = None
        if self.auth_method == "token":
            header = self.token_auth()
        elif self.auth_method == "signature":
            header = self.signature_auth(full_client_request)
        async with websockets.connect(self.ws_url, extra_headers=header, max_size=1000000000) as ws:
            # 发送 full client request
            await ws.send(full_client_request)
            res = await ws.recv()
            result = parse_response(res)
            if 'payload_msg' in result and result['payload_msg']['code'] != self.success_code:
                return result
            for seq, (chunk, last) in enumerate(ASRClient.slice_data(wav_data, segment_size), 1):
                # if no compression, comment this line
                payload_bytes = gzip.compress(chunk)
                audio_only_request = bytearray(generate_audio_default_header())
                if last:
                    audio_only_request = bytearray(generate_last_audio_default_header())
                audio_only_request.extend((len(payload_bytes)).to_bytes(4, 'big'))  # payload size(4 bytes)
                audio_only_request.extend(payload_bytes)  # payload
                # 发送 audio-only client request
                await ws.send(audio_only_request)
                res = await ws.recv()
                result = parse_response(res)
                if 'payload_msg' in result and result['payload_msg']['code'] != self.success_code:
                    return result
        return result

    async def execute(self):
        with open(self.audio_path, mode="rb") as _f:
            data = _f.read()
        audio_data = bytes(data)
        if self.format == "mp3":
            segment_size = self.mp3_seg_size
            return await self.segment_data_processor(audio_data, segment_size)
        if self.format != "wav":
            raise Exception("format should in wav or mp3")
        nchannels, sampwidth, framerate, nframes, wav_len = read_wav_info(
            audio_data)
        size_per_sec = nchannels * sampwidth * framerate
        segment_size = int(size_per_sec * self.seg_duration / 1000)
        return await self.segment_data_processor(audio_data, segment_size)


    def start_microphone_recording(self):
        """启动麦克风录制线程"""
        p = pyaudio.PyAudio()

        # 查找合适的输入设备
        device_index = None
        for i in range(p.get_device_count()):
            dev_info = p.get_device_info_by_index(i)
            if dev_info['maxInputChannels'] > 0 and dev_info['hostApi'] == 0:
                device_index = i
                break

        if device_index is None:
            print("未找到可用的输入设备")
            return

        stream = p.open(
            format=pyaudio.paInt16,
            channels=self.channel,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk_size,
        )

        print("开始录制... (按Ctrl+C停止)")
        while not self.stop_event.is_set():
            try:
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                if self.recording_enabled:
                    self.audio_queue.put(data)
            except IOError as e:
                # 输入溢出，跳过此块
                print("输入溢出，跳过此块")
                continue

        stream.stop_stream()
        stream.close()
        p.terminate()
        print("录制已停止")

    def audio_generator(self):
        """生成音频数据的生成器"""
        while not self.stop_event.is_set() or not self.audio_queue.empty():
            try:
                # 设置超时，避免永久阻塞
                data = self.audio_queue.get(timeout=0.5)
                yield data, False
            except queue.Empty:
                if self.stop_event.is_set():
                    # 发送结束信号
                    yield b'', True
                    return
                continue

        # 发送结束信号
        yield b'', True

    async def realtime_asr(self):
        """实时语音识别"""
        # 启动麦克风录制线程
        record_thread = threading.Thread(target=self.start_microphone_recording)
        record_thread.daemon = True
        record_thread.start()

        try:
            # 构建请求
            reqid = str(uuid.uuid4())
            request_params = self.construct_request(reqid)
            payload_bytes = str.encode(json.dumps(request_params))
            payload_bytes = gzip.compress(payload_bytes)
            full_client_request = bytearray(generate_full_default_header())
            full_client_request.extend((len(payload_bytes)).to_bytes(4, 'big'))
            full_client_request.extend(payload_bytes)

            # 认证头
            header = self.token_auth() if self.auth_method == "token" else self.signature_auth(full_client_request)

            async with websockets.connect(self.ws_url, extra_headers=header, max_size=1000000000) as ws:
                # 发送初始化请求
                await ws.send(full_client_request)
                res = await ws.recv()
                result = parse_response(res)

                # 检查初始化响应
                if 'payload_msg' in result and result['payload_msg']['code'] != self.success_code:
                    print(f"初始化失败: {result['payload_msg']}")
                    return

                print("连接已建立，开始语音识别...")

                # 创建音频生成器
                audio_gen = self.audio_generator()

                # 任务列表：发送音频和接收结果
                tasks = [
                    asyncio.create_task(self.send_audio_data(ws, audio_gen)),
                    asyncio.create_task(self.receive_results(ws))
                ]

                # 等待所有任务完成
                await asyncio.gather(*tasks)
        finally:
            self.stop_event.set()
            record_thread.join(timeout=1.0)

    async def send_audio_data(self, ws, audio_gen):
        """发送音频数据"""
        try:
            for data, last in audio_gen:
                # 压缩音频数据
                payload_bytes = gzip.compress(data) if data else b''

                # 构建音频请求
                if last:
                    audio_only_request = bytearray(generate_last_audio_default_header())
                else:
                    audio_only_request = bytearray(generate_audio_default_header())

                audio_only_request.extend((len(payload_bytes)).to_bytes(4, 'big'))
                audio_only_request.extend(payload_bytes)

                # 发送音频数据
                await ws.send(audio_only_request)

                # 控制发送速率
                await asyncio.sleep(0.05)
        except Exception as e:
            print(f"发送音频数据时出错: {e}")
        finally:
            # 确保发送结束标志
            if not self.stop_event.is_set():
                self.stop_event.set()

    async def receive_results(self, ws):
        """接收并处理识别结果"""
        try:
            while not self.stop_event.is_set():
                res = await asyncio.wait_for(ws.recv(), timeout=1.0)
                result = parse_response(res)

                # 处理识别结果
                if 'payload_msg' in result:
                    payload = result['payload_msg']
                    # 检查错误
                    if 'code' in payload and payload['code'] != self.success_code:
                        print(f"识别错误: {payload}")
                        continue
                    #print("paly load: ", payload)
                    # 输出识别结果
                    if 'result' in payload:
                        # 提取并打印识别文本
                        segments = payload.get('result', [])
                        is_end = False
                        internal_res = ""
                        for segment in segments:
                            text = segment.get('text', '')
                            if text:  # 只打印非空结果
                                internal_res = text
                            utterances = segment.get('utterances', [])
                            for utterance in utterances:
                                result_type = utterance.get('definite', [])
                                if result_type:
                                    is_end = True
                        if internal_res:
                            self.result_queue.put((internal_res, is_end))
                            #print(f"识别结果: {internal_res}, {is_end}")

        except asyncio.TimeoutError:
            # 超时是正常的，继续等待
            pass
        except Exception as e:
            print(f"接收结果时出错: {e}")
        finally:
            if not self.stop_event.is_set():
                self.stop_event.set()


def consume_result_queue():
    """<UNK>"""
    result_queue = queue.Queue()

def realtime_asr_test( **kwargs):
    """实时语音识别测试"""
    asr_client = ASRClient(**kwargs)

    # 启动结果处理线程
    def result_processing_loop():
        """在另一个线程中循环处理结果"""
        try:
            while not asr_client.stop_event.is_set():
                try:
                    result = asr_client.get_result()
                    if result is not None:
                        #print(f"result: {result}")
                        # 处理结果
                        text, is_end = result
                        print(f"获取结果: {text}， {is_end}")
                        # is_end 为 true 是一句话结束
                    else:
                        print(f"get empty result: {result}")
                except Exception as e:
                    time.sleep(0.1)  # 避免错误循环
        finally:
            print("结果处理线程结束")

    # 启动处理线程
    process_thread = threading.Thread(
        target=result_processing_loop,
        name="ResultProcessor",
        daemon=True
    )
    process_thread.start()

    # 创建新的事件循环
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)


    try:
        loop.run_until_complete(asr_client.realtime_asr())
    except KeyboardInterrupt:
        print("\n用户中断，停止识别")
        asr_client.stop_event.set()
    finally:
        # 清理资源
        tasks = asyncio.all_tasks(loop)
        for task in tasks:
            task.cancel()
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()


if __name__ == '__main__':
    realtime_asr_test(
        appid=api.APP_ID,
        token=api.APP_TOKEN,
        sample_rate=16000,  # 16kHz采样率
        bits=16,  # 16位深度
        channel=1,  # 单声道
        chunk_size=3200,  # 每次发送3200字节（约100ms音频）
        ws_url="wss://volc.vcloud.cn/ws/v1"
    )