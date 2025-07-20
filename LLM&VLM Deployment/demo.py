import asyncio
import threading
import queue
import signal
import sys
import time
from volcenginesdkarkruntime import AsyncArk
from prompt import SYSTEM_PROMPT
import api
import difflib

class VoiceAssistant:
    def __init__(self):
        # 初始化LLM客户端
        self.llm_client = AsyncArk(api_key=api.ARK_API_KEY)
        
        # 初始化TTS客户端
        from tts import TTSClient
        self.tts_client = TTSClient(appid=api.APP_ID, token=api.APP_TOKEN)
        
        # 初始化ASR客户端
        from asr import ASRClient
        self.asr_client = ASRClient(
            appid=api.APP_ID, 
            token=api.APP_TOKEN,
            sample_rate=16000,
            bits=16,
            channel=1,
            chunk_size=3200
        )
        
        # 控制标志
        self.is_running = True
        self.is_tts_playing = False  # TTS播放标志位
        self.conversation_history = [
            {"role": "system", "content": SYSTEM_PROMPT}
        ]
        
        # ASR线程对象
        self.asr_thread_obj = None
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        
        self.last_tts_text = ""  # 记录上一次TTS播报内容
    
    def _signal_handler(self, sig, frame):
        print("\n正在退出...")
        self.is_running = False
        self.asr_client.stop_event.set()
        self.tts_client.clean_up()
        sys.exit(0)
    
    async def get_llm_response(self, user_input: str) -> str:
        """获取LLM回复"""
        # 添加用户输入到对话历史
        self.conversation_history.append({"role": "user", "content": user_input})
        
        try:
            stream = await self.llm_client.chat.completions.create(
                model=api.LLM_MODEL,
                messages=self.conversation_history,
                stream=True
            )
            
            response = ""
            async for completion in stream:
                if completion.choices[0].delta.content:
                    content = completion.choices[0].delta.content
                    response += content
                    print(content, end="", flush=True)
            
            print()  # 换行
            
            # 添加助手回复到对话历史
            self.conversation_history.append({"role": "assistant", "content": response})
            
            return response
            
        except Exception as e:
            error_msg = f"LLM请求失败: {e}"
            print(error_msg)
            return error_msg
    
    async def text_to_speech(self, text: str):
        """文本转语音"""
        try:
            # 设置TTS播放标志位
            self.is_tts_playing = True
            print("TTS开始播放，ASR识别已暂停...")
            
            # 停止ASR识别
            if hasattr(self, 'asr_thread_obj') and self.asr_thread_obj and self.asr_thread_obj.is_alive():
                self.asr_client.stop_event.set()
                self.asr_thread_obj.join(timeout=1.0)
            
            # 停止麦克风录制
            self.asr_client.enable_recording(False)
            
            # 清空音频队列
            while not self.asr_client.audio_queue.empty():
                try:
                    self.asr_client.audio_queue.get_nowait()
                except queue.Empty:
                    break
            
            self.last_tts_text = text  # 记录TTS内容
            await self.tts_client.request_once(text)
            
            # 根据文本长度动态等待，假设TTS语速为5字/秒
            char_count = len(text)
            tts_speed = 5  # 每秒5字
            min_wait = 1.0  # 最小等待1秒
            max_wait = 100.0 # 最大等待10秒
            wait_time = min(max(char_count / tts_speed, min_wait), max_wait)
            await asyncio.sleep(wait_time)
            
            # 清除TTS播放标志位
            self.is_tts_playing = False
            print("TTS播放完成，ASR识别已恢复...")
            
        except Exception as e:
            self.is_tts_playing = False
            print(f"TTS失败: {e}")
    
    def is_similar_to_tts(self, text):
        # 关键词过滤
        keywords = ["乔青青", "很高兴和你聊天", "我是乔青青"]
        for kw in keywords:
            if kw in text:
                return True
        # 相似度过滤
        if self.last_tts_text:
            seq = difflib.SequenceMatcher(None, text, self.last_tts_text)
            if seq.ratio() > 0.7:  # 相似度阈值可调整
                return True
        return False

    def speech_to_text(self) -> str:
        """语音转文本 - 使用同步方式"""
        try:
            print("请说话...")
            
            # 等待一下，确保TTS播放完成
            time.sleep(1.0)
            
            # 重置ASR客户端状态
            self.asr_client.stop_event.clear()
            
            # 启用麦克风录制
            self.asr_client.enable_recording(True)
            
            # 启动ASR识别
            def asr_thread():
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                try:
                    loop.run_until_complete(self.asr_client.realtime_asr())
                except Exception as e:
                    print(f"ASR错误: {e}")
                finally:
                    loop.close()
            
            # 在后台线程运行ASR
            self.asr_thread_obj = threading.Thread(target=asr_thread, daemon=True)
            self.asr_thread_obj.start()
            
            # 等待识别结果
            last_text = ""
            start_time = time.time()
            while self.is_running and (time.time() - start_time) < 10:  # 最多等待10秒
                try:
                    # 检查TTS是否正在播放
                    if self.is_tts_playing:
                        time.sleep(0.1)
                        continue
                    
                    result = self.asr_client.get_result()
                    if result is not None:
                        text, is_end = result
                        if text and text != last_text:
                            # 过滤助手TTS内容
                            if self.is_similar_to_tts(text):
                                print(f"过滤掉助手TTS内容: {text}")
                                last_text = text
                                continue
                            print(f"识别到: {text}")
                            last_text = text
                            if is_end:
                                return text
                    time.sleep(0.1)
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"获取识别结果错误: {e}")
                    break
            
            return ""
            
        except Exception as e:
            print(f"ASR失败: {e}")
            return ""
    
    async def conversation_loop(self):
        """主对话循环"""
        print("=== 深蓝AI语音助手 ===")
        print("开始语音对话，请说话...")
        print("按 Ctrl+C 退出")
        print("-" * 30)
        
        # 播放欢迎语音
        welcome_text = "你好！我是深蓝学院的阿兰，很高兴和你聊天！"
        print(f"助手: {welcome_text}")
        await self.text_to_speech(welcome_text)
        
        while self.is_running:
            try:
                # 语音识别
                user_input = self.speech_to_text()
                
                if not user_input:
                    print("没有识别到语音，请重试")
                    continue
                
                print(f"用户: {user_input}")
                
                # 检查退出指令
                if any(keyword in user_input for keyword in ["退出", "再见", "拜拜", "结束"]):
                    goodbye_text = "再见！下次再聊哦！"
                    print(f"助手: {goodbye_text}")
                    await self.text_to_speech(goodbye_text)
                    break
                
                # 获取LLM回复
                print("助手: ", end="")
                response = await self.get_llm_response(user_input)
                
                # 语音合成
                await self.text_to_speech(response)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"对话出错: {e}")
                continue
        
        print("\n对话结束")

async def main():
    """主函数"""
    assistant = VoiceAssistant()
    await assistant.conversation_loop()

if __name__ == "__main__":
    asyncio.run(main())