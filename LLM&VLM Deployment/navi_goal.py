#!/usr/bin/env python3
# coding=utf-8
"""
Voice-controlled Navigation Client

This script integrates ASR, LLM, and TTS to publish navigation goals
to a ROS topic when hearing `--Nav(x,y)` commands.
"""
import asyncio
import threading
import queue
import signal
import sys
import time
import re

import rospy
from geometry_msgs.msg import PoseStamped

from volcenginesdkarkruntime import AsyncArk
from prompt import SYSTEM_PROMPT
import api
from tts import TTSClient
from asr import ASRClient

class VoiceAssistant:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('navi_goal', anonymous=True, disable_signals=True)
        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=1)

        # Initialize LLM client
        self.llm_client = AsyncArk(api_key=api.ARK_API_KEY)
        # Initialize TTS client
        self.tts_client = TTSClient(appid=api.APP_ID, token=api.APP_TOKEN)
        # Initialize ASR client
        self.asr_client = ASRClient(
            appid=api.APP_ID,
            token=api.APP_TOKEN,
            sample_rate=16000,
            bits=16,
            channel=1,
            chunk_size=3200
        )
        # Control flags
        self.is_running = True
        self.is_tts_playing = False
        self.conversation_history = [
            {"role": "system", "content": SYSTEM_PROMPT}
        ]
        self.asr_thread_obj = None
        self.last_tts_text = ""

        # Handle SIGINT
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, sig, frame):
        print("\nShutting down...")
        self.is_running = False
        self.asr_client.stop_event.set()
        self.tts_client.clean_up()
        rospy.signal_shutdown('User exit')
        sys.exit(0)

    def send_goal(self, x, y):
        """Publish a navigation goal to ROS"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        print(f"Published navigation goal: ({x}, {y})")

    async def get_llm_response(self, user_input: str) -> str:
        # Append user input
        self.conversation_history.append({"role": "user", "content": user_input})
        try:
            stream = await self.llm_client.chat.completions.create(
                model=api.LLM_MODEL,
                messages=self.conversation_history,
                stream=True
            )
            response = ""
            async for chunk in stream:
                delta = chunk.choices[0].delta.content
                if delta:
                    response += delta
                    print(delta, end="", flush=True)
            print()
            self.conversation_history.append({"role": "assistant", "content": response})
            return response
        except Exception as e:
            err = f"LLM request failed: {e}"
            print(err)
            return err

    async def text_to_speech(self, text: str):
        self.is_tts_playing = True
        print("TTS playing... ASR paused.")
        if self.asr_thread_obj and self.asr_thread_obj.is_alive():
            self.asr_client.stop_event.set()
            self.asr_thread_obj.join(timeout=1.0)
        self.asr_client.enable_recording(False)
        while not self.asr_client.audio_queue.empty():
            try: self.asr_client.audio_queue.get_nowait()
            except queue.Empty: break
        self.last_tts_text = text
        await self.tts_client.request_once(text)
        # Estimate wait time
        wait_time = min(max(len(text) / 5, 1.0), 10.0)
        await asyncio.sleep(wait_time)
        self.is_tts_playing = False
        print("TTS finished. ASR resumed.")

    def is_similar_to_tts(self, text):
        keywords = ["乔青青", "很高兴和你聊天", "我是乔青青"]
        for kw in keywords:
            if kw in text:
                return True
        if self.last_tts_text:
            import difflib
            ratio = difflib.SequenceMatcher(None, text, self.last_tts_text).ratio()
            if ratio > 0.7:
                return True
        return False

    def speech_to_text(self) -> str:
        print("Please speak...")
        time.sleep(1.0)
        self.asr_client.stop_event.clear()
        self.asr_client.enable_recording(True)
        def asr_thread():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self.asr_client.realtime_asr())
            except Exception as e:
                print(f"ASR error: {e}")
            finally:
                loop.close()
        self.asr_thread_obj = threading.Thread(target=asr_thread, daemon=True)
        self.asr_thread_obj.start()
        last_text = ""
        start = time.time()
        while self.is_running and (time.time() - start) < 10:
            if self.is_tts_playing:
                time.sleep(0.1)
                continue
            try:
                result = self.asr_client.get_result()
                if result:
                    text, is_end = result
                    if text and text != last_text and not self.is_similar_to_tts(text):
                        print(f"Heard: {text}")
                        last_text = text
                        if is_end:
                            return text
            except Exception:
                pass
            time.sleep(0.1)
        return ""

    async def conversation_loop(self):
        print("=== Voice Navigation Assistant ===")
        print("Speak to set a navigation goal. Ctrl+C to exit.")
        # Welcome message
        welcome = "你好，我是导航助手。请告诉我目的地。"
        print(f"Assistant: {welcome}")
        await self.text_to_speech(welcome)
        while self.is_running and not rospy.is_shutdown():
            user_input = self.speech_to_text()
            if not user_input:
                print("Didn't catch that, please try again.")
                continue
            print(f"User: {user_input}")
            if any(k in user_input for k in ["退出","再见","结束"]):
                bye = "导航结束，再见！"
                print(f"Assistant: {bye}")
                await self.text_to_speech(bye)
                break
            # Get LLM response
            print("Assistant: ", end="")
            response = await self.get_llm_response(user_input)
            # Detect navigation command
            nav = re.search(r"--Nav\(([-\d\.]+),([-\d\.]+)\)", response)
            if nav:
                x, y = float(nav.group(1)), float(nav.group(2))
                self.send_goal(x, y)
            await self.text_to_speech(response)
        print("Conversation ended.")

async def main():
    assistant = VoiceAssistant()
    await assistant.conversation_loop()

if __name__ == '__main__':
    asyncio.run(main())
