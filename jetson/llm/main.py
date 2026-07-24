#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import logging
import socket

# --- [1. Python 3.8 로깅 오류 완벽 해결 패치] ---
def _universal_findCaller(self, *args, **kwargs):
    try:
        f = sys._getframe(3)
        if f:
            co = f.f_code
            return (co.co_filename, f.f_lineno, co.co_name, None)
    except Exception:
        pass
    return ("(unknown file)", 0, "(unknown function)", None)

logging.Logger.findCaller = _universal_findCaller

# --- [2. ROS Melodic 경로 설정] ---
ros_paths = [
    '/usr/lib/python2.7/dist-packages',
    '/opt/ros/melodic/lib/python2.7/dist-packages'
]
for path in ros_paths:
    if os.path.exists(path) and path not in sys.path:
        sys.path.append(path)

import rospy 
import json
import threading
import time
from std_msgs.msg import String
from dotenv import load_dotenv

import llm_module2
import realtime_stt2
import tts_module2
import config

LLM_SUPPORT_DIR = os.environ.get(
    "LLM_DIR", os.path.join(os.path.expanduser("~"), "llm"))
load_dotenv(os.path.join(LLM_SUPPORT_DIR, ".env"))
load_dotenv()

class VoiceControlNode:
    def __init__(self):
        logging.getLoggerClass().findCaller = _universal_findCaller
        rospy.init_node('voice_control_node', anonymous=True)
        
        self.command_pub = rospy.Publisher('/llm_command', String, queue_size=10)
        self.tts_event_pub = rospy.Publisher('/tts_event', String, queue_size=10)
        self.status_sub = rospy.Subscriber('/robot_status', String, self.status_callback)

        self.current_robot_state = "IDLE"
        self.current_target_room = ""    
        self.delivered_history = []       
        self.target_mission = []          
        self.is_listening_paused = False 
        self.llm_client = llm_module2.get_llm_client()
        if self.llm_client is None:
            raise RuntimeError(
                "OpenAI client initialization failed. Check "
                "{}/.env and OPENAI_API_KEY.".format(LLM_SUPPORT_DIR))
        self.last_speech_time = 0
        self.stt_engine = None
        self.speech_lock = threading.Lock() # 추가됨: 음성 씹힘 방지용 스레드 락

        self.pdf_responses = {
            "SCENARIO_1": "네, '{}호'로 '택배' 배송 명령을 접수했습니다. 출발하겠습니다.",
            "SCENARIO_2": "알겠습니다. 첫 번째 목적지는 {}호 두 번째 목적지는 {}호로 출발하겠습니다.",
            "SCENARIO_3": "세 곳의 목적지를 순차 배송으로 설정했습니다. 출발하겠습니다.",
            "SCENARIO_4": "두 곳의 목적지를 순차 배송으로 설정합니다. 출발하겠습니다.",
            "SCENARIO_5": "{}호에 도착했습니다. 요청하신 택배입니다. 보관함에서 물건을 꺼내 주세요.",
            "SCENARIO_6": "알겠습니다. 목적지를 알려주세요.", 
            "SCENARIO_6_ASK": "알겠습니다. 목적지를 알려주세요.", 
            "SCENARIO_8": "물품 수령이 확인되었습니다. 다음 목적지로 이동합니다.",
            # [요청 반영] 멘트 수정 금지: 텍스트는 그대로 유지합니다.
            "SCENARIO_9": "배송을 모두 완료했습니다. 이제 처음 출발했던 장소로 복귀합니다.",
            # [요청 반영] 시나리오 10, 11, 12, 14 완전 삭제
            "SCENARIO_13": "수령인이 부재중입니다. 물건을 배송 실패함으로 등록하고 다음 목적지로 이동합니다.",
            "SCENARIO_16_MOVING": "네, 현재 '{}호'로 택배를 배송하는 중입니다.",
            "SCENARIO_16_RETURNING": "네, 현재 모든 배송을 마치고 출발지로 복귀하는 중입니다.",
            "SCENARIO_16_IDLE": "현재 로봇은 대기 중이며 택배 배송 명령을 기다리고 있습니다.",
            "SCENARIO_17_DONE": "네, 이전에 배송한 {}호 배송은 완료되었습니다.",
            "SCENARIO_17_PROGRESS": "네, {}호 배송은 완료되었고, 지금은 다음 목적지인 {}호로 이동하고 있습니다.",
            "SCENARIO_17_FIRST_MOVE": "네, 현재 첫 번째 목적지인 {}호로 이동하고 있습니다. 아직 완료된 배송은 없습니다.",
            "SCENARIO_17_ALL_DONE": "네, 요청하신 {} 배송을 모두 완료했습니다.",
            "SCENARIO_WAITING_FOR_USER": "{}호에서 현재 물품 수령을 위해 대기 중입니다.", 
            "SCENARIO_18": "네, 복도 끝에는 '531호'와 '532a호', '532b호'가 있습니다. 어느 곳으로 배송할까요?",
            "SCENARIO_19": "알겠습니다. 직전 배송지인 '{}호'로 다시 이동하겠습니다.",
            "SCENARIO_20": "네, 배송을 시작하겠습니다. 목적지가 어디인가요?",

            "SCENARIO_21": "네, 정지했습니다.", 
            "SCENARIO_22": "이동을 다시 시작합니다.",
            
            "INVALID_ROOM": "죄송합니다. 요청하신 ‘{}호’는 배송이 불가능한 구역입니다.다시 말씀해주세요."
        }

    def status_callback(self, msg):
        raw_msg = msg.data.strip().upper()
        # [요청 반영] 불필요한 시나리오 10, 11, 12, 14 리스트에서 제거
        manual_actions = ["SCENARIO_13"]
        if raw_msg in manual_actions:
            self.play_robot_speech(self.pdf_responses[raw_msg])
            return

        if ":" in raw_msg:
            state, room = raw_msg.split(":")
            self.current_robot_state = state
            self.current_target_room = room 
            if state == "ARRIVED" and room not in self.delivered_history:
                self.delivered_history.append(room)
        else:
            # --- [안전장치 1] 내가 정지시켰는데 자율주행 노드가 멋대로 IDLE을 보내면 무시! ---
            if raw_msg == "IDLE" and self.current_robot_state == "PAUSED":
                rospy.loginfo(" [상태 방어] 정지 상태이므로 자율주행 노드의 IDLE 신호를 무시하고 목적지를 기억합니다.")
                return
            # -------------------------------------------------------------------------

            self.current_robot_state = raw_msg
            if raw_msg in ["RETURNING", "IDLE"]: 
                self.current_target_room = ""
                if raw_msg == "IDLE":
                    self.target_mission = []
                    
                    # --- 초기 위치로 복귀 시 호출어 수면 모드로 전환 ---
                    if self.stt_engine:
                        self.stt_engine.is_awake = False
                        rospy.loginfo("[상태 전환] 초기 위치 복귀 완료. 호출어 수면 모드로 전환됩니다.")
                    # -------------------------------------------------------------

        if raw_msg.startswith("SCENARIO_"):
            if time.time() - self.last_speech_time < 2.0: return 
            if raw_msg == "SCENARIO_5":
                self.play_robot_speech(
                    self.pdf_responses["SCENARIO_5"].format(
                        self.current_target_room),
                    completion_event="SCENARIO_5_DONE")
            elif raw_msg in self.pdf_responses:
                self.play_robot_speech(self.pdf_responses[raw_msg])

    def play_robot_speech(self, text, completion_event=None):
        self.is_listening_paused = True
        self.last_speech_time = time.time()
        rospy.loginfo(f"🔊 [로봇]: {text}")
        
        # --- 17개 다운로드 파일 자동 매칭 스위치 ---
        text_to_key = {v: k for k, v in self.pdf_responses.items()}
        
        def speak():
            # 추가됨: 스레드 락을 걸어 음성이 연속으로 들어와도 앞 음성이 끝날 때까지 대기
            with self.speech_lock:
                if self.stt_engine: self.stt_engine.set_pause(True)
                
                played_local = False
                
                if text in text_to_key:
                    scenario_key = text_to_key[text]
                    wav_path = os.path.join("sounds", f"{scenario_key}.wav")
                    
                    if os.path.exists(wav_path):
                        os.system(f"aplay {wav_path} -q 2>/dev/null")
                        played_local = True
                
                if not played_local:
                    tts_module2.speak(text, self.llm_client)
                    
                if self.stt_engine: self.stt_engine.set_pause(False)
                self.is_listening_paused = False
                if completion_event:
                    self.tts_event_pub.publish(completion_event)
                    rospy.loginfo(
                        f"[TTS event] published {completion_event}")
            
        threading.Thread(target=speak, daemon=True).start()

    def on_transcription_received(self, text):
        if self.is_listening_paused: return
        
        json_cmd = llm_module2.parse_command_to_json(text, self.llm_client)
        scenario = json_cmd.get("command", "UNKNOWN")
        payload = json_cmd.get("payload", [])
        if not isinstance(payload, list): payload = [str(payload)] if payload else []
        interpretation = json_cmd.get("interpretation", "의도를 해석하는 중...")

        print(f" [llm최종해석]: \"{interpretation}\"") 

        if scenario == "UNKNOWN": return 

        busy_states = ["MOVING", "RETURNING", "ARRIVED", "SCENARIO_5", "SCENARIO_8"]
        is_busy = any(bs in self.current_robot_state for bs in busy_states)
        
        # [수정됨] 수령 확인(SCENARIO_8)은 작업 중이어도 차단하지 않도록 예외 처리 추가
        if is_busy and scenario not in ["SCENARIO_16", "SCENARIO_17", "SCENARIO_21", "SCENARIO_8"]:
            rospy.loginfo(f"🚫 [상태 알림] 로봇이 작업 중({self.current_robot_state})이라 명령을 수행할 수 없습니다.")
            return

        if scenario in ["SCENARIO_1", "SCENARIO_2", "SCENARIO_3", "SCENARIO_4"]:
            clean_payload = [str(p).lower().replace("호", "").strip() for p in payload if p]
            
            if not all(r in llm_module2.VALID_ROOMS for r in clean_payload):
                invalid_rooms = [r for r in clean_payload if r not in llm_module2.VALID_ROOMS]
                self.play_robot_speech(self.pdf_responses["INVALID_ROOM"].format(", ".join(invalid_rooms)))
                return 

            if not clean_payload: return

            # --- [실제 로봇 버그 완벽 해결 1] ---
            # 자율주행 노드가 '정지' 상태로 잠겨있다면 새 목적지를 무시해버리므로
            # 잠금 해제(22번) 신호를 먼저 보낸 뒤 새 목적지를 주입합니다.
            if self.current_robot_state == "PAUSED":
                self.command_pub.publish(json.dumps({"command": "SCENARIO_22", "payload": []}))
                time.sleep(0.2) # 자율주행 노드가 잠금을 풀 시간을 아주 잠깐 줌
            # -----------------------------------

            self.target_mission = clean_payload
            self.delivered_history = []
            
            # --- [수정된 부분] LLM이 시나리오 번호를 잘못 분류해도(ex. 2곳인데 SCENARIO_1 리턴), 
            # 목적지 개수(len)에 맞춰 정확한 음성과 시나리오를 강제로 올바르게 매칭하도록 변경 ---
            if len(clean_payload) == 1: 
                scenario = "SCENARIO_1"
                resp = self.pdf_responses["SCENARIO_1"].format(clean_payload[0])
            elif len(clean_payload) == 2: 
                scenario = "SCENARIO_2"
                resp = self.pdf_responses["SCENARIO_2"].format(clean_payload[0], clean_payload[1])
            else: 
                scenario = "SCENARIO_3"
                resp = self.pdf_responses["SCENARIO_3"]
            # ------------------------------------------------------------------------------------------

            self.play_robot_speech(resp)
            self.command_pub.publish(json.dumps({"command": scenario, "payload": clean_payload}))

        elif scenario == "SCENARIO_6":
            if not self.target_mission:
                self.play_robot_speech(self.pdf_responses["SCENARIO_6_ASK"])
            else:
                self.play_robot_speech(self.pdf_responses["SCENARIO_6"])
                self.command_pub.publish(json.dumps({"command": "SCENARIO_6", "payload": self.target_mission}))

        elif scenario == "SCENARIO_17":
            if self.current_robot_state == "RETURNING":
                text = self.pdf_responses["SCENARIO_16_RETURNING"]
            elif "ARRIVED" in self.current_robot_state or self.current_robot_state == "SCENARIO_5":
                text = self.pdf_responses["SCENARIO_WAITING_FOR_USER"].format(self.current_target_room)
            elif self.current_robot_state == "IDLE":
                hist = ", ".join(self.target_mission) if self.target_mission else "이전"
                text = self.pdf_responses["SCENARIO_17_ALL_DONE"].format(hist)
            elif "MOVING" in self.current_robot_state:
                if not self.delivered_history:
                    text = self.pdf_responses["SCENARIO_17_FIRST_MOVE"].format(self.current_target_room)
                else:
                    text = self.pdf_responses["SCENARIO_17_PROGRESS"].format(self.delivered_history[-1], self.current_target_room)
            else: 
                text = "현재 배송 작업을 진행 중입니다."
            self.play_robot_speech(text)

        elif scenario == "SCENARIO_16":
            if "ARRIVED" in self.current_robot_state or self.current_robot_state == "SCENARIO_5":
                text = self.pdf_responses["SCENARIO_WAITING_FOR_USER"].format(self.current_target_room)
            elif self.current_robot_state == "RETURNING": 
                text = self.pdf_responses["SCENARIO_16_RETURNING"]
            elif self.current_robot_state == "IDLE": 
                text = self.pdf_responses["SCENARIO_16_IDLE"]
            else: 
                text = self.pdf_responses["SCENARIO_16_MOVING"].format(self.current_target_room)
            self.play_robot_speech(text)

        elif scenario == "SCENARIO_19":
            if self.delivered_history:
                last_room = self.delivered_history[-1]
                text = self.pdf_responses["SCENARIO_19"].format(last_room)
                self.play_robot_speech(text)
                self.command_pub.publish(json.dumps({"command": "SCENARIO_1", "payload": [last_room]}))
            else:
                self.play_robot_speech("이전에 배송한 기록이 없습니다. 목적지를 말씀해 주세요.")

        elif scenario == "SCENARIO_20": 
            self.play_robot_speech(self.pdf_responses["SCENARIO_20"])
            
        elif scenario == "SCENARIO_21":
            self.play_robot_speech(self.pdf_responses["SCENARIO_21"])
            self.command_pub.publish(json.dumps({"command": "SCENARIO_21", "payload": []}))
            # --- [안전장치 2] 명령 접수 즉시 AI 노드의 상태를 '정지'로 강제 고정 ---
            self.current_robot_state = "PAUSED"
            # -------------------------------------------------------------------------
            
        elif scenario == "SCENARIO_22":
            if self.current_robot_state == "PAUSED":
                self.play_robot_speech(self.pdf_responses["SCENARIO_22"])
                self.command_pub.publish(json.dumps({
                    "command": "SCENARIO_22",
                    "payload": []
                }))
            else:
                rospy.loginfo("🚫 [명령 차단] 현재 정지(PAUSED) 상태가 아니므로 재출발 명령을 무시합니다.")

        # --- [추가됨] 수령 확인(다 꺼냈어) 명령 포워딩 ---
        elif scenario == "SCENARIO_8":
            if "ARRIVED" in self.current_robot_state or self.current_robot_state == "SCENARIO_5":
                # 발화는 main_node.py에서 상태를 바꾸면 자동으로 실행되므로 여기선 publish만 함
                self.command_pub.publish(json.dumps({"command": "SCENARIO_8", "payload": []}))
            else:
                rospy.loginfo("🚫 [명령 차단] 목적지에 도착하여 대기 중인 상태가 아니므로 수령 확인을 무시합니다.")
        # ----------------------------------------------------------------------------------------

        elif scenario in self.pdf_responses: 
             self.play_robot_speech(self.pdf_responses[scenario])

    def run(self):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            rospy.loginfo(" [시스템] 인터넷 연결 상태 정상입니다.")
        except OSError:
            rospy.logerr("[경고] 인터넷이 연결되어 있지 않습니다. 연결을 확인해주세요.")
            rospy.logerr("(인터넷이 없으면 기능들이 동작하지 않습니다.)")

        self.stt_engine = realtime_stt2.RealtimeSTT(self.on_transcription_received, self.llm_client)
        threading.Thread(target=self.stt_engine.start_listening, daemon=True).start()
        rospy.loginfo(" 음성 제어 노드 가동 중")
        rospy.spin()

if __name__ == "__main__":
    VoiceControlNode().run()
