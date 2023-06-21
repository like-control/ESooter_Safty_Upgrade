import threading
import serial
import time
import pygame
import pynmea2
import json
import datetime
import uuid
import hmac
import hashlib
import requests
import platform
import sys
import os
import RPi.GPIO as GPIO
from geopy.geocoders import Nominatim

#소리 함수
pygame.init()

song_path1 = '/home/pi/warning1.mp3'
song_path2 = '/home/pi/warning2.mp3'
song_path3 = '/home/pi/warning3.mp3'

def play_music1():
    pygame.mixer.music.load(song_path1)
    pygame.mixer.music.play(0)

def play_music2():
    pygame.mixer.music.load(song_path2)
    pygame.mixer.music.play(0)

def play_music3():
    pygame.mixer.music.load(song_path3)
    pygame.mixer.music.play(0)

def stop_music():
    pygame.mixer.music.stop()


# SMS 함수
protocol = 'https'
domain = 'api.coolsms.co.kr'
prefix = ''
count = 0


def unique_id():
    return str(uuid.uuid1().hex)


def get_iso_datetime():
    utc_offset_sec = time.altzone if time.localtime().tm_isdst else time.timezone
    utc_offset = datetime.timedelta(seconds=-utc_offset_sec)
    return datetime.datetime.now().replace(tzinfo=datetime.timezone(offset=utc_offset)).isoformat()


def get_signature(key, msg):
    return hmac.new(key.encode(), msg.encode(), hashlib.sha256).hexdigest()


def get_headers(api_key, api_secret):
    date = get_iso_datetime()
    salt = unique_id()
    combined_string = date + salt

    return {
        'Authorization': 'HMAC-SHA256 ApiKey=' + api_key + ', Date=' + date + ', salt=' + salt + ', signature=' +
                         get_signature(api_secret, combined_string),
        'Content-Type': 'application/json; charset=utf-8'
    }


def get_url(path):
    url = '%s://%s' % (protocol, domain)
    if prefix != '':
        url = url + prefix
    url = url + path
    return url

def send_many(parameter):
    api_key = 'NCSC5VXWFTYGEK2Z'
    api_secret = 'ZHV90B4K1L52AZCZSDND74EJ9B0LJNTT'
    parameter['agent'] = {
        'sdkVersion': 'python/4.2.0',
        'osPlatform': platform.platform() + " | " + platform.python_version()
    }
    return requests.post(get_url('/messages/v4/send-many'), headers=get_headers(api_key, api_secret), json=parameter)

angleser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.001)

def geocoding_reverse(lat_lng_str): 
    geolocoder = Nominatim(user_agent = 'South Korea', timeout=None)
    address = geolocoder.reverse(lat_lng_str)

    return address


#메인 함수

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.output(18, GPIO.HIGH)

last_triggered_time = time.time()

play_music3()
time.sleep(1)

while True:
    if angleser.inWaiting():

        testString = angleser.readline().decode('utf-8')
        real = testString[testString.index('*')+1:testString.index('.')]
        anglevalue = abs(int(real))


        if anglevalue is not None and anglevalue >= 20 and anglevalue < 60:
            print(anglevalue, "warning")

            if not pygame.mixer.music.get_busy() or pygame.mixer.music.get_pos() == -1:
               play_music1()
        
        elif anglevalue is not None and anglevalue >= 60:
            print(anglevalue, "danger")
            first_triggered_time = time.time()
            print(last_triggered_time)

            if not pygame.mixer.music.get_busy() or pygame.mixer.music.get_pos() == -1:
                play_music2()

                while anglevalue >= 60:
                    if angleser.inWaiting():
                        testString = angleser.readline().decode('utf-8')
                        real = testString[testString.index('*')+1:testString.index('.')]
                        anglevalue = abs(int(real))

                    time.sleep(0.1)
                    play_music2()

                    last_triggered_time = time.time()
                    print("넘어짐 시간 ", int(last_triggered_time - first_triggered_time), "초 경과")

                    if last_triggered_time - first_triggered_time > 5 : # 넘어짐 감지 시간 기준
                        # GPS 모듈 연결된 USB 포트 경로
                        serial_port = '/dev/ttyAMA0'

                        # 시리얼 통신 객체 생성
                        serial = serial.Serial(serial_port, baudrate=9600, timeout=0.5)
                        if not serial.isOpen():
                            serial.open()
                        while True:
                            serial_data = serial.readline().decode('ascii', errors='replace')
                            # GPS 데이터 수신 및 파싱
                            try:
                                data = serial.readline().decode()
                                if data.startswith('$GPGGA'):
                                    msg = pynmea2.parse(data)
                                    # 주소 정보 추출
                                    lat = msg.latitude
                                    lon = msg.longitude
                                    #address = f"위도 {lat:.5f}, 경도 {lon:.5f}"
                                    address = geocoding_reverse(f"{lat:.5f}, {lon:.5f}")
                                    break
                            except Exception as e:
                                print(e)

                        # 문자 내용 작성
                        if count == 0:
                            data = {
                                'messages': [
                                    {
                                        'to': '01055353533',
                                        'from': '01041377938',
                                        'text': "현재 사고 위치는 "+(address)+" 부근입니다."
                                    },
                                ]
                            }
                            count += 1
                            print (data)
                            res = send_many(data)
                            print(json.dumps(res.json(), indent=2, ensure_ascii=False))
    
                        serial.close()

                        GPIO.output(18, GPIO.LOW)
                        GPIO.cleanup()
                        
                        os._exit(0)



        else:
            print(anglevalue)
            stop_music()