import RPi.GPIO as GPIO
import time

# 设置GPIO引脚
BUZZER_PIN = 35

# 定义音符的频率（赫兹）
NOTE_FREQS = {
    '3-': 261 ,  # E
    '4-': 294,  # F
    '5-': 329,  # G
    '6-': 349,  # A
    '7-': 392,  # B
    '1': 440,  # C
    '2': 494,  # D
    '3': 523   # C5
}

# 定义音符的节奏（每个音符的持续时间，单位：秒）
NOTE_DURATIONS = {
    'whole': 1.0,
    'half': 0.5,
    'quarter': 0.25
}

# 初始化GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# 播放单个音符
def play_tone(frequency, duration):
    if frequency > 0:
        pwm = GPIO.PWM(BUZZER_PIN, frequency)
        pwm.start(50)  # 启动PWM，50%占空比
        time.sleep(duration)
        pwm.stop()

# 播放一段音乐
def play_melody(melody):
    for note, duration in melody:
        play_tone(NOTE_FREQS[note], duration)
        time.sleep(0.01)  # 音符间的间隔

# 例子：播放指定的音调
melody = [
    ('6-', NOTE_DURATIONS['quarter']),  # A
    ('7-', NOTE_DURATIONS['quarter']),  # B
    ('1', NOTE_DURATIONS['half']),    # C
     ('1', NOTE_DURATIONS['quarter']), 
    ('7-', NOTE_DURATIONS['quarter']),  # B
    ('1', NOTE_DURATIONS['half']),    # C
    ('3', NOTE_DURATIONS['half']),  # E
    ('7-', NOTE_DURATIONS['whole']),
     ('7-', NOTE_DURATIONS['half']), # B
    ('3-', NOTE_DURATIONS['half']),  # E
    ('6-', NOTE_DURATIONS['half']),  # A
    ('6-', NOTE_DURATIONS['quarter']),  # A
    ('5-', NOTE_DURATIONS['quarter']),  # G
    ('6-', NOTE_DURATIONS['half']),  # A
    ('1', NOTE_DURATIONS['half']),    # C
    ('5-', NOTE_DURATIONS['whole']),  # G
]

try:
    play_melody(melody)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()  # 清理GPIO设置
