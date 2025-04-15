import Hobot.GPIO as GPIO

CAN_SIGNAL = 36

def main():
    # 设置物理编码
    GPIO.setmode(GPIO.BOARD)
    # 屏蔽警告
    GPIO.setwarnings(False)
    # 设置输出模式
    GPIO.setup(CAN_SIGNAL, GPIO.OUT)
    # 设置通道的输出值
    GPIO.output(CAN_SIGNAL, GPIO.HIGH)

if __name__ == "__main__":
    main()
