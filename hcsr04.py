import time
import RPi


class HCSR04Result:
    'DHT11 sensor result returned by DHT11.read() method'

    ERR_NO_ERROR = 0
    ERR_MISSING_DATA = 1
    ERR_CRC = 2

    error_code = ERR_NO_ERROR
    temperature = -1
    humidity = -1

    def __init__(self, error_code, distance):
        self.error_code = error_code
        self.distance = distance

    def is_valid(self):
        return self.error_code == HCSR04Result.ERR_NO_ERROR


class HCSR04:
    'DHT11 sensor reader class for Raspberry'

    __trig = 0
    __echo = 1

    def __init__(self, trig, echo):
        self.__trig = trig
        self.__echo = echo

    def read(self):
        RPi.GPIO.setup(self.__trig, RPi.GPIO.OUT)
        RPi.GPIO.setup(self.__echo, RPi.GPIO.IN)

        # send initial high
        self.__send_and_sleep(RPi.GPIO.HIGH, 0.00001)

        # pull down to low
        self.__send_and_sleep(RPi.GPIO.LOW, 0)

        startTime = time.time()
        stopTime = time.time()

        while 0 == RPi.GPIO.input(self.__echo):
            startTime = time.time()

        while 1 == RPi.GPIO.input(self.__echo):
            stopTime = time.time()

        TotalTime = stopTime - startTime

        return round(((TotalTime * 34300) / 2),1)

    def __send_and_sleep(self, output, sleep):
        RPi.GPIO.output(self.__trig, output)
        time.sleep(sleep)
