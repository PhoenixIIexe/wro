from time import sleep, time
import RPi.GPIO as GPIO
import numpy as np
import cv2

GPIO.setmode(GPIO.BCM)
#---------------------------------------------------------------------#
#Обозначение мотора
pin1_d = 22
pin1_sp = 27

GPIO.setup(pin1_d, GPIO.OUT)
GPIO.setup(pin1_sp, GPIO.OUT)

motor = GPIO.PWM(pin1_sp, 100)
#---------------------------------------------------------------------#
#Обозначение серва
servoPIN = 17

GPIO.setup(servoPIN, GPIO.OUT)
GPIO.output(servoPIN, True)

sr = GPIO.PWM(servoPIN, 100)
sr.start(1)

#---------------------------------------------------------------------#
#Обозначение границ черного и белого
hsv_min = np.array((0, 0, 80), np.uint8)
hsv_max = np.array((255, 255, 255), np.uint8)
font = cv2.FONT_HERSHEY_COMPLEX

#---------------------------------------------------------------------#


class Wro:
    #Функция управления серва
    @classmethod
    def turn(self, angle):
        duty = -angle + 5.5
        sr.ChangeDutyCycle(duty)
        
    #---------------------------------------------------------------------#
    #Функция управления мотора
    @classmethod
    def go(self, m1):
        GPIO.output(pin1_d, GPIO.HIGH if m1 >= 0 else GPIO.LOW)
        motor.start(abs(m1) if abs(m1) <= 100 else 100)

    #---------------------------------------------------------------------#
    #Фунция движения
    @classmethod
    def line(self, m1):
        sens = self.sensor()
        print(sens)
        self.turn(sens * 4) 
        sleep(0.2)
        self.go(m1)

    #---------------------------------------------------------------------#
    #Определение границ
    @classmethod
    def sensor(self):
        list_x = []
        list_y = []

        img = cv2.VideoCapture(0).read()[1]
        img = img[200:480, 0:640]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv, hsv_min, hsv_max)
        contours, hierarchy = cv2.findContours( thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        min_x = 310
        min_y = 480
        
        
        i = 0
        while thresh[55, i] != 255 and i != 639:
            i += 1
        left_x = i
        while thresh[55, i] != 0 and i != 639:
            i += 1
        right_x = i
        
        for cnt in contours :
            approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)

            cv2.drawContours(img, [approx], 0, (0, 255, 255), 5) 

            n = approx.ravel() 
            for i in range(0, len(n), 2):
                list_x.append(int(n[i]))
                list_y.append(int(n[i+1]))
                # x = n[i]

                # y = n[i + 1]

                # string = str(x) + " " + str(y) 
                # if(i == 0):
                #     cv2.putText(img, "Arrow tip", (x, y),

                #                     font, 0.5, (255, 0, 0)) 
                # else:
                #     cv2.putText(img, string, (x, y), 

                #                 font, 0.5, (0, 255, 0)) 
        
        
        # cv2.line(img, (0, 55), (640, 55), (0, 255, 0), 5)
        # cv2.line(img, ((right_x+left_x) // 2, 55), (320, 280), (255, 0, 0), 5)
        
        # cv2.imshow('img', img)
        # cv2.waitKey(1)
        
        return (right_x+left_x) / 640 - 1
    #---------------------------------------------------------------------# 

#---------------------------------------------------------------------#          
#Программа
temp = round(time())
while round(time()) - temp < 20:
    Wro.line(70)


#---------------------------------------------------------------------# 
#Выключение всех моторов и закрытие окон
cv2.destroyAllWindows()
motor.stop()
sr.stop()
GPIO.cleanup()