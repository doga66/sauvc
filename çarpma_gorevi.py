"""
export OPENBLAS_CORETYPE= ARMV8
sudo chmod 666 /dev/ttyACM0
cd yolov5/  #bura farklı olabilir doga!!!!!
python3 çarpma_görevi.py 
"""

import torch
import cv2
import time
import numpy as np
from pySerialTransfer import pySerialTransfer as txfer
import math


class struct(object):
    RollS = 0
    PitchS = 0
    HeadingS = 0
    frontLidarS = 0
    leftLidarS = 0
    rightLidarS = 0
    altitudeS = 0
    data1S = 0
    data2S = 0
    data3S = 0
    data4S = 0
    data5S = 0

class struct1(object):
    arac_ileri_degeri = 0
    arac_x_degeri = 0
    arac_y_degeri = 0
    arac_yengec_degeri = 0
    degree = 0
    lidarKontrol = False
    cisimKontrol = False

rovDataTx = struct1 # gönderilen
rovDataRx = struct # alınan

a = 0
b = 0
c = 0
d = 0
bool1 = False
bool2 = False

fark_x =0
fark_y = 0
hiz_x =0
hiz_y = 0
farkx_ =0
farky_ = 0
hizx_ =0
hizy_ = 0
alan =0
prevCircle = None 
dist = lambda x1,y1,x2,y2: (x1-x2)*2+(y1-y2)*2
radius = 0
circle_x = 0
circle_y = 0
oncam_ortala = True
wait_alan = False
wait_alan_2 = False
farkSayac = 0
now_time2 = 0
bool3 = False
aKontrolBool = False
await_kontrol = False
aAlanKontrolSayacFark = 0
waitAlanSayacFark = 0 
wait_alan2 = False

lidarwaitBool = False
lidarwaitBool2 = False
lidarwaitBool3 = False 
lidarwaitBool4 = False
lidarwaitBool5 = False
lidarwaitBool6 = False
lidarwaitBool7 = False
lidarwaitBool8 = False
lidarwaitBool9 = False
lidarwaitBool10 = False

height,width = 480 , 640
rovDataTx.lidarKontrol = False
rovDataTx.cisimKontrol = False
rovDataRx.frontLidarS = 0
lidarWaitTimerFark = 0
rovDataTx.degree = 0
gyroTimer1 = time.time()
gyroTimer3 = time.time()

model = torch.hub.load('ultralytics/yolov5', 'custom', path='bestEftelya.pt', force_reload = True )


cam = cv2.VideoCapture(0)
width1 = int(cam.set(cv2.CAP_PROP_FRAME_WIDTH,640))
height1 = int(cam.set(cv2.CAP_PROP_FRAME_HEIGHT,480))

cap_alt = cv2.VideoCapture(2)
width2 = int(cap_alt.set(cv2.CAP_PROP_FRAME_WIDTH,640))
height2 = int(cap_alt.set(cv2.CAP_PROP_FRAME_HEIGHT,480))


fourcc = cv2.VideoWriter_fourcc(*'MJPG')

writer1 = cv2.VideoWriter('yabay10.avi', fourcc, 6.0, (width, height))
writer2 = cv2.VideoWriter('yabay20.avi', fourcc, 6.0, (width, height))
writer3 = cv2.VideoWriter('yabay30.avi', fourcc, 6.0, (width, height))
# writer4 = cv2.VideoWriter('yabay40.avi', fourcc, 6.0, (width, height))
# writer5 = cv2.VideoWriter('yabay11.avi', fourcc, 6.0, (width, height))
# writer6 = cv2.VideoWriter('yabay12.avi', fourcc, 6.0, (width, height))
# writer7 = cv2.VideoWriter('yabay13.avi', fourcc, 6.0, (width, height))
# writer = cv2.VideoWriter('yabay7.avi', fourcc, 10.0, (width, height))


newTimeri = time.time()


link = txfer.SerialTransfer('/dev/ttyACM0')
link.open()

time.sleep(2) # allow some time for the Arduino to completely reset

startTime = time.time()
print("suya at")
while 1:
    time_now1 = time.time()
    now_time =int( time_now1-startTime)

    # ret, frame_alt = cap_alt.read()
    ret2, frame = cam.read()
    # if ret2 is None: eğer kamerada temazsızlık durumu olursa bu yorum satırı acilmalidir
    #     continue
    writer2.write(frame)
    # writer3.write(frame_alt)


    if now_time >10.0:
       
       
        result = model(frame)
        df = result.pandas().xyxy[0]
        
        farkx_ = 0
        farky_ = 0
        hizx_ = 0
        hizy_ = 0
        alan = 0
        rovDataTx.arac_x_degeri = 0  
        
        for ind in df.index:
            if df['class'][ind] == 0 and df['confidence'][ind] >= 0.5 :
                x1, y1 = int(df['xmin'][ind]), int(df['ymin'][ind])
                x2, y2 = int(df['xmax'][ind]), int(df['ymax'][ind])
                label = df['name'][ind]
                conf =  df['confidence'][ind]
                text = label + " " + str(conf.round(decimals=2))
                
                noktax = int(((x2+x1)/2)) 
                noktay = int((y2+y1)/2)
                nokta = noktax,noktay
                
                alan = (x2-x1) * (y2-y1)
                
                farkx_ = 320 - noktax
                farky_ = 240 - noktay
                
                hizx_= - farkx_ * 0.8
                hizy_ = - farky_  * 0.8
                
                
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,0,0),2)
                cv2.putText(frame, text,(x1,y1-5), cv2.FONT_ITALIC,2 ,(0,0,0),2)
                cv2.putText(frame, ".", (nokta), cv2.FONT_ITALIC, 1, (0, 0, 255), 4)

        


        tespit_edilen_direk_sayısı=0

        if (tespit_edilen_direk_sayısı==1):
            rovDataTx.arac_ileri_degeri =  100
            rovDataTx.arac_x_degeri = farkx_
            rovDataTx.arac_y_degeri = 0

        if (tespit_edilen_direk_sayısı==0):
            rovDataTx.arac_ileri_degeri = -100
            rovDataTx.arac_x_degeri = 0
            rovDataTx.arac_y_degeri = 0

            time.sleep(5) #süre değişebilir.
            #5 saniye -100 ile geriye gitsin.

            rovDataTx.arac_ileri_degeri = 0
            #sonra ileri gitme dursun.

        if (tespit_edilen_direk_sayısı==2):
            
           # İlk direk (sol taraftaki direk)
          x1, y1 = int(df['xmin'][ind]), int(df['ymin'][ind])
          x2, y2 = int(df['xmax'][ind]), int(df['ymax'][ind])

          # İkinci direk (sağ taraftaki direk)
          x3, y3 = int(df['xmin'][ind+1]), int(df['ymin'][ind+1])
          x4, y4 = int(df['xmax'][ind+1]), int(df['ymax'][ind+1])

          # İlk direğin merkezi
          noktax1 = int((x2 + x1) / 2)
          noktay1 = int((y2 + y1) / 2)

          # İkinci direğin merkezi
          noktax2 = int((x4 + x3) / 2)
          noktay2 = int((y4 + y3) / 2)

          # Eğer ikinci direk daha sağda ise, ona doğru hareket et
          if noktax2 > noktax1:
            farkx_ = 320 - noktax2
            hizx_ = -farkx_ * 0.8
          else:
            farkx_ = 320 - noktax1
            hizx_ = -farkx_ * 0.8

          rovDataTx.arac_ileri_degeri = 100  # İleri git
          rovDataTx.arac_x_degeri = hizx_  # Sağdaki direğe hareket et
          rovDataTx.arac_y_degeri = 0  # Y ekseni üzerinde hareket etme



    # print("****")
    writer1.write(frame)
    # writer4.write(frame_alt)
         
 
    # print("basınc : ",rovDataRx.altitudeS)

    sendSize = 0
    sendSize = link.tx_obj(rovDataTx.arac_ileri_degeri, start_pos=sendSize)
    sendSize = link.tx_obj(rovDataTx.arac_y_degeri, start_pos=sendSize)
    sendSize = link.tx_obj(rovDataTx.arac_x_degeri, start_pos=sendSize)
    sendSize = link.tx_obj(rovDataTx.arac_yengec_degeri, start_pos=sendSize)
    sendSize = link.tx_obj(rovDataTx.degree, start_pos=sendSize)

    link.send(sendSize)
    time.sleep(0.15)

    if not link.available():
        # print("while'a girdi")
        if link.status < 0:
            if link.status == txfer.CRC_ERROR:
                print('ERROR: CRC_ERROR')
            elif link.status == txfer.PAYLOAD_ERROR:
                print('ERROR: PAYLOAD_ERROR')
            elif link.status == txfer.STOP_BYTE_ERROR:
                print('ERROR: STOP_BYTE_ERROR')
            else:
                print('ERROR: {}'.format(link.status))

    recSize = 0
    rovDataRx.RollS = link.rx_obj(obj_type=type(rovDataRx.RollS),obj_byte_size= sendSize, start_pos=recSize)
    rovDataRx.PitchS = link.rx_obj(obj_type=type(rovDataRx.PitchS),obj_byte_size= sendSize, start_pos=recSize + 4)
    rovDataRx.HeadingS = link.rx_obj(obj_type=type(rovDataRx.HeadingS),obj_byte_size= sendSize, start_pos=recSize + 8)
    rovDataRx.frontLidarS = link.rx_obj(obj_type=type(rovDataRx.frontLidarS),obj_byte_size= sendSize, start_pos=recSize + 12)
    rovDataRx.leftLidarS = link.rx_obj(obj_type=type(rovDataRx.leftLidarS),obj_byte_size= sendSize, start_pos=recSize + 16)
    rovDataRx.rightLidarS = link.rx_obj(obj_type=type(rovDataRx.rightLidarS),obj_byte_size= sendSize, start_pos=recSize + 20)
    rovDataRx.altitudeS = link.rx_obj(obj_type=type(rovDataRx.altitudeS),obj_byte_size= sendSize, start_pos=recSize + 24)
    rovDataRx.data1S = link.rx_obj(obj_type=type(rovDataRx.data1S),obj_byte_size= sendSize, start_pos=recSize + 28)
    
    # cv2.imshow("frame",frame)
    if cv2.waitKey(1) & now_time >= 10000:
        break

cam.release()
cap_alt.release()

writer1.release()
writer2.release()
writer3.release()
# writer4.release()

link.close()

cv2.destroyAllWindows()












        
