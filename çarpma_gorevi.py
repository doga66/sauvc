from main import *
from çarpma_görevi import *
import numpy as np
import time 
import cv2 as cv
import pandas as pd

class DirekController:
    def __init__(self,direkController):
        self.direkController=direkController #önce alana girince direk kontolü yapsın.
        if (direkController==0):
           self.move(0,-100,0)
           time.sleep(5)
        else:
            self.direct_detection()


    def move(self):
        self.move()



    def calculate_center(self, df, ind):
    # İlgili veri çerçevesinden gerekli verileri alarak orta noktayı hesapla
           x1, y1 = int(df['xmin'][ind]), int(df['ymin'][ind])
           x2, y2 = int(df['xmax'][ind]), int(df['ymax'][ind])

            
           noktax = int(((x2+x1)/2)) 
           noktay = int((y2+y1)/2)
           nokta = noktax,noktay
           
           farkx_ = 320 - noktax
           farky_ = 240 - noktay
           
    
           return farkx_,farky_




    def direct_detection(self, tespit_edilen_direk_sayısı):
          if tespit_edilen_direk_sayısı == 1:
            self.calculate_center(df, ind) 
            if(-30 <farkx_< 30 and -30 <farky_< 30):
                self.move(0,0,100) 
            
          elif tespit_edilen_direk_sayısı == 2:
            # İlk direk (sol taraftaki direk)
           left_direct_center=self.calculate_center()

          # İkinci direk (sağ taraftaki direk)
           right_direct_center=self.calculate_center(self)

          # Eğer ikinci direk daha sağda ise, ona doğru hareket et
           if right_direct_center > left_direct_center:
              
              self.move()  # noktax2 - 320   
            
           else:
            

        # Geriye hareket ve ilerleme işlemlerini kontrol etmek için örnek kodları burada entegre edebilirsiniz
        #self.move_back_and_forward(tespit_edilen_direk_sayısı)

    #def move_back_and_forward(self, tespit_edilen_direk_sayısı):
       #if tespit_edilen_direk_sayısı == 0:
            # 5 saniye geri gidip durma işlemi
            #self.move(-100, 0, 0)
            #time.sleep(5)
            #self.move(0, 0, 0)
        # Diğer durumlar için gereken işlemleri buraya ekleyebilirsiniz


# Kullanım örneği:
rov_controller = DirekController.Controller()
rov_controller.direct_detection(tespit_edilen_direk_sayısı, df, ind)
