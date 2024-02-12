from main import *
from eftelya import *
import numpy as np
import time


class Bucket():
    def __init__(self, bluePoint, *RedPoints ):
        self.bluePoint = bluePoint
        self.RedPoints = RedPoints
        

        if len(self.bluePoint) == 0:
            self.goToPoint(self.RedPoints)
        else:
            self.goToPoint(self.bluePoint)


    def goToPoint(self, point):
        self.point = point
        self.mid = ((self.point[0][0] + self.point[1][0]) / 2 , (self.point[0][1] + self.point[1][1]) / 2 )
        self.area = abs(self.point[0][0] - self.point[1][0]) * abs(self.point[0][1] - self.point[1][1])
        
        self.oneAxisCenter()
        self.moveForward()
        self.checkArea()

    def oneAxisCenter(self):
        """
        gonderilen noktaya göre donme gerçeklestirecek ve kova yi ortalamis olacak
        """
        degree = self.calculateAngle(self.mid)
        rov.turnDegrees(degree)

    def calculateAngle(self, point):

        angle = np.arctan2(self.mid[1] - height/2, self.mid[0] - width/2) * 180.0 / np.pi 
        return angle
        
    def moveForward(self):
        """
        ileri hareket gerçekleştirecek
        """
        rov.move(0,0,100)

    
    def checkArea(self):
        """
        alan kontrolu yapacak
        """
        if self.area < 20000:
            self.moveForward()
        else:
            #alt kamera da goruntu isleme yapacak
            pass


    def dropBall(self):
        """
        topu birakacak
        """
        pass




class Bucket2(Bucket):
    def __init__(self):
        self.goBackThreeSecond(self)



    def goBackThreeSecond(self):
        """
        3 saniye geri gidecek
        """
        rov.move(0,0,-100)
        timePartTwo = time.time()
        while time.time() - timePartTwo < 3:
            pass
        rov.move(0,0,0)
        
        self.goForwardThreeSecond(self)

    def goForwardThreeSecond(self):
        """
        3 saniye ileri gidecek
        """
        rov.move(0,0,100)
        timePartTwo = time.time()
        while time.time() - timePartTwo < 3:
            pass
        rov.move(0,0,0)

        self.searchBall(self)

    def searchBall(self):
        """
        top arayacak
        """
        pass
        self.oneAxisCenter(self)
    


    def takeBall(self):
        """
        topu alacak
        """
        pass