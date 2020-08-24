#!/usr/bin/env python
from OpenGL.GL import *
from OpenGL.GLU import *
import cv2
from PyQt5 import QtGui, QtCore, QtOpenGL, QtWidgets


class CvWidget(QtOpenGL.QGLWidget):

    def __init__(self, fps=15, parent=None):
        QtOpenGL.QGLWidget.__init__(self, parent)
        self.cvImage = None
        self.aspectRatio = 1
        self.size = (400, 400)
        if(fps > 0):
            interval = max(int(1000 / fps), 1)
            self.startTimer(interval)

    def resizeEvent(self,event):
        QtWidgets.QWidget.resizeEvent(self,event)
        self.size = (self.width(), self.height())
        
    def timerEvent(self, event):
        if self.isVisible():
            self.update()

    def paintGL(self):
        if not self.isVisible() or self.cvImage is None:
            return
        img = self.cvImage.copy()
        if type(img) == type(None):
            return
        # Resize image to fit screen
        newHeight = int(self.height()*self.aspectRatio)
        size = min(newHeight,self.width())
        offset = ((self.width() - size) / 2,
                  (self.height() - int(size / self.aspectRatio)) / 2)
        width, height = (size,int(size / self.aspectRatio))
        
        img = self.imageProc(img)
        img = cv2.resize(img, (width, height))

        # Need to flip image because GL buffer has 0 at bottom
        img = cv2.flip(img, flipCode = 0)
        fmt = GL_RGB
        t = GL_UNSIGNED_BYTE
        glViewport(offset[0],offset[1],width,height)
        glClearColor(1.0,1.0,1.0,1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT )
        glEnable(GL_ALPHA_TEST)
        glAlphaFunc(GL_GREATER,0)

        glPixelStorei(GL_PACK_ALIGNMENT, 1)
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
        glMatrixMode(GL_PROJECTION);
        matrix = glGetDouble( GL_PROJECTION_MATRIX )
        glLoadIdentity();
        glOrtho(0.0, width, 0.0, height, -1.0, 1.0)
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix()
        glLoadIdentity()
        glRasterPos2i(0,0)
        glDrawPixels(width, height, fmt, t, img)
        glPopMatrix()
        glFlush()

    def imageProc(self, image):
        return image

    def setImage(self, image, bgr=True):
        if not bgr:
            self.cvImage = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        else:
            self.cvImage = image;
        self.aspectRatio = image.shape[1] / float(image.shape[0])

    def initializeGL(self):

        glClearColor(0,0,0, 1.0)

        glClearDepth(1.0)              
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()                    
        gluPerspective(45.0,1.33,0.1, 100.0) 
        glMatrixMode(GL_MODELVIEW)

if __name__ == '__main__':
    app = QtWidgets.QApplication(['cv_widget'])
    texturePath = "/home/biomed/Pictures/-.png"
    image = cv2.imread(texturePath)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    widget = CvWidget()
    widget.setImage(image)

    widget.show()
    app.exec_()



