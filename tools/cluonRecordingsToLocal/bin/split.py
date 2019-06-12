#!/usr/bin/python3

import cv2 as cv
import glob
import os
images = glob.glob('*.png')
try:
    os.mkdir('left')
    os.mkdir('right')
    for fname in images:
        img = cv.imread(fname)
        (rows, cols) = img.shape[0:2]
        # left lens
        left = img[0:rows, 0:cols//2]
        # right lens
        right = img[0:rows, cols//2:cols]
    
        os.chdir('left')
        imgname = fname[:-4] + '_left.png' # fname is *.png
        cv.imwrite(imgname, left)
    
        os.chdir('../right')
        imgname = fname[:-4] + '_right.png'
        cv.imwrite(imgname, right)
        os.chdir('..')
except:
    for fname in images:
        img = cv.imread(fname)
        (rows, cols) = img.shape[0:2]
        # left lens
        left = img[0:rows, 0:cols//2]
        # right lens
        right = img[0:rows, cols//2:cols]
    
        os.chdir('left')
        imgname = fname[:-4] + '_left.png' # fname is *.png
        cv.imwrite(imgname, left)
    
        os.chdir('../right')
        imgname = fname[:-4] + '_right.png'
        cv.imwrite(imgname, right)
        os.chdir('..')
