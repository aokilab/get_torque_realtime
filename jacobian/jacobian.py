#!/usr/bin/env python3


#import scipy.io
from scipy.io import loadmat

# MATファイル読み込み
#mat_data = scipy.io.loadmat('jacobian-4.mat')

mat = loadmat("jacobian-4.mat")

Jv = mat["Jv"]

print(Jv)
