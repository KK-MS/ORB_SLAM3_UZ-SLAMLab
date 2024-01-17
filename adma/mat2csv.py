import pandas as pd
import numpy as np
from datetime import datetime, timedelta
from collections import defaultdict
import sys
import csv
import mat73



class ADMA_Processor:

  G = 9.81
  rate_factor = (np.pi/180)
  IMU_FREQ = 100
  data_file = None

  def __init__(self, args):
    self.data_file = args[1]
     
  def get_mat_data(self):
     print("Reading file...")
     data_dict = mat73.loadmat(self.data_file)
     return data_dict
  
  def get_rate_body(self, dict):
    rate_body_x = []
    rate_body_y = []
    rate_body_z = []

    for i in range(len(dict['INS_Time_Week'])):
        if(i > 0):
            rate_body_x.append(dict['Rate_Body_X'][i] * self.rate_factor)
            rate_body_y.append(dict['Rate_Body_Y'][i] * self.rate_factor)
            rate_body_z.append(dict['Rate_Body_Z'][i] * self.rate_factor)

    return rate_body_x, rate_body_y, rate_body_z

  def get_acc_body(self, dict):
    acc_body_x = []
    acc_body_y = []
    acc_body_z = []

    for i in range(len(dict['INS_Time_Week'])):
        if(i > 0):
            acc_body_x.append(dict['Acc_Body_X'][i] * self.G)
            acc_body_y.append(dict['Acc_Body_Y'][i] * self.G)
            acc_body_z.append(dict['Acc_Body_Z'][i] * self.G)

    return acc_body_x, acc_body_y, acc_body_z
  

adma = ADMA_Processor(sys.argv)

dict = adma.get_mat_data()

acc_body_x,acc_body_y,acc_body_z = adma.get_acc_body(dict)
rate_body_x,rate_body_y,rate_body_z = adma.get_rate_body(dict)

with open('adma_data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    field = ["acc_x","acc_y","acc_z","gyro_x","gyro_y","gyro_z"]
    writer.writerow(field)

    for count, ts in enumerate(acc_body_x):
      field = [
              "{:.18f}".format(float(rate_body_x[count])), 
              "{:.18f}".format(float(rate_body_y[count])),
              "{:.18f}".format(float(rate_body_z[count])),
              "{:.18f}".format(float(acc_body_x[count])), 
              "{:.18f}".format(float(acc_body_y[count])),
              "{:.18f}".format(float(acc_body_z[count]))]
      writer.writerow(field)
    