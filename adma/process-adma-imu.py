import pandas as pd
import numpy as np
from datetime import datetime, timedelta
from collections import defaultdict
import sys
import csv
import cv2
import os
import time
import mat73
import pytz



class ADMA_Processor:

  G = 9.81
  rate_factor = (np.pi/180)
  IMU_FREQ = 100
  data_file = None
  video_file = None
  rootdir = None

  def __init__(self, args):
    self.data_file = args[1]
    self.video_file = args[2]
    self.rootdir = args[3]
     
  def get_mat_data(self):
     print("Reading file...")
     data_dict = mat73.loadmat(self.data_file)
     return data_dict
  
  def gen_timestamps(self, dict):
     timestamps_ns = []
     time_week = dict['GNSS_Time_Week'][1]
     time_ms = dict['GNSS_Time_msec'][1]
     leap_seconds = 18

     for i in range(len(dict['GNSS_Time_Week'])):
      if(i > 0):
        time_week = dict['GNSS_Time_Week'][i]
        time_ms = dict['GNSS_Time_msec'][i]
        gps_epoch = datetime(1980, 1, 6, tzinfo=pytz.utc)
        unix_ts = (gps_epoch + timedelta(weeks=time_week, milliseconds=time_ms, seconds=-leap_seconds)).timestamp()
        unix_ts = unix_ts * 1e9
        timestamps_ns.append(unix_ts)
     
     return timestamps_ns
  
  def get_rate_body(self, dict):
    rate_body_x = []
    rate_body_y = []
    rate_body_z = []

    for i in range(len(dict['GNSS_Time_Week'])):
      if(i > 0):
        rate_body_x.append(dict['Rate_Body_X'][i] * self.rate_factor)
        rate_body_y.append(dict['Rate_Body_Y'][i] * self.rate_factor)
        rate_body_z.append(dict['Rate_Body_Z'][i] * self.rate_factor)

    return rate_body_x, rate_body_y, rate_body_z
  
  def get_acc_body(self, dict):
    acc_body_x = []
    acc_body_y = []
    acc_body_z = []

    for i in range(len(dict['GNSS_Time_Week'])):
      if(i > 0):
        acc_body_x.append(dict['Acc_Body_X'][i] * self.G)
        acc_body_y.append(dict['Acc_Body_Y'][i] * self.G)
        acc_body_z.append(dict['Acc_Body_Z'][i] * self.G)

    return acc_body_x, acc_body_y, acc_body_z
  
  def get_duplicate_ts(self, timestamps):
    D = defaultdict(list)
    for i,item in enumerate(timestamps):
      D[item].append(i)
    D = {k:v for k,v in D.items() if len(v)>1}
    return D
  
  def get_video_ts(self, ts):
    video = cv2.VideoCapture(self.video_file)
    frame_ts = []
    while(video.isOpened()):
      frame_exists, curr_frame = video.read()
      if frame_exists:
        nsec = int((video.get(cv2.CAP_PROP_POS_MSEC)) * 1e6)
        new_ts = ts[0] + nsec
        frame_ts.append(new_ts)
        print("===============")
        print(ts[0])
        print(int((video.get(cv2.CAP_PROP_POS_MSEC)) * 1e6))
        print(new_ts)
        print("===============")
      else:
        print("No of ts: " + str(len(frame_ts)))
        break
    return frame_ts
  
  def find_nearest_ts(self,ts_high_freq, K):
    return ts_high_freq[min(range(len(ts_high_freq)), key = lambda i: abs(ts_high_freq[i]-K))]
  
  def save_frame_by_frame(self, filepath, timestamps):
    cap = cv2.VideoCapture(filepath)
    basepath = "./" + str(self.rootdir) + "/mav0/cam0/data/"
    counter = 0
    while(True):
        if cap.isOpened():
          ret,frame = cap.read()
          filename = str(timestamps[counter]) + ".png"
          imagePath = basepath + filename
          print(counter)
          counter = counter + 1
          if (ret == True):
            cv2.imwrite(imagePath, frame)
            if cv2.waitKey(10) & 0xFF == ord('q'):
              break
          else:
            print("Frame is empty!\n")
            break
    cap.release()
    return True
  
  def write_ts_to_file(self, timestamps, filename):
     file_path = "./" + str(self.rootdir) + "/mav0/" + str(filename) + ".txt"
     f = open(file_path, "w")
     for ts in timestamps:
        f.write(str(ts)+"\n")
     f.close()


     
  
def main():

  adma = ADMA_Processor(sys.argv)

  if not os.path.exists(adma.rootdir):
    os.makedirs(adma.rootdir)
    os.makedirs(adma.rootdir + "/mav0")
    os.makedirs(adma.rootdir + "/mav0/cam0")
    os.makedirs(adma.rootdir + "/mav0/cam0/data")
    os.makedirs(adma.rootdir + "/mav0/imu0")

  dict = adma.get_mat_data()

  timestamps = adma.gen_timestamps(dict)
  rate_body_x, rate_body_y, rate_body_z = adma.get_rate_body(dict)
  acc_body_x, acc_body_y, acc_body_z = adma.get_acc_body(dict)

  ts_new = []
  output = adma.get_duplicate_ts(timestamps)
  for item in output:
    length = len(output[item])
    for counter, indices in enumerate(output[item]):
      print(indices)
      if(counter > 0):
        ts_new.append(int(timestamps[indices] + (1e9 / adma.IMU_FREQ * (counter))))
      else:
        ts_new.append(int(timestamps[indices]))

  for i in ts_new:
    print(int(i))

  print("Get video frame timestamps\n")
  frame_ts = adma.get_video_ts(ts_new)
  print("Done!\n")

  print("Find the nearest ts\n")
  new_frame_ts = []
  for counter, ts in enumerate(frame_ts):
      nearest = adma.find_nearest_ts(ts_new, ts)
      print("TS: " + str(ts))
      print("NEAREST: " + str(nearest))
      for count, ts_high in enumerate(ts_new):
         if ts_high == nearest:
            print("TS_HIGH: " + str(ts_high))
            new_frame_ts.append(int(ts_high))
  print("Done\n")
  print("Save frame by frame\n")
  adma.save_frame_by_frame(adma.video_file, new_frame_ts)
  print("Done\n")
  print("Save timestamps in timestamps.txt\n")
  adma.write_ts_to_file(new_frame_ts,"timestamps")
  adma.write_ts_to_file(ts_new, "imu_timestamps")
      

  with open(str(adma.rootdir) + '/mav0/imu0/data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    field = ["#timestamp [ns]","w_RS_S_x [rad s^-1]","w_RS_S_y [rad s^-1]","w_RS_S_z [rad s^-1]","a_RS_S_x [m s^-2]","a_RS_S_y [m s^-2]","a_RS_S_z [m s^-2]"]
    writer.writerow(field)

    for count, ts in enumerate(ts_new):
      field = [
              int(ts),
              "{:.18f}".format(float(rate_body_x[count])), 
              "{:.18f}".format(float(rate_body_y[count])),
              "{:.18f}".format(float(rate_body_z[count])),
              "{:.18f}".format(float(acc_body_x[count])), 
              "{:.18f}".format(float(acc_body_y[count])),
              "{:.18f}".format(float(acc_body_z[count]))]
      writer.writerow(field)



  

if __name__ == "__main__":
   main()