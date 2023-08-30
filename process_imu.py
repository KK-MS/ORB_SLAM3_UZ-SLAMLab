import pandas as pd
import sys
import csv
import cv2
import os
import time
from datetime import datetime


class IMU_Processor:
    
  ts_list = None
  acc_list = None
  gyro_list = None
  ts_ns = None
  
  def __init__(self, acc_path, gyro_path):
     self.ts_list = self.load_ts(acc_path)
     self.acc_list = self.load_acc(acc_path)
     self.gyro_list = self.load_gyro(gyro_path)

  def load_ts(self, filepath):
    timestamps = pd.read_csv(filepath, usecols=[0,1])
    return timestamps

  def load_acc(self, filepath):
    accelerations = pd.read_csv(filepath, usecols=[2,3,4])
    return accelerations

  def load_gyro(self, filepath):
      velocities = pd.read_csv(filepath, usecols=[2,3,4])
      return velocities
  
  def convert_date(self, date):
     first_date = date
     dt_object = datetime.strptime(first_date, "%Y-%m-%dT%H:%M:%S.%fZ")
     unix_timestamp = dt_object.timestamp()
     unix_timestamp_ns = int(unix_timestamp * 1e9)
     self.ts_ns = unix_timestamp_ns
     return unix_timestamp_ns
  
  def gen_timestamps(self):
     timestamps_ns = []
     for index, row in self.ts_list.iterrows():
        timestamps_ns.append(self.convert_date(row[1]))
     return timestamps_ns
  
  def gen_csv(self,timestamps_ns):
     with open('./data/mav0/imu0/data.csv', 'w', encoding='UTF8') as f:
        writer = csv.writer(f)
        header = ["#timestamp [ns]","w_RS_S_x [rad s^-1]","w_RS_S_y [rad s^-1]","w_RS_S_z [rad s^-1]","a_RS_S_x [m s^-2]","a_RS_S_y [m s^-2]","a_RS_S_z [m s^-2]"]
        writer.writerow(header)
        for i in range(len(timestamps_ns)):
           data = [timestamps_ns[i], self.gyro_list.iloc[i,0], self.gyro_list.iloc[i,1], self.gyro_list.iloc[i,2], self.acc_list.iloc[i,0],self.acc_list.iloc[i,1], self.acc_list.iloc[i,2]]
           writer.writerow(data)
        return True
  
  def save_frame_by_frame(self, filepath, timestamps):
    cap = cv2.VideoCapture(filepath)
    basepath = "./data/mav0/cam0/data/"
    counter = 0
    while(True):
       if cap.isOpened():
          ret,frame = cap.read()
          filename = str(timestamps[counter]) + ".png"
          imagePath = basepath + filename
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
     file_path = "./data/mav0/" + str(filename) + ".txt"
     f = open(file_path, "w")
     for ts in timestamps:
        f.write(str(ts)+"\n")
     f.close()

  # def find_nearest_ts(self, ts_high_freq, ts):
  #    return min(ts_high_freq, key=lambda x:abs(ts))
  def find_nearest_ts(self,ts_high_freq, K):
    return ts_high_freq[min(range(len(ts_high_freq)), key = lambda i: abs(ts_high_freq[i]-K))]
  
  
  def interpolate_timestamps(self, camera_timestamps, inertial_timestamps):
    interpolated_camera_timestamps = []

    for inertial_idx, inertial_timestamp in enumerate(inertial_timestamps):
        if inertial_idx == 0:
            interpolated_camera_timestamps.append(camera_timestamps[0])
            continue

        prev_inertial_timestamp = inertial_timestamps[inertial_idx - 1]
        prev_camera_timestamp = interpolated_camera_timestamps[-1]

        time_difference = inertial_timestamp - prev_inertial_timestamp
        camera_time_difference = time_difference * (camera_timestamps[1] - camera_timestamps[0]) / (inertial_timestamps[1] - inertial_timestamps[0])

        interpolated_camera_timestamp = prev_camera_timestamp + camera_time_difference
        interpolated_camera_timestamps.append(int(interpolated_camera_timestamp))

    return interpolated_camera_timestamps

def main():
   print(sys.argv)
   rootdir = "./data"
   if not os.path.exists(rootdir):
     os.makedirs(rootdir)
     os.makedirs(rootdir + "/mav0")
     os.makedirs(rootdir + "/mav0/cam0")
     os.makedirs(rootdir + "/mav0/cam0/data")
     os.makedirs(rootdir + "/mav0/imu0")

   imu = IMU_Processor(sys.argv[1], sys.argv[2])
   imu2 = IMU_Processor(sys.argv[3], sys.argv[4])

  ##  imu.convert_date()
  ##  imu2.convert_date()

   timestamps = imu.gen_timestamps()
   #print(timestamps)
   timestamps_high = imu2.gen_timestamps()

   new_ts_list = []
   print(timestamps_high)
   for counter, ts in enumerate(timestamps):
      nearest = imu.find_nearest_ts(timestamps_high, ts)
      print("TS: " + str(ts))
      print("NEAREST: " + str(nearest))
      for count, ts_high in enumerate(timestamps_high):
         if ts_high == nearest:
            print("TS_HIGH: " + str(ts_high))
            new_ts_list.append(ts_high)

   #new_ts_list = imu.interpolate_timestamps(timestamps, timestamps_high)


   ##print(new_ts_list)
   ##imu.gen_csv(timestamps)
   #imu2.gen_csv(timestamps_high)
   imu2.gen_csv(timestamps_high)

   ##print(new_ts_list)       
  
   imu.save_frame_by_frame(sys.argv[5], timestamps)
   #imu.write_ts_to_file(new_ts_list, "timestamps")
   imu.write_ts_to_file(timestamps, "timestamps")
  
   

if __name__ == "__main__":
   main()
        