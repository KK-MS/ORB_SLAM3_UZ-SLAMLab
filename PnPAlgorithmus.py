import numpy as np
import cv2 
from cv2 import aruco
import time
import math
import statistics


# class PnPSolver():
    

marker0_gps = [47.98088436540192, 10.233872780711508]
marker1_gps = [47.98086506470829, 10.233779573950567]

Passmarker = np.array(sorted([31,30]))#,MS2,MS3,MS4]) #Liste mit den Markern

#Ground Truth Punkte in der Form: [[x1,y1,z1]
#                                  [x2,y2,z2]]
# Jeder Marker hat vier Ecken, die ale vermessen werden müssen. Muss mit dem Markernummern korrespondieren!

    
sidelength = 280
Passpunkte = np.array([[0.0,0,0],
                       [sidelength,0,0],
                       [sidelength,sidelength,0],
                       [0,sidelength,0]])
 
max_distance = sidelength * 17

aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
Aufnahme = cv2.VideoCapture(r"/home/yassine/Schreibtisch/video_output_left.mp4")

#Kalibrierdaten laden

Kmatrix = np.array([[382.613,   0.        , 320.183],
                    [  0.        , 382.613, 236.455],
                    [  0.        ,   0.        ,   1.        ]])


distCoeffs = np.array([[ 0.,  0.0, -0.0,  0.0, -0.0]])

marker0_vecs = []
marker0_frames = []
marker1_vecs = []
marker1_frames = []
ts_list = []
pos_list = []
rot_list = []
frameNo_list = []
frameTS_list = []
marker0_traj_pos = []
marker1_traj_pos = []
dist_0_list = []
dist_1_list = []

    
    # def __init__():
    #     print("Starting")

def rot_params_rv( rvecs):
    R = cv2.Rodrigues(rvecs)[0]
    roll = 180*math.atan2(-R[2][1], R[2][2])/math.pi
    pitch = 180*math.asin(R[2][0])/math.pi
    yaw = 180*math.atan2(-R[1][0], R[0][0])/math.pi
    rot_params= [roll,pitch,yaw]
    return rot_params

def filter_indices( marker0_frames,frameNo_list):
    """
    This function returns the indices in marker0_frames, for which there is an entry in the trajectory file

    """
    indices = np.intersect1d(marker0_frames,frameNo_list,return_indices = True)[1]
    return indices

def check_before_marker(r_vec,t_vec,max_distance):
    """
    This function returns True if the measured marker may be used for further calculations
 
    Parameters
    ----------
    r_vec : TYPE
        DESCRIPTION.
    t_vec : TYPE
        DESCRIPTION.
    max_distance : TYPE, optional
        DESCRIPTION. The default is 4500.
 
    Returns
    -------
    bool
        DESCRIPTION.
 
    """
    Rmat = cv2.Rodrigues(r_vec)[0]
    if rot_params_rv(cv2.Rodrigues(Rmat.T)[0])[1]<0:
        return False
    else:
        if np.linalg.norm(t_vec) < max_distance:
            return True

def MeasureAndShow( Kmatrix, distCoeffs, Passmarker, Passpunkte):
    # Capture frame-by-frame
    frameCount = 0
    ret, frame = Aufnahme.read()
    
    if ret:
        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        # aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        # Display the resulting frame
        cv2.imshow('Live View',frame_markers)
        
        if corners: #Check if markers are detected
            
            if np.intersect1d(ids,Passmarker).size > 0:
          
                # schauen, ob die vorgegebenen Nummern erkannt worden sind und den Indices rausfinden
                indices = np.intersect1d(Passmarker,ids,return_indices=True)[2]
                for i in range(len(indices)):
                    _, rVec, tVec = cv2.solvePnP(Passpunkte.astype('float32'), corners[indices[i]], Kmatrix, distCoeffs, flags = cv2.SOLVEPNP_ITERATIVE)
                    
                    Rmat = cv2.Rodrigues(rVec)
                    #tVec = np.dot(-Rmat[0].T,tVec)
                    # print('Marker Nr.: ' + str(ids[indices[i]]))
                    # print (str(tVec))
                    # print(np.linalg.norm(tVec))
                    frame_number = Aufnahme.get(cv2.CAP_PROP_POS_FRAMES)
                    if(check_before_marker(rVec, tVec, max_distance)):
                        if (ids[indices[i]] == Passmarker[0]):
                            if int(frame_number) in frameNo_list:
                                marker0_vecs.append([tVec,frame_number])
                                marker0_frames.append(frame_number)
                        elif (ids[indices[i]] == Passmarker[1]):
                            if int(frame_number) in frameNo_list:
                                marker1_vecs.append([tVec,frame_number])
                                marker1_frames.append(frame_number)
                        # now write the data to a text file! x,y,z_time_number of recognized points
    

      
    return ret

def draw_cameras( t_vecs,r_vecs):
    import matplotlib.pyplot as plt
    
    
    x = [t_vecs[i][0] for i in range(len(t_vecs))]
    y = [t_vecs[i][1] for i in range(len(t_vecs))]
    z = [t_vecs[i][2] for i in range(len(t_vecs))]
    
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(projection='3d')
    ax.scatter(x, y, z)
    
    border = np.amax(np.abs(t_vecs))
    ax.set_xlim([-border, border])
    ax.set_ylim([-border, border])
    ax.set_zlim([-border, border])
    
    # ax.plot([0,1], [0,1],zs=[0,1],color = 'g')
    
    #now draw direction of the cameras
    length = np.linalg.norm(t_vecs[0]-t_vecs[1])*200
    vec = np.array([[0],
                    [0],
                    [length]])
    for i in range(len(t_vecs)):
        
        r_mat = cv2.Rodrigues(np.array(rot_list[i]))[0] 
        p1 = t_vecs[i]
        p2 = t_vecs[i] + np.dot(vec.T,r_mat)[0]
        
        ax.plot([p1[0],p2[0]], [p1[1],p2[1]],zs=[p1[2],p2[2]],color = 'g')
    
    plt.show()
    
def split_data( data):
    tsl = []
    pl  = []
    rl = []
    fnl = []
    ftsl = []
    for i in range(len(data)):
        line = data[i].split(" ")
        tsl.append(float(line[0]))
        pl.append(np.array([float(line[1]),float(line[2]),float(line[3])]))
        rl.append(np.array([float(line[4]),float(line[5]),float(line[6])]))
        fnl.append(int(line[8]))
        ftsl.append(int(line[9]))
    
    return tsl, pl, rl, fnl, ftsl

    
def calculate_orb_points():
        
    print("Die Messung startet, zum Beenden s drücken.")
    
    # Read data from trajectory file and split into lists
    file = r"/home/yassine/ws/ORB_SLAM3_UZ-SLAMLab/CameraTrajectory_mod.txt"
    with open(file, 'r') as infile:
        # Read the contents of the file into memory.
        data = infile.read()
    
    # Return a list of the lines, breaking at line boundaries.
    my_list = data.splitlines()
    ts_list, pos_list, rot_list, frameNo_list, frameTS_list = split_data(my_list)
    
    print(pos_list)
    
    
    
    while not (cv2.waitKey(1) & 0xFF == ord('s')) :
        ret = MeasureAndShow(Kmatrix, distCoeffs, Passmarker, Passpunkte)
        
    marker_indices = filter_indices(marker0_frames, frameNo_list)
    
    # Calculate distances between first position from where marker was detected
    # and the next positions where same marker gets detected
    marker1_indices = []
    for i in marker1_vecs:
        print(len(marker1_vecs))
        print(i)
        print("\n")
        d3 = np.linalg.norm(marker1_vecs[0][0]-i[0])
        index = frameNo_list.index(int(i[1]))
        print(index)
        print("\n")
        marker1_indices.append(index)
        dist_1_list.append(d3)
    
    print(marker1_indices[0])
    orb_vec_1 = pos_list[marker1_indices[0]]
    dist_1_orb_list = []
    for i in marker1_indices:
        dist_1_orb_list.append(np.linalg.norm(orb_vec_1-pos_list[i]))
        
    max_dist_marker_1 = max(dist_1_list)
    
    scale1 = dist_1_list[dist_1_list.index(max_dist_marker_1)] / dist_1_orb_list[dist_1_list.index(max_dist_marker_1)]
    
    marker1_vec1 = [arr[0] for arr in marker1_vecs[0][0]]
    marker1_point = pos_list[int(marker1_indices[0])] + (marker1_vec1 / scale1)
    
    marker0_indices = []
    for i in marker0_vecs:   
        d3 = np.linalg.norm(marker0_vecs[0][0]-i[0])
        index = frameNo_list.index(int(i[1]))
        marker0_indices.append(index)
        dist_0_list.append(d3)
        
    orb_vec_0 = pos_list[marker0_indices[0]]
    dist_0_orb_list = []
    for i in marker0_indices:
        dist_0_orb_list.append(np.linalg.norm(orb_vec_0-pos_list[i]))
        
    max_dist_marker_0 = max(dist_0_list)
    scale0 = dist_0_list[dist_0_list.index(max_dist_marker_0)] / dist_0_orb_list[dist_0_list.index(max_dist_marker_0)]
    marker0_vec1 = [arr[0] for arr in marker0_vecs[0][0]]
    marker0_point = pos_list[int(marker0_indices[0])] + (marker0_vec1 / scale0)
    
        
    Aufnahme.release()
    cv2.destroyAllWindows()
    print("Messung durch den User beendet")
            
    return marker0_point, marker1_point
    
    

# obj = PnPSolver()

# obj.calculate_orb_points()
