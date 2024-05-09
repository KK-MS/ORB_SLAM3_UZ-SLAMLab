import matplotlib.pyplot as plt
import pandas as pd
import geopy.distance
import numpy as np
import scipy
import math
import cv2
import sys
from mapping import CustomGoogleMapPlotter
import folium
from cv2 import aruco
import time
import statistics
import matplotlib


matplotlib.use("TkAgg")
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



def plotgps(lat,lon):
    plt.scatter(lon, lat, color='blue', marker='o')

    # Beschrifte die Achsen und den Plot
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('GPS-Koordinaten')
    
    # Zeige den Plot an
    plt.show()
    

def create_trajectory_map(trajectories, output_file='trajectories_map.html', colors=['blue', 'red', 'green']):
    # Erstelle eine Folium-Karte
    m = folium.Map(location=trajectories[0][0], zoom_start=15)

    # Füge Polylinien für jede Trajektorie hinzu
    c = 0
    for trajectory in trajectories:
        folium.PolyLine(locations=trajectory, color=colors[c]).add_to(m)
        c = c+1
    # Speichere die Karte als HTML-Datei
    m.save(output_file)


def calcEuclDist(p1, p2):
    dist = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
    return dist

def calcGpsDist(coords1, coords2):
    return geopy.distance.geodesic(coords1,coords2).m

def calcScale(gpsDist, euclDist):
    return  gpsDist / euclDist
    

def reOrientate(absolute,model):
    """
    Transforms a model point cloud "model" into a corresponding, absolute pointcloud "absolute"
    Gives out the rotation, translation and scale of the affine Transformation performed.
    Expects a numpy array with the following form: np.transpose(np.array(([x1,y1,z1],
                                                                          [x2,y2,z2],
                                                                          [x3,y3,z3])))
    """
    d = absolute.copy()
    m = model.copy()

    #calculate the mean vector
    dd = np.mean(d,0)
    md = np.mean(m,0)


    l,w = m.shape

    dc = (d - np.tile(dd, (l,1))).T
    mc = (m - np.tile(md, (l,1))).T
    # divide by l because l is the dimension of the euclidean space (=3)
    H = np.dot(mc,dc.T)/l

    U,D,VT = np.linalg.svd(H)

    V = VT.T

    V_=V.copy()
    V_[:,2]=-V_[:,2]

    R = np.dot(V,U.T)

    if np.linalg.det(R)<0:
        R = np.dot(V_,U.T)

    # Aus Umeyama paper:
    sigma_x = np.mean(np.sum(mc*mc,0))
    scale = np.trace(np.dot(np.diag(D),np.eye(3)))/sigma_x
    T = dd - scale*np.dot(R,md)
    #testen:
    #print ("m = " +str(T+ scale*np.dot(R,d.T).T))
    return R,T,scale

def get_rotation_angle(gps_p1, gps_p2, cart_p1, cart_p2):
    # alpha = gamma - beta
    distance1, rho1 = polar(cart_p2[0], cart_p2[1])
    print("\n===================\n")
    print("Distance 1: " + str(distance1))
    print("RHO1: " + str(rho1))
    beta = 90 - rho1
    print("BETA: " + str(beta))
    distance2, rho2 = transGeo2Polar(cart_p1[0], cart_p1[1], cart_p2[0], cart_p2[1])
    print("Distance2: " + str(distance2))
    print("RHO2: " + str(rho2))
    gamma = 360 - rho2
    print("GAMMA: " + str(gamma))
    
    return gamma - beta

def polar(x, y) -> tuple:
  """returns rho, theta (degrees)"""
  return np.hypot(x, y), np.degrees(np.arctan2(y, x))

def transGeoCart2Geo(fLat0, fLon0, sx, sy):

    # Radius Erdmittelpunkt bis Äquator
    r = 6378137.0
    # Exzentrizität
    en = 0.0818191908426
    #Pi, sin
    pi = np.pi
    # Radius Richtung Norden
    rn = (r*(1-en*en))/((1-en*en*np.sin(fLat0)*np.sin(fLat0))**1.5)
    # Radius Richtung Osten
    ro = r/((1-en**2*np.sin(fLat0)**2)**(0.5))
    # Skalierungsfaktor Norden in m/° Abstand in Meter nach Norden, zwischen Breitengraden
    sfn = rn*(pi/180)
    # Skalierungsfaktor Osten in m/° Abstand in Meter nach Osten, zwischen Längengraden
    sfo = ro*(pi/180)*np.cos(fLat0*pi/180)
    # Abstand zwischen den beiden GPS-Punkten in X-Richtung Osten in Meter
    #s = haversine((lat0,lon0),(lat1,lon1))/1000
    # Retransformation

    fLon1 = (fLon0+(sx/sfo))
    fLat1 = (fLat0+(sy/sfn))

    return [float(fLat1), float(fLon1)]

def transGeo2Polar(fLat0, fLon0, fLat1, fLon1):

    # Radius Erdmittelpunkt bis Äquator
    r = 6378137.0
    # Exzentrizität
    en = 0.0818191908426
    #Pi, sin
    pi = np.pi
    # Radius Richtung Norden
    rn = (r*(1-en*en))/((1-en*en*np.sin(fLat0)*np.sin(fLat0))**1.5)
    # Radius Richtung Osten
    ro = r/((1-en**2*np.sin(fLat0)**2)**(0.5))
    # Skalierungsfaktor Norden in m/° Abstand in Meter nach Norden, zwischen Breitengraden
    sfn = rn*(pi/180)
    # Skalierungsfaktor Osten in m/° Abstand in Meter nach Osten, zwischen Längengraden
    sfo = ro*(pi/180)*np.cos(fLat0*pi/180)
    # Abstand zwischen den beiden GPS-Punkten in X-Richtung Osten in Meter
    sx = (fLon1-fLon0)*sfo
    # Abstand zwischen den beidn GPS-Punkten in Y-Richtung Norden in Meter
    sy = (fLat1-fLat0)*sfn
    # Distanz zwischen den beiden GPS-Punkten
    s = np.sqrt(sy**2+sx**2)
    #arctan2 - angle in rad
    angle_rad = np.arctan2(sx, sy)
    # angle in degree between -180 / 180
    angle_deg = np.degrees(angle_rad)
    bearingangle = (angle_deg+360) % 360
    #s = haversine((lat0,lon0),(lat1,lon1))/1000
    return [float(s),float(bearingangle)]
    #return [float(sx), float(sy)]

def geo_to_polar(reference_latitude, reference_longitude, target_latitude, target_longitude):
    # Umrechnung von Grad zu Bogenmaß
    ref_lat_rad = math.radians(reference_latitude)
    ref_lon_rad = math.radians(reference_longitude)
    target_lat_rad = math.radians(target_latitude)
    target_lon_rad = math.radians(target_longitude)

    # Radius der Erde (angenommen)
    earth_radius = 6371  # in Kilometern

    # Berechnung der Differenzen
    delta_lon = target_lon_rad - ref_lon_rad
    delta_lat = target_lat_rad - ref_lat_rad

    # Polarkoordinaten relativ zum neuen Ursprung
    r = earth_radius * math.sqrt(delta_lat**2 + (math.cos((ref_lat_rad + target_lat_rad) / 2) * delta_lon)**2)
    theta = math.atan2(math.sin(delta_lon) * math.cos(target_lat_rad),
                       math.cos(ref_lat_rad) * math.sin(target_lat_rad) -
                       math.sin(ref_lat_rad) * math.cos(target_lat_rad) * math.cos(delta_lon))

    return r, theta



def cart2geo(geo_point1, geo_point2, cart_p3):
    """
    Converts the cartesian point cart_p3 into geo coordinates given a common 
    reference geo_point1 and a second point geo_point2

    Parameters
    ----------
    geo_point1 : list
        The common reference in geo-coordinates in 1x2 list.
    geo_point2 : list
        geo-coordinates in 1x2 list.
    cart_p3 : list
        The cartesian point to be transformed.

    Returns
    -------
    the point cart_p3 transformed to geo-coordinates.

    """
    coordinates = transGeo2Polar(geo_point1[0], geo_point1[1], geo_point2[0], geo_point2[1])
    alpha = polar(coordinates[0],coordinates[1])[1]

    beta = polar(cart_p3[1],cart_p3[0])[1]
    gamma = beta - alpha
    distance = np.linalg.norm(cart_p3)
    x = math.sin(math.radians(gamma))*distance
    y = math.cos(math.radians(gamma))*distance
    return(transGeoCart2Geo(geo_point1[0], geo_point1[1], x, y))

def rot_params_rv(rvecs):
    R = cv2.Rodrigues(rvecs)[0]
    roll = 180*math.atan2(-R[2][1], R[2][2])/math.pi
    pitch = 180*math.asin(R[2][0])/math.pi
    yaw = 180*math.atan2(-R[1][0], R[0][0])/math.pi
    rot_params= [roll,pitch,yaw]
    return rot_params

def vecs2geo(geo_point1, geo_point2, vecs, marker0_vec, marker1_vec, scale_vecs = True):
    """
    This function transforms a list of metric 3d-vectors into geo-coordinates. 
    The geo_point 1 and geo_point2 are at the beginning and end of the vectors.
    Set Scale_vecs to True if the vectors need to be scaled with the two 
    geo points so that they are metric.

    Parameters
    ----------
    geo_point1 : list
        The common reference in geo-coordinates in 1x2 list.
    geo_point2 : list
        geo-coordinates in 1x2 list.
    vecs : list
        nx3 list containing the x,y, and z coordinates for every row of the list.

    Returns
    -------
    Latitude list, longitude list.

    """
    
    #erst ggf. skalieren:
    
    # if scale_vecs == True:
    #     eucl_dist = np.linalg.norm(vecs[0]-vecs[-1])
    #     gps_dist = transGeo2Polar(geo_point1[0], geo_point1[1], geo_point2[0], geo_point2[1])[0]
    #     scale = gps_dist/eucl_dist
    #     vecs = vecs*scale
    
    if scale_vecs == True:
        eucl_dist = np.linalg.norm(marker0_vec-marker1_vec)
        gps_dist = transGeo2Polar(geo_point1[0], geo_point1[1], geo_point2[0], geo_point2[1])[0]
        scale = gps_dist/eucl_dist
        vecs = vecs*scale
    
    """
    Winkel finden, sodass der Punkt P2 x0 hat.
    """
    
    
    #angle = np.arctan2(vecs[-1][0],vecs[-1][1])
    angle = np.arctan2(marker1_vec[0],marker1_vec[1])

    r_mat = np.array([[np.cos(angle),-np.sin(angle),0],
                      [np.sin(angle),np.cos(angle),0],
                      [0,0,1]])
    print(vecs)
    vecs_new = np.dot(vecs,r_mat.T)

    """
    Jetzt noch die Vektoren von Norden auf den zweiten GPS-Punkt drehen
    """

    bearing = np.radians(transGeo2Polar(geo_point1[0], geo_point1[1], geo_point2[0], geo_point2[1])[1])

    r_mat = np.array([[np.cos(bearing),-np.sin(bearing),0],
                      [np.sin(bearing),np.cos(bearing),0],
                      [0,0,1]])


    vecs_new = np.dot(vecs_new,r_mat)
    
    # Uncomment these lines for a Plot
    # x = vecs_new[:,0]
    # y = vecs_new[:,1]
    # z = vecs_new[:,2]

    # fig = plt.figure()
    # ax = fig.add_subplot(projection='3d')
    # plt.xlim(-100, 100)
    # plt.ylim(-100, 100)
    # ax.scatter(x, y, z)

    # ax.set_xlabel('X Label')
    # ax.set_ylabel('Y Label')
    # ax.set_zlabel('Z Label')
    # ax.set_zlim3d(-10, 10)
    # plt.show()
    
    
    lat_list = []
    lon_list = []
    for vec in vecs_new:

        lat, lon = transGeoCart2Geo(geo_point1[0], geo_point1[1], vec[0], vec[1])
        lat_list.append(lat)
        lon_list.append(lon)
    

    return lat_list,lon_list


#data = np.load('C:/Users/Yassine.Zouad/Desktop/masterarbeit/plot/nparray_all_webcam.npy', allow_pickle=True)
       
print("Die Messung startet, zum Beenden s drücken.")

# Read data from trajectory file and split into lists
file = r"/home/yassine/ws/ORB_SLAM3_UZ-SLAMLab/CameraTrajectory_mod.txt"
with open(file, 'r') as infile:
    # Read the contents of the file into memory.
    data = infile.read()

# Return a list of the lines, breaking at line boundaries.
my_list = data.splitlines()
ts_list, pos_list, rot_list, frameNo_list, frameTS_list = split_data(my_list)

while not (cv2.waitKey(1) & 0xFF == ord('s')) :
    ret = MeasureAndShow(Kmatrix, distCoeffs, Passmarker, Passpunkte)
    
marker_indices = filter_indices(marker0_frames, frameNo_list)

# Calculate distances between first position from where marker was detected
# and the next positions where same marker gets detected
marker1_indices = []
for i in marker1_vecs:
    d3 = np.linalg.norm(marker1_vecs[0][0]-i[0])
    index = frameNo_list.index(int(i[1]))
    marker1_indices.append(index)
    dist_1_list.append(d3)

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

# Display trajectory and markers


fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(projection='3d')

ax.scatter(np.array(pos_list)[:,0], np.array(pos_list)[:,1], np.array(pos_list)[:,2])
ax.scatter(marker0_point[0], marker0_point[1], marker0_point[2], color = 'g')
ax.scatter(marker1_point[0], marker1_point[1], marker1_point[2], color = 'g')

plt.show()

    
Aufnahme.release()
cv2.destroyAllWindows()
print("Messung durch den User beendet")


df = pd.read_csv(r'/home/yassine/ws/plot/orb_trajectories/CameraTrajectory.txt', sep=' ', encoding='cp1252')

df.head()

t = df.time
X = df.x
Y = df.y
Z = df.z
QW = df.qw
QX = df.qx
QY = df.qy
QZ = df.qz

data = np.column_stack((t, X, Y, Z, QW, QX, QY, QZ))
data = data.astype(np.float64)




geo_point1 = [47.98088436540192, 10.233872780711508]
geo_point2 = [47.98086506470829, 10.233779573950567]
geo_endpoint = [47.98086506470829, 10.233779573950567]



x = data[:,1]
y = data[:,2]
z = data[:,3]


fig = plt.figure()
ax = fig.add_subplot(projection='3d')



# For each set of style and range settings, plot n random points in the box
# defined by x in [23, 32], y in [0, 100], z in [zlow, zhigh].

ax.scatter(x, y, z)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()

"""
Now transform so that the measurement is planar to the ground
"""

vecs = data[:,1:4]

xs = vecs[:,0]
ys = vecs[:,1]
zs = vecs[:,2]

# Form a inhomogeneous system of linear equations
A = np.hstack((vecs[:,:2], np.ones((len(xs),1))))
b = vecs[:,2]
res = scipy.optimize.lsq_linear(A, b)

# ebenengleichung: ax+by+c = z
a = res.x[0]
b = res.x[1]
c = res.x[2]

# eckpunkte der Wunschebene
corners = np.array([[0,0,0],
                    [1,0,0],
                    [1,1,0],
                    [0,1,0]],dtype = 'float64')

# Transformieren der Eckpunkte in die vorhandene Ebene
corners_transformed_z = []
for i in range(len(corners)):
    corners_transformed_z.append(corners[i][0]*a+corners[i][1]*b+c)

corners_transformed = corners.copy()
corners_transformed[:,2] = corners_transformed_z

#R,t,s = reOrientate(corners_transformed,corners)
R,t,s = reOrientate(corners, corners_transformed)

gpsDist = calcGpsDist(geo_point1, geo_endpoint)

vecs_transformed = np.dot(R,vecs.T).T


gps_array_lat,gps_array_lon = vecs2geo(geo_point1, geo_point2, vecs_transformed, marker0_point, marker1_point, scale_vecs = True)

gps_point_list = [[gps_array_lat[i],gps_array_lon[i]] for i in range(len(gps_array_lat))]

#plotgps(gps_array_lat[0],gps_array_lon[0])

x_axis_x = []
x_axis_y = []
y_axis_x = []
y_axis_y = []
x_axis = []
y_axis = []

lat = geo_point1[0]
lon = geo_point1[1]
for i in range(1000):
    x_axis_x.append(lat)
    x_axis_y.append(geo_point1[1])
    x_axis.append([lat, geo_point1[1]])
    y_axis_x.append(geo_point1[0])
    y_axis_y.append(lon)
    y_axis.append([geo_point1[0], lon])
    lat = lat + 0.000001
    lon = lon + 0.000001    

trajectories = [gps_point_list, x_axis, y_axis]    
create_trajectory_map(trajectories)
    
initial_zoom = 20
gmap = CustomGoogleMapPlotter(gps_array_lat[0], gps_array_lon[0], initial_zoom,
                              map_type='satellite')
gmap.color_scatter(gps_array_lat, gps_array_lon, color = 'c', colormap='coolwarm', size = 0.05)
gmap.color_scatter([geo_point1[0],geo_endpoint[0],geo_point1[0],geo_point2[0]] ,
                    [geo_point1[1],geo_endpoint[1],geo_point1[1],geo_point2[1]] ,
                    color = 'r', colormap='coolwarm', size = 0.4)


gmap.color_scatter(x_axis_x, x_axis_y, color = 'g', colormap='coolwarm', size = 0.05)
gmap.color_scatter(y_axis_x, y_axis_y, color = 'g', colormap='coolwarm', size = 0.05)
    
gmap.color_scatter

print("Generating map.\n")
gmap.draw("karte.html")