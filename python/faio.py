#!/usr/bin/env python3


from __future__ import print_function

import time
# import import_pcl
import mfpfh.pcl_interface.import_pcl as import_pcl
import mfpfh.pcl_interface.pcl_types as pcl_types

import os
import numpy as np
np.set_printoptions(threshold=np.inf)
import matplotlib
from matplotlib import cm
import matplotlib.pyplot as plt
from scipy import stats
import pandas

#for use sklearn
from sklearn.mixture import GaussianMixture
from sklearn.cluster import KMeans

from sklearn.metrics import silhouette_score,silhouette_samples

#for check elbow point
from kneed import KneeLocator

#for change  value from name
import re

#for save pcd file
#import open3d as o3d




def valid_imshow_data(data):
    data = np.asarray(data)
    if data.ndim == 2:
        return True
    elif data.ndim == 3:
        if 3 <= data.shape[2] <= 4:
            return True
        else:
            print('The "data" has 3 dimensions but the last dimension '
                  'must have a length of 3 (RGB) or 4 (RGBA), not "{}".'
                  ''.format(data.shape[2]))
            return False
    else:
        print('To visualize an image the data must be 2 dimensional or '
              '3 dimensional, not "{}".'
              ''.format(data.ndim))
        return False

def load_features_folder(folder:str, object:str, scale2load:float) -> list:
    #clouds = []
    for file in os.listdir(f"{folder}/{object}/features"):
        filename = file[0:file.rfind('.')]
        fileext = file[file.rfind('.')+1:]
        if fileext != "pcd":
            continue
        #print(type(filename))
        #print(filename)
        keywords = filename.split('_')
        #print(keywords)
        print(keywords[1])
        for info in keywords:
            if info[0] == 's':
                scale = float(info[1:])
            elif info[0] == 'r':
                radius = float(info[1:])
            elif info[0:2] == 'px':
                px = float(info[2:])
            elif info[0:2] == 'py':
                py = float(info[2:])
            elif info[0:2] == 'pz':
                pz = float(info[2:])
            elif info[0:2] == 'ox':
                ox = float(info[2:])
            elif info[0:2] == 'oy':
                oy = float(info[2:])
            elif info[0:2] == 'oz':
                oz = float(info[2:])
            elif info[0:2] == 'ow':
                ow = float(info[2:])
            # else:
            #     print(f"unrecognized: {info}")
        #if scale == scale2load:
            #print(f"Loading {file}")
            #clouds.append(import_pcl.read(f"{folder}/{object}/features/{file}"))
            #print()
    #return clouds
    return filename

def concatenate_clouds(clouds:list) -> np.array:
    result = np.nan
    for cloud in clouds:
        if result is not np.nan:
            result = np.concatenate([result, cloud.points])
        else:
            result = cloud.points
    return result


#### definition for fonction GMM  ########

def trace_histro(dict1):
    c=len(dic1)
    c=list(range(c)) 
    lst = []
    for i in c:
        globals()[f"c{i}"] = dic1[i]    #creater variable dynamic 


    plt.figure()
    #plt.title("Les courbes de FPFH en class 0~5")
    plt.subplot(2,3,1)
    plt.title("class 0")
    plt.xlabel('33 bins')
    plt.ylabel('Value for FPFH')
    for i in c0:                 
        a1=plt.plot(fpfh.points[i],color='g')
    plt.subplot(2,3,2)
    plt.title("class 1")
    plt.xlabel('33 bins')
    plt.ylabel('Value for FPFH')
    for i in c1:                 
        a2=plt.plot(fpfh.points[i],color='g')
    plt.subplot(2,3,3)
    plt.title("class 2")
    plt.xlabel('33 bins')
    plt.ylabel('Value for FPFH')
    for i in c2:                 
        a3=plt.plot(fpfh.points[i],color='g')
    plt.subplot(2,3,4)
    plt.title("class 3")
    plt.xlabel('33 bins')
    plt.ylabel('Value for FPFH')
    for i in c3:                 
        a4=plt.plot(fpfh.points[i],color='g')
    plt.subplot(2,3,5)
    plt.title("class 4")
    plt.xlabel('33 bins')
    plt.ylabel('Value for FPFH')
    for i in c4:                 
        a5=plt.plot(fpfh.points[i],color='g')
    plt.subplot(2,3,6)
    plt.title("class 5")
    plt.xlabel('33 bins')
    plt.ylabel('Value for FPFH')
    for i in c5:                 
        a6=plt.plot(fpfh.points[i],color='g')
    

def trace_bar(dict1):
    c=len(dic1)
    c=list(range(c)) 
    lst = []
    for i in c:
        globals()[f"c{i}"] = dic1[i]    #creater variable dynamic 

    c=list(range(33))

    plt.figure()
    plt.title("Les courbes de FPFH en class 0~5")
    plt.subplot(2,3,1)
    plt.title("class 0")
    plt.xlabel('33 bins')
    plt.ylabel('Value for FPFH')
    for i in c0:                 
        a1=plt.bar(c,fpfh.points[i],color='r')
    plt.subplot(2,3,2)
    plt.title("class 1")
    plt.xlabel('33 bins')
    plt.ylabel('Value for FPFH')
    for i in c1:                 
        a2=plt.bar(c,fpfh.points[i],color='g')
    plt.subplot(2,3,3)
    plt.title("class 2")
    plt.xlabel('33 bins')
    plt.ylabel('Value for FPFH')
    for i in c2:                 
        a3=plt.bar(c,fpfh.points[i],color='b')
    plt.subplot(2,3,4)
    plt.title("class 3")
    plt.xlabel('33 bins')
    plt.ylabel('Value for FPFH')
    for i in c3:                 
        a4=plt.bar(c,fpfh.points[i],color='c')
    plt.subplot(2,3,5)
    plt.title("class 4")
    plt.xlabel('33 bins')
    plt.ylabel('Value for FPFH')
    for i in c4:                 
        a5=plt.bar(c,fpfh.points[i],color='m')
    plt.subplot(2,3,6)
    plt.title("class 5")
    plt.xlabel('33 bins')
    plt.ylabel('Value for FPFH')
    for i in c5:                 
        a6=plt.bar(c,fpfh.points[i],color='y')
    plt.legend([a1,a2,a3,a4,a5,a6],["c0","c1","c2","c3","c4","c5"],loc="upper right")



def save_in_dic(gmm,pointc):

    a=np.shape(pointc)
    a=a[0]
    v=list(range(a)) 
    labels=gmm.predict(pointc)

    dic={}
    for x in v:
        if v[x] in dic:
            dic[v[x]] += [labels[x]]
        else:
            dic[v[x]] = [labels[x]]
    

    dic1={}
    for y in v:
        if labels[y] in dic1:
            dic1[labels[y]] += [v[y]]
        else:
            dic1[labels[y]] = [v[y]]
    
    return dic,dic1
    


def gmm_weights_class(pointc,number):
    print(np.shape(pointc))
    a=np.shape(pointc)
    a=a[0]
    print("size of this set FPFH is %d" %a)

    print("le poids de chaque classe est:")
    gmm = GaussianMixture(n_components=number,random_state=0).fit(pointc)
    print(gmm.weights_)
    
    return gmm

def sk_kmeans(pointc,number):
    km = KMeans(n_clusters=number,init = 'k-means++',random_state=0).fit(pointc)

    return km

####### end   ######


def correlation_FPFH():
    mylist=[]
    for file in os.listdir(f"{folder}/{object}/features"):
        filename = file[0:file.rfind('.')]
        fileext = file[file.rfind('.')+1:]
        if fileext != "pcd":
            continue
        keywords = filename.split('_')
        if keywords[1] == 's1.340000': 
            #print(f"{folder}/{object}/features/{filename}.{extension}")
            mylist.append(f"{folder}/{object}/features/{filename}.{extension}")
            #fpfh = import_pcl.read(f"{folder}/{object}/features/{filename}.{extension}")
        else:
            print(f"we dont need this scale")
            #fpfh = import_pcl.read(f"{folder}/{object}/features/{filename}.{extension}")
    for i in mylist:
        print(type(i))
        for j in mylist:
            fpfh = import_pcl.read(f"{i}")
            fpfh2 = import_pcl.read(f"{j}")
            correlation=np.corrcoef(fpfh.points,fpfh2.points)
            if(valid_imshow_data(correlation)):
                plt.matshow(correlation, cmap='jet_r')
                plt.colorbar()
                i.strip("/home/wzeng/multiscale-fpfh/output/plateforms_wo_sensor_origin/features/")    #title not need this part str , go away
                j.strip("/home/wzeng/multiscale-fpfh/output/plateforms_wo_sensor_origin/features/")
                #plt.title(f"Global at {i}")
                plt.title(f"mac at {i} \n and {j}")
                #plt.savefig(f"mac {filename} and {filename2}.png")  #save resu
                #plt.savefig(f"/home/wzeng/zeng-stage/data_diff_desciptor_mac/mac {i}.png")  #save resu with path specifie
                #plt.close()  #too much window open, need to close to keep the memoire
                plt.show()
    return 0


def correlation_for_2file():  #choose two files fpfh, compare each other to see the correlation
    filename = "features_s0.167500_plateforms_r1.340000_px-0.625000_py-0.250000_pz1.180000_ox-0.653281_oy0.270598_oz0.653282_ow0.270598"
    filename2 ="features_s0.167500_plateforms_r1.340000_px-1.709083_py-0.250000_pz0.627632_ox0.000000_oy0.309017_oz0.000000_ow0.951057"
    fpfh = pcl_types.PointCloud        #declare the type
    fpfh2 = pcl_types.PointCloud       #for second input
    
    fpfh = import_pcl.read(f"{folder}/{object}/features/{filename}.{extension}")
    fpfh2 = import_pcl.read(f"{folder}/{object}/features/{filename2}.{extension}")


    correlation=np.corrcoef(fpfh.points,fpfh2.points)      #matrice correlation
    
    if(valid_imshow_data(correlation)):
        plt.matshow(correlation, cmap='jet_r')
        plt.colorbar()
        plt.title(f"mac at {filename}")
        #plt.title(f"mac at {filename} \n and {filename2}")
        plt.show()
    


def see_autocorrelation():       #choose one files fpfh, compare itself to see the autocorrelation
    ### my way to import data
    #filePath = "/home/wzeng/multiscale-fpfh/output/plateforms_wo_sensor_origin/features"
    #for filename in os.walk(filePath):
    #    result=filename[2]
    


    #fpfh = pcl_types.PointCloud        #declare the type
    #for i in result:
        #fpfh = import_pcl.read(f"{folder}/{object}/features/{i}")     #input pcd one by one
        correlation=np.corrcoef(fpfh.points)          #matrice auto correlation
        if(valid_imshow_data(correlation)):
            plt.matshow(correlation, cmap='jet_r')
            plt.colorbar()
            #plt.title(f"mac at {i}")
            plt.title(filename)
            plt.show()




if __name__ == "__main__":




    extension = "pcd"
    folder = "/home/zeng/multiscale-fpfh/output"
    #object = "plateforms_wo_sensor_origin"
    object = "plateforms_normals_reso3"
    scales = [1.34, 0.67, 0.335, 0.1675, 0.08375, 0.041875, 0.020938, 0.010469]

    filePath = "/home/zeng/multiscale-fpfh/output/plateforms_normals_reso3/features_test_0.16"    #for k to filename
    fileList=os.listdir(filePath)


######   this part for resume all views and find k value for all (every 33 dans un set)


fpfh = pcl_types.PointCloud        #declare the type

c = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]]   #Initialize container size and prepare to import FPFH descriptors
#d = [[]]
#k= [[]]

                

for i in fileList:                   #Import FPFH descriptors by entering a file of the specified scale
    fpfh = import_pcl.read(f"{folder}/plateforms_normals_reso3/features_test_0.16/{i}")     #input pcd one by one
    a=np.shape(fpfh.points)
    a=[0]
    c=np.append(c,fpfh.points,axis=0)

#print(c[0])
c=np.delete(c,0,axis=0)
#print(np.shape(c))

    #d=c.ravel()
    #d=d.resize((1,a))
    #d=np.append(d,d,axis=0)
#print(d)
#print(np.shape(d))




#c=np.delete(c,0,axis=0)

# After importing all descriptors in a scale, confirm their reasonable number of clusters

WCSS=[]
for x in range(1,200):  #200 is the default number of test clusters, which increases or decreases with the number of samples
    kmeans=sk_kmeans(c,x)
    WCSS.append(kmeans.inertia_)

kneedle = KneeLocator(range(1,200),WCSS,S=1.0, curve="convex", direction="decreasing")

#for check the result
#print("the best cluster number for this data is:",kneedle.knee)
#kneedle.plot_knee() 
#plt.show()


#Save the result to a txt
fw =open("test.txt",'w')

#kmeans=sk_kmeans(c,b)
kmeans=sk_kmeans(c,kneedle.knee)
#print(kmeans.cluster_centers_,end=' ')
print(kmeans.cluster_centers_,end="",file = fw)
fw.close()

#Remarks

#The results of the clustering output also need to change the format to PCD with the following rules

#EXAMPLE
"""
[[2.17179019e-01 7.82745955e-02 1.81724317e-01 1.84680684e-01
  1.99991225e+00 9.42966267e+01 2.20794893e+00 3.40774921e-01
  2.42939361e-01 5.94187767e-02 1.90520281e-01 3.13126054e-01
  2.54504455e-01 6.99510445e-01 2.24185191e+00 6.95919511e+00
  7.91894786e+01 6.60092993e+00 2.33484576e+00 7.04917871e-01
  2.85617399e-01 4.16022233e-01 1.03034245e+01 1.01901935e+01
  9.64464865e+00 9.05199320e+00 7.86649770e+00 7.60766428e+00
  7.64102196e+00 8.77626226e+00 9.48271373e+00 9.68927951e+00
  9.74630079e+00]
 [3.21901841e-01 1.81938457e-01 5.37140881e-01 9.09158973e-01
  7.20802683e+00 8.01427168e+01 8.20913297e+00 1.39699478e+00
  6.92870640e-01 1.55864284e-01 2.44253252e-01 1.04659990e+00
  1.20530376e+00 3.09101545e+00 7.15609868e+00 1.49244089e+01
  4.42295991e+01 1.54685795e+01 7.30192206e+00 3.03811548e+00
  1.27608652e+00 1.26227067e+00 1.17561175e+01 1.14537786e+01
  1.04083755e+01 8.72065249e+00 7.15361230e+00 4.60755391e+00
  6.50124570e+00 7.95753505e+00 9.72327021e+00 1.04493777e+01
  1.12684811e+01]
 [3.21305729e+00 1.51017886e+00 3.70240209e+00 5.63896060e+00
  1.40080366e+01 4.41831531e+01 1.31647499e+01 6.36674443e+00
  3.52767816e+00 1.59098334e+00 3.09405572e+00 6.60060627e+00
  5.41710090e+00 6.27193154e+00 8.08174613e+00 1.16790621e+01
  2.22841625e+01 1.23256229e+01 8.61149998e+00 6.44771674e+00
  5.56201356e+00 6.71853747e+00 1.36668681e+01 1.13517073e+01
  9.77251214e+00 7.52578432e+00 5.82444395e+00 4.13818654e+00
  5.85382164e+00 7.91790197e+00 9.70636273e+00 1.11322553e+01
  1.31101562e+01]]


===============================================>   Change the name to  test.txt=====> features_s1.340000_plateforms_normals_reso3_all_in_one.pcd


# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS fpfh
SIZE 4
TYPE F
COUNT 33
WIDTH 3                                           <======= Number of descriptors after corresponding clustering
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 3                                          <======= Number of descriptors after corresponding clustering
DATA ascii
2.17179019e-01 7.82745955e-02 1.81724317e-01 1.84680684e-01
1.99991225e+00 9.42966267e+01 2.20794893e+00 3.40774921e-01
2.42939361e-01 5.94187767e-02 1.90520281e-01 3.13126054e-01
2.54504455e-01 6.99510445e-01 2.24185191e+00 6.95919511e+00
7.91894786e+01 6.60092993e+00 2.33484576e+00 7.04917871e-01
2.85617399e-01 4.16022233e-01 1.03034245e+01 1.01901935e+01
9.64464865e+00 9.05199320e+00 7.86649770e+00 7.60766428e+00
7.64102196e+00 8.77626226e+00 9.48271373e+00 9.68927951e+00
9.74630079e+00
3.21901841e-01 1.81938457e-01 5.37140881e-01 9.09158973e-01
7.20802683e+00 8.01427168e+01 8.20913297e+00 1.39699478e+00
6.92870640e-01 1.55864284e-01 2.44253252e-01 1.04659990e+00
1.20530376e+00 3.09101545e+00 7.15609868e+00 1.49244089e+01               <=======Eliminate spaces and brackets
4.42295991e+01 1.54685795e+01 7.30192206e+00 3.03811548e+00
1.27608652e+00 1.26227067e+00 1.17561175e+01 1.14537786e+01
1.04083755e+01 8.72065249e+00 7.15361230e+00 4.60755391e+00
6.50124570e+00 7.95753505e+00 9.72327021e+00 1.04493777e+01
1.12684811e+01
3.21305729e+00 1.51017886e+00 3.70240209e+00 5.63896060e+00
1.40080366e+01 4.41831531e+01 1.31647499e+01 6.36674443e+00
3.52767816e+00 1.59098334e+00 3.09405572e+00 6.60060627e+00
5.41710090e+00 6.27193154e+00 8.08174613e+00 1.16790621e+01
2.22841625e+01 1.23256229e+01 8.61149998e+00 6.44771674e+00
5.56201356e+00 6.71853747e+00 1.36668681e+01 1.13517073e+01
9.77251214e+00 7.52578432e+00 5.82444395e+00 4.13818654e+00
5.85382164e+00 7.91790197e+00 9.70636273e+00 1.11322553e+01
1.31101562e+01





"""