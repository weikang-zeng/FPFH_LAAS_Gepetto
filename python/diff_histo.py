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
import open3d as o3d

#for sd
import math



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
    folder = "/home/wzeng/multiscale-fpfh/output"
    object = "plateforms_wo_sensor_origin"
    #object = "plateforms_normals_reso3"
    scales = [1.34, 0.67, 0.335, 0.1675, 0.08375, 0.041875, 0.020938, 0.010469]
    

    #filename = "features_s1.340000_plateforms_r1.340000_px-0.832041_py1.057208_pz0.049622_ox0.050955_oy0.059661_oz-0.647446_ow0.758062"
    #filename = "features_s0.670000_plateforms_r1.340000_px-0.625000_py-0.250000_pz1.180000_ox-0.653281_oy0.270598_oz0.653282_ow0.270598"
    #filename = "features_s0.335000_plateforms_r1.340000_px-1.709083_py-0.250000_pz0.627632_ox0.000000_oy0.309017_oz0.000000_ow0.951057"
    #filename = "features_s1.340000_plateforms_r1.340000_px-0.832041_py1.057208_pz0.049622_ox0.050955_oy0.059661_oz-0.647446_ow0.758062"
    #filename2 = "features_s1.340000_plateforms_normals_reso3_r1.340000_px-1.965000_py-0.250000_pz-0.160000_ox0.000000_oy0.000000_oz0.000000_ow1.000000_k22"

    #fpfh = pcl_types.PointCloud
    #fpfh = import_pcl.read(f"{folder}/{object}/features/{filename}.{extension}")


    #fpfh2 = pcl_types.PointCloud
    #fpfh2 = import_pcl.read(f"{folder}/{object}/features_test_with_k/{filename2}.{extension}")

    #fpfh = pcl_types.PointCloud
    #fpfh = import_pcl.read(f"/home/wzeng/multiscale-fpfh/data/cloud/scene_side/pcl_features_for_plot.pcd")


    
    """
    num_size_fpfh=np.shape(fpfh.points)
    num_size_fpfh=num_size_fpfh[0]                    #get features number size

    km=sk_kmeans(fpfh.points,19)     # for sklearn kmeans
    a=km.fit_predict(fpfh.points)    #Return the index of the cluster each sample belongs to.

    #### result  
    #[13 13 13 13 13 13 17 12 12 12 12 12 12 12 13 13 13 16 13 13 13 17 12 12.....2 12 15 15  3 3  3  3 17  5  5 12 12 12 15 15  3  3  3  3 17 12 12 12 12 12 15 15 15 15 15 15 15 15 15 15 15]

    #print(km.cluster_centers_)
    #pourcentage ={}
    #for x in a:                              #save result in dict
        #pourcentage[x] = pourcentage.get(x,0)+1
   
    pourcentage = [] # lists are already indexed by integers from 0.
    list_a = list(a) #just to be sure but if a is a list, it's not needed
    for val in range(0, 19):
        pourcentage.append(list_a.count(val))
    #### result  
    #{12: 124, 17: 56, 18: 191, 10: 127, 0: 203, 16: 83, 9: 188, 4: 115, 1: 213, 13: 22, 6: 65, 5: 69, 3: 180, 2: 185, 15: 84, 14: 53, 7: 25, 8: 131, 11: 30}
    
    for i in range(0,19):                    #result/num_size = pourcentage of the cluster each sample
        pourcentage[i]=pourcentage[i]/num_size_fpfh
    #### result
    #{12: 0.05783582089552239, 17: 0.026119402985074626, 18: 0.08908582089552239, 10: 0.05923507462686567, 0: 0.09468283582089553, 16: 0.03871268656716418, 9: 0.08768656716417911, 4: 0.05363805970149254, 1: 0.09934701492537314, 13: 0.010261194029850746, 6: 0.03031716417910448, 5: 0.032182835820895525, 3: 0.08395522388059702, 2: 0.08628731343283583, 15: 0.03917910447761194, 14: 0.024720149253731342, 7: 0.01166044776119403, 8: 0.06110074626865672, 11: 0.013992537313432836}

    print(pourcentage)
    """
    """
    sum=0
    for i in pourcentage:   #just for check pourcentage total is 100
        sum=sum+pourcentage[i]
    print(pourcentage)
    print(sum)
    #### result
    #1.0
    """


    """
    filePath = "/home/wzeng/multiscale-fpfh/output/plateforms_normals_reso3/features_test"    #for resume all views
    fileList=os.listdir(filePath)
    n=0

    fpfh = pcl_types.PointCloud #declare the type
    for i in fileList:
        fpfh = import_pcl.read(f"{folder}/plateforms_normals_reso3/features_test/{i}")
        WCSS=[]
        for x in range(1,100):
            kmeans=sk_kmeans(fpfh.points,x)
            WCSS.append(kmeans.inertia_) 
        kneedle = KneeLocator(range(1,100),WCSS,S=1.0, curve="convex", direction="decreasing")
        oldname=filePath+os.sep+fileList[n]
        newname=oldname.replace(".pcd","_k"+str(kneedle.knee)+".pcd")
        print(newname)
        os.rename(oldname,newname)
        print(oldname,'=========>',newname)
        n+=1
    """













    
    #print(kmeans.cluster_centers_,end=' ')


    
    fpfh2 = pcl_types.PointCloud
    fpfh2 = import_pcl.read(f"/home/wzeng/7.19c_e/scene_front_far/features_clustered.pcd")
    fpfh = pcl_types.PointCloud
    fpfh = import_pcl.read(f"/home/wzeng/7.19c_e/scene_front_far/features_scene_voxel.pcd")
    #compare les resultat point features avec les resu clusting
    c1=list(range(0,66,2)) 
    c2=list(range(1,66,2)) 
    a=186
    b=217
    s=0
    s1=0

    plt.title('histrogramme de feature de point scene 62530 (red) et clusting resulatat 217(blue) ')
    plt.bar(c1,fpfh.points[a],color='r')
    e=fpfh.points[a]
    print(e)
    plt.bar(c2,fpfh2.points[b],color='b')
    f=fpfh2.points[b]
    print(f)
    for i in range(0,33,1):
        s=pow((e[i]-f[i]),2)
        s1=s1+s
    print(s1)
    plt.show()


    



