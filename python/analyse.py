#!/usr/bin/env python3

# import open3d as o3d
from __future__ import print_function

import time
# import import_pcl
import mfpfh.pcl_interface.import_pcl as import_pcl
import mfpfh.pcl_interface.pcl_types as pcl_types

import os
import numpy as np
#np.set_printoptions(threshold=np.inf)
import matplotlib
import matplotlib.pyplot as plt
from scipy import stats
import pandas

#import numpy as np


from sklearn.mixture import GaussianMixture

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

if __name__ == "__main__":

    #start = time.time()
    """  my way to import data
    filePath = "/home/wzeng/multiscale-fpfh/output/plateforms_wo_sensor_origin/features"
    for filename in os.walk(filePath):
        result=filename[2]
    """
    """
    for file in os.listdir(f"{folder}/{object}/features"):
        filename = file[0:file.rfind('.')]
        fileext = file[file.rfind('.')+1:]
        if fileext != "pcd":
            continue
        keywords = filename.split('_')
        if keywords[1] == 's1.340000': 
            fpfh = import_pcl.read(f"{folder}/{object}/features/{filename}.{extension}")
    """


    extension = "pcd"
    folder = "/home/wzeng/multiscale-fpfh/output"
    object = "plateforms_wo_sensor_origin"
    #object = "plateforms_normals_reso3"
    scales = [1.34, 0.67, 0.335, 0.1675, 0.08375, 0.041875, 0.020938, 0.010469]


#########   choose un scale, collect all fpfh desciptor with this scales, compare each other to see the correlation  #########
    """
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
    """     
###############    end this part #####################

#########   choose two files fpfh, compare each other to see the correlation  #########
    
    #filename = "features_s1.340000_plateforms_normals_reso3_r1.340000_px-1.965000_py-0.250000_pz-0.160000_ox0.000000_oy0.000000_oz0.000000_ow1.000000"    #for reso3 
    filename = "features_s0.167500_plateforms_r1.340000_px-0.625000_py-0.250000_pz1.180000_ox-0.653281_oy0.270598_oz0.653282_ow0.270598"
    #filename2 ="features_s0.167500_plateforms_r1.340000_px-1.709083_py-0.250000_pz0.627632_ox0.000000_oy0.309017_oz0.000000_ow0.951057"
    fpfh = pcl_types.PointCloud        #declare the type
    #fpfh2 = pcl_types.PointCloud       #for second input
    #result2=flipud(result)
    
    

    #for i in result:
        #for j in result:
            #fpfh = import_pcl.read(f"{folder}/{object}/features/{i}")     #input pcd one by one
            #fpfh2 = import_pcl.read(f"{folder}/{object}/features/{j}")     #input2 pcd one by one
    fpfh = import_pcl.read(f"{folder}/{object}/features/{filename}.{extension}")
    #fpfh2 = import_pcl.read(f"{folder}/{object}/features/{filename2}.{extension}")
            #glob_diff = np.zeros((fpfh.header.height*fpfh.header.width, fpfh.header.height*fpfh.header.width))    #return a array with size fpfh.header.height*fpfh.header.width
    #print(np.size(fpfh.points))
    #print(np.shape(fpfh.points))
    #print(fpfh.points)
    gm = GaussianMixture(n_components=7,random_state=0).fit(fpfh.points)
    o3.visualization.draw_geometries([fpfh.points])
    #print(gm.means_)
    #gm.predict()
    #correlation=np.corrcoef(fpfh.points)          #matrice auto correlation    
    #correlation=np.corrcoef(fpfh.points,fpfh2.points)      #matrice correlation
    '''
    if(valid_imshow_data(correlation)):
        plt.matshow(correlation, cmap='jet_r')
        plt.colorbar()
        plt.title(f"mac at {filename}")
        #plt.title(f"mac at {filename} \n and {filename2}")
        plt.show()
    '''
###############    end this part #####################



#########   choose one files fpfh, compare itself to see the autocorrelation  #########
    """
    ### my way to import data
    filePath = "/home/wzeng/multiscale-fpfh/output/plateforms_wo_sensor_origin/features"
    for filename in os.walk(filePath):
        result=filename[2]
    


    fpfh = pcl_types.PointCloud        #declare the type
    for i in result:
        fpfh = import_pcl.read(f"{folder}/{object}/features/{i}")     #input pcd one by one
        correlation=np.corrcoef(fpfh.points)          #matrice auto correlation
        if(valid_imshow_data(correlation)):
            plt.matshow(correlation, cmap='jet_r')
            plt.colorbar()
            plt.title(f"mac at {i}")
            plt.show()
    """ 
###############    end this part ##################### 
    
    
    
    
    
    #-----------------------c'est les merde  la bas -------------------------------    
    #print(np.size(glob_diff))
    #print(np.shape(glob_diff))
    #print(glob_diff)
    
    #j=np.arange(0, i)
    #print(np.size(fpfh.points))
    #print(np.shape(fpfh.points))
    #print(fpfh.points)
    #a=np.sum(fpfh.points,axis=0)
    #print(a)
    #a=np.diff(fpfh.points)
    #a=np.diff(fpfh.points,axis=0)
    #print(a)
    #print(np.shape(a))
    #a=np.square(a)
    #print(a)
    #a=np.sum(a,axis=1)
    #print(a)
    #print(np.shape(a))

    """
    a=a*a
    print(a.sum(axis=0))
    """


    #print((fpfh.points[0,0]-fpfh.points[1,0])*(fpfh.points[0,0]-fpfh.points[1,0]))
    


        
    #correlation2=np.corrcoef(fpfh.points2)

    #print(correlation[0:1])
    #print(np.shape(correlation))







#i=np.arange(0, fpfh.header.height*fpfh.header.width)

#j=np.flipud(i)

#print(fpfh.points[j])
"""
i=np.arange(0, fpfh.header.height*fpfh.header.width)
j=np.arange(0, fpfh.header.height*fpfh.header.width)
a=np.arange(0, 33)
for k in a:
    #squared_sum_diff = 0
    diff = np.diff(fpfh.points[i, k],fpfh.points[j, k])
    squared_sum_diff += (diff * diff)
glob_diff[i, j] = squared_sum_diff
"""



"""
for i in range(0, fpfh.header.height*fpfh.header.width):
        #print(i)
    for j in range(0, fpfh.header.height*fpfh.header.width):               #i,j same size
        squared_sum_diff = 0
        for k in range(0, 33):     # k 0~32
            #print(fpfh.points[i, k])
            diff = fpfh.points[i, k] - fpfh.points[j, k]
                #print(diff)
                #diff = fpfh.points[i, 0, k] - fpfh.points[j, 0, k]
            squared_sum_diff += (diff * diff)
                #print(squared_sum_diff)
                
        glob_diff[i, j] = squared_sum_diff
"""
    #i=np.arange(0, fpfh.header.height*fpfh.header.width)
    #print(i)
    #k=np.arange(0, 33)
    #print(i[1:])
    #for k in range(0, 33):
        #print(fpfh.points[i[1:], k])
    #print(np.size(glob_diff))
    #print(np.shape(glob_diff))           
    #print(glob_diff[0:1])
#print(glob_diff)
    #print(glob_diff, file=open("output.txt","a"))
    #pandas_data = pandas.DataFrame(glob_diff)
    #pandas_data.describe().to_csv(f"/home/wzeng/multiscale-fpfh/python/log/mac/test.csv")
"""
    for scale in scales[0:1]:
        print(f"Computing for scale {scale}")
        clouds = load_features_folder(folder=folder, object=object, scale2load=scale)
        
        print(f"Number of clouds: {len(clouds)}.")
        
        features = concatenate_clouds(clouds)
        features_s = np.squeeze(features)
        print(f"Lets go for {features_s.shape} features...")
        # print(stats.describe(features_s))
        #! following impossible...
        # glob_diff = np.zeros((features.shape[0], features.shape[0]))

        pandas_data = pandas.DataFrame(features_s)
        pandas_data.describe().to_csv(f"/home/wzeng/multiscale-fpfh/python/log/test2_scale_{scale}.csv")
        myFig = plt.figure()
        pandas_data.boxplot(showfliers=False)
        myFig.savefig(f"/home/wzeng/multiscale-fpfh/python/log/test2_scale_{scale}_boxplot.s{i}vg", format="svg")

        del(clouds)
        del(features)
        del(features_s)
        del(pandas_data)
        del(myFig)
"""

    # for i in range(0, features.shape[0]):
    #     for j in range(0, features.shape[0]):
    #         diff_vec = features[i, 0, :] - features[j, 0, :]
    #         glob_diff[i, j] = np.sum(diff_vec * diff_vec)
"""
if(valid_imshow_data(glob_diff)):
         plt.figure()
         plt.matshow(glob_diff, cmap='jet')
         plt.colorbar()
         plt.title(f"Global at old fonction")
"""      


    


#end = time.time()
#print(end -start)