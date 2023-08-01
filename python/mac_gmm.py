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
    #object = "plateforms_wo_sensor_origin"
    object = "plateforms_normals_reso3"
    scales = [1.34, 0.67, 0.335, 0.1675, 0.08375, 0.041875, 0.020938, 0.010469]
    """
    filename = "features_s1.340000_plateforms_normals_reso3_r1.340000_px0.535474_py0.420000_pz-0.160000_ox0.000000_oy0.000000_oz0.965926_ow-0.258819_k22"
    #filename = "features_s0.670000_plateforms_r1.340000_px-0.625000_py-0.250000_pz1.180000_ox-0.653281_oy0.270598_oz0.653282_ow0.270598"
    #filename = "features_s0.335000_plateforms_r1.340000_px-1.709083_py-0.250000_pz0.627632_ox0.000000_oy0.309017_oz0.000000_ow0.951057"
    #filename = "features_s1.340000_plateforms_r1.340000_px-0.832041_py1.057208_pz0.049622_ox0.050955_oy0.059661_oz-0.647446_ow0.758062"
    filename2 = "features_s1.340000_plateforms_normals_reso3_r1.340000_px-1.965000_py-0.250000_pz-0.160000_ox0.000000_oy0.000000_oz0.000000_ow1.000000_k22"
    fpfh = pcl_types.PointCloud
    fpfh = import_pcl.read(f"{folder}/{object}/features_test_with_k/{filename}.{extension}")
    fpfh2 = pcl_types.PointCloud
    fpfh2 = import_pcl.read(f"{folder}/{object}/features_test_with_k/{filename2}.{extension}")
    #filename3 = "features_s1.340000_plateforms_normals_reso3_r1.340000_px-0.625000_py0.910474_pz0.510000_ox0.183013_oy0.183013_oz-0.683013_ow0.683013_k17"
    #fpfh3 = pcl_types.PointCloud
    #fpfh3 = import_pcl.read(f"{folder}/{object}/features_test_with_k/{filename3}.{extension}")
    #filename4 = "features_s1.340000_plateforms_normals_reso3_r1.340000_px-0.960000_py-0.830237_pz1.000474_ox-0.250000_oy0.433013_oz0.433013_ow0.750000_k13"
    #fpfh4 = pcl_types.PointCloud
    #fpfh4 = import_pcl.read(f"{folder}/{object}/features_test_with_k/{filename4}.{extension}")
    #filename5 = "features_s1.340000_plateforms_normals_reso3_r1.340000_px-1.785474_py-0.250000_pz0.510000_ox0.000000_oy0.258819_oz-0.000000_ow0.965926_k16"
    #fpfh5 = pcl_types.PointCloud
    #fpfh5 = import_pcl.read(f"{folder}/{object}/features_test_with_k/{filename5}.{extension}")

    #print(np.shape(fpfh.points))


    
    #gmm=gmm_weights_class(fpfh.points,16)        #input data set FPFH and number of class  output is the weights for every class
    #print(gmm.means_)
    #gmm2=gmm_weights_class(fpfh2.points,16)        #input data set FPFH and number of class  output is the weights for every class
    #print(gmm2.n_features_in_)
    #gmm3=gmm_weights_class(fpfh3.points,16)
    #print(gmm3.means_)
    #gmm4=gmm_weights_class(fpfh4.points,16)
    #print(gmm4.n_features_in_)
    #gmm5=gmm_weights_class(fpfh5.points,16)
    #print(gmm5.means_)
    
    km=sk_kmeans(fpfh.points,16)     # for sklearn kmeans
    print(km.inertia_)
    km2=sk_kmeans(fpfh2.points,16)     # for sklearn kmeans
    print(km2.inertia_)
    #km3=sk_kmeans(fpfh3.points,16)     # for sklearn kmeans
    #print(km3.inertia_)
    #km4=sk_kmeans(fpfh4.points,16)     # for sklearn kmeans
    #print(km4.inertia_)
    #km5=sk_kmeans(fpfh5.points,16)     # for sklearn kmeans
    #print(km5.inertia_)
    """




    """
    k1=0
    WCSS=[]        #within-cluster sum of squares
    for x in range(1,200):
        kmeans=sk_kmeans(fpfh.points,x)
        #k=kmeans.inertia_
        #k=k-k1
        #if k<-100000:
        #    print(len(WCSS))
        #print(k)
        #k1=kmeans.inertia_
        WCSS.append(kmeans.inertia_)     #Sum of squared distances of samples to their closest cluster center, weighted by the sample weights if provided. 
    #WCSS.pop(0)
    #plt.plot(range(1,50),WCSS)
    kneedle = KneeLocator(range(1,200),WCSS,S=1.0, curve="convex", direction="decreasing")
    print(kneedle.knee)
    kneedle.plot_knee()
    #plt.title(filename)
    plt.title(f"K par rapport a la somme des erreurs des carres avec difference vue")
    plt.xlabel('Number of clusters')
    plt.ylabel('WCSS change value')

    plt.show()
    """
    

    """
    SAVE=[]
    for x in range(2,100):
        
        km = KMeans(n_clusters=x,init = 'k-means++',random_state=0)
        X=fpfh.points
        y_km=km.fit_predict(X)

        cluster_labels=np.unique(y_km)
        n_clusters=cluster_labels.shape[0]
        silhouette_score_cluster=silhouette_score(X,km.labels_)
        SAVE.append(silhouette_score_cluster)
    print(SAVE)
    print(max(SAVE))
    print(SAVE.index(max(SAVE)))
    """





    """
    #filePath = "/home/wzeng/multiscale-fpfh/output/plateforms_wo_sensor_origin/features"
    #for filename in os.walk(filePath):
    #    result=filename[2]
    


    #fpfh = pcl_types.PointCloud        #declare the type
    #for i in result:
    #    fpfh = import_pcl.read(f"{folder}/{object}/features/{i}")     #input pcd one by one

    WCSS=[]        #within-cluster sum of squares
    for x in range(1,20):
        kmeans=sk_kmeans(fpfh.points,x)
        WCSS.append(kmeans.inertia_)     #Sum of squared distances of samples to their closest cluster center, weighted by the sample weights if provided. 
    plt.plot(range(1,20),WCSS,marker='o',label='f270598')
    plt.title(filename)
    #plt.title(f"K par rapport a la somme des erreurs des carres avec difference vue")
    plt.xlabel('Number of clusters')
    plt.ylabel('WCSS')

    plt.figure()
    filename = "features_s0.167500_plateforms_r1.340000_px-1.709083_py-0.250000_pz0.627632_ox0.000000_oy0.309017_oz0.000000_ow0.951057"
    fpfh = pcl_types.PointCloud
    fpfh = import_pcl.read(f"{folder}/{object}/features/{filename}.{extension}")

    WCSS=[]        #within-cluster sum of squares
    for x in range(1,20):
        kmeans=sk_kmeans(fpfh.points,x)
        WCSS.append(kmeans.inertia_)     #Sum of squared distances of samples to their closest cluster center, weighted by the sample weights if provided. 
    plt.plot(range(1,20),WCSS,marker='o',label='f951057')
    plt.title(filename)
    plt.xlabel('Number of clusters')
    plt.ylabel('WCSS')

    plt.figure()
    filename = "features_s0.167500_plateforms_r1.340000_px-0.832041_py1.057208_pz0.049622_ox0.050955_oy0.059661_oz-0.647446_ow0.758062"
    fpfh = pcl_types.PointCloud
    fpfh = import_pcl.read(f"{folder}/{object}/features/{filename}.{extension}")
    

    WCSS=[]        #within-cluster sum of squares
    for x in range(1,20):
        kmeans=sk_kmeans(fpfh.points,x)
        WCSS.append(kmeans.inertia_)     #Sum of squared distances of samples to their closest cluster center, weighted by the sample weights if provided. 
    plt.plot(range(1,20),WCSS,marker='o',label='f758062')
    plt.title(filename)
    plt.xlabel('Number of clusters')
    plt.ylabel('WCSS')

    plt.figure()
    filename = "features_s0.167500_plateforms_r1.340000_px0.406024_py0.499083_pz0.254083_ox-0.148778_oy-0.048341_oz0.939347_ow-0.305212"
    fpfh = pcl_types.PointCloud
    fpfh = import_pcl.read(f"{folder}/{object}/features/{filename}.{extension}")

    WCSS=[]        #within-cluster sum of squares
    for x in range(1,20):
        kmeans=sk_kmeans(fpfh.points,x)
        WCSS.append(kmeans.inertia_)     #Sum of squared distances of samples to their closest cluster center, weighted by the sample weights if provided. 
    plt.plot(range(1,20),WCSS,marker='o',label='f305212')
    plt.title(filename)
    plt.xlabel('Number of clusters')
    plt.ylabel('WCSS')



    
    plt.legend()
    
    plt.show()
    """
    
    """
    kmeans = KMeans(n_clusters=4,init = 'k-means++',random_state=0)
    X=fpfh.points
    y_kmeans = kmeans.fit_predict(X)
    plt.scatter(X[y_kmeans == 0, 0], X[y_kmeans == 0, 1], s = 60, c = 'red', label = 'Cluster1')
    plt.scatter(X[y_kmeans == 1, 0], X[y_kmeans == 1, 1], s = 60, c = 'blue', label = 'Cluster2')
    plt.scatter(X[y_kmeans == 2, 0], X[y_kmeans == 2, 1], s = 60, c = 'green', label = 'Cluster3')
    plt.scatter(X[y_kmeans == 3, 0], X[y_kmeans == 3, 1], s = 60, c = 'violet', label = 'Cluster4')
    plt.scatter(X[y_kmeans == 4, 0], X[y_kmeans == 4, 1], s = 60, c = 'yellow', label = 'Cluster5') 
    plt.scatter(kmeans.cluster_centers_[:, 0], kmeans.cluster_centers_[:, 1], s = 100, c = 'black', label = 'Centroids')
    plt.xlabel('Annual Income (k$)') 
    plt.ylabel('Spending Score (1-100)') 
    plt.legend() 

    plt.show()

    #corr.see_autocorrelation()
    """

    """
    #km=KMeans(n_clusters=3,init='k-means++',n_init=10,max_iter=300,tol=1e-4,random_state=0)
    km = KMeans(n_clusters=6,init = 'k-means++',random_state=0)
    X=fpfh.points
    y_km=km.fit_predict(X)

    cluster_labels=np.unique(y_km)
    n_clusters=cluster_labels.shape[0]
    silhouette_score_cluster=silhouette_score(X,km.labels_)
    print("Silhouette Score When Cluster Number Set : %.3f" % silhouette_score_cluster)
    silhouette_vals=silhouette_samples(X,y_km,metric='euclidean')
    y_ax_lower,y_ax_upper=0,0
    yticks=[]
    for i,c in enumerate(cluster_labels):
        c_silhouette_vals=silhouette_vals[y_km==c]
        c_silhouette_vals.sort()
        y_ax_upper+=len(c_silhouette_vals)
        color=cm.jet(float(i)/n_clusters)
        plt.barh(range(y_ax_lower,y_ax_upper),
                c_silhouette_vals,
                height=1.0,
                edgecolor='none',
                color=color)
        yticks.append((y_ax_lower+y_ax_upper)/2.0)
        y_ax_lower+=len(c_silhouette_vals)
    silhouette_avg=np.mean(silhouette_vals)
    plt.axvline(silhouette_avg,
                color='red',
                linestyle='--')
    plt.yticks(yticks,cluster_labels+1)
    plt.ylabel("Cluster")
    plt.xlabel("Silhouette Coefficients")
    #plt.savefig('./fig4.png')
    plt.show()
    """



### add k  value for every feature
filePath = "/home/wzeng/multiscale-fpfh/output/plateforms_normals_reso3/features_test_0.16"    #for k to filename
#filePath = "/home/wzeng/multiscale-fpfh/output/plateforms_normals_reso3/features_test"    #for resume all views
#for filename in os.walk(filePath):
#    result=filename[2]

fileList=os.listdir(filePath)
n=0

#print(fileList)
"""
for i in fileList:
    oldname=filePath+os.sep+fileList[n]
    a=newname.replace(".pcd","_k"+str(kneedle.knee)+".pcd")
    print(a)
    n=n+1
"""



######   this part for resume all views and find k value for all (every 33 dans un set)

"""
fpfh = pcl_types.PointCloud        #declare the type

c = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]]
d = [[]]
k= [[]]

                

for i in fileList:
    fpfh = import_pcl.read(f"{folder}/plateforms_wo_sensor_origin/features_test_1.34/{i}")
    #fpfh = import_pcl.read(f"{folder}/plateforms_normals_reso3/features_test/{i}")     #input pcd one by one
    a=np.shape(fpfh.points)
    a=[0]



    c=np.append(c,fpfh.points,axis=0)
print(c[0])
c=np.delete(c,0,axis=0)
print(np.shape(c))
"""
    #d=c.ravel()
    #d=d.resize((1,a))
    #d=np.append(d,d,axis=0)
#print(d)
#print(np.shape(d))




#c=np.delete(c,0,axis=0)


"""
WCSS=[]
for x in range(1,200):
    kmeans=sk_kmeans(c,x)
    WCSS.append(kmeans.inertia_)

kneedle = KneeLocator(range(1,200),WCSS,S=1.0, curve="convex", direction="decreasing")
print(kneedle.knee)
kneedle.plot_knee()
 
plt.show()
"""

"""
fw =open("test.txt",'w')

#kmeans=sk_kmeans(c,b)
kmeans=sk_kmeans(c,18)
#print(kmeans.cluster_centers_,end=' ')
print(kmeans.cluster_centers_,end="",file = fw)
fw.close()
"""

"""
##############   Segmentation des clusters

fpfh = pcl_types.PointCloud        #declare the type

c=[[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]
b=0
d=0

for i in fileList:
    fpfh = import_pcl.read(f"{folder}/plateforms_normals_reso3/feature_test/{i}")     #input pcd one by one
    a=np.shape(fpfh.points)
    a=a[0]
    b=b+a
    d=d+1

    c=np.append(c,fpfh.points,axis=0)
print(b)
print(d)

c=np.delete(c,0,axis=0)
b=round(b/d)


print(b)


fw =open("test.txt",'w')

kmeans=sk_kmeans(c,b)
#kmeans=sk_kmeans(c,13)
#print(kmeans.cluster_centers_,end=' ')
print(kmeans.cluster_centers_,end="",file = fw)
fw.close()




 
#plt.show()
"""



####### this part for k value in the filename

fpfh = pcl_types.PointCloud        #declare the type
for i in fileList:
    fpfh = import_pcl.read(f"{folder}/plateforms_normals_reso3/features_test_0.16/{i}")  
    print(i)
    #fpfh = import_pcl.read(f"{folder}/{object}/features_test_0.16/{i}")     #input pcd one by one

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
####### changer k value for test

for n in range(1,439):

    oldname=filePath+os.sep+fileList[n]
    oldname3=filePath+os.sep+fileList[n]



    oldname2=re.findall(r"_k(.+?).pcd",fileList[n])
    kk=list(map(int,oldname2))
    for i in kk:
    #newname=oldname.replace("_","_k"+str(kneedle.knee)+".pcd")
        i=i+10
        #print(i)
        oldname=oldname[:-6]
        newname=oldname.replace("_k","_k"+str(i)+".pcd")
        os.rename(oldname3,newname)
        #print(oldname,'=========>',newname)
"""














