#! /usr/bin/env python3
# T. Lasguignes 25-08-2021

##** Global includes
import argparse
import open3d as o3d
import os
import matplotlib.pyplot as plt
import numpy as np
from timeit import default_timer as timer
from datetime import datetime
import time

##** Projects includes
import mfpfh.types.model
import mfpfh.pcl_interface.pcl_types as mpcl
from mfpfh.pcl_interface import import_pcl
from mfpfh.segmentation.spherical import SphericalSegmentation
from mfpfh.vlad.encoder import Encoder
from termco.tc import fc, bc, ff, rf

from sklearn.cluster import MiniBatchKMeans
import pickle

def app():
    now = datetime.now()
    date_str = now.strftime("%m-%d-%Y")
    time_str = now.strftime("%H-%M-%S")

    parser = argparse.ArgumentParser()
    parser.add_argument("--savepath", "-sp", help="save repository path",
                        default="/local/users/tlasguigne/work/dev/multiscale-fpfh/output/VLAD/", type=str)
    parser.add_argument("--vladpath", "-vp", help="VLAD repository path",
                        default="/local/users/tlasguigne/work/dev/multiscale-fpfh/output/VLAD_plateforms/", type=str)
    parser.add_argument("--objpath", "-op", help="path to the object if different from <'save'/'object'> (the complete path should be <'objpath'/'object'>",
                        default="", type=str)
    parser.add_argument("--object", "-o", help="name of the object to use as model",
                        default="plateforms")
    parser.add_argument("--scene", "-st", help="Path to the scene used as target.",
                        default="/local/users/tlasguigne/work/dev/multiscale-fpfh/data/cloud/scene_front/scene_front.pcd", type=str)
    parser.add_argument("--n_cluster", help="clusters for k-means algorithm",
                        default=10, type=int)
    parser.add_argument("--recompute", help="do not check if k-means results exists, simply recompute",
                        action='store_true')
    parser.add_argument("--saveresults", "-s", help="Save all the results (k-means if not provided, localisation results, ...)",
                        action='store_true')
    parser.add_argument("--scenefeatures", "-sf", help="Path of the features for the scene if provided.",
                        default="", type=str)
    parser.add_argument("--top", "-t", help="Number of top candidates to consider",
                        default=10, type=int)
    args = parser.parse_args()

    start = timer()
    if not args.objpath:
        folder = args.savepath
    else:
        folder = args.objpath
    object = args.object
    scales = [1.34, 0.67, 0.335, 0.1675, 0.08375, 0.041875]
    scene_path = args.scene
    if args.saveresults:
        savepath_complete = os.path.join(args.savepath, date_str, time_str)
        os.makedirs(savepath_complete)
    
    model = mfpfh.types.model.MFPFHModel()
    model.build_from_pcl_folder(folder, object)

    scene_features_from_file = args.scenefeatures is not ""

    n_cluster = args.n_cluster

    ##! Load scene
    pcd = o3d.io.read_point_cloud(scene_path)

    ##! segment scene
    segmenter = SphericalSegmentation(pcd)
    centers, neigh, pcds = segmenter.segment(0.67)

    ##! encode model
    for scale_to_use in scales:
        scale_to_use = model.get_closest_scale(scale_to_use)
        encoder = Encoder(scale_to_use, f"{object}_codebook_n{n_cluster}_s{scale_to_use}.pkl", args.vladpath,
                          n_cluster, model, False, args.saveresults)
        
        print(f"{ff.bold}Computing scale {scale_to_use}...{ff.reset}")
        if args.recompute:
            ##* get codebook
            encoder.compute_codebook()

            if args.saveresults:
                ##* save codebook
                codebook_filename = f"{object}_codebook_n{n_cluster}_s{scale_to_use}.pkl"
                codebook_path = os.path.join(args.savepath, date_str, time_str, codebook_filename)
                encoder.save_codebook(codebook_path)

            ##* encode views
            for view_number in range(len(model)):
                _ = encoder.get_encoded_view(view_number)

        #! load scale features of the scene
        if scene_features_from_file:
            filename = f"pcl_features_s{scale_to_use:.6f}_{os.path.basename(scene_path)}"
            filepath = os.path.join(args.scenefeatures, filename)
            print(f"Loading <{filepath}>")
            data_loaded = import_pcl.read(filepath)
            raw_fpfh = data_loaded.points
        else:
            working_pcd = pcd
            working_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamRadius(radius=(0.05)))
            raw_fpfh = o3d.pipelines.registration.compute_fpfh_feature(working_pcd, o3d.geometry.KDTreeSearchParamRadius(radius=scale_to_use))
            
        vlad_parts = {}
        candidates = {}
        candidates_list = []
        ##! encode scene
        for _, partID in enumerate(pcds):
            vlad_parts[partID] = encoder.vlad(raw_fpfh[neigh[partID],:])

            ##! find candidates
            candidates[partID] = encoder.match_in_dict(query=vlad_parts[partID])
            candidates_list.append((partID, candidates[partID][0, 0], candidates[partID][0, 1]))

        candidates_list.sort(key=lambda x:x[2])
        pcd.paint_uniform_color([0.2, 0.2, 0.2])
        valid_clouds = [pcd]
        for partID, viewID, value in candidates_list:
            if len(valid_clouds) < args.top+1:
                valid_clouds.append(pcds[partID])
            else:
                break
        if os.path.basename(os.path.normpath(args.scenefeatures)) == "bauzil_room":
            if os.path.basename(scene_path) == "front_no_noise_r1.340000_px0.045000_py-0.250000_pz1.000474_ox-0.500000_oy0.000000_oz0.866025_ow0.000000.pcd":
                o3d.visualization.draw_geometries(valid_clouds,
                                                window_name=f"Result at scale {scale_to_use} (top {args.top})",
                                                front=[-0.945, -0.054, 0.323],
                                                lookat=[1.188, 2.715, 2.294],
                                                up=[0.322, 0.020, 0.947],
                                                zoom=0.44)
            elif os.path.basename(scene_path) == "side_no_noise_r1.340000_px-0.625000_py0.420000_pz1.000474_ox0.353553_oy0.353553_oz-0.612373_ow0.612372.pcd":
                o3d.visualization.draw_geometries(valid_clouds,
                                                window_name=f"Result at scale {scale_to_use} (top {args.top})",
                                                front=[0.036, -0.891, 0.452],
                                                lookat=[0.411, 0.590, 2.108],
                                                up=[-0.004, 0.452, 0.892],
                                                zoom=0.32)
            elif os.path.basename(scene_path) == "top_no_noise_r1.340000_px-0.625000_py-0.920000_pz1.000474_ox-0.353553_oy0.353553_oz0.612372_ow0.612373.pcd":
                o3d.visualization.draw_geometries(valid_clouds,
                                                window_name=f"Result at scale {scale_to_use} (top {args.top})",
                                                front=[-0.125, 0.851, 0.51],
                                                lookat=[1.486, 0.087, 2.582],
                                                up=[0.054, -0.507, 0.86 ],
                                                zoom=0.34)
            else:
                o3d.visualization.draw_geometries(valid_clouds)
        else:
            o3d.visualization.draw_geometries(valid_clouds)


    ##! results

    end = timer()
    print(f"Elapsed time: {end - start}s")

if __name__ == "__main__":
    app()




