#!/usr/bin/env python
__version__ = "1.0"
import argparse
import os
import json
import numpy as np



def coordinate_adjustment(H_file, pcl_tofix):
    H = []
    with open(H_file) as f:
        for line in f:
            row = np.array([float(x) for x in line.split()])
            H.append(row)
    H=np.array(H)
    
    points = []
    with open(pcl_tofix) as f:
        for line in f:
            point = np.array([float(x) for x in line.split()])
            point1 = np.matmul(H, np.concatenate((point[:3],np.array([1]))))
            points.append(np.concatenate((point1[:3],point[3:])))
    with open(pcl_tofix,'w') as f:
        for point in points:
            f.write('{:.10f} {:.10f} {:.10f} {} {} {} \n'.format(*point))
   
    
def main():
    parser = argparse.ArgumentParser(description='Correct point cloud using SfM and NaviTrack')
    parser.add_argument('--tf', metavar='file', type=str, required=True,
                    default='',
                    help='Transform Raw.')
    parser.add_argument('--xyz', metavar='file', type=str, required=True,
                    help='XYZ point cloud file to move.')
    args = parser.parse_args()

    coordinate_adjustment(args.tf, args.xyz)
main()