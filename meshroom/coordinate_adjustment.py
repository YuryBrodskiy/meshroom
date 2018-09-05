#!/usr/bin/env python
__version__ = "1.0"
import argparse
import os
import json
import numpy as np

def similarity_transform(from_points, to_points):
    assert len(from_points.shape) == 2, \
        "from_points must be a m x n array"
    assert from_points.shape == to_points.shape, \
        "from_points and to_points must have the same shape"

    N, m = from_points.shape
    mean_from = from_points.mean(axis = 0)
    mean_to = to_points.mean(axis = 0)
    delta_from = from_points - mean_from # N x m
    delta_to = to_points - mean_to       # N x m
    print(mean_from)
    print(mean_to)
    sigma_from = (delta_from * delta_from).sum(axis = 1).mean()
    sigma_to = (delta_to * delta_to).sum(axis = 1).mean()
    print(sigma_from)
    print(sigma_to)
    cov_matrix = delta_to.T.dot(delta_from) / N
    U, d, V_t = np.linalg.svd(cov_matrix, full_matrices = True)
    cov_rank = np.linalg.matrix_rank(cov_matrix)
    S = np.eye(m)
    if cov_rank >= m - 1 and np.linalg.det(cov_matrix) < 0:
        S[m-1, m-1] = -1
    elif cov_rank < m-1:
        raise ValueError("colinearility detected in covariance matrix:\n{}".format(cov_matrix))
    R = U.dot(S).dot(V_t)
    c = (d * S.diagonal()).sum() / sigma_from
    t = mean_to - c*R.dot(mean_from)
    print("scale: ", c)
    Mtmp = np.eye(4)

    Mtmp[:3,:3] = c*R

    Mtmp[:3,3] = t

    return Mtmp

def invH(H):
    R = H[:3,:3] 
    t = H[:3,3]
    Mtmp = np.eye(4)
    Mtmp[:3,:3] = R.T
    Mtmp[:3,3] = -np.matmul(R.T,t)
    return Mtmp

def get_PCA(data_fit):
    print(data_fit.shape)
    mean = data_fit.mean(axis=1)
    diff = mean.reshape(4,1) - data_fit
    cov_mat = np.matmul(diff,diff.transpose()) 
    eig_val, eig_vec=np.linalg.eig(cov_mat[:3,:3])  
    ind = np.argsort(eig_val)
    eig_val = eig_val[ind]
    R = eig_vec[:,ind]
    check = np.cross(R[:,0],R[:,1]).dot(R[:,2])
    Mtmp = np.eye(4)
    #Mtmp[:3,:3] = R
    Mtmp[:3,3] = mean[:3]  
    return Mtmp

def get_PCA_align(A,B):
    M_a = get_PCA(A)
    M_b = get_PCA(B)
    return np.matmul(M_b,invH(M_a))

def prepare_arrays(from_points, to_points):
    assert len(from_points.shape) == 2, \
        "from_points must be a m x n array"
    assert from_points.shape[0] == 4, \
        "only homo coordinates accceped"
    
        
    Mt = get_PCA_align(from_points, to_points)
    N = np.min([from_points.shape[0],to_points.shape[0]])
    F = from_points
    T = np.matmul(Mt,to_points)
    F2T = []
    for i in range(F.shape[0]):
        diff = T - F[:,i].reshape(4,1)
        dist2 = (diff*diff).sum(axis=0)
        F2T.append(dist2)
    F2T=np.array(F2T)
    F=[]
    Fs=np.copy(from_points)
    T=[]
    Ts=np.copy(to_points)
    print(F2T.shape)
    while F2T.shape[0]!=1:
        ind = np.unravel_index(np.argmin(F2T, axis=None), F2T.shape)
        F.append(Fs[:3, ind[0]])
        T.append(Ts[:3, ind[1]])
        Fs=np.delete(Fs,ind[0],1)
        Ts=np.delete(Ts,ind[1],1)
        F2T=np.delete(F2T,ind[0],axis=0)
        F2T=np.delete(F2T,ind[1],axis=1)
   
    return np.array(F).T,np.array(T).T



def icl(from_points, to_points):
    from_points_temp = from_points
    result = np.eye(4)
    for i in range(300):
        #print("Index",i)
        Mtmp = similarity_transform(from_points_temp, to_points)
        #print(Mtmp.shape, from_points_temp.shape)
        from_points_temp = np.matmul(Mtmp, from_points_temp)
        result = np.matmul(Mtmp, result)
    return result

def getName(id, views):
    for view in views:
        if view["poseId"] == id:
            return os.path.normpath(view["path"])
    raise IndexError()
    return None

def coordinate_adjustment(navi_model_file, av_file, pcl_tofix):
    navi_points = []
    navi_points_names = []
    to_meters = 1./100.0
    with open(navi_model_file) as f:
        data = json.load(f)
        for item in data:
            navi_points.append([item["x"]*to_meters,item["y"]*to_meters,item["z"]*to_meters, 1])
            navi_points_names.append(os.path.normpath(item["TexturePath"]))
    navi_points = np.array(navi_points).T
    assert navi_points.shape[0] == 4, "correct shape"
    av_points = []
    av_points_name = []
    with open(av_file) as f:
        data = json.load(f)
        poses = data["poses"]
        for item in poses:
            point=item["pose"]["transform"]["center"]
            id = item["poseId"]
            name = getName(id, data["views"])
            av_points.append([float(point[0]),float(point[1]),float(point[2]), 1])
            av_points_name.append(name)
    av_points = np.array(av_points).T
    indexs = []
    for i, name in enumerate(navi_points_names):
        try:
           indexs.append([i, av_points_name.index(name)])
        except:
            pass
    indexs = np.array(indexs)
#   print(indexs)
#   print(len(navi_points_names))
#   print(len(av_points_name))
#
#   #print([navi_points_names[i] for i in indexs[:,0] ])
#   #print([av_points_name[i] for i in indexs[:,1] ])
#
#   print([i for i in indexs[:,0] ])
#   print([i for i in indexs[:,1] ])
#   
    assert av_points.shape[0] == 4, "correct shape"
    print(av_points.shape, navi_points.shape)
    av_points_temp = np.array([av_points[:,i] for i in indexs[:,1] ]).T
    navi_points_temp = np.array([navi_points[:,i] for i in indexs[:,0] ]).T
    print(av_points_temp.shape, navi_points_temp.shape)
    ret_M =  similarity_transform(av_points_temp[:3,:].T, navi_points_temp[:3,:].T)
    
    points = []
    with open(pcl_tofix) as f:
        for line in f:
            point = np.array([float(x) for x in line.split()])
            point1 = np.matmul(ret_M, np.concatenate((point[:3],np.array([1]))))
            points.append(np.concatenate((point1[:3],point[3:])))
        for point in navi_points.T:
            points.append(np.concatenate((point[:3],np.array([255,0,0]))))
        for point in av_points.T:
            point1 = np.matmul(ret_M, point)
            points.append(np.concatenate((point1[:3],np.array([0,255,0]))))
    with open(pcl_tofix,'w') as f:
        for point in points:
            f.write('{:.10f} {:.10f} {:.10f} {} {} {} \n'.format(*point))
   
    
def main():
    parser = argparse.ArgumentParser(description='Correct point cloud using SfM and NaviTrack')
    parser.add_argument('--track', metavar='file', type=str, required=True,
                    default='',
                    help='Navi track to align, json .')
    parser.add_argument('--sfm', metavar='file', type=str,required=True,
                    help='sfm from alicevison json with camera poses.')
    parser.add_argument('--xyz', metavar='file', type=str, required=True,
                    help='XYZ point cloud file to move.')
    args = parser.parse_args()

    coordinate_adjustment(args.track, args.sfm, args.xyz)
main()