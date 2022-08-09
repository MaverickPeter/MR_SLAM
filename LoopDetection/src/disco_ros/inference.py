import argparse
import math
import numpy as np
import socket
import importlib
import os
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
import sys
import torch
import torch.nn as nn
from torch.autograd import Variable
from torch.backends import cudnn
import cv2
from sklearn.neighbors import NearestNeighbors
from sklearn.neighbors import KDTree
from torchvision import transforms, utils

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
from utils import *
from loading_pointclouds import *
import models.DiSCO as SC
from tensorboardX import SummaryWriter
import loss.loss_function
import gpuadder
import config as cfg
import scipy.io as scio

cudnn.enabled = True

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def infer(input_filename):
    if cfg.INPUT_TYPE == 'point':
        query = load_pc_file_infer(input_filename)
        query = np.array(query, dtype=np.float32)
    elif cfg.INPUT_TYPE == 'image':
        query = np.load(input_filename)
    
    model = SC.SCNet(global_feat=True, feature_transform=True, max_pool=False,
                                        output_dim=cfg.FEATURE_OUTPUT_DIM, num_points=cfg.NUM_POINTS)
    corr2soft = SC.Corr2Softmax(200., 0.)

    corr2soft = corr2soft.to(device)
    model = model.to(device)

    resume_filename = cfg.LOG_DIR + cfg.MODEL_FILENAME
    print("Resuming From ", resume_filename)
    checkpoint = torch.load(resume_filename)
    saved_state_dict = checkpoint['state_dict']
    saved_corr2soft_dict = checkpoint['corr2soft']

    model.load_state_dict(saved_state_dict)
    corr2soft.load_state_dict(saved_corr2soft_dict)
    model = nn.DataParallel(model)

    out, _, _, _ = infer_model(model, corr2soft, query)
    out_show = out.reshape(1, 32, 32)

    # imshow(out_show)
    # print("output descriptor: ",out)
    # np.save("./output_des.npy", out.cpu().numpy())

    return out

def infer_model(model, corr2soft, query):

    model.eval()
    corr2soft.eval()
    is_training = False
    
    with torch.no_grad():
        feed_tensor = torch.from_numpy(query).float()
        feed_tensor = feed_tensor.to(device)
        feed_tensor = feed_tensor.view((-1, cfg.num_height, cfg.num_ring, cfg.num_sector))
        out, outfft, fft_result, unet_out = model(feed_tensor)

    model.train()
    return out, outfft, fft_result, unet_out

def imshow(tensor, title=None):
    unloader = transforms.ToPILImage()
    image = tensor.cpu().clone()  # we clone the tensor to not do changes on it
    image = image.squeeze(0)  # remove the fake batch dimension
    image = unloader(image)
    plt.imshow(image, cmap='jet')
    plt.show()


def GT_sc_angle_convert(gt_yaw, size):
    gt_yaw = gt_yaw % 360
    if gt_yaw > 180:
        gt_yaw -= 360
    elif gt_yaw < -180:
        gt_yaw += 360
    
    gt_angle = gt_yaw
    # print("gt_",gt_angle)
    for batch_num in range(gt_angle.shape[0]):            
        if gt_angle[batch_num] <= -180.:
            gt_angle[batch_num] = gt_angle[batch_num] + 540.
        elif gt_angle[batch_num] >= 180.:
            gt_angle[batch_num] = gt_angle[batch_num] - 180.
        else:
            gt_angle[batch_num] = gt_angle[batch_num] + 180.
    gt_angle = np.ceil(gt_angle * float(cfg.num_sector) / 360.) - 1.
    return gt_angle

def fftshift2d(x):
    for dim in range(1, len(x.size())):
        n_shift = x.size(dim)//2
        if x.size(dim) % 2 != 0:
            n_shift = n_shift + 1  # for odd-sized images
        x = roll_n(x, axis=dim, n=n_shift)
    return x  # last dim=2 (real&imag)

def roll_n(X, axis, n):
    f_idx = tuple(slice(None, None, None) if i != axis else slice(0, n, None) for i in range(X.dim()))
    b_idx = tuple(slice(None, None, None) if i != axis else slice(n, None, None) for i in range(X.dim()))
    front = X[f_idx]
    back = X[b_idx]
    return torch.cat([back, front], axis)

def phase_corr(a, b, device, corr2soft):
    # a: template; b: source
    # imshow(a.squeeze(0).float())
    # [B, 1, cfg.num_ring, cfg.num_sector, 2]
    eps = 1e-15

    real_a = torch.from_numpy(a[...,0]).to(device)
    real_b = torch.from_numpy(b[...,0]).to(device)
    imag_a = torch.from_numpy(a[...,1]).to(device)
    imag_b = torch.from_numpy(b[...,1]).to(device)

    # compute a * b.conjugate; shape=[B,H,W,C]
    R = torch.FloatTensor(1, 1, cfg.num_ring, cfg.num_sector, 2).to(device)
    R[...,0] = real_a * real_b + imag_a * imag_b
    R[...,1] = real_a * imag_b - real_b * imag_a

    r0 = torch.sqrt(real_a ** 2 + imag_a ** 2 + eps) * torch.sqrt(real_b ** 2 + imag_b ** 2 + eps).to(device)
    R[...,0] = R[...,0].clone()/(r0 + eps).to(device)
    R[...,1] = R[...,1].clone()/(r0 + eps).to(device)

    corr = torch.ifft(R, 2)
    corr_real = corr[...,0]
    corr_imag = corr[...,1]
    corr = torch.sqrt(corr_real ** 2 + corr_imag ** 2 + eps)
    corr = fftshift2d(corr)

    corr = corr.squeeze(1)
    corr_wb = corr2soft(corr)
    corr_ang = torch.sum(corr_wb, 1, keepdim=False)

    angle = torch.argmax(corr)
    angle = angle % cfg.num_sector

    return angle, corr

def rotation_on_SCI(sc, rotation):
    # rotation to translation [-180:180] -> [-cfg.num_sector//2:cfg.num_sector//2]
    if rotation > 0:
        t = rotation / 180. * (cfg.num_sector // 2)
        t = np.floor(t).astype(int)
        patch = sc[:, (cfg.num_sector-t):cfg.num_sector]
        col, row = cfg.num_sector, cfg.num_ring
        center = (col // 2, row // 2)
        t_x, t_y = t, 0.

        M = cv2.getRotationMatrix2D(center, 0.0, 1.0)
        sc = cv2.warpAffine(sc, M, (col, row))

        N = np.float32([[1,0,t_x],[0,1,t_y]])
        sc = cv2.warpAffine(sc, N, (col, row))
        sc[:, 0:t] = patch
    else:
        t = -rotation / 180. * (cfg.num_sector // 2)
        t = np.floor(t).astype(int)
        patch = sc[:, 0:t]
        col, row = cfg.num_sector, cfg.num_ring
        center = (col // 2, row // 2)
        t_x, t_y = -t, 0.

        M = cv2.getRotationMatrix2D(center, 0.0, 1.0)
        sc = cv2.warpAffine(sc, M, (col, row))

        N = np.float32([[1,0,t_x],[0,1,t_y]])
        sc = cv2.warpAffine(sc, N, (col, row))
        sc[:, (cfg.num_sector-t):cfg.num_sector] = patch
        # plt.imshow(sc)
        # plt.show()
    return sc

if __name__ == "__main__":
    # params
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_filename', default='./test.bin',
                        help='input file name [default: ./test.bin]')
    parser.add_argument('--dimension', type=int, default=1024)
    parser.add_argument('--input_type', default='point',
                        help='Input of the network, can be [point] or scan [image], [default: point]')
    FLAGS = parser.parse_args()

    cfg.INPUT_FILENAME = FLAGS.input_filename
    cfg.FEATURE_OUTPUT_DIM = 1024
    cfg.num_ring = 40
    cfg.num_sector = 120
    cfg.num_height = 20
    cfg.max_length = 1

    cfg.LOG_DIR = 'log/polar_density_both_40_120_20/'
    cfg.MODEL_FILENAME = "model.ckpt"
    cfg.INPUT_TYPE = FLAGS.input_type

    disco = infer(cfg.INPUT_FILENAME)