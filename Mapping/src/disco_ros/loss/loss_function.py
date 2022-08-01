import numpy as np
import math
import torch
import config as cfg
import torch.nn as nn

from torchvision import transforms, utils
import matplotlib.pyplot as plt

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def phase_corr(a, b, device):
    # a: template; b: source
    # imshow(a.squeeze(0).float())
    # [B, 1, cfg.num_ring, cfg.num_sector, 2]
    eps = 1e-15

    real_a = a[...,0]
    real_b = b[...,0]
    imag_a = a[...,1]
    imag_b = b[...,1]

    # compute a * b.conjugate; shape=[B,H,W,C]
    R = torch.FloatTensor(a.shape[0], 1, cfg.num_ring, cfg.num_sector, 2).to(device)
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

    corr_marginize = torch.sum(corr, 3, keepdim=False)
    angle = torch.max(corr_marginize)
    corr_softmax = nn.functional.softmax(corr_marginize.clone(), dim=-1)

    # indice = np.linspace(0, 1, cfg.num_sector)
    # indice = torch.tensor(np.reshape(indice, (-1, cfg.num_sector))).to(device)
    # angle = torch.sum((cfg.num_sector - 1) * corr_softmax * indice, dim=-1)     
    return angle, corr

def one_dim_phase_corr(a, b, device):
    # a: template; b: source
    # imshow(a.squeeze(0).float())
    # [B, 1, cfg.num_ring, cfg.num_sector, 2]
    eps = 1e-15

    real_a = a[...,0]
    real_b = b[...,0]
    imag_a = a[...,1]
    imag_b = b[...,1]

    # compute a * b.conjugate; shape=[B,H,W,C]
    R = torch.FloatTensor(a.shape[0], 1, cfg.num_ring, cfg.num_sector, 2).to(device)
    R[...,0] = real_a * real_b + imag_a * imag_b
    R[...,1] = real_a * imag_b - real_b * imag_a

    r0 = torch.sqrt(real_a ** 2 + imag_a ** 2 + eps) * torch.sqrt(real_b ** 2 + imag_b ** 2 + eps).to(device)
    R[...,0] = R[...,0].clone()/(r0 + eps).to(device)
    R[...,1] = R[...,1].clone()/(r0 + eps).to(device)

    corr = torch.ifft(R, 1)
    corr_real = corr[...,0]
    corr_imag = corr[...,1]
    corr = torch.sqrt(corr_real ** 2 + corr_imag ** 2 + eps)
    corr = fftshift2d(corr)
    corr_marginize = torch.sum(corr, 2, keepdim=False)

    #angle = torch.max(corr_marginize)
    corr_softmax = torch.softmax(corr_marginize, 2)

    indice = np.linspace(0, 1, cfg.num_sector)
    indice = torch.tensor(np.reshape(indice, (-1, cfg.num_sector))).to(device)
    angle = torch.sum((cfg.num_sector - 1) * corr_softmax * indice, dim=-1)     
    return angle, corr, corr_marginize

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

def best_pos_distance(query, pos_vecs):
    num_pos = pos_vecs.shape[1]
    query_copies = query.repeat(1, int(num_pos), 1)
    diff = ((pos_vecs - query_copies)**2).sum(2)
    min_pos, _ = diff.min(1)
    max_pos, _ = diff.max(1)
    return min_pos, max_pos


def triplet_loss(q_vec, pos_vecs, neg_vecs, margin, use_min=False, lazy=False, ignore_zero_loss=False):
    min_pos, max_pos = best_pos_distance(q_vec, pos_vecs)

    # PointNetVLAD official code use min_pos, but i think max_pos should be used
    if use_min:
        positive = min_pos
    else:
        positive = max_pos

    num_neg = neg_vecs.shape[1]
    batch = q_vec.shape[0]
    query_copies = q_vec.repeat(1, int(num_neg), 1)
    positive = positive.view(-1, 1)
    positive = positive.repeat(1, int(num_neg))

    loss = margin + positive - ((neg_vecs - query_copies)**2).sum(2)
    loss = loss.clamp(min=0.0)
    if lazy:
        triplet_loss = loss.max(1)[0]
    else:
        triplet_loss = loss.sum(1)
    if ignore_zero_loss:
        hard_triplets = torch.gt(triplet_loss, 1e-16).float()
        num_hard_triplets = torch.sum(hard_triplets)
        triplet_loss = triplet_loss.sum() / (num_hard_triplets + 1e-16)
    else:
        triplet_loss = triplet_loss.mean()
    return triplet_loss


def triplet_loss_wrapper(q_vec, pos_vecs, neg_vecs, other_neg, corr2soft, fft_result, heading, randomYaw, m1, m2, use_min=False, lazy=False, ignore_zero_loss=False):
    return triplet_loss(q_vec, pos_vecs, neg_vecs, m1, use_min, lazy, ignore_zero_loss)


def quadruplet_loss(q_vec, pos_vecs, neg_vecs, other_neg, corr2soft, fft_result, heading, randomYaw, m1, m2, use_min=False, lazy=False, ignore_zero_loss=False):
    min_pos, max_pos = best_pos_distance(q_vec, pos_vecs)

    # PointNetVLAD official code use min_pos, but i think max_pos should be used
    if use_min:
        positive = min_pos
    else:
        positive = max_pos

    num_neg = neg_vecs.shape[1]
    batch = q_vec.shape[0]
    query_copies = q_vec.repeat(1, int(num_neg), 1)
    positive = positive.view(-1, 1)
    positive = positive.repeat(1, int(num_neg))

    loss = m1 + positive - ((neg_vecs - query_copies)**2).sum(2)
    loss = loss.clamp(min=0.0)
    if lazy:
        triplet_loss = loss.max(1)[0]
    else:
        triplet_loss = loss.sum(1)
    if ignore_zero_loss:
        hard_triplets = torch.gt(triplet_loss, 1e-16).float()
        num_hard_triplets = torch.sum(hard_triplets)
        triplet_loss = triplet_loss.sum() / (num_hard_triplets + 1e-16)
    else:
        triplet_loss = triplet_loss.mean()

    other_neg_copies = other_neg.repeat(1, int(num_neg), 1)
    second_loss = m2 + positive - ((neg_vecs - other_neg_copies) ** 2).sum(2)
    second_loss = second_loss.clamp(min=0.0)
    if lazy:
        second_loss = second_loss.max(1)[0]
    else:
        second_loss = second_loss.sum(1)

    if ignore_zero_loss:
        hard_second = torch.gt(second_loss, 1e-16).float()
        num_hard_second = torch.sum(hard_second)
        second_loss = second_loss.sum() / (num_hard_second + 1e-16)
    else:
        second_loss = second_loss.mean()

    query_rot, pos_rot, _, _ = torch.split(fft_result, [1, cfg.TRAIN_POSITIVES_PER_QUERY, cfg.TRAIN_NEGATIVES_PER_QUERY, 1], dim=0)
    query_rot = query_rot.expand_as(pos_rot)
    
    heading = torch.from_numpy(heading)
    heading = heading.squeeze()

    query_yaw, pos_yaw, _, _ = torch.split(heading, [1, cfg.TRAIN_POSITIVES_PER_QUERY, cfg.TRAIN_NEGATIVES_PER_QUERY, 1], dim=0)
    gt_yaw = torch.ones(pos_yaw.shape).to(device)
    gt_yaw = gt_yaw * randomYaw
    gt_scale = torch.ones(pos_yaw.shape).to(device)
    gt_scale = gt_scale * 64

    print('gt_yaw',gt_yaw)
    
    gt_angle = -gt_yaw
    for batch_num in range(gt_angle.shape[0]):            
        if gt_angle[batch_num] <= -180.:
            gt_angle[batch_num] = gt_angle[batch_num] + 540.
        elif gt_angle[batch_num] >= 180.:
            gt_angle[batch_num] = gt_angle[batch_num] - 180.
        else:
            gt_angle[batch_num] = gt_angle[batch_num] + 180.

    gt_angle = torch.ceil(gt_angle * float(cfg.num_sector) / 360.) - 1.

    angle, corr = phase_corr(query_rot, pos_rot, device)

    corr = corr.squeeze(1)
    corr_wb = corr2soft(corr)
    corr_arg = torch.sum(corr_wb, 1, keepdim=False)
    pred = torch.argmax(corr_arg.detach(), dim=-1)

    compute_celoss = torch.nn.CrossEntropyLoss(reduction="sum").to(device)
    compute_l1 = torch.nn.L1Loss().to(device)
    yaw_loss_l1 = compute_l1(pred, gt_angle.float())
    yaw_ce_loss = compute_celoss(corr_arg, gt_angle.long())

    total_loss = (triplet_loss + second_loss) + yaw_ce_loss

    return total_loss, yaw_ce_loss, yaw_loss_l1, corr

def GT_angle_convert(this_gt, size):
    for batch_num in range(this_gt.shape[0]):
        if this_gt[batch_num] >= 90:
            this_gt[batch_num] = cfg.num_ring - (this_gt[batch_num] - 90.) / 180 * size
        else:
            this_gt[batch_num] = (90 - this_gt[batch_num]) * size / 180
        this_gt[batch_num] = this_gt[batch_num] // 1 + (this_gt[batch_num] % 1 + 0.5)//1
        if this_gt[batch_num].long() == size:
            this_gt[batch_num] = this_gt[batch_num] - 1
    return this_gt.long()

def GT_angle_sc_convert(this_gt, size):
    for batch_num in range(this_gt.shape[0]):
        if this_gt[batch_num] >= 90:
            this_gt[batch_num] = cfg.num_ring - (this_gt[batch_num] - 90.) / 180 * size
        else:
            this_gt[batch_num] = (90 - this_gt[batch_num]) * size / 180
        this_gt[batch_num] = this_gt[batch_num] // 1 + (this_gt[batch_num] % 1 + 0.5)//1
        if this_gt[batch_num].long() == size:
            this_gt[batch_num] = this_gt[batch_num] - 1
    return this_gt.long()

def imshow(tensor, title=None):
    unloader = transforms.ToPILImage()
    image = tensor.cpu().clone()  # we clone the tensor to not do changes on it
    image = image.squeeze(0)  # remove the fake batch dimension
    image = unloader(image)
    plt.imshow(image, cmap='jet')
    plt.show()