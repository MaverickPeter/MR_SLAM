import os
import torch
import config as cfg
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree
import torchvision.transforms.functional as fn
from torch_radon import Radon, ParallelBeam, RadonFanbeam

# np.set_printoptions(precision=4)
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

def forward_fft(input):
    median_output = torch.fft.fft2(input, dim=(-2, -1),norm="ortho")
    median_output_r = median_output.real
    median_output_i = median_output.imag
    
    output = torch.sqrt(median_output_r ** 2 + median_output_i ** 2)
    output = fftshift2d(output)
    # imshow(output.squeeze()[0,...])
    return output, median_output

def forward_row_fft(input):
    median_output = torch.fft.fft2(input, dim=-1,norm="ortho")
    median_output_r = median_output.real
    median_output_i = median_output.imag
    output = torch.sqrt(median_output_r ** 2 + median_output_i ** 2)
    # output = fftshift2d(output)
    return output, median_output

def forward_column_fft(input):
    median_output = torch.fft.fft2(input, dim=-2,norm="ortho")
    median_output_r = median_output.real
    median_output_i = median_output.imag
    output = torch.sqrt(median_output_r ** 2 + median_output_i ** 2)
    # output = fftshift2d(output)
    return output, median_output

def fftshift1d(x):
    x_new = x.clone()
    for i in range(len(x)):
        x_new[i] = x[len(x)//2-i]

    return x_new

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

def fast_fft_corr(a, b):
    a = fn.normalize(a, mean=a.mean(), std=a.std())
    b = fn.normalize(b, mean=b.mean(), std=b.std())
    # imshow(a)
    # imshow(b)
    a_fft = torch.fft.fft2(a, dim=-2,norm="ortho")
    b_fft = torch.fft.fft2(b, dim=-2,norm="ortho")
    corr = torch.fft.ifft2(a_fft*b_fft.conj(), dim=-2, norm="ortho")
    corr = torch.sqrt(corr.real**2 + corr.imag**2)
    corr = torch.sum(corr,dim=-1).view(-1)
    # corr = fftshift2d(corr)
    corr = fftshift1d(corr)
    dist = 1 - torch.max(corr)/(0.15*cfg.num_ring*cfg.num_sector)
    # corr = corr[0:cfg.num_ring//2] # only calculate the first half of the corr (180 degree range)
    # angle = torch.argmax(corr)
    # angle = torch.argmax(corr) % cfg.num_sector - cfg.num_ring//2
    angle = cfg.num_ring//2 - torch.argmax(corr)
    dist = dist.cpu().numpy()
    angle = angle.cpu().numpy()
    return dist, angle

def GT_angle_convert(gt_yaw, size):
    gt_yaw = gt_yaw % 360
    if gt_yaw > 180:
        gt_yaw -= 360
    elif gt_yaw < -180:
        gt_yaw += 360
    
    gt_angle = gt_yaw

    # if gt_angle <= -180.:
    #     gt_angle = gt_angle + 540.
    # elif gt_angle >= 180.:
    #     gt_angle = gt_angle - 180.
    # else:
    #     gt_angle = gt_angle + 180.
    gt_angle = np.round(gt_angle * float(size) / 360.)
    return gt_angle

def solve_translation(query, positive, rot_angle, device):
    # query = torch.from_numpy(query).to(device)
    # positive = torch.from_numpy(positive).to(device)
    H, W = query.shape
    # caculate the translation of b relative to a
    angles = torch.FloatTensor(np.linspace(0, 2*np.pi, H).astype(np.float32)).to(device)
    # Compensate for the rotation of the query to calculate the translation
    angles = angles - rot_angle # in radians
    # # take one half of the spectrum for correlation
    # angles = angles[0:H//2]
    # query = query[0:H//2,:]
    # positive = positive[0:H//2,:]
    # matrices of the overdetermined linear system
    A = torch.stack([torch.cos(angles), torch.sin(angles)], dim=1)
    b = torch.FloatTensor(H).to(device)
    x = torch.FloatTensor(1).to(device)
    y = torch.FloatTensor(1).to(device)

    for i in range(H):
        query_fft = torch.fft.fft2(query[i,:], dim=-1, norm="ortho")
        positive_fft = torch.fft.fft2(positive[i,:], dim=-1, norm="ortho")
        corr = torch.fft.ifft2(query_fft*positive_fft.conj(), dim=-1, norm="ortho")
        corr = torch.sqrt(corr.imag**2 + corr.real**2)
        corr = fftshift1d(corr)
        shift = torch.argmax(corr) - W//2
        # print('shift: ', shift)
        b[i] = shift
    x, y = solve_overdetermined_linear_system(A, b, method='svd')
    # calculate the error of overdetermined linear system
    # error = torch.norm(torch.matmul(A, x) - b)
    # print('error: ', error)
    err = A @ torch.cat([x, y]) - b

    x = x.cpu().numpy()
    y = y.cpu().numpy()
    # print('predicted x, y: ', x, y)

    return x, y, err

# solve the overdetermined linear system using SVD by torch
def solve_overdetermined_linear_system(A, b, method='pinv'):
    # import pdb; pdb.set_trace()
    # A: [B, H, W, C]
    # b: [B, H, W]
    # method: 'pinv' or 'svd'
    assert method in ['pinv', 'svd']
    B = A.size(0)
    A = A.view(B, -1)
    b = b.view(B, -1)
    if method == 'pinv':
        return torch.pinverse(A) @ b
    else:
        u, s, v = torch.svd(A, some=False)
        s_new = torch.zeros(A.shape).to(device)
        for i in range(len(s)):
            s_new[i, i] = 1/s[i]
        s_inv = s_new.t()

        return v.t() @ s_inv @ u.t() @ b


# solve the overdetermined linear system using SVD by numpy
def solve_overdetermined_linear_system_numpy(A, b, method='pinv'):
    # A: [B, H, W, C]
    # b: [B, H, W]
    # method: 'pinv' or 'svd'
    assert method in ['pinv', 'svd']
    B = A.shape[0]
    A = A.reshape(B, -1)
    b = b.reshape(B, -1)
    if method == 'pinv':
        return np.linalg.pinv(A) @ b
    else:
        u, s, v = np.linalg.svd(A, full_matrices=False)
        s_new = np.zeros(A.shape)
        for i in range(len(s)):
            s_new[i, i] = 1/s[i]
        s_inv = s_new.T

        return v.T @ s_inv @ u.T @ b


def imshow(tensor, title=None):
    image = tensor.cpu().clone()  # we clone the tensor to not do changes on it
    image = image.squeeze(0)  # remove the fake batch dimension
    plt.imshow(image, cmap='jet')
    # plt.colorbar()
    plt.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    plt.savefig(title, format='png', dpi=1000, bbox_inches='tight', pad_inches=0)
    plt.show()