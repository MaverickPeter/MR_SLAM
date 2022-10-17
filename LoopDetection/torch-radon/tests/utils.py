import numpy as np
import random
import os
import shutil
from nose.tools import assert_less, assert_equal
import matplotlib.pyplot as plt
import random
import torch

from torch_radon_cuda import SymbolicFunction, symbolic_discretize, symbolic_forward


def relative_error(ref, x):
    return np.linalg.norm(ref - x) / (np.linalg.norm(ref) + 1e-6)


def random_symbolic_function(height, width):
    f = SymbolicFunction(height, width)

    for i in range(30):
        w = random.uniform(0, 1)
        cx = random.uniform(-width/2, width/2)
        cy = random.uniform(-height/2, height/2)
        rx = random.uniform(width/32, width/4)
        ry = random.uniform(height/32, height/4)

        if random.randint(0, 1) < 0.25:
            f.add_ellipse(w, cx, cy, rx, ry)
        else:
            f.add_gaussian(5*w, cx, cy, 0.5/rx**2, 0.5/ry**2)

    return f


class TestHelper:
    def __init__(self, name):
        self.out_folder = os.path.join(os.path.dirname(os.path.realpath(__file__)), "output")
        self.name = name
        self.count = 0

        if os.path.exists(self.out_folder):
            shutil.rmtree(self.out_folder)
        os.makedirs(self.out_folder)
        print(self.out_folder, self.name)

    def save_output_image(self, task):
        path = os.path.join(self.out_folder, f"{self.name}_{task}_{self.count}.png")
        self.count += 1
        plt.savefig(path, dpi=100, bbox_inches='tight')
        plt.close()

    def compare_images(self, target, res, max_error, description):
        is_half = res.dtype == torch.float16
        res = res.float().cpu().numpy()
        batch_size = res.shape[0]

        for i in range(batch_size):
            forward_error = relative_error(target, res[i])

            desc = f"Batch: {i}/{batch_size}, {description}\nError: {forward_error:.2e}, Max error: {max_error:.2e}"

            if forward_error > max_error:
                fig, ax = plt.subplots(1, 3, figsize=(10, 7))
                ax = ax.ravel()
                ax[0].imshow(target)
                ax[0].axis("off")
                ax[0].set_title("Target")
                ax[1].imshow(res[i])
                ax[1].axis("off")
                ax[1].set_title("Result")
                ax[2].imshow(np.abs(target - res[i]))
                ax[2].axis("off")
                ax[2].set_title("Difference")
                fig.suptitle(desc)
                fig.tight_layout()
                if is_half:
                    self.save_output_image("forward_half")
                else:
                    self.save_output_image("forward")
                break

            print(desc)
            assert_less(forward_error, max_error)

    # makes sure that <y, Ax> = < A^T y, x>
    def backward_check(self, x, y, radon, description, back_max_error):
        batch_size = x.shape[0]

        test_sino = y / torch.max(y)
        target = torch.sum(test_sino * y, dim=(1, 2)) / (radon.volume.voxel_size[0] * radon.volume.voxel_size[1])
        bp = radon.backward(test_sino)
        value = torch.sum(bp * x, dim=(1, 2))

        for i in range(batch_size):
            backward_error = abs(target[i] - value[i]) / abs(target[i])
            desc = f"Batch: {i}/{batch_size}, {description}\nError: {backward_error:.2e}, Max error: {back_max_error:.2e}"

            if backward_error > back_max_error:
                fig, ax = plt.subplots(1, 2)
                ax = ax.ravel()
                ax[0].imshow(x[i].cpu().numpy())
                ax[0].axis("off")
                ax[1].imshow(bp[i].cpu().numpy())
                ax[1].axis("off")
                fig.suptitle(desc)
                fig.tight_layout()
                self.save_output_image("backward")
                break

            assert_less(backward_error, back_max_error)
