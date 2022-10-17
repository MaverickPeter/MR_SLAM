import matplotlib.pyplot as plt
import numpy as np
import torch

from torch_radon import Radon
from torch_radon.solvers import Landweber
from utils import show_images

batch_size = 1
n_angles = 512
image_size = 512

img = np.load("phantom.npy")
device = torch.device('cuda')

# instantiate Radon transform
angles = np.linspace(0, np.pi, n_angles, endpoint=False)
radon = Radon(image_size, angles)

landweber = Landweber(radon)

# estimate step size
alpha = landweber.estimate_alpha(image_size, device)

with torch.no_grad():
    x = torch.FloatTensor(img).reshape(1, 1, image_size, image_size).to(device)
    sinogram = radon.forward(x)

    # use landweber iteration to reconstruct the image
    # values returned by 'callback' are stored inside 'progress'
    reconstruction, progress = landweber.run(torch.zeros(x.size(), device=device), sinogram, alpha, iterations=500,
                                             callback=lambda xx: torch.norm(xx - x).item())

plt.plot(progress)
plt.xlabel("Iteration")
plt.ylabel("Reconstruction error")
plt.title("Landweber Reconstruction Error")
print("Landweber Error", torch.norm(x - reconstruction).item() / torch.norm(x).item())

titles = ["Original Image", "Sinogram", "Reconstruction"]
show_images([x, sinogram, reconstruction], titles, keep_range=False)
plt.show()
