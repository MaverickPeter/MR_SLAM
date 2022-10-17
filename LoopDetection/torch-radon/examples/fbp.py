import matplotlib.pyplot as plt
import numpy as np
import torch
from utils import show_images

from torch_radon import Radon

device = torch.device('cuda')

img = np.load("phantom.npy")
image_size = img.shape[0]
n_angles = image_size

# Instantiate Radon transform. clip_to_circle should be True when using filtered backprojection.
angles = np.linspace(0, np.pi, n_angles, endpoint=False)
radon = Radon(image_size, angles, clip_to_circle=True)

with torch.no_grad():
    x = torch.FloatTensor(img).to(device)

    sinogram = radon.forward(x)
    filtered_sinogram = radon.filter_sinogram(sinogram)
    fbp = radon.backprojection(filtered_sinogram)

print("FBP Error", torch.norm(x - fbp).item())


# Show results
titles = ["Original Image", "Sinogram", "Filtered Sinogram", "Filtered Backprojection"]
show_images([x, sinogram, filtered_sinogram, fbp], titles, keep_range=False)
plt.show()