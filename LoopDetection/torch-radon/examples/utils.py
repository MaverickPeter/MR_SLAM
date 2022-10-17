import numpy as np
import matplotlib.pyplot as plt


def show_images(imgs, titles=None, keep_range=True, shape=None, figsize=(8, 8.5)):
    imgs = [x.cpu().numpy() for x in imgs]
    combined_data = np.array(imgs)

    if titles is None:
        titles = [str(i) for i in range(combined_data.shape[0])]

    # Get the min and max of all images
    if keep_range:
        _min, _max = np.amin(combined_data), np.amax(combined_data)
    else:
        _min, _max = None, None

    if shape is None:
        shape = (1, len(imgs))

    fig, axes = plt.subplots(*shape, figsize=figsize, sharex=True, sharey=True)
    ax = axes.ravel()
    for i, (img, title) in enumerate(zip(imgs, titles)):
        ax[i].imshow(img, cmap=plt.cm.Greys_r, vmin=_min, vmax=_max)
        ax[i].set_title(title)
