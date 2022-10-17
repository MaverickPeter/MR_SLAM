from alpha_transform import AlphaShearletTransform
from alpha_transform.fourier_util import my_ifft_shift

from .utils import normalize_shape
import numpy as np
import torch
import os


class ShearletTransform:
    """
    Implementation of Alpha-Shearlet transform based on https://github.com/dedale-fet/alpha-transform/tree/master/alpha_transform.

    Once the shearlet spectrograms are computed all the computations are done on the GPU.

    :param width: Width of the images
    :param height: Height of the images
    :param alphas: List of alpha coefficients that will be used to generate shearlets
    :param cache: If specified it should be a path to a directory that will be used to cache shearlet coefficients in
        order to avoid recomputing them at each instantiation of this class.

    .. note::
        Support both float and double precision.
    """

    def __init__(self, width, height, alphas, cache=None):
        cache_name = f"{width}_{height}_{alphas}.npy"
        if cache is not None:
            if not os.path.exists(cache):
                os.makedirs(cache)

            cache_file = os.path.join(cache, cache_name)
            if os.path.exists(cache_file):
                shifted_spectrograms = np.load(cache_file)
            else:
                alpha_shearlet = AlphaShearletTransform(width, height, alphas, real=True, parseval=True)
                shifted_spectrograms = np.asarray([my_ifft_shift(spec) for spec in alpha_shearlet.spectrograms])
                np.save(cache_file, shifted_spectrograms)
        else:
            alpha_shearlet = AlphaShearletTransform(width, height, alphas, real=True, parseval=True)
            shifted_spectrograms = np.asarray([my_ifft_shift(spec) for spec in alpha_shearlet.spectrograms])

        self.shifted_spectrograms = torch.FloatTensor(shifted_spectrograms)

        self.shifted_spectrograms_d = torch.DoubleTensor(shifted_spectrograms)

    def _move_parameters_to_device(self, device):
        if device != self.shifted_spectrograms.device:
            self.shifted_spectrograms = self.shifted_spectrograms.to(device)
            self.shifted_spectrograms_d = self.shifted_spectrograms_d.to(device)

    @normalize_shape(2)
    def forward(self, x):
        """
        Do shearlet transform of a batch of images.

        :param x: PyTorch GPU tensor with shape :math:`(d_1, \\dots, d_n, h, w)`.
        :returns: PyTorch GPU tensor containing shearlet coefficients.
            Has shape :math:`(d_1, \\dots, d_n, \\text{n_shearlets}, h, w)`.
        """
        self._move_parameters_to_device(x.device)

        c = torch.fft.rfft(x, 2, norm="forward")
        print(self.shifted_spectrograms.size(), c.size())

        if x.dtype == torch.float64:
            cs = torch.einsum("fij,bijc->bfijc", self.shifted_spectrograms_d, c)
        else:
            cs = torch.einsum("fij,bijc->bfijc", self.shifted_spectrograms, c)

        return torch.fft.irfft(cs, 2, norm="forward")

    @normalize_shape(3)
    def backward(self, cs):
        """
        Do inverse shearlet transform.

        :param cs: PyTorch GPU tensor containing shearlet coefficients,
            with shape :math:`(d_1, \\dots, d_n, \\text{n_shearlets}, h, w)`.
        :returns: PyTorch GPU tensor containing reconstructed images.
            Has shape :math:`(d_1, \\dots, d_n, h, w)`.
        """

        cs_fft = torch.fft.rfft(cs, 2, norm="forward")
        print(self.shifted_spectrograms.size(), cs_fft.size())

        if cs.dtype == torch.float64:
            res = torch.einsum("fij,bfijc->bijc", self.shifted_spectrograms_d, cs_fft)
        else:
            print(self.shifted_spectrograms.size(), cs_fft.size())
            res = torch.einsum("fij,bfijc->bijc", self.shifted_spectrograms, cs_fft)

        return torch.fft.irfft(res, 2, norm="forward")
