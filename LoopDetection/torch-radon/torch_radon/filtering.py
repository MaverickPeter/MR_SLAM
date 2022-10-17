import numpy as np
import torch
import torch.fft

try:
    import scipy.fft

    fftmodule = scipy.fft
except ImportError:
    import numpy.fft

    fftmodule = numpy.fft


class FourierFilters:
    def __init__(self):
        self.cache = dict()

    def get(self, size: int, filter_name: str, device):
        key = (size, filter_name)

        if key not in self.cache:
            ff = torch.FloatTensor(self.construct_fourier_filter(size, filter_name)).view(1, 1, -1, 1).to(device)
            self.cache[key] = ff

        return self.cache[key].to(device)

    @staticmethod
    def construct_fourier_filter(size, filter_name):
        """Construct the Fourier filter.

        This computation lessens artifacts and removes a small bias as
        explained in [1], Chap 3. Equation 61.

        Parameters
        ----------
        size: int
            filter size. Must be even.
        filter_name: str
            Filter used in frequency domain filtering. Filters available:
            ram-lak (ramp), shepp-logan, cosine, hamming, hann.

        Returns
        -------
        fourier_filter: ndarray
            The computed Fourier filter.

        References
        ----------
        .. [1] AC Kak, M Slaney, "Principles of Computerized Tomographic
               Imaging", IEEE Press 1988.

        """
        filter_name = filter_name.lower()

        n = np.concatenate((np.arange(1, size / 2 + 1, 2, dtype=np.int),
                            np.arange(size / 2 - 1, 0, -2, dtype=np.int)))
        f = np.zeros(size)
        f[0] = 0.25
        f[1::2] = -1 / (np.pi * n) ** 2

        # Computing the ramp filter from the fourier transform of its
        # frequency domain representation lessens artifacts and removes a
        # small bias as explained in [1], Chap 3. Equation 61
        fourier_filter = 2 * np.real(fftmodule.fft(f))  # ramp filter
        if filter_name == "ramp" or filter_name == "ram-lak":
            pass
        elif filter_name == "shepp-logan":
            # Start from first element to avoid divide by zero
            omega = np.pi * fftmodule.fftfreq(size)[1:]
            fourier_filter[1:] *= np.sin(omega) / omega
        elif filter_name == "cosine":
            freq = np.linspace(0, np.pi, size, endpoint=False)
            cosine_filter = fftmodule.fftshift(np.sin(freq))
            fourier_filter *= cosine_filter
        elif filter_name == "hamming":
            fourier_filter *= fftmodule.fftshift(np.hamming(size))
        elif filter_name == "hann":
            fourier_filter *= fftmodule.fftshift(np.hanning(size))
        else:
            print(
                f"[TorchRadon] Error, unknown filter type '{filter_name}', available filters are: 'ramp', 'shepp-logan', 'cosine', 'hamming', 'hann'")

        return fourier_filter[:size//2+1]
