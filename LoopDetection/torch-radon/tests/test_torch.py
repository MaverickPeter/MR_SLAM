import torch
import numpy as np
import torch_radon as tr
from unittest import TestCase


class TestTorch(TestCase):
    def test_differentiation(self):
        device = torch.device('cuda')
        x = torch.FloatTensor(1, 64, 64).to(device)
        x.requires_grad = True
        angles = torch.FloatTensor(np.linspace(0, 2 * np.pi, 10).astype(np.float32)).to(device)

        radon = tr.ParallelBeam(64, angles)

        # check that backward is implemented for fp and bp
        y = radon.forward(x)
        z = torch.mean(radon.backward(y))
        z.backward()
        self.assertIsNotNone(x.grad)

    def test_shapes(self):
        """
        Check using channels is ok
        """
        device = torch.device('cuda')
        angles = torch.FloatTensor(np.linspace(0, 2 * np.pi, 10).astype(np.float32)).to(device)
        radon = tr.ParallelBeam(64, angles)

        # test with 2 batch dimensions
        x = torch.FloatTensor(2, 3, 64, 64).to(device)
        y = radon.forward(x)
        self.assertEqual(y.size(), (2, 3, 10, 64))
        z = radon.backward(y)
        self.assertEqual(z.size(), (2, 3, 64, 64))

        # no batch dimensions
        x = torch.FloatTensor(64, 64).to(device)
        y = radon.forward(x)
        self.assertEqual(y.size(), (10, 64))
        z = radon.backward(y)
        self.assertEqual(z.size(), (64, 64))
