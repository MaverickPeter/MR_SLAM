import torch
import numpy as np
from alpha_transform import AlphaShearletTransform
from torch_radon.shearlet import ShearletTransform
from nose.tools import assert_less
from .utils import relative_error


def test_shearlet():
    img_size = 128
    device = torch.device("cuda")
    scales = [0.5] * 2

    x = np.random.uniform(0, 1, (img_size, img_size))

    shearlet = ShearletTransform(img_size, img_size, scales)
    alpha_shearlet = AlphaShearletTransform(img_size, img_size, scales, real=True, parseval=True)
    print(alpha_shearlet.spectrograms[0].dtype)

    coeff_ = alpha_shearlet.transform(x, do_norm=False)
    rec_ = alpha_shearlet.adjoint_transform(coeff_, do_norm=False)


    with torch.no_grad():
        # check with double precision
        dx = torch.DoubleTensor(x).to(device)
        coeff = shearlet.forward(dx)
        rec = shearlet.backward(coeff)

        x_err_d = (torch.norm(rec - dx) / torch.norm(dx)).item()
        coef_err_d = relative_error(coeff_, coeff.cpu().numpy())
        rec_err_d = relative_error(rec_, rec.cpu().numpy())
        print(x_err_d, coef_err_d, rec_err_d)

        # check with single precision
        dx = torch.FloatTensor(x).to(device)
        coeff = shearlet.forward(dx)
        rec = shearlet.backward(coeff)

        x_err = (torch.norm(rec - dx) / torch.norm(dx)).item()
        coef_err = relative_error(coeff_, coeff.cpu().numpy())
        rec_err = relative_error(rec_, rec.cpu().numpy())
        print(x_err, coef_err, rec_err)

        assert_less(x_err_d, 1e-15)
        assert_less(coef_err_d, 1e-15)
        assert_less(rec_err_d, 1e-15)

        assert_less(x_err, 1e-6)
        assert_less(coef_err, 1e-6)
        assert_less(rec_err, 1e-6)

    # print(np.allclose(coeff_, coeff.cpu().numpy()))
    # print(np.allclose(rec_, rec.cpu().numpy()))
    #
    # print("Alpha forward", timeit.timeit(lambda: alpha_shearlet.transform(im, do_norm=False), number=10))
    # print("Alpha backward", timeit.timeit(lambda: alpha_shearlet.adjoint_transform(coeff_, do_norm=False), number=10))
    # print("My forward", timeit.timeit(lambda: shearlet.forward(x), number=30))
    # print("My backward", timeit.timeit(lambda: shearlet.backward(coeff), number=30))
