import torch


def _normalize_shape(x, d):
    old_shape = x.size()[:-d]
    x = x.view(-1, *(x.size()[-d:]))
    return x, old_shape


def _unnormalize_shape(y, old_shape):
    if isinstance(y, torch.Tensor):
        y = y.view(*old_shape, *(y.size()[1:]))
    elif isinstance(y, tuple):
        y = [yy.view(*old_shape, *(yy.size()[1:])) for yy in y]

    return y


class ShapeNormalizer:
    def __init__(self, d):
        self.d = d
        self.old_shape = None

    def normalize(self, x):
        x, self.old_shape = _normalize_shape(x, self.d)
        return x

    def unnormalize(self, y):
        if self.old_shape is None:
            raise Exception("Calling `unnormalize` before `normalize` ")

        return _unnormalize_shape(y, self.old_shape)


def normalize_shape(d):
    """
    Input with shape (batch_1, ..., batch_n, s_1, ..., s_d) is reshaped to (batch, s_1, s_2, ...., s_d)
    fed to f and output is reshaped to (batch_1, ..., batch_n, s_1, ..., s_o).
    :param d: Number of non-batch dimensions
    """

    def wrap(f):
        def wrapped(self, x, *args, **kwargs):
            x, old_shape = _normalize_shape(x, d)

            y = f(self, x, *args, **kwargs)

            return _unnormalize_shape(y, old_shape)

        wrapped.__doc__ = f.__doc__
        return wrapped

    return wrap


def projection_property_maker(name):
    @property
    def prop(self):
        return getattr(self.projection.cfg, name)

    @prop.setter
    def prop(self, value):
        setattr(self.projection.cfg, name, value)

    return prop


def expose_projection_attributes(pyclass, attributes: list):
    """Exposes the attributes of the projection directly from the class.
    For example, after exposing "det_spacing_u" (internal name) as "det_spacing" (exposed name), setting radon.det_spacing = 32
    is equivalent to setting  radon.projection.cfg.det_spacing_u = 32

    Args:
        pyclass: A python class (not instance of a class but the actual class)
        attributes: List of attributes, each element can be a string (then exposed_name = internal_name) or a tuple (exposed_name, internal_name)
    """
    for x in attributes:
        exposed_name, internal_name = x if isinstance(x, tuple) else (x, x)
        setattr(pyclass, exposed_name, projection_property_maker(internal_name))
