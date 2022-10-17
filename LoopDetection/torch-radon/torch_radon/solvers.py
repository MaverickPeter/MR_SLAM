import torch
from .utils import _normalize_shape, _unnormalize_shape


def normalize(x):
    size = x.size()
    x = x.view(size[0], -1)
    norm = torch.norm(x, dim=1)
    x /= norm.view(-1, 1)
    return x.view(*size), torch.max(norm).item()


class Landweber:
    """
    Class that implements Landweber iteration to solve :math:`\\min_{x \\in C}  \\|Ax-y\\|_2^2`
    (see `Wikipedia page <https://en.wikipedia.org/wiki/Landweber_iteration>`_).

    The iteration used is :math:`x_{n+1} = \\mathcal{P}_C(x - \\alpha A^T A x_n)` where :math:`\\mathcal{P}_C` is the
    projection onto :math:`C`.

    :param operator: Instance of a class that implements products :math:`A x` (``operator.forward(x)``) and :math:`A^T y`
        (``operator.backward(y)``).
    :param projection: Function that implements :math:`\\mathcal{P}_C(\\cdot)`, if not specified no projection is used.
    :param grad: If true gradient will be enabled, more memory will be used but it will be possible to backpropagate.
    """

    def __init__(self, operator, projection=None, grad=False):
        self.operator = operator
        self.projection = lambda x: x if projection is None else projection
        self.grad = grad

    def estimate_alpha(self, img_size, device, n_iter=50, batch_size=8):
        """
        Use power iteration on :math:`A^T A` to estimate the maximum step size that still guarantees convergence.

        .. note::
            Because this computation is not exact it is advised to use a value of alpha lower that the one estimated by
            this method (for example multiplying the estimate by 0.95).

        :param img_size: Size of the image
        :param device: GPU device that will be used for computation
        :param n_iter: Number of iterations
        :param batch_size: Number of vectors used in the power iteration.
        :return: Estimated value for alpha
        """
        with torch.no_grad():
            x = torch.randn((batch_size, img_size, img_size), device=device)
            x, _ = normalize(x)
            for i in range(n_iter):
                next_x = self.operator.backward(self.operator.forward(x))
                x, v = normalize(next_x)

            return 2.0 / v

    def run(self, x_zero, y, alpha, iterations=100, callback=None):
        """
        Execute Landweber iterations.

        :param x_zero: Initial solution guess used as a starting point for the iteration
        :param y: Value of y in :math:`\\min_{x \\in C}  \\|Ax-y\\|_2^2`
        :param alpha: Step size, can be estimated using :attr:`estimate_alpha`
        :param iterations: Number of iterations
        :param callback: Optional function that will be called at each iteration with :math:`x_n` as argument.
            Values returned by ``callback`` will be stored in a list and returned together with the computed solution
        :return: If ``callback`` is specified returns ``x, values`` where ``x`` is the solution computed by the
            Landweber iteration and ``values`` is the list of values returned by ``callback`` at each iteration. If
            ``callback`` is not specified returns only ``x``
        """
        res = []
        with torch.set_grad_enabled(self.grad):
            x = x_zero.clone()
            for i in range(iterations):
                x = self.projection(x - alpha * self.operator.backward(self.operator.forward(x) - y))
                if callback is not None:
                    res.append(callback(x))

        if callback is not None:
            return x, res
        else:
            return x


def cg(forward, x, y, callback=None, max_iter=500, tol=1e-5):
    """
    Implements Conjugate Gradient algorithm for solving :math:`\\min_x  \\|Ax-y\\|_2^2`.

    .. note::
        For conjugate gradient to work the matrix :math:`A` must be symmetric positive definite. Otherwise use other
        solvers.

    :param forward: function that implements products :math:`A x` (``forward(x)``).
    :param x: Initial solution guess used as a starting point for the iteration
    :param y: Value of y in :math:`\\min_{x \\in C}  \\|Ax-y\\|_2^2`
    :param callback: Optional function that will be called at each iteration with :math:`x_n` and the residual as
            arguments. Values returned by ``callback`` will be stored in a list and returned together with the computed
            solution.
    :param max_iter: Maximum number of iterations.
    :param tol: Algorithm is stopped when :math:`\\frac{\\| Ax_n - y \\|}{\\| y \\|} \\leq \\text{tol}`
    :return: If ``callback`` is specified returns ``x, values`` where ``x`` is the solution computed by the
        Landweber iteration and ``values`` is the list of values returned by ``callback`` at each iteration. If
        ``callback`` is not specified returns only ``x``.
    """

    # normalize shape to be (batch, height, width)
    x, old_shape = _normalize_shape(x, 2)
    y, _ = _normalize_shape(y, 2)

    y_norm = torch.norm(y, dim=(1, 2))

    r = y - forward(x)
    p = r.clone()
    r_n = torch.sum(r ** 2, dim=(1, 2))

    values = []
    for i in range(max_iter):
        Ap = forward(p)
        alpha = (r_n / torch.sum(p * Ap, dim=(1, 2)).clamp_min(1e-8)).view(-1, 1, 1)
        x += alpha * p
        r_next = r - alpha * Ap

        if callback is not None:
            values.append(callback(x, r_next))

        r_next_n = torch.sum(r_next ** 2, dim=(1, 2))
        rs = torch.min(torch.sqrt(r_next_n) / y_norm).item()
        if rs <= tol:
            break

        beta = (r_next_n / r_n).view(-1, 1, 1)
        r_n = r_next_n
        p = r_next + beta * p
        r = r_next.clone()

    x = _unnormalize_shape(x, old_shape)

    if callback is None:
        return x
    else:
        return x, values


def cgne(operator, x, y, callback=None, max_iter=5000, tol=1e-5):
    """
    Implements Conjugate Gradient on the Normal Equations, an algorithm for solving :math:`\\min_x  \\|Ax-y\\|_2^2`.

    :param operator: Instance of a class that implements products :math:`A x` (``operator.forward(x)``) and :math:`A^T y`
        (``operator.backward(y)``).
    :param x: Initial solution guess used as a starting point for the iteration
        :param y: Value of y in :math:`\\min_{x \\in C}  \\|Ax-y\\|_2^2`
    :param callback: Optional function that will be called at each iteration with :math:`x_n` as argument.
            Values returned by ``callback`` will be stored in a list and returned together with the computed solution
    :param max_iter: Maximum number of iterations
    :param tol: Algorithm is stopped when :math:`\\frac{\\| s \\|}{\\| y \\|} \\leq \\text{tol}`
    :return: If ``callback`` is specified returns ``x, values`` where ``x`` is the solution computed by the
        Landweber iteration and ``values`` is the list of values returned by ``callback`` at each iteration. If
        ``callback`` is not specified returns only ``x``
    """
    # normalize shape to be (batch, height, width)
    x, old_shape = _normalize_shape(x, 2)
    y, _ = _normalize_shape(y, 2)

    y_norm = torch.norm(y, dim=(1, 2))
    d = y - operator.forward(x)

    s = operator.backprojection(d)
    p = s.clone()
    s_norm = torch.sum(s ** 2, dim=(1, 2))

    values = []

    for i in range(max_iter):
        q = operator.forward(p)
        alpha_den = torch.sum(q ** 2, dim=(1, 2)).clamp_min(1e-8)
        alpha = (s_norm / alpha_den).view(-1, 1, 1)

        x = x + alpha * p
        d = d - alpha * q
        s_next = operator.backprojection(d)
        s_next_norm = torch.sum(s_next ** 2, dim=(1, 2))

        if callback is not None:
            values.append(callback(x))

        # if s_next_norm is small break
        sn = torch.max(torch.sqrt(s_next_norm) / y_norm).item()
        if sn <= tol:
            break

        beta = (s_next_norm / s_norm).view(-1, 1, 1)
        s = s_next
        s_norm = s_next_norm
        p = s + beta * p

    x = _unnormalize_shape(x, old_shape)

    if callback is None:
        return x
    else:
        return x, values
