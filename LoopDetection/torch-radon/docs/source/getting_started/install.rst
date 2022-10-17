Install Locally
=====================

Precompiled Package
---------------------
If you are running Linux you can install Torch Radon by running:

.. code-block:: bash

    wget -qO- https://raw.githubusercontent.com/matteo-ronchetti/torch-radon/master/auto_install.py  | python -

Docker Image
--------------
Docker images with PyTorch CUDA and Torch Radon are available `here <https://hub.docker.com/repository/docker/matteoronchetti/torch-radon>`_.

.. code-block:: bash

    docker pull matteoronchetti/torch-radon

To use the GPU in docker you need to use `nvidia-docker <https://github.com/NVIDIA/nvidia-docker>`_.


Compile from Source
---------------------

You need to have `CUDA <https://developer.nvidia.com/cuda-toolkit>`_ and `PyTorch <https://pytorch.org/get-started/locally/>`_ installed, then run:

.. code-block:: bash

    git clone https://github.com/matteo-ronchetti/torch-radon.git
    cd torch-radon
    python setup.py install

If you encounter any problem please contact the author or open an issue.