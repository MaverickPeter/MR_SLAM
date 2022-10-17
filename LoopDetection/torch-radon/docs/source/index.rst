:github_url: https://github.com/matteo-ronchetti/torch-radon

Torch Radon Documentation
=============================

Torch Radon is a fast CUDA implementation of transforms needed for
working with computed tomography data in Pytorch. It allows the training of end-to-end models that takes sinograms as inputs and produce images as output.

Main features:
 - All operations work directly on Pytorch GPU tensors.
 - Forward and back projections are differentiable and integrated with Pytorch `.backward()`.
 - Up to 50x faster than Astra Toolbox.
 - Supports half precision and can used togheter with amp for faster training.

Projection types:
 - Parallel Beam
 - Fan Beam
 - Cone Bean


.. toctree::
   :glob:
   :maxdepth: 1
   :caption: Getting Started

   getting_started/*

.. toctree::
   :glob:
   :maxdepth: 1
   :caption: Package reference

   modules/*

Indices and tables
==================

* :ref:`genindex`
