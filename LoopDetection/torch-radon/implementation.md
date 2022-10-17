The library can be divided into 4 layers:
1. **CUDA Library**: Library that implements the main functionalities (forward/backward)
without depending on the other layers. This makes it easy to port TorchRadon to frameworks
other than PyTorch and also other languages.
2. **PyTorch C++ Extension**: Connect the CUDA library to Python handling the allocation 
of PyTorch tensors and conversion from tensors to GPU pointers.
3. **Autograd Functions**: Handle differentiability.
4. **Radon Classes**: API intended to be used by the final user.

A key idea is that there are only 2 functions (forward/backward) and the projection settings
are specified through parameters. Parameters are grouped into classes, level 4 handles the
creation of these classes, levels 3 and 2 just forward these parameters to level 1. This is
done to minimize the amount of code in 2 and 3.

Classes in level 4 should accept user friendly parameters and convert them into Parameter
Classes.

The parameters classes are the following:
 - **Projection setting**: Contains information about the projection type and configuration.
 - **Volume setting**: Contains information about the conversion between volume coordinates
 and spatial coordinates, i.e. how to go from indices [i,j] to spatial coordinates (x, y).
 - **Execution Parameters**: Contains settings for the execution of the CUDA Kernel
 (for example the block size). This doesn't change the results of the computation but may
 affect computation times and can be tuned to get optimal performance. 
 - **Texture Cache**: Not really a parameter class, implements a cache used by Level 1 to
 avoid the reallocation of Textures.  
    
## Volume Setting
A 2D volume has 2 indices [i,j] which correspond to (-y, x).
A 3D volume has 3 indices [i, j, h] which correspond to (z, -y, x).