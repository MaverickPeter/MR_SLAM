from setuptools import setup
from torch.utils.cpp_extension import BuildExtension, CUDAExtension
import os
from make import build

with open("README.md", "r") as fh:
    long_description = fh.read()

cuda_home = os.getenv("CUDA_HOME", "/usr/local/cuda")
print(f"Using CUDA_HOME={cuda_home}")
build(cuda_home=cuda_home)

setup(name='torch_radon',
      version="2.0.0",
      author="Matteo Ronchetti",
      author_email="mttronchetti@gmail.com",
      description="Radon transform in PyTorch",
      long_description=long_description,
      long_description_content_type="text/markdown",
      url="https://github.com/matteo-ronchetti/torch-radon",

      packages=['torch_radon'],
      package_dir={
          'torch_radon': './torch_radon',
      },
      ext_modules=[
          CUDAExtension('torch_radon_cuda', [os.path.abspath('src/pytorch.cpp')],
                        include_dirs=[os.path.abspath('include')],
                        library_dirs=[os.path.abspath("objs")],
                        libraries=["m", "c", "gcc", "stdc++", "cufft", "radon"],
                        # strip debug symbols
                        extra_link_args=["-Wl,--strip-all"]
                        )
      ],
      cmdclass={'build_ext': BuildExtension},
      zip_safe=False,
      classifiers=[
          "Programming Language :: Python :: 3",
          "Operating System :: POSIX :: Linux",
          "License :: OSI Approved :: GNU General Public License v3 (GPLv3)"
      ],
      install_requires=[
          "scipy",
          "alpha-transform"
      ],
      )


"""
g++ -pthread -B /home/matteo/miniconda3/compiler_compat -L/home/matteo/miniconda3/lib -Wl,-rpath=/home/matteo/miniconda3/lib -Wl,--no-as-needed -Wl,--sysroot=/ /home/matteo/projects/torch-radon/build/temp.linux-x86_64-3.7/home/matteo/projects/torch-radon/src/pytorch.o -L/home/matteo/projects/torch-radon/objs -L/home/matteo/miniconda3/lib/python3.7/site-packages/torch/lib -L/usr/local/cuda-11/lib64 -lradon -lc10 -ltorch -ltorch_cpu -ltorch_python -lcudart -lc10_cuda -ltorch_cuda_cu -ltorch_cuda_cpp -o build/lib.linux-x86_64-3.7/torch_radon_cuda.cpython-37m-x86_64-linux-gnu.so -Wl,--strip-all -static-libgcc -static-libstdc++ -Wl,-static
g++ -shared /home/matteo/projects/torch-radon/build/temp.linux-x86_64-3.7/home/matteo/projects/torch-radon/src/pytorch.o -o build/lib.linux-x86_64-3.7/torch_radon_cuda.cpython-37m-x86_64-linux-gnu.so
g++ -pthread -B /home/matteo/miniconda3/compiler_compat -Wl,--sysroot=/ -Wsign-compare -DNDEBUG -g -fwrapv -O3 -Wall -Wstrict-prototypes -fPIC -I/home/matteo/projects/torch-radon/include -I/home/matteo/miniconda3/lib/python3.7/site-packages/torch/include -I/home/matteo/miniconda3/lib/python3.7/site-packages/torch/include/torch/csrc/api/include -I/home/matteo/miniconda3/lib/python3.7/site-packages/torch/include/TH -I/home/matteo/miniconda3/lib/python3.7/site-packages/torch/include/THC -I/usr/local/cuda-11/include -I/home/matteo/miniconda3/include/python3.7m -c -c /home/matteo/projects/torch-radon/src/pytorch.cpp -o /home/matteo/projects/torch-radon/build/temp.linux-x86_64-3.7/home/matteo/projects/torch-radon/src/pytorch.o -static -static-libgcc -static-libstdc++ -DTORCH_API_INCLUDE_EXTENSION_H '-DPYBIND11_COMPILER_TYPE="_gcc"' '-DPYBIND11_STDLIB="_libstdcpp"' '-DPYBIND11_BUILD_ABI="_cxxabi1011"' -DTORCH_EXTENSION_NAME=torch_radon_cuda -D_GLIBCXX_USE_CXX11_ABI=0 -std=c++14

"""