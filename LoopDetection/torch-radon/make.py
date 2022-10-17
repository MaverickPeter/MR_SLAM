import sys
from glob import glob
import os
import shutil
import subprocess
import re

from ptx_annotation import annotate_ptx


def mapper(src, dst):
    paths = glob(src)

    src_pre = src.index("*")
    src_post = src_pre - len(src) + 1
    pre = dst[:dst.index("*")]
    post = dst[dst.index("*") + 1:]

    return [(path, pre + path[src_pre:src_post] + post) for path in paths]


def run(command):
    print(f"\u001b[34m{command}\u001b[0m")
    if os.system(command) != 0:
        print("\u001b[31mERROR IN COMPILATION\u001b[0m")
        exit(-1)


def run_compilation(files, f):
    for src, dst in files:
        if not os.path.exists(dst) or os.path.getmtime(src) > os.path.getmtime(dst):
            print(f"\u001b[32mCompiling {src}\u001b[0m")
            command = f(src, dst)
            run(command)
        else:
            print(f"\u001b[32mAlready compiled {src}\u001b[0m")


def get_cuda_version(cuda_home):
    print("")
    print(cuda_home)

    nvcc_out = subprocess.run([f"{cuda_home}/bin/nvcc", "--version"], stdout=subprocess.PIPE).stdout.decode('utf-8')
    m = re.search(r"V[0-9]+.[0-9]+", nvcc_out)
    str_version = m.group(0)[1:]

    return int(str_version.replace(".", ""))


def build(compute_capabilities=(60, 70, 75, 80, 86), debug=False, cuda_home="/usr/local/cuda", cxx="g++",
          keep_intermediate=False):

    cuda_version = get_cuda_version(cuda_home)
    nvcc = f"{cuda_home}/bin/nvcc"
    include_dirs = ["./include"]
    intermediate_dir = "intermediates"

    # compute capabilities >= 80 are only for cuda >= 11
    if cuda_version <= 110:
        compute_capabilities = [x for x in compute_capabilities if x < 80]

    cu_files = mapper("src/*.cu", "objs/cuda/*.o")
    cpp_files = mapper("src/*.cpp", "objs/*.o")
    cpp_files = [x for x in cpp_files if x[0] != "src/pytorch.cpp"]

    all_objects = [y for x, y in cu_files + cpp_files]

    opt_flags = ["-g"] if debug else ["-DNDEBUG", "-O3"]

    include_flags = [f"-I{x}" for x in include_dirs]
    # -Wl,-Bstatic -lm
    cxx_flags = ["-std=gnu++11 -fPIC -D_GLIBCXX_USE_CXX11_ABI=0 -Wall"] + include_flags + opt_flags
    nvcc_base_flags = ["-std=c++11", f"-ccbin={cxx}", "-Xcompiler", "-fPIC",
                       "-Xcompiler -D_GLIBCXX_USE_CXX11_ABI=0"] + include_flags + opt_flags + [
                           "--generate-line-info --compiler-options -Wall", '-Xcudafe "--diag_suppress=unrecognized_gcc_pragma"']
    nvcc_flags = nvcc_base_flags + [f"-gencode arch=compute_{x},code=sm_{x}" for x in compute_capabilities]

    if keep_intermediate:
        if not os.path.exists(intermediate_dir):
            os.mkdir(intermediate_dir)
        nvcc_flags.append(f"-keep --keep-dir {intermediate_dir}")

    cxx_flags = " ".join(cxx_flags)
    nvcc_flags = " ".join(nvcc_flags)

    # create output directory
    if not os.path.exists("objs/cuda"):
        os.makedirs("objs/cuda")

    # compile
    run_compilation(cu_files, lambda src, dst: f"{nvcc} {nvcc_flags} -c {src} -o {dst}")
    run_compilation(cpp_files, lambda src, dst: f"{cxx} {cxx_flags} -c {src} -o {dst}")

    run(f"ar rc objs/libradon.a {' '.join(all_objects)}")

    if keep_intermediate:
        for path in os.listdir(intermediate_dir):
            if path.endswith(".ptx"):
                annotate_ptx(os.path.join(intermediate_dir, path))
            else:
                os.remove(os.path.join(intermediate_dir, path))


def clean():
    print(f"\u001b[32mCleaning\u001b[0m")
    for f in ["objs", "build", "dist", "intermediates", "torch_radon.egg-info"]:
        if os.path.exists(f):
            print(f"Removing '{f}'")
            shutil.rmtree(f)


def detect_compute_capabilities():
    import torch
    ccs = []
    for i in range(torch.cuda.device_count()):
        compute_capability = torch.cuda.get_device_capability(i)
        ccs.append(compute_capability[0]*10 + compute_capability[1])

    return ccs


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "clean":
        clean()
    else:
        cuda_home = os.getenv("CUDA_HOME", "/usr/local/cuda")
        compute_capabilities = (60, 70, 75, 80, 86)
        debug = "--debug" in sys.argv
        cxx = os.getenv("CXX", "g++")
        keep_intermediate = "--keep-intermediate" in sys.argv

        if "--local" in sys.argv:
            print("Detecting compute capabilities of local GPUs")
            compute_capabilities = detect_compute_capabilities()
            print("Compiling for compute capability", " and ".join([str(x) for x in compute_capabilities]))

        build(compute_capabilities, debug, cuda_home, cxx, keep_intermediate)
