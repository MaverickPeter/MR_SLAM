import torch
import os

if os.getenv("TORCH_RADON_DOC_BUILD"):
    print("Not importing cuda backend because this is just the doc build")
    VolumeCfg = None
    ProjectionCfg = None

    class ExecCfg:
        pass
else:
    from torch_radon_cuda import *
