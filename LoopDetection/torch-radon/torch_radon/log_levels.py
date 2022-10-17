from . import cuda_backend

DEBUG = 0
INFO = 1
WARN = 2
WARNING = 2
ERROR = 3

def set_log_level(level):
    level = max(0, min(int(level), ERROR))
    cuda_backend.set_log_level(level)