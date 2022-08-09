# GLOBAL
NUM_POINTS = 4096
FEATURE_OUTPUT_DIM = 1024
RESULTS_FOLDER = "results/"
OUTPUT_FILE = "results/results.txt"

LOG_DIR = 'log2/'
MODEL_FILENAME = "model.ckpt"

DATASET_FOLDER = '../data1/xxc/benchmark_datasets/'

# TRAIN
BATCH_NUM_QUERIES = 1
TRAIN_POSITIVES_PER_QUERY = 2
TRAIN_NEGATIVES_PER_QUERY = 4
DECAY_STEP = 200000
DECAY_RATE = 0.7
BASE_LEARNING_RATE = 0.0005
MOMENTUM = 0.9
OPTIMIZER = 'ADAM'
MAX_EPOCH = 20

MARGIN_1 = 0.5
MARGIN_2 = 0.2

BN_INIT_DECAY = 0.5
BN_DECAY_DECAY_RATE = 0.5
BN_DECAY_CLIP = 0.99

RESUME = False

TRAIN_FILE = 'generating_queries/training_queries_baseline.pickle'
TEST_FILE = 'generating_queries/test_queries_baseline.pickle'

# LOSS
LOSS_FUNCTION = 'quadruplet'
LOSS_LAZY = True
TRIPLET_USE_BEST_POSITIVES = False
LOSS_IGNORE_ZERO_BATCH = False

# EVAL6
EVAL_BATCH_SIZE = 1
EVAL_POSITIVES_PER_QUERY = 2
EVAL_NEGATIVES_PER_QUERY = 4

EVAL_DATABASE_FILE = '/home/xxc/data1/xxc/NCLT_kit/nclt_generating_queries/nclt_evaluation_database_sc_density.pickle'
EVAL_QUERY_FILE = '/home/xxc/data1/xxc/NCLT_kit/nclt_generating_queries/nclt_evaluation_query_sc_density.pickle'

num_ring = 40
num_sector = 120
num_height = 1
max_length = 1
max_height = 1

# ICP Parameters
max_icp_iter = 50
icp_tolerance = 0.001
icp_max_distance = 5.0
icp_fitness_score = 0.22

def cfg_str():
    out_string = ""
    for name in globals():
        if not name.startswith("__") and not name.__contains__("cfg_str"):
            #print(name, "=", globals()[name])
            out_string = out_string + "cfg." + name + \
                "=" + str(globals()[name]) + "\n"
    return out_string
