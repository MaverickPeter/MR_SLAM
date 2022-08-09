import argparse
import importlib
import math
import os
import socket
import sys
import time
import numpy as np
from sklearn.neighbors import KDTree, NearestNeighbors
from torchvision import transforms, utils
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import config as cfg
import evaluate
import loss.loss_function as SC_loss
import models.DiSCO as SC
import torch
import torch.nn as nn
from loading_pointclouds import *
from tensorboardX import SummaryWriter
from torch.autograd import Variable
from torch.backends import cudnn
import gpuadder
import cv2

os.environ["CUDA_VISIBLE_DEVICES"] = "0"

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
cudnn.enabled = True


parser = argparse.ArgumentParser()
parser.add_argument('--log_dir', default='log/', help='Log dir [default: log]')
parser.add_argument('--results_dir', default='results/',
                    help='results dir [default: results]')
parser.add_argument('--positives_per_query', type=int, default=2,
                    help='Number of potential positives in each training tuple [default: 2]')
parser.add_argument('--negatives_per_query', type=int, default=18,
                    help='Number of definite negatives in each training tuple [default: 18]')
parser.add_argument('--max_epoch', type=int, default=20,
                    help='Epoch to run [default: 20]')
parser.add_argument('--batch_num_queries', type=int, default=2,
                    help='Batch Size during training [default: 2]')
parser.add_argument('--learning_rate', type=float, default=0.000001,
                    help='Initial learning rate [default: 0.000005]')
parser.add_argument('--momentum', type=float, default=0.9,
                    help='Initial learning rate [default: 0.9]')
parser.add_argument('--optimizer', default='adam',
                    help='adam or momentum [default: adam]')
parser.add_argument('--decay_step', type=int, default=200000,
                    help='Decay step for lr decay [default: 200000]')
parser.add_argument('--decay_rate', type=float, default=0.7,
                    help='Decay rate for lr decay [default: 0.7]')
parser.add_argument('--margin_1', type=float, default=0.5,
                    help='Margin for hinge loss [default: 0.5]')
parser.add_argument('--margin_2', type=float, default=0.2,
                    help='Margin for hinge loss [default: 0.2]')
parser.add_argument('--loss_function', default='quadruplet', choices=[
                    'triplet', 'quadruplet'], help='triplet or quadruplet [default: quadruplet]')
parser.add_argument('--loss_not_lazy', action='store_false',
                    help='If present, do not use lazy variant of loss')
parser.add_argument('--loss_ignore_zero_batch', action='store_true',
                    help='If present, mean only batches with loss > 0.0')
parser.add_argument('--triplet_use_best_positives', action='store_true',
                    help='If present, use best positives, otherwise use hardest positives')
parser.add_argument('--resume', action='store_true',
                    help='If present, restore checkpoint and resume training')
parser.add_argument('--dataset_folder', default='../../dataset/',
                    help='PointNetVlad Dataset Folder')
parser.add_argument('--input_type', default='image',
                    help='Input of the network, can be [point] or scan [image], [default: point]')

FLAGS = parser.parse_args()
cfg.BATCH_NUM_QUERIES = 1
#cfg.EVAL_BATCH_SIZE = 12
cfg.NUM_POINTS = 4096
cfg.INPUT_TYPE = FLAGS.input_type
cfg.TRAIN_POSITIVES_PER_QUERY = FLAGS.positives_per_query
cfg.TRAIN_NEGATIVES_PER_QUERY = FLAGS.negatives_per_query
cfg.MAX_EPOCH = FLAGS.max_epoch
cfg.BASE_LEARNING_RATE = FLAGS.learning_rate
cfg.MOMENTUM = FLAGS.momentum
cfg.OPTIMIZER = FLAGS.optimizer
cfg.DECAY_STEP = FLAGS.decay_step
cfg.DECAY_RATE = FLAGS.decay_rate
cfg.MARGIN1 = FLAGS.margin_1
cfg.MARGIN2 = FLAGS.margin_2
cfg.FEATURE_OUTPUT_DIM = 256
cfg.LOSS_FUNCTION = FLAGS.loss_function
cfg.TRIPLET_USE_BEST_POSITIVES = FLAGS.triplet_use_best_positives
cfg.LOSS_LAZY = FLAGS.loss_not_lazy
cfg.LOSS_IGNORE_ZERO_BATCH = FLAGS.loss_ignore_zero_batch
cfg.overlap_num = 1
cfg.num_ring = 40
cfg.num_sector = 120
cfg.num_height = 20
cfg.max_length = 1

cfg.TRAIN_FILE = '/home/xxc/data1/xxc/NCLT_kit/nclt_generating_queries/training_queries_baseline_sc_density.pickle'
cfg.TEST_FILE = '/home/xxc/data1/xxc/NCLT_kit/nclt_generating_queries/test_queries_baseline_sc_density.pickle'

cfg.LOG_DIR = FLAGS.log_dir
if not os.path.exists(cfg.LOG_DIR):
    os.mkdir(cfg.LOG_DIR)
LOG_FOUT = open(os.path.join(cfg.LOG_DIR, 'log_train.txt'), 'w')
LOG_FOUT.write(str(FLAGS) + '\n')

cfg.RESULTS_FOLDER = FLAGS.results_dir

cfg.DATASET_FOLDER = FLAGS.dataset_folder

# Load dictionary of training queries
TRAINING_QUERIES = get_queries_dict(cfg.TRAIN_FILE)
TEST_QUERIES = get_queries_dict(cfg.TEST_FILE)

cfg.BN_INIT_DECAY = 0.5
cfg.BN_DECAY_DECAY_RATE = 0.5
BN_DECAY_DECAY_STEP = float(cfg.DECAY_STEP)
cfg.BN_DECAY_CLIP = 0.99

HARD_NEGATIVES = {}
TRAINING_LATENT_VECTORS = []

TOTAL_ITERATIONS = 0

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# ------------------------- Config Done ---------------------------------------

def get_bn_decay(batch):
    bn_momentum = cfg.BN_INIT_DECAY * \
        (cfg.BN_DECAY_DECAY_RATE **
         (batch * cfg.BATCH_NUM_QUERIES // BN_DECAY_DECAY_STEP))
    return min(cfg.BN_DECAY_CLIP, 1 - bn_momentum)


def log_string(out_str):
    LOG_FOUT.write(out_str + '\n')
    LOG_FOUT.flush()
    print(out_str)

# learning rate halfed every 5 epoch

def get_learning_rate(epoch):
    learning_rate = cfg.BASE_LEARNING_RATE * ((0.9) ** (epoch // 5))
    learning_rate = max(learning_rate, 0.00001)  # CLIP THE LEARNING RATE!
    return learning_rate


def train():
    global HARD_NEGATIVES, TOTAL_ITERATIONS
    bn_decay = get_bn_decay(0)

    if cfg.LOSS_FUNCTION == 'quadruplet'and cfg.INPUT_TYPE == 'point':
        loss_function = SC_loss.quadruplet_loss
    elif cfg.INPUT_TYPE == 'image':
        loss_function = SC_loss.quadruplet_loss
    else:
        loss_function = SC_loss.quadruplet_loss
    learning_rate = get_learning_rate(0)

    train_writer = SummaryWriter(os.path.join(cfg.LOG_DIR, 'train'))

    # -------------- Model formation -----------------
    if cfg.INPUT_TYPE == 'point':
        model = SC.PointNetVlad(global_feat=True, feature_transform=True,
                                max_pool=False, output_dim=cfg.FEATURE_OUTPUT_DIM, num_points=cfg.NUM_POINTS)
    elif cfg.INPUT_TYPE == 'image':
        model = SC.SCNet(global_feat=True, feature_transform=True,
                                max_pool=False, output_dim=cfg.FEATURE_OUTPUT_DIM, num_points=cfg.NUM_POINTS)
        corr2soft = SC.Corr2Softmax(200., 0.)

    corr2soft = corr2soft.to(device)
    model = model.to(device)

    parameters = filter(lambda p: p.requires_grad, model.parameters())

    # -------------- Optimizer -----------------
    if cfg.OPTIMIZER == 'momentum':
        optimizer = torch.optim.SGD(
            parameters, learning_rate, momentum=cfg.MOMENTUM)
    elif cfg.OPTIMIZER == 'adam':
        optimizer = torch.optim.Adam(parameters, learning_rate)
        optimizer_c2s = torch.optim.Adam(filter(lambda p: p.requires_grad, corr2soft.parameters()), lr=1e-3)
    else:
        optimizer = None
        exit(0)

    # -------------- Resume -----------------
    if FLAGS.resume:
        resume_filename = cfg.LOG_DIR + "model.ckpt"
        print("Resuming From ", resume_filename)
        checkpoint = torch.load(resume_filename)
        saved_state_dict = checkpoint['state_dict']
        starting_epoch = checkpoint['epoch']
        TOTAL_ITERATIONS = starting_epoch * len(TRAINING_QUERIES)

        model.load_state_dict(saved_state_dict)
        optimizer.load_state_dict(checkpoint['optimizer'])
    else:
        starting_epoch = 0

    model = nn.DataParallel(model)

    LOG_FOUT.write(cfg.cfg_str())
    LOG_FOUT.write("\n")
    LOG_FOUT.flush()

    # -------------- Training -----------------
    for epoch in range(starting_epoch, cfg.MAX_EPOCH):
        print(epoch)
        print()
        log_string('**** EPOCH %03d ****' % (epoch))
        sys.stdout.flush()
        
        train_one_epoch(model, optimizer, corr2soft, optimizer_c2s, train_writer, loss_function, epoch)
        
        log_string('EVALUATING...')
        cfg.OUTPUT_FILE = cfg.RESULTS_FOLDER + 'results_' + str(epoch) + '.txt'
        eval_recall = evaluate.evaluate_model(model, corr2soft)
        log_string('EVAL RECALL: %s' % str(eval_recall))

        train_writer.add_scalar("Val Recall", eval_recall, epoch)



def train_one_epoch(model, optimizer, corr2soft, optimizer_c2s, train_writer, loss_function, epoch):
    global HARD_NEGATIVES
    global TRAINING_LATENT_VECTORS, TOTAL_ITERATIONS

    is_training = True
    sampled_neg = 4000
    # number of hard negatives in the training tuple
    # which are taken from the sampled negatives
    num_to_take = 5

    # Shuffle train files
    train_file_idxs = np.arange(0, len(TRAINING_QUERIES.keys()))
    np.random.shuffle(train_file_idxs)

    for i in range(len(train_file_idxs)//cfg.BATCH_NUM_QUERIES):
        # for i in range (5):
        batch_keys = train_file_idxs[i *
                                     cfg.BATCH_NUM_QUERIES:(i+1)*cfg.BATCH_NUM_QUERIES]
        q_tuples = []

        faulty_tuple = False
        no_other_neg = False
        for j in range(cfg.BATCH_NUM_QUERIES):
            if (len(TRAINING_QUERIES[batch_keys[j]]["positives"]) < cfg.TRAIN_POSITIVES_PER_QUERY):
                # print("len pos",len(TRAINING_QUERIES[batch_keys[j]]["positives"]))
                faulty_tuple = True
                break

            # no cached feature vectors
            if (len(TRAINING_LATENT_VECTORS) == 0):
                q_tuples.append(get_query_tuple(TRAINING_QUERIES[batch_keys[j]], cfg.TRAIN_POSITIVES_PER_QUERY, cfg.TRAIN_NEGATIVES_PER_QUERY,
                                    TRAINING_QUERIES, hard_neg=[], other_neg=True))
                
                # q_tuples.append(get_rotated_tuple(TRAINING_QUERIES[batch_keys[j]],POSITIVES_PER_QUERY,NEGATIVES_PER_QUERY, TRAINING_QUERIES, hard_neg=[], other_neg=True))
                # q_tuples.append(get_jittered_tuple(TRAINING_QUERIES[batch_keys[j]],POSITIVES_PER_QUERY,NEGATIVES_PER_QUERY, TRAINING_QUERIES, hard_neg=[], other_neg=True))

            elif (len(HARD_NEGATIVES.keys()) == 0):
                query = get_feature_representation(
                    TRAINING_QUERIES[batch_keys[j]]['query'], model)
                random.shuffle(TRAINING_QUERIES[batch_keys[j]]['negatives'])
                negatives = TRAINING_QUERIES[batch_keys[j]
                                             ]['negatives'][0:sampled_neg]
                hard_negs = get_random_hard_negatives(
                    query, negatives, num_to_take)
                print(hard_negs)
                q_tuples.append(
                    get_query_tuple(TRAINING_QUERIES[batch_keys[j]], cfg.TRAIN_POSITIVES_PER_QUERY, cfg.TRAIN_NEGATIVES_PER_QUERY,
                                    TRAINING_QUERIES, hard_negs, other_neg=True))
                # q_tuples.append(get_rotated_tuple(TRAINING_QUERIES[batch_keys[j]],POSITIVES_PER_QUERY,NEGATIVES_PER_QUERY, TRAINING_QUERIES, hard_negs, other_neg=True))
                # q_tuples.append(get_jittered_tuple(TRAINING_QUERIES[batch_keys[j]],POSITIVES_PER_QUERY,NEGATIVES_PER_QUERY, TRAINING_QUERIES, hard_negs, other_neg=True))
            else:
                query = get_feature_representation(
                    TRAINING_QUERIES[batch_keys[j]]['query'], model)
                random.shuffle(TRAINING_QUERIES[batch_keys[j]]['negatives'])
                negatives = TRAINING_QUERIES[batch_keys[j]
                                             ]['negatives'][0:sampled_neg]

                hard_negs = get_random_hard_negatives(
                    query, negatives, num_to_take)
                hard_negs = list(set().union(
                    HARD_NEGATIVES[batch_keys[j]], hard_negs))
                print('hard', hard_negs)
                q_tuples.append(
                    get_query_tuple(TRAINING_QUERIES[batch_keys[j]], cfg.TRAIN_POSITIVES_PER_QUERY, cfg.TRAIN_NEGATIVES_PER_QUERY,
                                    TRAINING_QUERIES, hard_negs, other_neg=True))
                # q_tuples.append(get_rotated_tuple(TRAINING_QUERIES[batch_keys[j]],POSITIVES_PER_QUERY,NEGATIVES_PER_QUERY, TRAINING_QUERIES, hard_negs, other_neg=True))
                # q_tuples.append(get_jittered_tuple(TRAINING_QUERIES[batch_keys[j]],POSITIVES_PER_QUERY,NEGATIVES_PER_QUERY, TRAINING_QUERIES, hard_negs, other_neg=True))

            # if (q_tuples[j][3].shape[0] != cfg.NUM_POINTS):
            #     no_other_neg = True
            #     break

        if(faulty_tuple):
            log_string('----' + str(i) + '-----')
            log_string('----' + 'FAULTY TUPLE' + '-----')
            continue

        if(no_other_neg):
            log_string('----' + str(i) + '-----')
            log_string('----' + 'NO OTHER NEG' + '-----')
            continue

        queries = []
        positives = []
        negatives = []
        other_neg = []
        heading = []
        for k in range(len(q_tuples)):
            queries.append(q_tuples[k][0])
            positives.append(q_tuples[k][1])
            negatives.append(q_tuples[k][2])
            other_neg.append(q_tuples[k][3])
            heading.append(q_tuples[k][4])

        queries = np.array(queries, dtype=np.float32)
        queries = np.expand_dims(queries, axis=1)
        other_neg = np.array(other_neg, dtype=np.float32)
        other_neg = np.expand_dims(other_neg, axis=1)
        positives = np.array(positives, dtype=np.float32)
        negatives = np.array(negatives, dtype=np.float32)
        heading = np.array(heading, dtype=np.float32)

        log_string('----' + str(i) + ' / ' + str(len(train_file_idxs)//cfg.BATCH_NUM_QUERIES) +'-----')

        model.train()
        optimizer.zero_grad()
        heading = heading.squeeze()
        randomYaw = (np.random.rand() - 0.5) * 360.

        for dim in range(queries.shape[2]):
            queries[0,0,dim,...] = rotation_on_SCI(queries[0,0,dim,...], randomYaw)
        
        for b in range(positives.shape[1]):
            for dims in range(positives.shape[2]):
                positives[0,b,dims,...] = rotation_on_SCI(positives[0,b,dims,...], (heading[0]-heading[1+b])/np.pi*180)

        
        # -------------------- Run Model ------------------------------
        import time
        time_start = time.time()
        output_queries, output_positives, output_negatives, output_other_neg, outfft, fft_result, unet_out = run_model(model, queries, positives, negatives, other_neg)
        time_end = time.time()
        print("forward time: ", time_end - time_start)
        loss, yaw_loss, yaw_loss_l1, corr = loss_function(output_queries, output_positives, output_negatives, output_other_neg, corr2soft, fft_result, heading, randomYaw, cfg.MARGIN_1, cfg.MARGIN_2, use_min=cfg.TRIPLET_USE_BEST_POSITIVES, lazy=cfg.LOSS_LAZY, ignore_zero_loss=cfg.LOSS_IGNORE_ZERO_BATCH)  # time_start = time.time()
        
        loss.backward()
        optimizer.step()
        optimizer_c2s.step()

        log_string('batch loss: %f' % (loss - yaw_loss))
        log_string('batch yaw loss: %f' % yaw_loss_l1)
        train_writer.add_scalar("Loss", (loss - yaw_loss).cpu().item(), TOTAL_ITERATIONS)
        train_writer.add_scalar("Yaw_Loss", yaw_loss.cpu().item(), TOTAL_ITERATIONS)
        train_writer.add_scalar("Yaw_Loss_L1", yaw_loss_l1.cpu().item(), TOTAL_ITERATIONS)

        if yaw_loss_l1 > 20.:
            train_writer.add_image("query_unet", unet_out[0,0,...].unsqueeze(0).detach().cpu(), TOTAL_ITERATIONS)
            train_writer.add_image("positive_unet", unet_out[1,0,...].unsqueeze(0).detach().cpu(), TOTAL_ITERATIONS)
        else:
            train_writer.add_image("query_unet_good", unet_out[0,0,...].unsqueeze(0).detach().cpu(), TOTAL_ITERATIONS)
            train_writer.add_image("positive_unet_good", unet_out[1,0,...].unsqueeze(0).detach().cpu(), TOTAL_ITERATIONS)
        train_writer.add_image("query", outfft[0,0,...].unsqueeze(0).detach().cpu(), TOTAL_ITERATIONS)
        train_writer.add_image("positive", outfft[1,0,...].unsqueeze(0).detach().cpu(), TOTAL_ITERATIONS)
        train_writer.add_image("corr", corr[0,...].unsqueeze(0).detach().cpu(), TOTAL_ITERATIONS)

        TOTAL_ITERATIONS += cfg.BATCH_NUM_QUERIES

        # -------------------- Evaluating ---------------------------
        if (epoch >= 6 and i % (1400 // cfg.BATCH_NUM_QUERIES) == 29):
            TRAINING_LATENT_VECTORS = get_latent_vectors(
                model, TRAINING_QUERIES)
            print("Updated cached feature vectors")

        if (i % (6000 // cfg.BATCH_NUM_QUERIES) == 101):
            if isinstance(model, nn.DataParallel):
                model_to_save = model.module
            else:
                model_to_save = model
            save_name = cfg.LOG_DIR + cfg.MODEL_FILENAME
            torch.save({
                'epoch': epoch,
                'iter': TOTAL_ITERATIONS,
                'state_dict': model_to_save.state_dict(),
                'optimizer': optimizer.state_dict(),
                'corr2soft': corr2soft.state_dict(),
            },
                save_name)
            print("Model Saved As " + save_name)


def get_feature_representation(filename, model):
    model.eval()
    queries = load_pc_files([filename])
    queries = np.array(queries, dtype=np.float32)
        
    # query_batch = queries.shape[0]
    # size = queries.shape[1]
    # queries_point = np.zeros([query_batch, cfg.num_height * cfg.num_ring * cfg.num_sector])
    # for batch_queries in range(query_batch):
    #     queries_batch = queries[batch_queries,:,:].transpose()
    #     queries_batch = queries.flatten()
    #     transer = gpuadder.GPUTransformer(queries_batch, size, cfg.max_length, cfg.num_ring, cfg.num_sector, cfg.num_height, cfg.overlap_num)
    #     transer.transform()
    #     point_t = transer.retreive()
    #     point_t = point_t.reshape(-1, 3)
    #     point_t = point_t[...,2]
    #     #point_t = (point_t + 1.0)/ 2.0
    #     queries_point[batch_queries,...] = point_t.reshape(1, cfg.num_height * cfg.num_ring * cfg.num_sector)
        

    # if(BATCH_NUM_QUERIES-1>0):
    #    fake_queries=np.zeros((BATCH_NUM_QUERIES-1,1,NUM_POINTS,3))
    #    q=np.vstack((queries,fake_queries))
    # else:
    #    q=queries
    with torch.no_grad():
        feed_tensor = torch.from_numpy(queries).float()
        # feed_tensor = feed_tensor.unsqueeze(1)
        #feed_tensor = feed_tensor[...,2]
        feed_tensor = feed_tensor.to(device)
        feed_tensor = feed_tensor.view((-1, cfg.num_height, cfg.num_ring, cfg.num_sector))
        output, outfft, fft_result, unet_out = model(feed_tensor)
    output = output.detach().cpu().numpy()
    #output = output.squeeze(1)
    output = output.reshape(output.shape[1],-1)
    model.train()
    return output


def get_random_hard_negatives(query_vec, random_negs, num_to_take):
    global TRAINING_LATENT_VECTORS

    latent_vecs = []
    for j in range(len(random_negs)):
        latent_vecs.append(TRAINING_LATENT_VECTORS[random_negs[j]])

    latent_vecs = np.array(latent_vecs)
    query_vec = np.squeeze(np.array([query_vec]))
    query_vec = query_vec.reshape(1,-1)
    #print("query_vec",query_vec.shape)
    #print("latent_vecs",latent_vecs.shape)
    nbrs = KDTree(latent_vecs)
    distances, indices = nbrs.query(query_vec, k=num_to_take)
    hard_negs = np.squeeze(np.array(random_negs)[indices[0]])
    hard_negs = hard_negs.tolist()
    return hard_negs


def get_latent_vectors(model, dict_to_process):
    train_file_idxs = np.arange(0, len(dict_to_process.keys()))

    batch_num = cfg.BATCH_NUM_QUERIES * \
        (1 + cfg.TRAIN_POSITIVES_PER_QUERY + cfg.TRAIN_NEGATIVES_PER_QUERY + 1)
    q_output = []

    model.eval()

    for q_index in range(len(train_file_idxs)//batch_num):
        file_indices = train_file_idxs[q_index *
                                       batch_num:(q_index+1)*(batch_num)]
        file_names = []
        for index in file_indices:
            file_names.append(dict_to_process[index]["query"])
        queries = load_pc_files(file_names)
        queries = np.array(queries, dtype=np.float32)
        
        feed_tensor = torch.from_numpy(queries).float()
        feed_tensor = feed_tensor.to(device)
        feed_tensor = feed_tensor.view((-1, cfg.num_height, cfg.num_ring, cfg.num_sector))
        with torch.no_grad():
            out, outfft, fft_result, unet_out = model(feed_tensor)

        out = out.detach().cpu().numpy()

        q_output.append(out)

    q_output = np.array(q_output)
    if(len(q_output) != 0):
        q_output = q_output.reshape(-1, cfg.FEATURE_OUTPUT_DIM)

    # handle edge case
    for q_index in range((len(train_file_idxs) // batch_num * batch_num), len(dict_to_process.keys())):
        index = train_file_idxs[q_index]
        queries = load_pc_files([dict_to_process[index]["query"]])
        queries = np.array(queries, dtype=np.float32)
        
        with torch.no_grad():
            feed_tensor = torch.from_numpy(queries).float()
            #feed_tensor = feed_tensor[...,2]
            feed_tensor = feed_tensor.to(device)

            feed_tensor = feed_tensor.view((-1, cfg.num_height, cfg.num_ring, cfg.num_sector))
            o1, outfft, fft_result, unet_out = model(feed_tensor)

        output = o1.detach().cpu().numpy()
        output = output.reshape(output.shape[0],-1)
        if (q_output.shape[0] != 0):
            q_output = np.vstack((q_output, output))
        else:
            q_output = output

    model.train()
    print("latent",q_output.shape)
    return q_output


def run_model(model, queries, positives, negatives, other_neg, require_grad=True): 
    # ------------------------- INPUT_TYPE IMAGE--------------------------------
    if cfg.INPUT_TYPE == 'image':

        queries_tensor = torch.from_numpy(queries).float()
        positives_tensor = torch.from_numpy(positives).float()
        negatives_tensor = torch.from_numpy(negatives).float()
        other_neg_tensor = torch.from_numpy(other_neg).float()

        feed_tensor = torch.cat(
            (queries_tensor, positives_tensor, negatives_tensor, other_neg_tensor), 1)
        feed_tensor_vis = feed_tensor.clone()
        feed_tensor = feed_tensor.view((-1, cfg.num_height, cfg.num_ring, cfg.num_sector))
        feed_tensor.requires_grad_(require_grad)
        feed_tensor = feed_tensor.to(device)

        if require_grad:
            output, outfft, fft_result, unet_out = model(feed_tensor)
        else:
            with torch.no_grad():
                output, outfft, fft_result, unet_out = model(feed_tensor)

        output = output.view(cfg.BATCH_NUM_QUERIES, -1, cfg.FEATURE_OUTPUT_DIM)
        o1, o2, o3, o4 = torch.split(
            output, [1, cfg.TRAIN_POSITIVES_PER_QUERY, cfg.TRAIN_NEGATIVES_PER_QUERY, 1], dim=1)

        return o1, o2, o3, o4, outfft, fft_result, unet_out

def rotation_on_SCI(sc, rotation):
    # rotation to translation [-180:180] -> [-cfg.num_sector//2:cfg.num_sector//2]
    if rotation > 0:
        t = rotation / 180. * (cfg.num_sector // 2)
        t = np.floor(t).astype(int)
        patch = sc[:, (cfg.num_sector-t):cfg.num_sector]
        col, row = cfg.num_sector, cfg.num_ring
        center = (col // 2, row // 2)
        t_x, t_y = t, 0.

        M = cv2.getRotationMatrix2D(center, 0.0, 1.0)
        sc = cv2.warpAffine(sc, M, (col, row))

        N = np.float32([[1,0,t_x],[0,1,t_y]])
        sc = cv2.warpAffine(sc, N, (col, row))
        sc[:, 0:t] = patch
    else:
        t = -rotation / 180. * (cfg.num_sector // 2)
        t = np.floor(t).astype(int)
        patch = sc[:, 0:t]
        col, row = cfg.num_sector, cfg.num_ring
        center = (col // 2, row // 2)
        t_x, t_y = -t, 0.

        M = cv2.getRotationMatrix2D(center, 0.0, 1.0)
        sc = cv2.warpAffine(sc, M, (col, row))

        N = np.float32([[1,0,t_x],[0,1,t_y]])
        sc = cv2.warpAffine(sc, N, (col, row))
        sc[:, (cfg.num_sector-t):cfg.num_sector] = patch
        # plt.imshow(sc)
        # plt.show()
    return sc

def imshow(tensor, title=None):
    unloader = transforms.ToPILImage()
    image = tensor.cpu().clone()  # we clone the tensor to not do changes on it
    image = image.squeeze(0)  # remove the fake batch dimension
    image = unloader(image)
    plt.imshow(image, cmap='jet')
    plt.show()

def rotation_matrix(axis, theta):

    axis = axis/np.sqrt(np.dot(axis, axis))
    a = np.cos(theta/2.)
    b, c, d = -axis*np.sin(theta/2.)

    return np.array([[a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c)],
                  [2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b)],
                  [2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c]])


if __name__ == "__main__":
    train()
