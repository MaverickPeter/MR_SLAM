# PointNetVlad-Pytorch
Unofficial PyTorch implementation of PointNetVlad (https://github.com/mikacuy/pointnetvlad)

I kept almost everything not related to tensorflow as the original implementation.
The main differences are:
* Multi-GPU support
* Configuration file (config.py)
* Evaluation on the eval dataset after every epochs

This implementation achieved an average top 1% recall on oxford baseline of 84.81%

### Pre-Requisites
* PyTorch 0.4.0
* tensorboardX

### Generate pickle files
```
cd generating_queries/

# For training tuples in our baseline network
python generate_training_tuples_baseline.py

# For training tuples in our refined network
python generate_training_tuples_refine.py

# For network evaluation
python generate_test_sets.py
```

### Train
```
python train_pointnetvlad.py --dataset_folder $DATASET_FOLDER
```

### Evaluate
```
python evaluate.py --dataset_folder $DATASET_FOLDER
```

Take a look at train_pointnetvlad.py and evaluate.py for more parameters
