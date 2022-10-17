eval "$(/root/miniconda3/bin/conda shell.bash hook)"
python /code/travis/create_build_script.py
bash /code/travis/do_build.sh