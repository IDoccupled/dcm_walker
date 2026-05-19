#!/bin/bash

mkdir -p ~/venv/venv_dcm_walker

python3 -m venv ~/venv/venv_dcm_walker --system-site-packages --symlinks

source ~/venv/venv_dcm_walker/bin/activate

python3 -m pip install -r resource/requirements.txt

echo "venv_dcm_walker install done. Do <source ~/venv/venv_dcm_walker/bin/activate>"
