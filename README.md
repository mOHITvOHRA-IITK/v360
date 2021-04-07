# Visualthree60

## INTRODUCTION
This repository has some codes for estimating human dimensions from multiple image. This repository uses the human part detection module from the repo [Self Correction for Human Parsing](https://github.com/PeikeLi/Self-Correction-Human-Parsing)



## Requirements using Pip or Conda

1. Make directory `./v360`
2. `cd /path/to/v360`
3. Clone this repository


**Pip**
1. Install CUDA Toolkit from [nvidia](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html)
2. Install python3.6.9 or higher version

```
sudo apt install python3-pip
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

**Conda**
```
conda env create -f environment.yaml
conda activate v360
```


## Steps

1. Download the weights ([exp-schp-201908270938-pascal-person-part.pth](https://drive.google.com/file/d/1E5YwNKW2VOEayK9mWCS3Kpsxf-3z04ZE/view?usp=sharing)) and store in folder `./weights`.
2. Run the script `streamlit run streamlit_visual_try_on.py`


## Error
1. Streamlite command not found, add `export PATH="$HOME/.local/bin:$PATH"` in bashrc file.
