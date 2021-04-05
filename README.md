# Visualthree60

## INTRODUCTION
This repository has some codes for estimating human dimensions from single image. This repository uses the human part detection module from the repo[Self Correction for Human Parsing](https://github.com/PeikeLi/Self-Correction-Human-Parsing)



## Requirements

1. install CUDA Toolkit from [nvidia](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html)
2. install python3.6.9 or higher version
```
sudo apt install python3-pip
pip3 install --upgrade pip
pip3 install -r requirements.txt
```


## Steps

Download the weights ([exp-schp-201908270938-pascal-person-part.pth](https://drive.google.com/file/d/1E5YwNKW2VOEayK9mWCS3Kpsxf-3z04ZE/view?usp=sharing)) and store in folder `weights`.
Run the script `streamlit run streamlit_visual_try_on.py`
