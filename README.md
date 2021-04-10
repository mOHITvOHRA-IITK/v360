# Visualthree60

## INTRODUCTION
This repository has some codes for estimating human dimensions from multiple images. This repository uses the human part detection module from the repo [Self Correction for Human Parsing](https://github.com/PeikeLi/Self-Correction-Human-Parsing)



## Environment setting for server and client using Miniconda
1. Install [Miniconda](https://docs.conda.io/en/latest/miniconda.html)
2. Make directory `./v360`
3. `cd /path/to/v360`
4. Clone this repository


**FOR SERVER**

```
conda update -n base -c defaults conda
conda env create -f environment.yaml
conda activate v360
```

**FOR CLIENT**

```
conda update -n base -c defaults conda
conda env create -f environment_client.yaml
conda activate v360_client
```


## Note
1. Make sure to set the prefix in `environment.yaml` or `environment_client.yaml` files.



## Steps
1. Download the weights ([exp-schp-201908270938-pascal-person-part.pth](https://drive.google.com/file/d/1E5YwNKW2VOEayK9mWCS3Kpsxf-3z04ZE/view?usp=sharing)) and store in folder `./weights`.
2. To Run the server, type in the terminal `python socket_server.py` 
3. To run the client, type in the terminal `streamlit run client.py`






