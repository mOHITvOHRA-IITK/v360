# Visualthree60


## INTRODUCTION
This repository has some codes for estimating human dimensions from multiple images. This repository uses the human part detection module from the repo [Self Correction for Human Parsing](https://github.com/PeikeLi/Self-Correction-Human-Parsing)



## Environment setting for server and client using Miniconda
1. Install [Miniconda](https://docs.conda.io/en/latest/miniconda.html)
2. Clone this repository
3. `cd /path/to/v360`



**FOR SERVER**

```
conda update -n base -c defaults conda
conda env create -f environment.yaml
conda activate v360
```

**STEPS**
* Download the weights ([exp-schp-201908270938-pascal-person-part.pth](https://drive.google.com/file/d/1E5YwNKW2VOEayK9mWCS3Kpsxf-3z04ZE/view?usp=sharing)) and store in folder `./weights`.
* To Run the server, type in the terminal `python socket_server.py` 



**FOR CLIENT**

```
conda update -n base -c defaults conda
conda env create -f environment_client.yaml
conda activate v360_client
```

**STEPS**
* To run the client, type in the terminal `streamlit run client.py`



## Note
1. Environment setting takes significant amount of time (20-40 minutes).
2. Check Connection to server using `ping 172.26.174.143` in the terminal, or can change the server IP in the script `client_class.py`









