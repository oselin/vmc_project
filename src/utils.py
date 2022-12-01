import numpy as np

def label_data(data: np.array):
    text = ""

    text += f"STD:  {np.std(data)}\n"
    text += f"MEAN: {np.mean(data)}\n"
    text += f"MIN:  {np.amin(data)}\n"
    
    return text


def strong_avoid(occ_map,r):
    strong_occ_map = np.zeros(occ_map.shape)
    for i in range(r-1,occ_map.shape[0]-r-1):
        for j in range(r-1,occ_map.shape[1]-r):
            if occ_map[i,j] == 1:
                for a in range(-r,r):
                    for b in range(-r,r):
                        #if ((i+a<=occ_map.shape[0]) and (j+b<=occ_map.shape[1]) and (i+a>=0) and (j+b>=0)):
                        strong_occ_map[i+a,j+b] = 0.7  
    return strong_occ_map