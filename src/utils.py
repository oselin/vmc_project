import numpy as np

def label_data(data: np.array):
    text = ""

    text += f"STD:  {np.std(data)}\n"
    text += f"MEAN: {np.mean(data)}\n"
    text += f"MIN:  {np.amin(data)}\n"
    
    return text


def strong_avoid(occ_map,r):
    strong_occ_map = np.zeros(occ_map.shape)
    for i in range(r,occ_map.shape[0]-r):
        for j in range(r,occ_map.shape[1]-r):
            if occ_map[i,j] == 1:
                for a in range(-r,r):
                    for b in range(-r,r):
                        strong_occ_map[i+a,j+b] = 1  
    return strong_occ_map