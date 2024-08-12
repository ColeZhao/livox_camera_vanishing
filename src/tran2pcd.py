import numpy as np
import pandas as pd

point_data = np.load("/home/collar/data/sample/pcds/000000.npy")

np_to_csv = pd.DataFrame(data=point_data)
np_to_csv.to_csv("/home/collar/data/sample/pcds/000000.csv")