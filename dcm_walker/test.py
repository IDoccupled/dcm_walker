import numpy as np

test_array = np.empty((0,5,3))

for step in range(5): # 5 steps
    test_list = []
    for i in range(5): # 5 dcms in each step
        test_list.append([step, step+i+1, step+i+2]) # 3 dimensions for each dcm
    test_array = np.concatenate((test_array, np.array(test_list).reshape(1, -1, 3)), axis=0)
    
print(test_array)