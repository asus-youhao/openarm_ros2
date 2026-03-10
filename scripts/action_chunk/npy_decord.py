import numpy as np

chunk_data = np.load('action_chunk.npy')
print('Shape:', chunk_data.shape)

if chunk_data.ndim == 2:
    print('Rows:', chunk_data.shape[0])
    print('Each row size:', chunk_data.shape[1])
elif chunk_data.ndim == 1:
    print('1D array, size:', chunk_data.shape[0])
else:
    print('Unsupported shape:', chunk_data.shape)