from numpy import load
data_box = load('./Viewport/bbox_2d_tight/0.npy', allow_pickle=True) 
print('BBBox')
print(data_box)

data_semantic = load('./Viewport/semantic/0.npy', allow_pickle=True) 
print('Semantic')
print(data_semantic[0])
print(data_semantic[0].size)
print(data_semantic[1].size)


data_instance = load('./Viewport/instance/0.npy', allow_pickle=True) 
print('Instance')
print(data_instance[0])
print(data_instance[0].size)
print(data_instance[1].size)
