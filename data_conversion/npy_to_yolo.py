from numpy import load
import os
import shutil

width = 1280
height = 720
bbox_2d_tight_dir = r'D:\kasia_mgr\output\Viewport\bbox_2d_tight'
rgb_dir = r'D:\kasia_mgr\output\Viewport\rgb'
yolo_data_dir = r'D:\kasia_mgr\yolo_data'
counter = 0
data_dir_name = 'train'

for root, dirs, files in os.walk(bbox_2d_tight_dir):
    for file in files:
        if file.endswith(".png"):
            shutil.copyfile(os.path.join(rgb_dir, file), os.path.join(yolo_data_dir, 'images', data_dir_name, file))
            counter += 1
        if file.endswith(".npy"):
            data_box = load(os.path.join(bbox_2d_tight_dir, file), allow_pickle=True) 
            file_name, _ = os.path.splitext(file)
            new_file = open(os.path.join(yolo_data_dir, 'labels', data_dir_name, file_name + '.txt'), "w")

            for data_lines in data_box:
                category = data_lines[5]-1
                object_width = data_lines[8] - data_lines[6]
                object_height = data_lines[9] - data_lines[7]
                X = (data_lines[6] + object_width/2)/width
                Y = (data_lines[7] + object_height/2)/height
                object_width_from_X = object_width/width
                object_width_from_Y = object_height/height
                new_file.write("%s %.6f %.6f %.6f %.6f\n" % (category, X, Y, object_width_from_X, object_width_from_Y))

            new_file.close()
        if counter >= 1000:
            data_dir_name = 'val'
