import os
import random
import shutil
from pathlib import Path

# 设置数据的路径
data_dir = Path("/home/ywen/labelme2voc/Raw_Picture")  # 更改为你的文件夹路径

# 创建训练集、验证集、测试集文件夹
(train_dir, val_dir, test_dir) = [data_dir / x for x in ["train", "val", "test"]]
for d in (train_dir, val_dir, test_dir):
    d.mkdir(exist_ok=True)

# 获取所有jpg图片
all_files = list(data_dir.glob("*.jpg"))

# 随机打乱文件顺序
random.shuffle(all_files)

# 计算切分点
total_files = len(all_files)
train_end = int(total_files * 0.7)
val_end = train_end + int(total_files * 0.2)

# 分配文件到对应的文件夹
for i, file in enumerate(all_files):
    json_file = file.with_suffix('.json')
    # 根据索引判断文件应该放在哪个文件夹
    if i < train_end:
        dest = train_dir
    elif i < val_end:
        dest = val_dir
    else:
        dest = test_dir
    # 复制图片文件
    shutil.copy(file, dest / file.name)
    # 如果存在对应的JSON文件，也复制
    if json_file.exists():
        shutil.copy(json_file, dest / json_file.name)

print("所有图片文件已按7:2:1的比例随机分配到训练集、验证集和测试集中。")
# import os
# import random
# import shutil
# from pathlib import Path

# # 设置数据的路径
# data_dir = Path("/home/ywen/labelme2voc/Raw_Picture")  # 更改为你的文件夹路径

# # 创建训练集、验证集、测试集文件夹
# (train_dir, val_dir, test_dir) = [data_dir / x for x in ["train", "val", "test"]]
# for d in (train_dir, val_dir, test_dir):
#     d.mkdir(exist_ok=True)

# # 获取所有jpg图片
# all_files = list(data_dir.glob("*.jpg"))
# # 筛选出有对应json文件的图片
# files = [f for f in all_files if f.with_suffix('.json').exists()]

# # 随机打乱文件顺序
# random.shuffle(files)

# # 计算切分点
# total_files = len(files)
# train_end = int(total_files * 0.7)
# val_end = train_end + int(total_files * 0.2)

# # 分配文件到对应的文件夹
# for i, file in enumerate(files):
#     json_file = file.with_suffix('.json')
#     if i < train_end:
#         dest = train_dir
#     elif i < val_end:
#         dest = val_dir
#     else:
#         dest = test_dir
#     # 复制图片和JSON文件
#     shutil.copy(file, dest / file.name)
#     shutil.copy(json_file, dest / json_file.name)

# print("文件已按7:3:1的比例分配到训练集、验证集和测试集中。")