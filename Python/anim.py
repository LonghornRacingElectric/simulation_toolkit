import imageio
import os

# Generate animation
file_dir = "./simulations/qss/qss_outputs/ymd_animation"
fps = 30
output_path = "./simulations/qss/qss_outputs/ymd_cv.gif"

for file in os.listdir(file_dir):
    if file.count(".") > 1:
        index = file.index(".")
        new_name = file[:index] + "p" + file[index + 1:]
        os.rename(f"{file_dir}/{file}", f"{file_dir}/{new_name}")

# Collect sorted file list
images = sorted([f for f in os.listdir(file_dir) if f.endswith(".png")])
images += images[::-1]

# Write video
with imageio.get_writer(output_path, mode='I', fps=fps, loop=0) as writer:
    for filename in images:
        image = imageio.imread(os.path.join(file_dir, filename))
        writer.append_data(image)