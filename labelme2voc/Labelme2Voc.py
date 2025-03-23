import argparse
import glob
import os
import os.path as osp
import sys

import imgviz
import numpy as np
import shutil
import labelme

def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("input_dir", help="input annotated directory")
    parser.add_argument("output_dir", help="output dataset directory")
    parser.add_argument("--labels", help="labels file", required=True)
    parser.add_argument(
        "--noviz", help="no visualization", action="store_true"
    )
    args = parser.parse_args()

    if osp.exists(args.output_dir):
        print("Output directory already exists:", args.output_dir)
        sys.exit(1)
    os.makedirs(args.output_dir)
    os.makedirs(osp.join(args.output_dir, "JPEGImages"))
    os.makedirs(osp.join(args.output_dir, "SegmentationClass"))
    os.makedirs(osp.join(args.output_dir, "SegmentationClassPNG"))
    if not args.noviz:
        os.makedirs(
            osp.join(args.output_dir, "SegmentationClassVisualization")
        )
    print("Creating dataset:", args.output_dir)

    class_names = []
    class_name_to_id = {}
    for i, line in enumerate(open(args.labels).readlines()):
        class_id = i - 1  # starts with -1
        class_name = line.strip()
        class_name_to_id[class_name] = class_id
        if class_id == -1:
            assert class_name == "__ignore__"
            continue
        elif class_id == 0:
            assert class_name == "_background_"
        class_names.append(class_name)
    class_names = tuple(class_names)
    print("class_names:", class_names)
    out_class_names_file = osp.join(args.output_dir, "class_names.txt")
    with open(out_class_names_file, "w") as f:
        f.writelines("\n".join(class_names))
    print("Saved class_names:", out_class_names_file)

    for img_file in glob.glob(osp.join(args.input_dir, "*.jpg")):
        base = osp.splitext(osp.basename(img_file))[0]
        json_file = osp.join(args.input_dir, base + ".json")
        out_img_file = osp.join(args.output_dir, "JPEGImages", base + ".jpg")

        # Ensure the image is always copied to JPEGImages
        shutil.copy(img_file, out_img_file)

        if osp.exists(json_file):
            label_file = labelme.LabelFile(filename=json_file)
            img = labelme.utils.img_data_to_arr(label_file.imageData)
            lbl, _ = labelme.utils.shapes_to_label(
                img_shape=img.shape,
                shapes=label_file.shapes,
                label_name_to_value=class_name_to_id,
            )
            out_png_file = osp.join(
                args.output_dir, "SegmentationClassPNG", base + ".png"
            )
            labelme.utils.lblsave(out_png_file, lbl)
            np.save(osp.join(args.output_dir, "SegmentationClass", base + ".npy"), lbl)
            if not args.noviz:
                viz = imgviz.label2rgb(
                    lbl,
                    imgviz.rgb2gray(img),
                    font_size=15,
                    label_names=class_names,
                    loc="rb",
                )
                imgviz.io.imsave(
                    osp.join(args.output_dir, "SegmentationClassVisualization", base + ".jpg"),
                    viz
                )
        else:
            # Handle images without a corresponding JSON file
            img = imgviz.io.imread(img_file)
            lbl = np.zeros(img.shape[:2], dtype=np.int32)  # Assume background class
            np.save(osp.join(args.output_dir, "SegmentationClass", base + ".npy"), lbl)
            labelme.utils.lblsave(osp.join(args.output_dir, "SegmentationClassPNG", base + ".png"), lbl)
            if not args.noviz:
                viz = imgviz.label2rgb(lbl, imgviz.rgb2gray(img), font_size=15, label_names=["_background_"], loc="rb")
                imgviz.io.imsave(osp.join(args.output_dir, "SegmentationClassVisualization", base + ".jpg"), viz)

if __name__ == "__main__":
    main()
