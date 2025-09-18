# Sample: Running EfficientSAM with Eye Gaze image reprojection as prompt

This sample shows how to run [EfficientSAM](https://github.com/yformer/EfficientSAM/) on Aria RGB images with Eye Gaze as the point prompt.

## Install dependencies

```
#
# Create your python environment
#
rm -rf $HOME/projectaria_tools_python_env
python3 -m venv $HOME/projectaria_tools_python_env
source $HOME/projectaria_tools_python_env/bin/activate

#
# Go to this folder sample (cd tools/samples/python/SAM_eye_gaze_prompt)
#  and install EfficientSAM and its dependencies
#
python -m pip install torch torchvision

# `cd` to the folder of this sample and

git clone https://github.com/yformer/EfficientSAM.git
cd EfficientSAM
git checkout 0a113aab2cadb83945e824fbb21a31d65274be65
python -m pip install .
cp -r ./weights ../
unzip ../weights/efficient_sam_vits.pt.zip -d ../weights
cd ..
rm -rf ./EfficientSAM

#
# Install or use your existing projectaria_tools
#
python -m pip install projectaria_tools
```

## Run the code sample

Running on sample data requires a VRS file and an MPS eye gaze output:
```
python SAM_eye_prompt_demo.py --vrs ../../../../data/mps_sample/sample.vrs --eyegaze ../../../../data/mps_sample/eye_gaze/generalized_eye_gaze.csv
```

> [!NOTE]
> * We run `small` and `tiny` EfficientSAM models side by side by default, but you can run one model by using `--model_list small` or `--model_list tiny`
> * On each image is shown:
>   * the Eye Gaze reprojection is shown on each image
>   * the EfficientSAM mask

![EfficientSAM With Eye Gaze prompt](EfficientSAM_eyegaze_prompt.png)


```bibtex
@article{xiong2023efficientsam,
  title={EfficientSAM: Leveraged Masked Image Pretraining for Efficient Segment Anything},
  author={Yunyang Xiong, Bala Varadarajan, Lemeng Wu, Xiaoyu Xiang, Fanyi Xiao, Chenchen Zhu, Xiaoliang Dai, Dilin Wang, Fei Sun, Forrest Iandola, Raghuraman Krishnamoorthi, Vikas Chandra},
  journal={arXiv:2312.00863},
  year={2023}
}
```
