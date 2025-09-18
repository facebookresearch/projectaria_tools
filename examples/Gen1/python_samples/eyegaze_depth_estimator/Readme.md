## Eye Gaze Depth Estimator

### Description:

This is a code sample of using a custom patch-matching algorithm inspired by
SFM / stereo reconstruction techniques to perform eye
gaze depth estimation given the yaw and pitch angles of the eye gaze. The
algorithm is experimental and not extensively evaluated but can be used
(at your own risk) as an alternative to the vergence-model obtained using MPS.
Suggestions and Pull requests are welcome for any improvement.

### How to use:

#### Create your python environment

```
rm -rf $HOME/projectaria_tools_python_env
python3 -m venv $HOME/projectaria_tools_python_env
source $HOME/projectaria_tools_python_env/bin/activate
```

#### Go to this folder sample (cd tools/samples/python/eyegaze_depth_estimator)

```
python eye_gaze_Depth_estimator.py --vrs_file ../../../../data/mps_sample/sample.vrs --eyegaze_data_file ../../../../data/mps_sample/eye_gaze/generalized_eye_gaze.csv  --output_csv_filepath gaze_with_depth.csv
```

#### Visualize output

```
viewer_mps --vrs ../../../../data/mps_sample/sample.vrs --mps_folder ../../../../data/mps_sample/ --eyegaze gaze_with_depth.csv --points ../../../../data/mps_sample/trajectory/global_points.csv.gz --trajectory ../../../../data/mps_sample/trajectory/closed_loop_trajectory.csv
```
