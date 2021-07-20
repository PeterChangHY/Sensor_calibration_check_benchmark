# Sensor Calibration Benchmark

## Auto_Calibration_simulator(stereo)

1. download fastandstraight data folder
```
$scp -r /mnt/vault1/home/peterchang/sensor_calibration_benchmark_data/fastandstraight_bags_front_radar_and_odom YOUR_LOCATION/
```


3. build `auto_calibration_simulator`:
```
$cd work/dirve
$make release && make install
```

4. change the parent_dir in `data/fastandstraight_data_sampled.yaml` to `YOUR_LOCATION/fastandstraight_bags`

5. optional: modify data/auto_calibration.prototxt

5. run scripts to run `auto_calibration_simulator` on each selected data
Note: In order to handle load the relative filepath and log, I made some changes on auto_calibration_simulator code. You can pick this [commit](https://github.com/PeterChangHY/drive/commit/5c63a80782c4edf78f5f694d241ef8821dc66639), if you want to run it on your side. 
 ```
 ./script/auto_calibration_simulator_runner.py -d data/fastandstraight_data_sampled.yaml -w work/drive/ -o OUT_DIR
 ```

6. outpout

    1. all tsv file will be copied to OUT_DIR with the bag name as prefix
    2. `run_result.txt` will show the each data's status
    3. all calibration will be copied to OUT_DIR

## Data Dumper

dump raw data from the list in yaml 
```
    python dumper.py -d fastandstraight_data_sampled.yaml --output_dir output
```
dump raw data from the bag paths in yaml 
```
    python dumper.py --bags bag_path1 bag_path2... --output_dir output
```


## Run percpetion_unified_simulator

0. prepare you data, download the data from

```
    scp data1:/mnt/vault0/simulator_bags/general/ YOUR_PATH
```

1. prepare you drive 
```
    $make release && make install
```

2. modify `data/fastandstraight_data_sampled.yaml`'s parent_dir to where the data is

3. run the script
```
    ./script/auto_perception_simulator_runner.py -d data/fastandstraight_data_sampled.yaml -w work/drive/ -o OUT_DIR
```

4. then it will run unified_perception_simulator on each bag specified at `fastandstraight_data_sampled.yaml`, then we can analyze the log to compare the result, check `parse_unified_simulation_log.py`
