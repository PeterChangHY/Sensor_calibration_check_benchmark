# Sensor Calibration Benchmark

## Auto_Calibration_simulator(stereo)

1. download fastandstraigh data
```
$scp data1:/home/peterchang/sensor_calibration_benchmark_data/fastandstraight_bags.zip YOUR_LOCATION/
```

2. unzip it
```
$unzip YOUR_LOCATION/fastandstraight_bags.zip YOUR_LOCATION/
```

3. build `auto_calibration_simulator`:
```
$cd work/dirve
$make release && make install
```

4. change the parent_dir in `fastandstraight_data_sampled.yaml` to `YOUR_LOCATION/fastandstraight_bags`

5. optional: modify data/auto_calibration.prototxt

5. run scripts to run `auto_calibration_simulator` on each selected data
Note: In order to handle load the relative filepath and log, I made some changes on auto_calibration_simulator code. You can pick this [commit](https://github.com/PeterChangHY/drive/commit/5c63a80782c4edf78f5f694d241ef8821dc66639), if you want to run it on your side. 
 ```
 ./script/auto_calibration_simulator_runner.py -d data/fastandstraight_data_sampled.yaml -w work/drive/ -o OUT_DIR
 ```

6. outpout

    1. all tsv file will be copied to OUT_DIR with the bag name as prefix
    2. `run_result.txt` will show the each data's status
    3. all calibration
