To compile, simply run:
```
./build_release.sh
```

To run the examples, use the following commands for each file (these files correspond to the three experiments we mention below):
```
./build/release/bin/Estimator ./samples/static_bottom_up.csv
./build/release/bin/Estimator ./samples/using_RBQ_quat.csv
```

The output will be saved as:
```
./samples/static_bottom_up.csv.output.csv
./samples/using_RBQ_quat.csv.output.csv
```