# jetson_Detector_and_Laser
This components compute a virtual laser using Realsense D455 cams and use Jetson-Inference to detect objects like vehicles and pedestrians


## Configuration parameters
Jetson_Inferences repo is needed to use this component

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <jetson_detector's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/jetson_detector config
```
