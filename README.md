# BaggingBot
ee106a final project

## Framework
### Sensing
* camera_server.py: 
    * serves ImageSrv & CamInfoSrv from head camera
* object_server.py
    * serves ObjectSrv from ImageSrv
    * ObjectSrv.objects is a list of Point(x,y,z) indicating the center of detected objects
    * CV is based on HSV threshold and ray-plane intersection
* bag_server.py?
    * should return some handle location to pick up the bag

### Planning
Not designed yet. maybe follow lab7 structure?
