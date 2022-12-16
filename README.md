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

### Transport
* Execution.py:
   * Main driver cntaining demo sequence
* sawyer_node.py:
   * Sawyer control node that wraps around motion planner & action server
   * Accepts simple directives based on input objects
* cv_serv.py:
   * Glue logic that interacts with sensing package
* path_planner.py:
   * slightly tuned path planner library from previous lab
