# cv
Summarize CV package of Walkie robot 

Service with payload 
use CV_type msg to send the type of CV service to be used
then use the return infomation 



## To Run 
1. rosrun cv_connector CV_connector.py
2. cv_main.py must be running 
3. roslaunch zed_wrapper zed2i.launch
4. rosservice call /CV_connect/req_cv "cv_type:
  type: 1