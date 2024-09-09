# Overview

rosapi is part of the rosbridge suite and provides a simple service to get meta info from the system. For example, the following will provide list of topics currently running:

```
ros2 service call /rosapi/topics rosapi_msgs/srv/Topics
```

or services:

```
ros2 service call /rosapi/services rosapi_msgs/srv/Services
```