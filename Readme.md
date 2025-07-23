# Bopa_logger
Module for gather logs and transfer it to dedicated host

___
### Build and run
```
colcon build --packages-select bopa_logger
. install/setup.bash
ros2 launch bopa_logger logger.launch.py
```
---
### Usage example

| Command                                                                            | Result                                                                                            |
|------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------|
| ```ros2 topic pub /log_trigger std_msgs/msg/String "{data: 'date 2025-07-20'}" --once``` | Fetch all logs by date in single archive and transfer it to dedicated server                      |
| ```ros2 topic pub /log_trigger std_msgs/msg/String "{data: 'latest'}" --once```          | Fetch latest logs single archive and transfer it to dedicated server                              |
| ```ros2 topic pub /log_trigger std_msgs/msg/String "{data: 'all'}" --once```             | Fetch all logs(available on system) by date in single archive and transfer it to dedicated server |

___

### File structure
```
bopa_logger/
├─ CMakeLists.txt
├─ package.xml
├─ setup.py
├─ setup.cfg
└─ launch/
    └─ logger.launch.py 
└─ config/
    └─ config.yaml
└─ src/
   ├─ logger_collector/
   │  ├─ __init__.py
   │  └─ LogCollector.py
   └─ nodes/
        └─ run.py
```

___
### Configuration
Configuration could be found in
```
bopa_logger/
└─ config/
    └─ config.yaml
```

Where

| Param       | Description                                           |
|-------------|-------------------------------------------------------|
| remote_host | ipV4, ipV6, or domain name of remote host             |
| remote_user | UserName                                              |
| remote_pass | Password                                              |
| remote_path | Local Path where logs will be stored. Ex: /home/test/ |
| transfer_method | "paramiko". Now only supported transfer using sftp|

---
### Entry point
Simple executable as entry point for node
```
bopa_logger/
└─ src/
   └─ nodes/
        └─ run.py
```
---

### Classes and source code
```
bopa_logger/
└─ src/
   ├─ logger_collector/
      ├─ __init__.py
      └─ LogCollector.py

```