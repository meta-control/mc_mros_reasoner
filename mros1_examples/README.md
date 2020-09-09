### Execution
Activate your virtual environment, source your ws and launch the reasoner, passing as parameter the path of the ontology file to initialize the KB:
```
source [path/to]/venv3.6_ros/bin/activate
source mros1_reasoner_ws/devel/setup.bash
roslaunch mros1_reasoner run.launch onto:=[path to your workspace]/src/mros1_reasoner/mros1_examples/owl/example_f1_fd1.owl
```