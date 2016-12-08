# gtp_tutorials
Tutorials for using GTP in ROS

## Usage

You need to have a running instance of gtp,
with all the pipeline for the worldstate update and robot control. Have a look at `adream_hri_experiments` package.

Then you can simply rosrun the scripts, after having a look at them and their documentation.

## action.py node

This python script is mainly just an interface to call action servers and services from GTP and Co.

It can be used to plan and execute tasks manually, and also serves as a demonstration of GTP usage.

### Example

Let's plan a pick and place action, and then execute it.
First we need to fetch the world state from `toaster`, we call the `update` sevice of GTP.
This only "triggers" the update, wich will actually be done upon reception of the required data.


```bash
~ ❯❯❯ rosservice call /gtp/update
success: True
message: ''
```

```bash
~ ❯❯❯ rosrun gtp_tutorials action.py pick PR2_ROBOT -o GREY_TAPE
sending goal
step: wait for update
step: planning task
step: no solution
Plan result:
id: 
  taskId: -1
  alternativeId: -1
solutionParts: []
status: no_solution
success: False
```
```bash
~ ❯❯❯ rosrun gtp_tutorials action.py pick PR2_ROBOT -o GREY_TAPE
sending goal
step: planning task
step: post processing solution
Plan result:
id: 
  taskId: 0
  alternativeId: 0
solutionParts: 
  - 
    agent: PR2_ROBOT
    armId: 0
    id: 0
    name: approach
    type: Manipulate
  - 
    agent: PR2_ROBOT
    armId: 0
    id: 1
    name: engage
    type: Manipulate
  - 
    agent: PR2_ROBOT
    armId: 0
    id: 2
    name: grasp
    type: Manipulate
  - 
    agent: PR2_ROBOT
    armId: 0
    id: 3
    name: escape
    type: Manipulate
status: OK
success: True
```
```bash
~ ❯❯❯ rosrun gtp_tutorials action.py place PR2_ROBOT -o GREY_TAPE -p 0,0
sending goal
step: planning task
step: no solution
Plan result:
id: 
  taskId: -1
  alternativeId: -1
solutionParts: []
status: no_solution
success: False
```
```bash
~ ❯❯❯ rosrun gtp_tutorials action.py place PR2_ROBOT -o GREY_TAPE -p 0,0
sending goal
step: planning task
step: post processing solution
Plan result:
id:
  taskId: 1
  alternativeId: 0
solutionParts:
  -
    agent: PR2_ROBOT
    armId: 0
    id: 0
    name: approach
    type: Manipulate
  -
    agent: PR2_ROBOT
    armId: 0
    id: 1
    name: engage
    type: Manipulate
  -
    agent: PR2_ROBOT
    armId: 0
    id: 2
    name: release
    type: Manipulate
  -
    agent: PR2_ROBOT
    armId: 0
    id: 3
    name: escape
    type: Manipulate
status: OK
success: True
```

