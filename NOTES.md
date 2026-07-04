# NOTES

## Phase 0 — 공식 모델 검증

nq=70  nv=69  nu=63  ngeom=131

### Option
- timestep: 0.002
- integrator: implicitfast
- cone: elliptic
- impratio: 10.0
- iterations (solver): 100
- ls_iterations: 50
- gravity: [0.0, 0.0, -9.81]

### Joints (all)
| name | type | range | damping | armature | actuator | kp | forcerange |
|---|---|---|---|---|---|---|---|
| floating_base | free | unlimited | 0 | 0 | - | - | - |
| left_wheel_steer_joint | hinge | [-1.58, 1.58] | 200 | 0.5 | left_wheel_steer | 5000.0 | [-2000.0, 2000.0] |
| left_wheel_drive_joint | hinge | unlimited | 2 | 0 | left_wheel_drive | 1.0 | unlimited |
| right_wheel_steer_joint | hinge | [-1.58, 1.58] | 200 | 0.5 | right_wheel_steer | 5000.0 | [-2000.0, 2000.0] |
| right_wheel_drive_joint | hinge | unlimited | 2 | 0 | right_wheel_drive | 1.0 | unlimited |
| rear_wheel_steer_joint | hinge | [-1.58, 1.58] | 200 | 0.5 | rear_wheel_steer | 5000.0 | [-2000.0, 2000.0] |
| rear_wheel_drive_joint | hinge | unlimited | 2 | 0 | rear_wheel_drive | 1.0 | unlimited |
| lift_joint | slide | [-0.5, 0.0] | 5e+03 | 10 | lift_joint | 10000.0 | [-100000.0, 100000.0] |
| head_joint1 | hinge | [-0.2317, 0.6951] | 5 | 0.1 | head_joint1 | 200.0 | [-10.0, 10.0] |
| head_joint2 | hinge | [-0.35, 0.35] | 5 | 0.1 | head_joint2 | 200.0 | [-10.0, 10.0] |
| arm_l_joint1 | hinge | [-3.14, 3.14] | 30 | 0.5 | arm_l_joint1 | 600.0 | [-61.4, 61.4] |
| arm_l_joint2 | hinge | [0.0, 3.14] | 30 | 0.5 | arm_l_joint2 | 600.0 | [-61.4, 61.4] |
| arm_l_joint3 | hinge | [-3.14, 3.14] | 20 | 0.5 | arm_l_joint3 | 600.0 | [-31.7, 31.7] |
| arm_l_joint4 | hinge | [-2.9361, 1.0786] | 20 | 0.5 | arm_l_joint4 | 600.0 | [-31.7, 31.7] |
| arm_l_joint5 | hinge | [-3.14, 3.14] | 20 | 0.5 | arm_l_joint5 | 600.0 | [-31.7, 31.7] |
| arm_l_joint6 | hinge | [-1.57, 1.57] | 20 | 0.5 | arm_l_joint6 | 600.0 | [-31.7, 31.7] |
| arm_l_joint7 | hinge | [-1.8201, 1.5804] | 3 | 0.1 | arm_l_joint7 | 200.0 | [-5.1, 5.1] |
| finger_l_joint1 | hinge | [-1.5708, 1.5708] | 1 | 0.05 | finger_l_joint1 | 20.0 | [-2.0, 2.0] |
| finger_l_joint2 | hinge | [0.0, 3.1416] | 1 | 0.05 | finger_l_joint2 | 20.0 | [-2.0, 2.0] |
| finger_l_joint3 | hinge | [-1.5708, 0.0] | 1 | 0.05 | finger_l_joint3 | 20.0 | [-2.0, 2.0] |
| finger_l_joint4 | hinge | [-1.5708, 0.0] | 1 | 0.05 | finger_l_joint4 | 20.0 | [-2.0, 2.0] |
| finger_l_joint5 | hinge | [-0.6109, 0.6109] | 1 | 0.05 | finger_l_joint5 | 20.0 | [-2.0, 2.0] |
| finger_l_joint6 | hinge | [0.0, 2.0071] | 1 | 0.05 | finger_l_joint6 | 20.0 | [-2.0, 2.0] |
| finger_l_joint7 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_l_joint7 | 20.0 | [-2.0, 2.0] |
| finger_l_joint8 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_l_joint8 | 20.0 | [-2.0, 2.0] |
| finger_l_joint9 | hinge | [-0.6109, 0.6109] | 1 | 0.05 | finger_l_joint9 | 20.0 | [-2.0, 2.0] |
| finger_l_joint10 | hinge | [0.0, 2.0071] | 1 | 0.05 | finger_l_joint10 | 20.0 | [-2.0, 2.0] |
| finger_l_joint11 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_l_joint11 | 20.0 | [-2.0, 2.0] |
| finger_l_joint12 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_l_joint12 | 20.0 | [-2.0, 2.0] |
| finger_l_joint13 | hinge | [-0.6109, 0.6109] | 1 | 0.05 | finger_l_joint13 | 20.0 | [-2.0, 2.0] |
| finger_l_joint14 | hinge | [0.0, 2.0071] | 1 | 0.05 | finger_l_joint14 | 20.0 | [-2.0, 2.0] |
| finger_l_joint15 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_l_joint15 | 20.0 | [-2.0, 2.0] |
| finger_l_joint16 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_l_joint16 | 20.0 | [-2.0, 2.0] |
| finger_l_joint17 | hinge | [-0.6109, 0.6109] | 1 | 0.05 | finger_l_joint17 | 20.0 | [-2.0, 2.0] |
| finger_l_joint18 | hinge | [0.0, 2.0071] | 1 | 0.05 | finger_l_joint18 | 20.0 | [-2.0, 2.0] |
| finger_l_joint19 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_l_joint19 | 20.0 | [-2.0, 2.0] |
| finger_l_joint20 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_l_joint20 | 20.0 | [-2.0, 2.0] |
| arm_r_joint1 | hinge | [-3.14, 3.14] | 30 | 0.5 | arm_r_joint1 | 600.0 | [-61.4, 61.4] |
| arm_r_joint2 | hinge | [-3.14, 0.0] | 30 | 0.5 | arm_r_joint2 | 600.0 | [-61.4, 61.4] |
| arm_r_joint3 | hinge | [-3.14, 3.14] | 20 | 0.5 | arm_r_joint3 | 600.0 | [-31.7, 31.7] |
| arm_r_joint4 | hinge | [-2.9361, 1.0786] | 20 | 0.5 | arm_r_joint4 | 600.0 | [-31.7, 31.7] |
| arm_r_joint5 | hinge | [-3.14, 3.14] | 20 | 0.5 | arm_r_joint5 | 600.0 | [-31.7, 31.7] |
| arm_r_joint6 | hinge | [-1.57, 1.57] | 20 | 0.5 | arm_r_joint6 | 600.0 | [-31.7, 31.7] |
| arm_r_joint7 | hinge | [-1.5804, 1.8201] | 3 | 0.1 | arm_r_joint7 | 200.0 | [-5.1, 5.1] |
| finger_r_joint1 | hinge | [-1.5708, 1.5708] | 1 | 0.05 | finger_r_joint1 | 20.0 | [-2.0, 2.0] |
| finger_r_joint2 | hinge | [-3.1416, 0.0] | 1 | 0.05 | finger_r_joint2 | 20.0 | [-2.0, 2.0] |
| finger_r_joint3 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_r_joint3 | 20.0 | [-2.0, 2.0] |
| finger_r_joint4 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_r_joint4 | 20.0 | [-2.0, 2.0] |
| finger_r_joint5 | hinge | [-0.6109, 0.6109] | 1 | 0.05 | finger_r_joint5 | 20.0 | [-2.0, 2.0] |
| finger_r_joint6 | hinge | [0.0, 2.0071] | 1 | 0.05 | finger_r_joint6 | 20.0 | [-2.0, 2.0] |
| finger_r_joint7 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_r_joint7 | 20.0 | [-2.0, 2.0] |
| finger_r_joint8 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_r_joint8 | 20.0 | [-2.0, 2.0] |
| finger_r_joint9 | hinge | [-0.6109, 0.6109] | 1 | 0.05 | finger_r_joint9 | 20.0 | [-2.0, 2.0] |
| finger_r_joint10 | hinge | [0.0, 2.0071] | 1 | 0.05 | finger_r_joint10 | 20.0 | [-2.0, 2.0] |
| finger_r_joint11 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_r_joint11 | 20.0 | [-2.0, 2.0] |
| finger_r_joint12 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_r_joint12 | 20.0 | [-2.0, 2.0] |
| finger_r_joint13 | hinge | [-0.6109, 0.6109] | 1 | 0.05 | finger_r_joint13 | 20.0 | [-2.0, 2.0] |
| finger_r_joint14 | hinge | [0.0, 2.0071] | 1 | 0.05 | finger_r_joint14 | 20.0 | [-2.0, 2.0] |
| finger_r_joint15 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_r_joint15 | 20.0 | [-2.0, 2.0] |
| finger_r_joint16 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_r_joint16 | 20.0 | [-2.0, 2.0] |
| finger_r_joint17 | hinge | [-0.6109, 0.6109] | 1 | 0.05 | finger_r_joint17 | 20.0 | [-2.0, 2.0] |
| finger_r_joint18 | hinge | [0.0, 2.0071] | 1 | 0.05 | finger_r_joint18 | 20.0 | [-2.0, 2.0] |
| finger_r_joint19 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_r_joint19 | 20.0 | [-2.0, 2.0] |
| finger_r_joint20 | hinge | [0.0, 1.5708] | 1 | 0.05 | finger_r_joint20 | 20.0 | [-2.0, 2.0] |

### Finger joints detail (finger_l_*, finger_r_*)
| name | actuator present | kp | forcerange |
|---|---|---|---|
| finger_l_joint1 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint2 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint3 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint4 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint5 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint6 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint7 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint8 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint9 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint10 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint11 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint12 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint13 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint14 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint15 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint16 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint17 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint18 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint19 | yes | 20.0 | [-2.0, 2.0] |
| finger_l_joint20 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint1 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint2 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint3 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint4 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint5 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint6 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint7 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint8 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint9 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint10 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint11 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint12 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint13 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint14 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint15 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint16 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint17 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint18 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint19 | yes | 20.0 | [-2.0, 2.0] |
| finger_r_joint20 | yes | 20.0 | [-2.0, 2.0] |

### Geoms
| name | type | contype | conaffinity | friction | condim |
|---|---|---|---|---|---|
| floor | plane | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed1> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed2> | box | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed3> | mesh | 0 | 0 | [5.0, 1.0, 0.3] | 3 |
| <unnamed4> | cylinder | 1 | 1 | [5.0, 1.0, 0.3] | 3 |
| <unnamed5> | mesh | 0 | 0 | [5.0, 1.0, 0.3] | 3 |
| <unnamed6> | cylinder | 1 | 1 | [5.0, 1.0, 0.3] | 3 |
| <unnamed7> | mesh | 0 | 0 | [5.0, 1.0, 0.3] | 3 |
| <unnamed8> | cylinder | 1 | 1 | [5.0, 1.0, 0.3] | 3 |
| <unnamed9> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed10> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed11> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed12> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed13> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed14> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed15> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed16> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed17> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed18> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed19> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed20> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed21> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed22> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed23> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed24> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed25> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed26> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed27> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed28> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed29> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed30> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed31> | box | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed32> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed33> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed34> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed35> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed36> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed37> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed38> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed39> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed40> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed41> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed42> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed43> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed44> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed45> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed46> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed47> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed48> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed49> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed50> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed51> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed52> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed53> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed54> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed55> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed56> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed57> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed58> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed59> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed60> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed61> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed62> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed63> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed64> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed65> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed66> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed67> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed68> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed69> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed70> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed71> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed72> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed73> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed74> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed75> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed76> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed77> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed78> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed79> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed80> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed81> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed82> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed83> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed84> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed85> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed86> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed87> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed88> | box | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed89> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed90> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed91> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed92> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed93> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed94> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed95> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed96> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed97> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed98> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed99> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed100> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed101> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed102> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed103> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed104> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed105> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed106> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed107> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed108> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed109> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed110> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed111> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed112> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed113> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed114> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed115> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed116> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed117> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed118> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed119> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed120> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed121> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed122> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed123> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed124> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed125> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed126> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed127> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed128> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed129> | mesh | 0 | 0 | [1.0, 0.005, 0.0001] | 3 |
| <unnamed130> | mesh | 1 | 1 | [1.0, 0.005, 0.0001] | 3 |

### Collision geom summary
- mesh-type collision geoms (contype/conaffinity != 0): 60
- primitive-type collision geoms: 7
- finger/hand collision geom types found: ['mesh']
### Divergence test (5s, gravity only, zero control)
- steps: 2500
- max |qacc|: 1.719204e+02
- diverged (>= 1e5): False

### Finger collision mesh vs primitive judgement
**결론: finger/hand collision geom은 전부 mesh 기반이다 (primitive 없음).** Phase 1에서 capsule/box primitive collision으로 교체하는 작업이 필요하다.

## Phase 1 — hand_only 씬 + collision 정비 (2026-07-04)

### 모델 구성

`models/hand_only.xml`: 공식 `ffw_sh5.xml`의 `hx5_r_base` 서브트리(오른손, 20 finger joint)를
그대로 복사해 독립 씬으로 재구성. 팔/베이스/왼손/헤드 전부 제외.

- **손 고정 방식**: `hx5_r_base`에 `freejoint`를 부여하고, `hand_mocap`(mocap body)과
  `<equality><weld .../></equality>`로 연결. 물리 엔진이 constraint force로 손을 목표 위치에 붙잡는
  방식 — `qpos` 직접 대입이 아니므로 프로젝트의 kinematic override 금지 규칙을 위반하지 않는다.
- **캔**: `freejoint` cylinder, r=0.033 h=0.11 mass=0.35, `pos="0.13 0.06 0.15"` (hx5_r_base 기준
  world 좌표). 아래 "손 좌표계 매핑" 참고.
- **손 좌표계 매핑** (`quat="0.5 0.5 0.5 0.5"`로 world에 부착): hx5_r_base의 local 좌표축이
  local X(curl 방향) → world Y, local Y(스프레드 방향) → world Z, local Z(손가락 연장 방향) → world X로
  매핑되도록 배치. 즉 4개 손가락이 world Z 방향으로 수직으로 늘어서고, 캔을 world X 방향에서
  수평으로 접근해 감싸쥐는 배치. FK만으로 fingertip 궤적을 스윕(curl 0→1)해 실측한 결과:
  index/middle/ring/pinky 모두 curl≈0.4~0.6 부근에서 거의 같은 (world X, world Y) 지점에 수렴 —
  캔을 그 지점에 두면 4개 손가락이 서로 다른 높이(world Z)에서 같은 캔을 감싸는 형태가 된다.
  엄지는 curl=1.0에서도 그 지점까지 닿지 않음 (아래 "알려진 한계" 참고).

### Mesh AABB 측정 → capsule 변환 (`tests/measure_hand_meshes.py`)

Phase 0에서 finger/hand collision이 전부 mesh였음을 확인했으므로, 오른손 9종 distinct mesh의
local-frame AABB를 측정해 capsule(또는 palm은 box)로 치환했다. 방법: 각 mesh의 최대 extent 축을
capsule의 장축으로 삼고, 나머지 두 축 half-extent의 평균을 반지름으로 사용. fingertip(thumb_tip,
finger_tip)은 실측 반지름에 +1mm 패딩(계획서 지시대로 패드 효과).

| mesh | 장축 | radius | fromto (capsule) |
|---|---|---|---|
| base_unit | - (box) | - | box size 0.02421×0.05536×0.06126 |
| thumb_mcp | X | 0.011375 | -0.000125 0.00225 0 → 0.037125 0.00225 0 |
| thumb_mcp2 | Y | 0.0115 | 0 0.00525 0 → 0 0.02275 0 |
| thumb_ip | Y | 0.0105 | 0 0.001 0 → 0 0.03125 0 |
| thumb_tip | Y (+1mm) | 0.012 | 0 0.0025 0.0015 → 0 0.0415 0.0015 |
| finger_mcp | Z | 0.014825 | 0.00685 0 0.00858 → 0.00685 0 0.01818 |
| finger_pip/dip | Z | 0.0105 | 0 0 0.004255 → 0 0 0.034505 |
| finger_tip | Z (+1mm) | 0.01225 | 0.0015 0 0.006005 → 0.0015 0 0.025755 |

손 geom(`class="collision"`)에는 shadow_hand 레시피대로 `solimp="0.5 0.99 0.0001"`
`solref="0.005 1"` 적용, `option timestep="0.001" integrator="implicitfast" cone="elliptic"
impratio="10"`.

### 버그: priority 때문에 캔의 solref/solimp가 기본값으로 fallback

**증상**: 위 recipe를 그대로 적용했는데도 관통 테스트에서 최대 28mm 관통 (한도 2mm 대비 14배).

**원인**: MuJoCo의 `priority` 규칙 — 두 geom의 priority가 다르면 friction/margin/gap뿐 아니라
**solref/solimp까지 전부 priority가 높은 geom의 값**을 사용한다. 캔에 `priority="1"`을 줬는데
캔 geom 자체에는 solref/solimp를 지정하지 않아서, 손가락 쪽에 공들여 넣은
`solimp="0.5 0.99 0.0001"` `solref="0.005 1"`이 완전히 무시되고 캔이 MuJoCo 엔진 기본값
(훨씬 무른 접촉)으로 계산되고 있었다. 계획서의 "손-캔 접촉에서 캔의 파라미터가 우선 적용되므로
캔 하나만 튜닝하면 된다"는 문장을 friction/condim에만 해당하는 것으로 오독한 것이 원인.

**수정**: `can_geom`에 `solimp="0.5 0.99 0.0001" solref="0.005 1"`을 직접 명시. 결과 28mm → 3.16mm.

### 테스트 방법론 수정 2건

1. **캔 낙하 후 바닥(geom `floor`) 관통을 손-캔 관통으로 오검출**: 캔은 freejoint라 중력으로 떨어지고
   테스트 시간(1.5~3s) 내에 바닥에 닿아 가라앉는다. penetration 계산 시 상대 geom의 body 이름이
   `finger_r_*` 또는 `hx5_r_base`인 경우만 카운트하도록 필터링.
2. **Phase 1 테스트에는 캔 위치 랜덤 노이즈(±5mm)를 넣지 않는다.** 계획서에서 ±5mm 노이즈는
   Phase 2의 grasp+lift 반복 테스트 항목이고, Phase 1은 "닫는 시나리오 20회 반복"만 명시한다.
   실제로 노이즈를 넣어봤더니 극히 일부 시드에서 캔이 이미 살짝 겹친 상태로 스폰되어(step 0에서
   3mm+ 관통) 오탐이 발생했다 — 이는 배치 여유 부족 문제이지 closing 동작의 문제가 아니므로,
   Phase 1은 결정론적 캔 위치로 20회 반복(현재는 사실상 동일 결과 반복)하는 것으로 정리했다.
   Phase 2에서 ±5mm 노이즈를 도입할 때는 캔-손가락 rest-pose 여유를 지금보다 넉넉히 둘 것.

### Finger actuator 튜닝

shadow_hand 스케일(kp 0.5~2, forcerange ±1~2 N·m) 안에서 `kp=0.7` `forcerange=±1.0 N·m`로 설정.
(주: 캔의 solref/solimp 수정 전/후로 kp=1.0/forcerange=1.5 vs kp=0.7/forcerange=1.0 사이 관통량
차이가 거의 없었음 — 이번 케이스의 지배 요인은 액추에이터 힘이 아니라 캔 쪽 solver 파라미터였다.
그래도 계획서 권장 범위 내에서 더 보수적인 쪽인 0.7/1.0으로 유지.)

### HX5-D20 PIP/DIP 커플링 검토

공식 `ffw_sh5.xml`의 `<actuator>` 섹션은 finger_r_joint1~20 **전부에 대해 독립적인 position
actuator**를 제공한다 (마디마다 1:1 actuator, tendon 없음). shadow_hand의 일부 언더액추에이션과
달리, ROBOTIS는 시뮬레이션 제어 목적상 이미 관절별 독립 제어를 전제로 MJCF를 배포한 것으로 보인다.
**결정: Phase 1에서는 fixed tendon을 도입하지 않고 독립 actuator 구조를 그대로 유지한다.**
근거: (1) 공식 모델이 이미 그렇게 구성됨, (2) 관통 테스트가 tendon 없이도 통과함(순응 파지가
안 되는 게 아니라 단순 관통 문제였고 이미 해결됨). Phase 2에서 순응 파지(conform)가 tendon 없이
안 나온다는 게 확인되면 그때 재검토한다.

### 알려진 한계 (Phase 2로 이월)

- 엄지가 4지가 수렴하는 지점(world Y≈0.06~0.08)까지 닿지 않는다 (curl=1.0에서도 Y≈0.03 근처).
  엄지가 캔을 마주보게 하려면 `thumb_r_cmc`(joint1) 또는 `thumb_r_mcp_yaw`(joint2)에 pre-shape
  회전을 줘야 한다 — Phase 2 튜닝 순서 (a) "파지 자세 조정"에서 다룬다.
- 캔은 아직 손에 안정적으로 붙잡히지 않는다(들어올리기 없음) — Phase 1의 목표가 아니라 Phase 2 목표.

### 결과

- `tests/test_phase_1.py`: 20회 반복(결정론적, 1.5s/회), 최대 finger-can 관통 **1.174mm** (한도 2mm) → PASS
  - 3초로 연장해도 동일(캔이 낙하 후 안정화되어 추가 관통 없음) — 재현성 확인
- RTF ≈ 32 (목표 0.5 이상, headless라 렌더링 오버헤드 없음)
- `tests/render_snapshot.py`: 오프스크린 시각 확인용 dev 툴. `--kinematic --grasp=<0~1>`로 FK 전용
  포즈 렌더, `--contacts`로 `mjVIS_CONTACTPOINT`/`mjVIS_CONTACTFORCE` 오버레이 (Phase 1 항목 6 충족).

---

## Phase 2 — 고정 손 grasp (2026-07-04)

### 3점 파지로 축소 (계획서 fallback 적용)

Phase 1에서 엄지가 4지 수렴 지점까지 닿지 않는 문제를 발견했는데, 5지 전부의 엄지 대립을
동시에 만족시키는 pre-shape를 찾는 데 시간이 오래 걸려서 계획서의 명시된 fallback을 적용:
**엄지 + 검지 + 중지 3점 파지로 단순화, 약지/새끼는 `models/hand_only.xml`에서
`range="0 0"`으로 고정** (파이썬 lock 아님, XML에서). 해당 actuator 8개도 제거
(`inheritrange`가 `range="0 0"`인 joint에 대해 컴파일 에러를 내서 — "no range defined" —
애초에 움직일 수 없는 관절의 actuator는 필요 없으므로 제거).

### 엄지 pre-shape: FK 스윕으로 탐색

`thumb_r_cmc`(joint1)와 `thumb_r_mcp_yaw`(joint2)는 grasp 스칼라와 무관한 **고정 pre-shape**
(계획서 표현으로는 grasp/thumb 슬라이더에 안 묶인 자세 파라미터)로 두고, 나머지 관절(mcp_pitch,
ip)만 grasp 스칼라로 움직인다. 그리드 서치(FK-only, `mj_forward`만)로 index/middle 수렴
지점 근처에서 엄지가 캔에 닿는 (j1, j2) 조합을 찾음: **j1=0.131 rad, j2=-1.309 rad.**

### 캔 재배치: `(0.105, 0.065, 0.16)`

index/middle 궤적과 엄지의 실제 도달 범위를 모두 만족하는 지점으로 캔을 다시 배치
(Phase 1의 `(0.13, 0.06, 0.15)`는 5지 전용으로 고른 지점이라 3점 파지엔 안 맞음).

### 버그: `range="0 0"` 인 joint에 `inheritrange="1"` actuator를 못 쓴다

**증상**: 약지/새끼를 잠그자마자 모델 컴파일 에러.
**원인**: MuJoCo가 폭이 0인 range를 "range 없음"으로 취급해서 `inheritrange`가 실패.
**수정**: 그 8개 관절의 `<position .../>` actuator를 통째로 제거 (어차피 못 움직이는
관절이라 actuator가 필요 없음).

### 버그: `data.ctrl[None] = hi` 가 배열 전체를 0으로 밀어버림

**증상**: actuator를 지운 관절까지 여전히 `CURL_JOINTS`에 남아 있어서
`actuator_for_joint()`가 `None`을 반환했는데, `test_phase_1.py`의 `close_hand()`가
`data.ctrl[None] = hi`를 그대로 실행 → **관통 테스트가 항상 0.000mm를 보고**(사실은
아무 것도 안 닫히고 있었음).
**원인**: numpy에서 `arr[None]`은 `arr[np.newaxis]`와 같아서, 전체 배열을 감싸는 새
view를 만든다. 여기에 스칼라를 대입하면 **원본 배열 전체가 그 스칼라로 broadcast되어
덮어써진다** — 즉 없는 인덱스에 대입한다고 에러가 나는 게 아니라 조용히 배열 전체를
망가뜨린다. 루프가 locked joint를 나중에 처리하면 그 시점에 이전 값들이 전부 0으로
리셋됨.
**수정**: `aid is None`이면 그냥 `continue`. **교훈**: numpy 배열에 정수 인덱스가 아닌
값(`None`, 잘못된 lookup 결과)을 대입 인덱스로 쓸 때는 반드시 사전에 유효성 검사할 것 —
조용히 틀린 결과를 낸다.

### 버그: 자체 충돌(self-collision) — capsule 근사가 mesh보다 뚱뚱해서 생긴 새 접촉

공식 모델의 `<contact><exclude .../></contact>`(부모-자식 링크 쌍)를 hand_only.xml로
옮길 때 빠뜨렸었다. 추가로, capsule 근사 때문에 원래 mesh에서는 안 닿던 **인접한 손가락의
mcp 링크끼리**(`finger_r_link5`-`9`, `9`-`13`, `13`-`17`) 새로 겹치는 것도 발견 —
mcp capsule 반지름(1.48cm)의 합이 손가락 간격(2.7cm)보다 커서 항상 겹침. 그리고
**팔레트(palm) collision이 단일 box라서** 실제 mesh보다 뭉툭해 여러 mid-finger 링크와
겹침. → `hx5_r_base`(palm) vs 모든 finger link, 그리고 인접 mcp 링크끼리 exclude 추가.
근본적으로 primitive 근사의 대가이고, 캔 파지 자체와는 무관한 자체-접촉이라 exclude가
타당하다고 판단.

### 근본 문제: 자유낙하하는 캔을 부드러운 액추에이터가 못 잡는다

`kp=0.7, forcerange=±1.0 N·m`(Phase 1에서 정한 값)로 관절을 움직여보면, 목표까지
**1.5초에 절반도 못 감**(고의로 약하게 설정한 force-limited actuator라 당연함 — 이게
바로 계획서가 원하는 "토크 포화로 자연스럽게 감싸쥐기"의 근거). 그런데 hand_only 씬엔
아직 테이블이 없어서(Phase 3에서 추가 예정) 캔이 `freejoint`로 중력 낙하하는데, 손가락이
반응하기도 전에(수십 ms 안에) 캔이 손가락의 유효 반경 밖으로 떨어져 버려서 **애초에
접촉이 일어나지 않았다.**

**시도 1 (실패)**: pre-shape에서 손가락을 캔에 최대한 가깝게(접촉 직전까지) 미리 굽혀두면
낙하 거리가 줄어드니 잡을 수 있을 거라 예상 → 그래도 여전히 100% 실패(캔이 그냥 바닥까지
직행). 액추에이터가 그 짧은 gap조차 제때 못 좁힘.

**해결**: hand_only에 **정적 지지대(`can_support`, box)를 캔 바로 아래 추가** — Phase 3의
테이블을 앞당겨 최소 형태로 넣은 것. 캔이 시작부터 지지된 상태(정지, 낙하 없음)이므로
grasp가 "떨어지는 물체를 붙잡는" 문제가 아니라 원래 계획서가 의도한 "이미 놓인 물체를
쥐는" 문제가 된다. `priority`/자체충돌 버그와 달리 이건 **모델링 스코프의 선택**이라
NOTES에 명시: pinky/ring의 고정 자세, palm box 여유와 안 겹치게 크기(0.05×0.045×0.01)와
위치(0.105, 0.085, 0.095)를 계산해서 배치했다.

### `src/grasp.py`

- `apply_grasp(model, data, grasp, thumb)`: grasp→index+middle의 pip/dip/tip,
  thumb→엄지의 mcp_pitch+ip. 둘 다 **[0,1]을 각 관절 range 전체가 아니라
  `[OPEN_FRAC, 1.0]` 구간에 매핑** — OPEN_FRAC은 rest 상태에서 캔 표면 바로 앞(접촉
  전)에 오도록 실측(`contact.dist`)으로 튜닝한 값 (finger=0.375, thumb=0.22).
  엄지 pre-shape(joint1,2)는 이 스칼라와 무관하게 항상 고정값 유지.
- `get_finger_can_contacts` / `is_grasped`: `mj_contactForce`로 실제 접촉 법선력을
  합산 — 위치 기반이 아니라 **순수 contact force 기반** 판정. `is_grasped`는 엄지
  포함 2개 이상의 손가락 그룹이 접촉 중이고 힘 합이 임계값 이상일 때만 True.

### `models/hand_only.xml`의 "pregrasp" keyframe

grasp=0, thumb=0 상태(위 OPEN_FRAC 반영)의 전체 qpos/ctrl을 저장. 캔 위치도 포함.
테스트 재현성의 기준점 — `test_phase_2.py`는 항상 이 keyframe에서 시작해 캔 위치에만
±5mm 노이즈를 준다.

### 결과 (`tests/test_phase_2.py`)

- ramp(1.0s, 0→1) → settle(1.0s) → mocap 10cm lift @ 2cm/s (5.0s) → 5.0s hold
- ±5mm 노이즈(x,y,z) 10회 반복: **10/10 성공** (목표 8/10)
  - net lift 9.88~9.95cm (목표 ≥8cm)
  - slip 0.48~1.31mm (한도 10mm) — hand 기준 상대 좌표로 측정, lift 자체의 이동은 제외
- Phase 0/1 회귀 테스트 재확인: 둘 다 여전히 PASS (Phase 1은 캔 위치를 Phase 2 배치에
  맞춰 갱신함 — NOTES 상단 Phase 1 절 참고)
- kinematic override 검사: `grep "qpos\[.*\]\s*="` 결과 전부 reset/초기배치/dev 시각화
  툴(`render_snapshot.py --kinematic`)에서만 발생, 런타임 시뮬레이션 루프에는 없음
