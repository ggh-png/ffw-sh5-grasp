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

---

## Phase 3 — arm_hand 씬 + 6DOF IK (2026-07-04, 완료)

### 구성

**`models/arm_hand.xml`**: 공식 `arm_r_link1-7` 체인 + Phase 1/2에서 검증한 손 서브트리를
그대로 결합. 어깨(`arm_base_link`)는 world에 완전 고정(`pos="0 0 1.0"`, 계획서 지시대로
~1.0m 높이). 테이블(top z=0.65) + 캔.

**`src/ik.py`**: `mj_jacSite` 기반 DLS. 계획서 지시대로 2단계 개발:
- `solve_position`: position-only 3DOF, 별도 scratch `MjData`에서만 반복(`mj_forward`만
  사용, 물리 스텝 없음) — **kinematic override 금지 규칙과의 경계를 명확히 함**: 이
  솔버는 절대 라이브 시뮬레이션의 `data`를 건드리지 않고 관절각 배열만 반환한다.
  호출자가 그 결과를 `data.ctrl[...]`에 넣는 책임을 진다.
- `solve_pose`: 6DOF hierarchical DLS(위치 DLS + 방향을 위치 자코비안 null-space에
  투영) + backtracking line search + multistart(랜덤 재시작).

### 버그 1: `mju_subQuat`는 로컬 프레임, `mj_jacSite`의 `jacr`은 월드 프레임

Hierarchical IK를 구현했는데도 방향 보정이 엉뚱한 관절에 나눠지는 문제 발생. 관절
개별 섭동 테스트로 확인: 실제로 방향에 영향 주는 관절과 계산된 gradient가 큰 관절이
서로 달랐다. 원인: `mju_subQuat`가 반환하는 회전 오차는 site의 **로컬 프레임**인데
`jacr`은 **월드 프레임**. **수정**: `ori_err_world = site_xmat.reshape(3,3) @ ori_err`.

### 버그 2: hierarchical만으로는 불충분 — 큰 오차에서 진동

방향 오차가 크면(spread 0.15rad 이상) 반복 횟수를 늘릴수록 결과가 더 나빠짐(진동의
징후). **해결**: backtracking line search 추가(step이 실제로 `pos_err+0.3*ori_err`를
줄일 때만 채택, 아니면 절반씩 최대 6회 재시도) + `solve_pose_multistart`(현재 시작점
+ random restart 8회).

### IK 단위 테스트: "reachable workspace" 재정의

전체 joint range 균등 샘플링(46% 수렴)에서 **HOME_Q 주변 ±0.2rad**(실제 텔레옵에서
쓰일 법한 영역)로 재정의, multistart와 함께 **100/100 (100%) 수렴**(목표 95%).

### 통합 pick 테스트 디버깅 — 진짜 원인을 찾기까지

**1차 시도 (거짓 리드): "팔의 잔차 위치 오차" 가설.** IK는 기구학적으로 정확히
수렴하는데(`solve_pose` 결과 pos_err<0.01mm), 실제로 그 관절각으로 서보 후 정착하면
site가 목표에서 15~20mm 벗어나 있었다. 사용자가 제시한 3단계 진단(텔레포트 테스트 →
actuator 포화/관절별 tracking error → ctrl 클램핑 검사)을 순서대로 실행:

- **텔레포트 테스트**(q_grasp를 별도 `MjData`에 직접 넣고 `mj_forward` 한 번만 실행,
  동역학 없음): 오차 0.004mm → 프레임/기준점 불일치 기각.
- **actuator 포화 검사**: 모든 관절이 forcerange 여유 충분(최대 11.5/31.7 N·m 사용) →
  포화 기각. 단, `qfrc_actuator ≈ kp × tracking_error`가 거의 정확히 성립하는 진짜
  비례 오차는 확인됨.
- **ctrl 클램핑 검사**: 보낸 값과 실제 ctrl 완전히 일치 → 기각.
- **60초 장기 정착 테스트**(추가로 실행): 18mm는 고정점이 아니라 **매우 느리게
  수렴하는 과도상태**였다(joint4 속도가 0.06→0.0008→...→0.000008로 계속 감소).
  `dampratio=1`가 관절 하나씩 독립이라고 가정하고 임계감쇠를 계산하는데, 실제로는
  7개 관절이 관성으로 결합된 시스템이라 그 가정이 안 맞아서 생기는 **결합계의
  저감쇠 모드**로 진단.

**처방 1 — position actuator를 motor(순수 토크) actuator로 교체 + 소프트웨어 PD +
중력/원심력 feedforward** (`src/arm_control.py`, unitree_mujoco 레퍼런스 패턴):

```
tau = qfrc_bias[joint]      (feedforward: 현재 상태의 중력/코리올리/원심력을 정확히 상쇄)
    + kp * (q_des - q)      (위치 피드백)
    - kd * qvel              (속도 피드백/능동 감쇠)
```

매 물리 스텝마다 재계산해서 `data.ctrl`(모터 토크)에 씀 — `data.qpos`는 건드리지 않음.
손은 기존 force-limited `<position>` actuator 유지(Phase 1/2에서 검증된 순응 파지
동작은 그대로 둠, 이건 팔의 강체 위치 결정 문제와는 별개).

**결과**: pregrasp 이동은 0.15mm까지 수렴(대성공). 그런데 **grasp 위치로 이동하면
여전히 18.5mm 오차** — 거의 그대로! 이상해서 접촉을 다시 추적했더니: `finger_r_link9`
(중지 MCP 관절, **잠긴 손가락이 아니라 실제로 쓰는 손가락**)이 테이블에 살짝
박혀 있었다(캔 바닥=테이블 상판=0.65로 정확히 같은 높이라 캔을 감싸려면 중지 knuckle이
테이블 높이까지 내려가야 함). **motor+PD+feedforward 자체는 처음부터 잘 작동하고
있었다** — 18mm는 제어기 문제가 아니라 **테이블과의 실제 기구학적 충돌**이었다.

**처방 1.5 — 캔 대신 목표를 3cm 올림**: 캔의 스폰 높이를 올려서 테이블 위에 띄우려
했으나 실패 — 캔은 진짜 `freejoint`라서 항상 `테이블 상판 + 캔 반높이`로 자유낙하해
버린다(캔을 "띄워두는" 것 자체가 kinematic override 없이는 불가능). 대신 **IK가
겨냥하는 목표 지점을 캔 중심보다 3cm 위**로 잡음 — 테이블과의 충돌은 사라졌지만
(site 오차 0.53mm), **새로운 문제**: 캔 중심이 아니라 3cm 위를 감싸쥐게 되면서
손가락-캔 상대 위치가 Phase 1/2에서 검증된 배치와 달라져 캔을 못 쥠(옆으로 계속
밀려남 — index/middle만 스치듯 닿고 엄지는 아예 안 닿음).

**진짜 원인 발견 — `grasp_target` site 정의 자체가 버그였다.** "3cm 올려도 캔을 못
쥐는" 게 이상해서, 현재 손가락-캔 상대 위치를 Phase 1/2(`hand_only.xml`)의 검증된
값과 직접 비교했다:

| | index tip rel to can | middle tip rel to can | thumb tip rel to can |
|---|---|---|---|
| hand_only.xml (검증됨) | (0.074, -0.013, 0.026) | (0.082, -0.013, -0.001) | (-0.058, 0.023, 0.041) |
| arm_hand.xml (버그) | (0.020, -0.054, -0.030) | (0.027, -0.053, -0.057) | (-0.113, -0.017, -0.014) |

세 손가락 **전부 똑같은 패턴**(약 -0.055, -0.041, -0.026)으로 어긋나 있었다 — 관절
설정 차이라면 손가락마다 다르게 어긋나야 하는데 다 똑같이 어긋난다는 건 **손바닥
자체가 통째로 잘못된 지점을 겨냥하고 있다는 신호**(site 오차 자체는 0.5mm로 극히
작았는데도). 원인을 역산: `grasp_target` site를 `pos="0.105 0.065 0.16"`으로 정의할
때, **이 숫자는 `hand_only.xml`에서 캔의 월드 좌표였는데, 그걸 palm 기준 로컬
오프셋인 것처럼 그대로 복붙**했다. 실제로는 팔목(`hx5_r_base`)이 world
`(0, 0, 0.15)`에 있었으므로, 로컬 오프셋을 구하려면 `(캔 월드좌표 − 팔목 월드좌표)`를
팔목의 회전행렬 역변환(`R^T`)으로 로컬 프레임에 투영해야 했다:

```python
diff_world = can_pos - palm_pos        # (0.105, 0.065, 0.01)
local_offset = palm_R.T @ diff_world   # (0.065, 0.01, 0.105)  <- 진짜 정답
```

`(0.105, 0.065, 0.16)`(틀림) vs `(0.065, 0.01, 0.105)`(맞음) — 완전히 다른 벡터였다.
**수정 후 재검증**: 손가락-캔 상대 위치가 `hand_only.xml` 기준값과 소수점 셋째
자리까지 일치, 테이블 충돌도 자연히 사라짐(`GRASP_TARGET_OFFSET`을 다시 0으로
되돌려도 문제없음 — 원래 기하가 맞으면 테이블도 안 건드린다).

### 최종 결과

- IK 단위 테스트: **100/100 (100%)**, pos_err median 0.006mm, ori_err median 0.055°
- 통합 pick 테스트: **10/10 (100%)**, net lift 9.54~9.57cm (목표 8cm 이상, 7/10 이상)
- Phase 0/1/2 회귀 테스트 전부 재확인 PASS
- kinematic override 검사(`grep "qpos\[.*\]\s*="`): 전부 reset/초기배치/scratch-IK-solver
  (라이브 시뮬레이션과 무관)/dev 시각화 툴에서만 발생, 런타임 루프에는 없음

### 교훈

1. **"강성을 올려도 안 바뀐다"는 신호를 진지하게 받아들여라.** 순수 비례오차라면
   kp에 반비례해야 하는데 5배를 올려도 거의 그대로였던 것은, 진짜 원인(접촉)이
   kp와 무관했기 때문이다. 증상과 안 맞는 가설은 데이터가 더 안 맞을 때까지
   밀어붙이지 말고 버려야 한다.
2. **동일한 오프셋이 여러 지점에 똑같이 나타나면 "각각의 설정"이 아니라 "공통 기준점"을
   의심해라.** 손가락 3개가 전부 같은 방향/크기로 어긋난 게 결정적 단서였다.
3. **worldbody에 직접 놓고 검증한 좌표를, 나중에 다른 body에 상대적인 local offset으로
   재사용할 때는 반드시 좌표계 변환을 명시적으로 계산할 것.** 숫자가 우연히 "그럴듯해
   보여도" 검증 없이 재사용하면 안 된다.

---

## Phase 4 — full_scene + 텔레옵 GUI + 데모 (2026-07-05, 완료)

### `models/full_scene.xml` 구성

공식 `assets/robotis_ffw/ffw_sh5.xml`(전체 로봇: 베이스+바퀴+리프트+헤드+양팔+양손)에서
출발해 텍스트 변환 스크립트로 다음을 적용(수작업 전사 대신 스크립트로 한 이유: 20개
손가락 관절 x 2손 분량의 캡슐 좌표를 손으로 옮기다 실수하는 게 이 프로젝트에서 제일
비쌌던 실수 종류라 — Phase 1/2/3 전부 좌표 부호/기준점 버그였다):

- **베이스 고정**: `base_link`의 `<freejoint name="floating_base"/>` 제거 → world에
  완전 고정(계획서 지시대로).
- **바퀴는 visual만**: 6개 바퀴 관절(steer 3 + drive 3) 및 대응 actuator 전부 제거,
  geom(mesh+충돌원기둥)은 그대로 둬서 시각적으로는 로봇처럼 보이되 움직이지 않음.
- **lift_joint / head_joint1-2**: 공식 `<position>` actuator 그대로 유지(각 kp 10000 /
  200, 이미 실차 스펙 기준으로 튜닝된 값 — 아래 hold 테스트로 중력 지지 확인).
- **양팔**: Phase 3에서 검증한 처방(motor 순수 토크 + `src/arm_control.py`
  feedforward+PD)을 오른팔뿐 아니라 **왼팔에도 동일 적용** — 왼팔도 같은 7-링크 결합계
  구조라 Phase 3와 같은 저감쇠 문제가 재발할 걸로 예상해서 처음부터 방지.
- **양손 캡슐 충돌**: Phase 1에서 실측한 오른손 캡슐 파라미터를 그대로 왼손에 미러링.
  중요한 단순화: 엄지·손바닥의 로컬 Y 오프셋을 제외한 나머지(손가락 mcp/pip/dip/tip
  캡슐 전부)는 로컬 프레임에서 **Y=0**이라 좌우 미러링이 부호 반전 없이 그대로
  재사용 가능함을 확인(공식 모델의 `finger_l_*` body pos들이 전부 `finger_r_*`의
  Y부호만 반전된 거울상이라는 것도 직접 대조로 확인). 엄지·손바닥만 Y 부호 반전.
- **약지/새끼 잠금**: Phase 2의 3점 파지 폴백(`range="0 0"`)을 양손에 동일 적용, 대응
  position actuator 삭제(Phase 2에서 발견한 "range=0 관절엔 inheritrange actuator 못
  붙임" 버그 재발 방지).
- **`grasp_target_r`/`grasp_target_l` site**: Phase 3에서 검증된 로컬 오프셋
  `(0.065, 0.01, 0.105)`을 오른손 그대로, 왼손은 Y 반전 `(0.065, -0.01, 0.105)`.
- **테이블+캔 배치**: 아래 "arm_base 평행이동" 참고.

### arm_base 평행이동으로 Phase 1-3 전부를 그대로 재사용

`models/arm_hand.xml`은 어깨(`arm_base_link`)를 world `(0,0,1.0)`에 무회전으로 고정했다.
공식 전체 모델은 어깨가 `base_link(0,0,0.15) → lift_link(0,0,0, 무관절) →
arm_base_link(0.0055,0,1.4316) + lift_joint(슬라이드, range -0.5~0)` 체인 끝에 있다 —
**체인 전체에 회전이 하나도 없다**(전부 `pos`만, `quat` 없음). 즉 lift_joint를 어떤
고정값(예: 홈 포즈 -0.5)으로 두면 어깨는 world `(0.0055, 0, 1.0816)`에 **무회전으로**
고정된 것과 동일 — `arm_hand.xml`의 `(0,0,1.0)`과는 순수 평행이동(`(0.0055, 0, 0.0816)`)
관계뿐이다. 회전이 없으므로:

- 어깨 기준 오른팔+오른손의 상대 기구학은 **완전히 동일**(각 관절의 부모 상대
  pos/quat이 arm_hand.xml과 토씨 하나 안 다름).
- 따라서 Phase 3의 `HOME_Q`(7개 관절각), 엄지 pre-shape, `grasp_target_r` 로컬 오프셋,
  캡슐 충돌 전부 **그대로 재사용 가능** — 유일하게 할 일은 테이블/캔의 **월드 좌표**를
  같은 평행이동만큼 옮기는 것(`(0.5,0,0.705)` → `(0.5055,0,0.7866)`, 테이블도 동일).
- 이 가설을 코드 수정 전에 먼저 **직접 검증**: `HOME_Q_R`를 새 모델에 그대로 넣고
  FK만 계산했더니 `grasp_target_r`가 캔 기준 정확히 "10cm 뒤 / 20cm 위"(Phase 3
  주석과 동일)에 위치 — 평행이동 가설이 맞았다. 이후 `tests/test_phase_4.py`의 pick
  테스트가 **10/10**으로 통과해 최종 확인.
- 왼팔은 이 프로젝트에서 캔을 잡아본 적이 없어 대응하는 `HOME_Q_L`이 없다 — 대신
  `src/ik.py`의 `solve_position`으로 어깨 반대편의 적당한 대기 자세를 한 번 풀어서
  그 결과를 그대로 채택(교차검증 없이 "그럴듯한 자세"를 그대로 씀 — 왼손 grasp이
  이 프로젝트 범위 밖이라 정밀 검증 대상이 아님, 아래 참고).

### 버그: IK 솔버가 `lift_joint`를 조용히 0으로 되돌려서 233mm 오차

`ik.py`의 `solve_position`/`solve_pose`는 자체 scratch `MjData`에서 매 호출마다
`mj_resetData`로 초기화한 뒤 **자신이 담당하는 관절(팔 7개)만** 써넣는다. `arm_hand.xml`
에서는 이게 안전했다 — 어깨보다 위에 있는 관절이 아예 없었으니까. `full_scene.xml`에서는
`lift_joint`가 어깨보다 위에 있는데, scratch가 매번 리셋되며 `lift_joint`가 기본값
0(=최고 높이, 홈 포즈 -0.5보다 0.5m 위)으로 돌아가 버렸다 — **팔은 정확히 풀리는데
어깨 자체가 엉뚱한 높이에 있는 것**과 동일한 효과. 처음 pick 테스트를 돌렸더니
IK가 200mm대 오차로 전부 실패해서 발견(Phase 3의 "site 오차 0.5mm인데 왜 안 잡히지"
류의 조용한 버그와 결이 비슷 — 이번엔 아예 IK 자체가 안 풀렸다는 점이 다름).

**수정**: `solve_position`/`solve_pose`/`solve_pose_multistart`에 `context_qpos` 파라미터
추가 — 지정하면 scratch를 `mj_resetData` 대신 그 값으로 시드(호출자가 담당하지 않는
모든 관절, 즉 `lift_joint`/헤드/반대쪽 팔/손가락을 라이브 시뮬레이션의 현재 값으로
채움). `arm_hand.xml`처럼 담당 관절 위에 아무것도 없는 모델은 `None`(기존 동작) 그대로
써도 무방 — Phase 3 회귀 테스트가 이를 확인. 라이브 `data`를 직접 건드리는 게 아니라
여전히 scratch 버퍼만 시드하는 것이므로 kinematic override 규칙과는 무관.

### GUI 툴킷 — dearpygui/PyQt/tkinter/imgui 전부 이 환경에서 실패, 브라우저로 우회

`tests/render_snapshot.py` 등에서 이미 쓰던 PIL은 문제없었지만, 계획서가 예시로 든
dearpygui는 **import 시점에 세그폴트**(Python 3.14용 사전 빌드 wheel이 없는 것으로
추정), tkinter/PyQt5/PySide6/wx는 아예 미설치(그리고 이 환경은 `pip install`이
`--break-system-packages` 없이는 안 됨), `imgui-bundle`+GLFW 조합은 import는 되지만
실제 GL 컨텍스트 생성 단계에서 `OpenGL.error.Error: Attempt to retrieve context when no
valid context` 발생. `mujoco.viewer.launch_passive` 자체는 문제없이 동작(Phase 1부터
`render_snapshot.py`로 검증된 경로).

**해결**: 슬라이더 패널을 별도 GUI 툴킷 대신 **루프백 HTTP + 브라우저**로 구현
(`src/teleop_app.py`). 이러면 계획서가 경고한 "GLFW/EGL 컨텍스트 충돌"이 애초에
성립 불가능해진다(브라우저는 완전히 별도 프로세스, 공유 GL 컨텍스트가 없음). 구조는
계획서가 요구한 것과 동일: HTTP 스레드는 타겟 값(EE pose, grasp/thumb, lift, 카메라
프리셋, contact viz 토글)만 쓰고 물리 루프가 읽음; 물리 루프는 읽은 값으로 매 프레임
IK+토크제어+grasp synergy를 갱신하고, HUD/joint monitor 값을 써서 HTTP 스레드가 읽게
함 — 양방향이 아니라 "GUI는 타겟만, 물리는 상태만"의 단방향 구조 그대로.

### 실시간 텔레옵 루프 설계

매 프레임(25Hz 목표, `steps_per_frame≈40`회의 1ms 물리 스텝): 슬라이더 타겟을 스냅샷 →
`solve_pose`(멀티스타트 아님, 이전 프레임의 관절해를 시드로 단일 시도, `max_iter=30`) →
그 결과를 `arm_control`에 매 서브스텝 재적용 → `grasp.apply_grasp` 양손 → `mj_step` →
카메라 프리셋/contact viz 플래그 반영 → `viewer.sync()` → 실측 루프 주파수로 HUD 갱신.
멀티스타트를 안 쓰는 이유: 텔레옵은 타겟이 슬라이더 조작으로 프레임마다 조금씩만
바뀌므로 이전 해가 이미 좋은 시드이고, 8회 랜덤 재시작은 매 프레임 감당하기엔 너무
비쌈(멀티스타트는 Phase 3처럼 큰 점프가 있는 스크립트 pick 시퀀스에서만 필요).
실측 루프 주파수 **~24.6Hz**(레퍼런스 영상의 23~26Hz와 일치, 목표 20Hz 이상 만족).

### `tests/test_phase_4.py` 결과

- Hold 테스트(양팔 `HOME_Q` 유지, lift/head 자세 유지, 5초): `max|qacc|=38.97`
  (한도 1e5), site 드리프트 오른팔 0.295mm / 왼팔 0.296mm(한도 2mm) — Phase 3의
  토크제어 처방이 왼팔에도 그대로 잘 통함을 확인.
- IK 단위 테스트: **100/100 (100%)**, pos_err median 0.006mm — Phase 3와 동일 수준
  (위 "평행이동" 가설이 맞다는 재확인).
- 통합 pick 테스트(오른손, 캔 노이즈 ±5mm, 10회): **10/10 (100%)**, net lift
  9.53~9.59cm — Phase 3의 9.54~9.57cm와 사실상 동일.
- Phase 0/1/2/3 회귀 전부 재확인 PASS.

### 알려진 한계

- **왼손 grasp은 미러링만 되어 있고 독자 검증은 안 됨.** 이 프로젝트엔 캔이 하나뿐이고
  오른손 전용으로 배치돼 있어서, 왼손 엄지 pre-shape/grasp synergy(`src/grasp.py`)는
  "기하학적으로 대칭이니 맞을 것"이라는 미러링 추론이지 Phase 2 수준의 실제 contact
  force 파지 성공률 검증을 받지 않았다. Hold 테스트가 최소한 발산/자기충돌은 없음을
  확인해줄 뿐이다.
- **슬라이더 텔레옵의 "5회 중 3회 성공" 기준은 에이전트가 직접 검증할 수 없다** —
  사람이 브라우저에서 슬라이더를 움직여야 하는 항목이라서. 대신 `tests/test_phase_4.py`
  가 동일한 IK+토크제어+grasp synergy 파이프라인을 스크립트로 10/10 재현해 물리적으로
  가능함을 검증하고, `docs/demo.gif`(`tests/record_demo.py`)로 그 스크립트 시퀀스를
  시각적으로 남겼다. 사람이 직접 슬라이더로 재현하는 것은 이 세션 밖의 몫으로 남는다.
- 데모 GIF는 실시간 텔레옵 GUI 자체의 화면 녹화가 아니라, 동일 파이프라인을 헤드리스로
  재생해 오프스크린 렌더링한 것이다(사람이 옆에서 슬라이더를 조작하는 장면은 아님).

### 교훈

1. **연산 자체가 코드 밖(자체 scratch 버퍼, 별도 MjData)에 있어도 "그 연산이 어떤
   맥락을 가정하는지"는 모델이 바뀌면 같이 검증해야 한다.** IK 솔버는 Phase 3부터
   한 줄도 안 바꿨는데, 그걸 감싸는 모델에 새 상위 관절(lift_joint)이 생기자 조용히
   깨졌다 — 코드가 옳아도 가정이 깨지면 틀린다.
2. **회전 없는 순수 평행이동 관계를 알아보면 검증된 숫자를 대량으로 재사용할 수
   있다.** Phase 3의 버그(캔의 world 좌표를 local 오프셋으로 착각)와 정반대 교훈 —
   이번엔 "두 프레임이 실제로 평행이동 관계"라는 걸 먼저 수식으로 확인하고 나서야
   안전하게 재사용했다(전자는 검증 없이 재사용해서 버그, 후자는 검증하고 재사용해서
   안전).
3. **GUI 툴킷은 계획 단계에서 예시로 든 대로 되리라는 보장이 없다.** dearpygui가 이
   환경에서 세그폴트를 낼 줄은 아무도 몰랐다 — 계획서 자체가 "dearpygui/tkinter 등"
   이라고 유연하게 열어뒀던 것이 실제로 도움이 됨. 아키텍처 요구사항(단방향 데이터
   흐름, GL 컨텍스트 충돌 회피)을 지키면 구체적 툴킷 선택은 환경에 맞춰 바꿀 수 있다.
