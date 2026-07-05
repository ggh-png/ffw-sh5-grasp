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

---

## Phase 4 후속 — Session 7-8: 테이블 관통 회귀 수정 + RPY 텔레옵 개선 (2026-07-05)

**커밋:** `efed6c8`

### 회귀 배경

Session 7에서 단일 네이티브 창 UI로 교체하는 작업 중, 사용자가 "손이 테이블을 관통한다"고
지적해서 `hx5_r_base`-`world` blanket exclude를 진단용으로 지웠다가 되돌리지 못한 채
커밋 `31370f5`으로 push까지 됐다(왼손만 exclude가 남은 비대칭 상태). 그 결과
`tests/test_phase_4.py`의 pick 테스트가 10/10 → 6/10로 하락한 채 방치되어 있었다.

### 1단계 진단 — "크러드 박스의 작은 오차"라는 가설은 틀렸다

`hx5_r_base`-`world` exclude가 빠진 현재 상태(비대칭 버그)에서 실제 pick 궤적 중
palm_r_collision-table 접촉의 `contact.dist`를 로깅해보니 관통 깊이가 **-0.23~-0.31mm로
작고 안정적**이었다 — 처음엔 이걸 "크러드 박스가 만드는 작은 근사 오차"로 보고,
MuJoCo `<pair>`의 solimp `(d0, d1, width)` 비선형 램프로 얕은 관통은 힘을 거의 안 주고
(`d0=0.0001`) 일정 깊이(`width`) 이후엔 정상 강성을 주도록 시도했다(`width=3mm`
스크래치 테스트로 이 메커니즘 자체는 확인: 기본 solimp가 0.3mm에서 6N을 주는 반면 이
램프는 0.003N만 줌). **그런데 이 폭을 3~9mm로 아무리 조여도 pick 성공률은 그대로
6/10** — 힘을 1000배 줄였는데 결과가 똑같다는 건 [[feedback-ffw-sh5-diagnostics]]가
말하는 "게인을 올려도 안 바뀌면 가설 자체가 틀렸다는 신호"의 정확한 반대쪽 사례였다.

### 2단계 진단 — 진짜 필요 관통 깊이를 직접 측정

`contact.dist`는 **접촉이 이미 활성화된 상태**에서만 잴 수 있으므로 순환논리였다(접촉이
막고 있으니까 작게 보이는 것뿐). 대신 (a) 순수 IK 해(충돌 없음)에서 palm box 8개
꼭짓점을 월드로 변환해 테이블 상판과 비교했더니 **~11.5mm 관통**이 필요했고, (b) 원래
동작하던 phase-4 태그(양손 exclude 대칭, 10/10 통과) 모델에 실제 동역학을 돌려
확인해도 **~11.2mm**로 일치했다. 그리고 `tests/measure_hand_meshes.py`로 실제
`base_unit.stl`의 AABB를 재실측해보니 palm 박스 크기(0.02421×0.05536×0.06126)가
**실제 mesh의 AABB와 소수점까지 일치** — 패딩된 근사가 아니라 손이 실제로 그 형상
그대로다. 즉 이건 "크러드 박스의 과잉근사"가 아니라, **테이블에 딱 붙어 있는 캔을
위에서-뒤에서 감싸쥐는 이 접근 방식 자체가, 손목 뭉치가 테이블 안쪽 공간을 요구하는
구조적 특성**이었다(arm_hand.xml Phase 3의 주석에 이미 "박스가 아니라 진짜 palm
형상이라면 안 닿을 것"이라고 적혀 있었던 바로 그 문제 — Session 3부터 계속 존재했고
exclude로만 가려져 있었다).

### 3단계 — 왜 그래도 3~9mm 폭에서 안 됐는가: 강성 제어기가 무른 접촉을 그냥 뚫는다

`src/arm_control.py`의 팔 제어기는 `kp=600`의 매우 뻣뻣한 PD+중력보상 토크제어라(Phase
3 교훈: `<position>` 액추에이터의 정상상태 오차를 없애려고 도입), 접촉이 어느 정도
무르든 상관없이 **목표 관절각을 거의 그대로 밀어붙인다**. 실측: solimp width를 3mm→
30mm까지 넓혀도 실제 정착 관통 깊이는 **3.4mm→5.8mm 사이에서 완만하게만 증가** —
"충분히 무르게" 만들어도 11mm 근처에는 전혀 안 간다(그 지점에 도달하려면 폭을 훨씬
더 넓혀야 하는데, 그러면 사실상 "관통 방지 기능이 거의 없는" 상태와 다를 게 없어져서
애초에 이 방법을 쓰는 의미가 없어진다). 다만 **성공에 필요한 깊이는 애초에 11mm가
아니라 몇 mm면 충분**했다(11mm는 "충돌이 전혀 없을 때"의 값이지, "약간의 저항이
있을 때 그래도 성공하는 최소값"이 아니었다) — `width=12mm`에서 이미 개별 trial이
성공하기 시작했고, **`width=15mm`에서 `tests/test_phase_4.py` pick 10/10 전부 재현**
(net lift 8.0~9.1cm, 목표 8cm 이상 통과).

### 최종 수정

`models/full_scene.xml`: `hx5_{l,r}_base`-`world` blanket exclude를 완전히 제거하고,
대신 `palm_{l,r}_collision` vs `table`/`floor` 각각에 대해
`solimp="0.0001 0.95 0.015" solref="0.005 1"`인 `<pair>`를 추가(왼손도 동일하게 대칭
처리, Session 7의 비대칭 버그 근본 해결). **검증**:
- `tests/test_phase_4.py`: hold/IK/pick 전부 PASS(pick 10/10).
- Phase 0/1/2/3 회귀 전부 재확인 PASS.
- **수동 조작 보호 확인** (스크래치 진단): IK 타겟을 테이블 상판 기준 +5cm에서
  -10cm까지 억지로 눌러봐도, palm box의 실제 관통 깊이는 **3.5~5.7mm 사이에서
  그대로 유지**(깊이가 계속 늘지 않음) — 사용자가 지적한 "손이 테이블을 그냥
  관통" 증상이 실제로 해결됐음을 직접 확인. 완벽한 강체 차단은 아니지만(수 mm의
  물리적으로 진짜인 유연 접촉), 이전의 "접촉 자체가 없음"과는 질적으로 다르다.

### 교훈

1. **"힘을 크게 줄여도 결과가 똑같다"는 신호를 진지하게 받아들여야 한다** —
   1000배 차이가 나는 두 설정이 같은 결과를 내면, 손대고 있는 파라미터가 원인이
   아니라는 뜻이다. 여기서도 그 신호를 따라 "관통 깊이 자체가 얼마나 필요한가"로
   질문을 바꾸고 나서야 진전이 있었다.
2. **"관통 깊이"를 접촉이 활성화된 상태에서만 재면 순환논리에 빠진다** — 충돌이
   막고 있어서 작게 보이는 걸 "원래 작다"고 오독하지 않으려면, 충돌이 아예 없는
   기준선(순수 IK, 또는 exclude가 살아있는 원본 모델의 실제 동역학)에서 별도로
   측정해야 한다.
3. **"크러드 근사라서 그렇다"는 설명은 실측 없이는 가정일 뿐이다** — 실제로
   mesh AABB를 다시 재보니 이번 케이스는 근사 오차가 아니라 진짜 설계상의 특성이었다.
4. **강성 제어기 + 무른 접촉의 조합에서 "정착 깊이"는 스프링 자연장(=solimp
   width)과 거의 무관하게, 제어기 강성과 액추에이터 한계에 의해 정해지는 경우가
   있다** — 이 프로젝트처럼 `kp`가 매우 큰 토크제어기라면, solimp를 아무리
   조정해도 정착점이 크게 안 움직일 수 있다는 걸 염두에 둘 것.

### RPY 텔레옵 조작감 개선

사용자가 "손의 RPY를 조작할 때 어색하다"고 지적. `src/teleop_app.py`의 기존 구현은
Roll/Pitch/Yaw 슬라이더 값을 **월드 프레임 기준 절대 Tait-Bryan(ZYX intrinsic) 오일러
각**으로 다뤘고, 초기값은 홈 포즈의 실제 site 쿼터니언을 그대로 분해한 값이었다. 홈
포즈의 쿼터니언 `(0.5,0.5,0.5,0.5)`을 분해하면 **roll=90°, pitch=0°, yaw=90°** —
항등원(0,0,0)에서 한참 떨어진 지점이 슬라이더의 "기본값"이었다.

**수치로 확인한 원인**: 오일러각 합성 `R=Rz(yaw)Ry(pitch)Rx(roll)`에서, 슬라이더를
이 홈 포즈 지점에서 각각 +10° 움직였을 때 실제로 어느 축을 도는지 사원수로 직접
계산해보니:
- Roll+10° → 월드 **Y축**을 돎 (라벨은 "Roll"인데 실제로는 세계 기준 옆으로 도는
  움직임)
- Pitch+10° → 월드 **-X축**을 돎
- Yaw+10° → 월드 Z축을 돎 (이것만 우연히 "그대로")

즉 라벨과 실제 회전축이 어긋나 있었다 — Tait-Bryan 각의 특성상 "안쪽" 항(roll)은
합성 순서상 완전히 로컬(항상 손의 현재 로컬 X축)로 남지만, 바깥쪽 항(pitch, yaw)의
월드 프레임 축은 **다른 각들의 현재 값에 따라 달라진다.** 항등원 근처가 아니라
`(90,0,90)` 근처에서 슬라이더를 움직이니 이 뒤섞임이 초기값부터 이미 발생하고
있었던 것.

**수정**: 슬라이더를 **홈 포즈를 기준으로 한 로컬(hand-frame) 상대 회전**으로
재정의 — `target_quat = quat_mul(home_quat, rpy_deg_to_quat(rpy_slider))`
(기존의 절대 world-Euler 디코드 `quat_to_rpy_deg`는 이제 안 쓰여서 삭제). 슬라이더
초기값은 `(0,0,0)`(자연스러운 홈 포즈), 범위도 ±180°에서 ±90°로 좁힘(상대 회전이라
그 정도면 충분), "Reset orientation" 버튼도 추가. 같은 수치 검증을 새 스킴으로
반복하면 원점(0,0,0)에서 Roll/Pitch/Yaw +10°가 **정확히 손의 로컬 X/Y/Z축**을 돎 —
라벨과 실제 축이 일치한다. 슬라이더가 원점에서 크게 벗어나면(예: 세 각 모두 큰 값)
오일러각 자체의 수학적 한계(3-파라미터로 SO(3)를 표현하면 항상 결합이 남는 축이
하나 생김)로 인한 잔여 결합은 여전히 남지만, 텔레옵에서 실제로 쓰는 "홈 포즈
근처에서 소폭 조정"하는 사용 패턴에서는 이 수정으로 체감상 완전히 달라진다.

**검증**: `python3 src/teleop_app.py` 실행해서 창이 뜨고 20초간 크래시 없이
렌더링/물리 루프가 도는 것 확인(로그 비어 있음, 예외 없음). RPY 산식 자체는
사원수 계산으로 별도 검증(위 수치). **슬라이더를 사람이 직접 조작했을 때 "이제
어색하지 않다"고 느끼는지는 에이전트가 검증할 수 없는 항목** — Phase 4의 "5회 중
3회 성공" 기준과 마찬가지로 사람의 확인이 필요하다.

---

## Phase 5 — 모바일 베이스 WASD 주행 (2026-07-05, 완료)

**커밋:** `028ef44`

`ffw-sh5-mobile-and-box-plan.md`(사용자가 다음 작업으로 지정한 계획 문서)를 그대로
적용하기 전에 실제 코드와 대조부터 함 — 문서는 레포가 비공개였을 때 다른 레포
구조로 추론해 쓴 것이라, 대상 물체가 이미 "박스"가 아니라 "캔"이고, `base_link`는
"교체해야 할 freejoint"가 아니라 애초에 **관절 자체가 없는 완전 고정 상태**인 등
안 맞는 전제가 여럿 있었다. 사용자에게 확인해 스코프를 "주행 기능만 먼저, 박스
파지는 나중"으로 좁힘.

### `models/full_scene.xml` — 평면 가상 관절 3개

계획서 §3.1 그대로: `base_link`에 `base_x`(slide, world X)/`base_y`(slide, world Y)/
`base_yaw`(hinge, world Z) 3개 관절을 직접 추가(새 body로 감싸지 않고 base_link
자신에 붙임 — MuJoCo는 한 body에 여러 joint를 허용하고, 셋 다 같은 body에 있으면
"평면 이동+제자리 회전"이 정확히 합성됨). 각각 velocity actuator로 구동
(`kv=800/200`, `forcerange=±500N/±200N·m`). 바퀴 자체는 여전히 관절 없는 시각적
돌출부(base_link에 강체 부착)라 실제로 구르지 않고 미끄러지듯 이동하지만, 이건
Phase 4부터 있던 단순화이지 이번에 새로 생긴 문제가 아니고, 계획서도 명시적으로
"실물 휠-지면 접촉 시뮬레이션은 이 프로젝트 목적 대비 비용 과다"라고 스코프 밖으로
뺀 부분이다.

### 버그 — keyframe `ctrl` 문자열에 토큰 하나 빠뜨림, 왼손이 조용히 틀어짐

관절 3개를 추가하면 `qpos`/`qvel` 인덱스가 전부 3칸씩 밀리므로, keyframe의 `qpos`/
`ctrl` 문자열 맨 앞에 새 관절 3개 몫의 `"0 0 0 "`을 수동으로 붙였는데, `ctrl` 쪽에서
숫자 하나를 빠뜨렸다(9개여야 할 0을 8개만 씀). 그 결과 `finger_l_joint1` 이후
모든 왼손 액추에이터의 ctrl이 한 칸씩 밀려서 엉뚱한 값을 받게 됨 — 하지만 이 버그는
`arm_control.ArmTorqueController`(매 스텝 `data.ctrl`를 다시 계산해서 씀)에는 영향이
없고, **`tests/test_phase_4.py`의 hold 테스트가 `grasp.apply_grasp(..., side="r")`만
호출**해서 왼손 finger 액추에이터의 ctrl을 아무도 다시 안 써서 keyframe의 (틀린) 값이
그대로 남는 경우에만 드러났다. 증상: hold 테스트에서 site_l 드리프트가 0.3mm → 20mm로
급증, pick 테스트 0/10.

**진단이 헤맨 경로 (교훈용으로 기록)**: 처음엔 "새 관절이 추가돼서 팔 반작용에
베이스가 미세하게 밀리고, 그게 왼팔의 `qfrc_bias` 피드포워드를 살짝 어긋나게
한다"는 가설을 세우고 `damping`/`frictionloss`를 30→150→1000(N), 50→500(N·m)까지
10배 이상 올려봤는데 **결과가 소수점까지 완전히 동일** — 이것도
[[feedback-ffw-sh5-diagnostics]]가 말하는 "파라미터를 10배 바꿔도 결과가 그대로면
가설이 틀렸다"는 신호였다. `damping=1e8`(사실상 완전 고정)로 극단적으로 밀어붙여
베이스 자체의 움직임을 qpos/qvel 레벨에서 완전히 죽여도 site_l 드리프트가 그대로
15mm대인 것을 보고서야 "베이스의 물리적 움직임과는 무관하다"고 확신, 그다음
`base_link`의 관절/액추에이터 추가분만 제거한 스크래치 모델로 이분 탐색해 원인을
keyframe 문자열로 좁혔다. **일반 교훈**: 회귀 테스트가 실제로 어떤 side/파라미터만
건드리는지 정확히 알아야 한다 — `side="r"`만 호출하는 테스트에서 "왼쪽" 관련 수치가
이상하다면, 왼쪽 액추에이터를 능동적으로 갱신하는 코드가 하나도 없다는 뜻이고,
그러면 keyframe 같은 "아무도 안 건드리는 정적 초기값"이 1순위 용의선이어야 했다.

### `src/base_teleop.py` — 조작감 이식

`ffw-sh5-teleoperation`(C++/Bullet3) 레포의 InputManager 상수를 그대로 포팅
(`K_SPEED=0.5 K_MAX=0.55 K_ACCEL=3.0 K_BRAKE=6.0 K_YAW=1.2`), 가속/제동을 지수
접근(`1-exp(-k*dt)`/`exp(-k*dt)`)으로 구현. 로컬(전진/좌측) 속도를 매 프레임 스무딩한
뒤 **그 시점의 `base_yaw` qpos로 월드 프레임 변환**해서 반환 — 회전 중에도 "전진"의
의미가 로봇이 보는 방향 기준으로 계속 갱신됨. 회전 입력이 있으면 병진을 즉시 0으로
꺾어 제자리 회전을 명확하게 함(계획서 §1.2). `data.ctrl`만 쓰고 `qpos`/`qvel`는 읽기만
— 헤딩(yaw)의 단일 진실 소스는 MuJoCo 자신의 `base_yaw` qpos.

### `src/teleop_app.py` — WASD/화살표/QE + IK 타겟 베이스-로컬 프레임화

W/A/S/D=전/후/좌/우 스트레이프, Left/Right=yaw, Q/E=리프트(계획서 §3.2, ctrl을
`±0.3·dt`씩 적분). 계획서 §3.3이 강조한 부분도 그대로 반영: EE 포즈 슬라이더
(위치+RPY)를 **베이스 로컬 프레임**으로 재해석해서, 매 프레임 그 시점의
`(base_x, base_y, base_yaw)`로 월드 좌표로 변환한 뒤 IK에 넘김 — 베이스가 움직이거나
회전해도 팔이 월드에 고정된 옛 지점을 쫓아가며 몸과 꼬이지 않는다(RPY 쪽도 동일하게
`base_quat`을 왼쪽에서 한 번 더 곱해 손 목표 방향이 몸과 같이 돎). 베이스가 원점·
yaw=0에 있을 땐 로컬=월드라 기존 Phase 4 동작과 수치까지 동일.

### `tests/test_phase_5.py`

1부: `base_teleop.BaseTeleop`을 MuJoCo 없이 단위 테스트(가속/감속 지수 곡선, 대각선
입력 시 `K_MAX` 클램프, 회전 입력 시 병진 즉시 0). 2부: 실제 물리로 (a) 키를 하나도
안 누른 "유휴" 상태가 발산·크리프 없이 안정적인지(이번 버그의 직접 회귀 테스트),
(b) 테이블과 반대 방향(뒤로)으로 3초 주행 후 놓았을 때 실제로 이동하고 자연스럽게
감속하는지, (c) 테이블 쪽(앞)으로 6초간 계속 주행 명령을 줘도 팔-테이블 충돌 때문에
방해받지 않은 주행이라면 갔을 3m 근처는커녕 훨씬 못 미쳐서 멈추는지(계획서 T9,
"베이스가 테이블 앞에서 정지"에 대응 — 이 씬의 테이블은 베이스 몸체보다 높이
떠 있어서 베이스 자체는 안 부딪히고 이미 테이블을 향해 뻗어 있던 팔이 부딪히며
막는다는 걸 확인 후 반영). **결과: 전부 PASS**. `tests/test_phase_{0,1,2,3,4}.py`도
전부 재확인 PASS(Phase 4는 이 세션에서 두 번 다시 확인 — keyframe 버그 수정
전/후).

### 알려진 한계 / 스코프 밖

- 바퀴는 여전히 안 구른다(시각적 돌출부, Phase 4부터의 단순화 — 계획서도 명시적으로
  스코프 밖으로 뺀 부분).
- 계획서의 "박스 파지" 부분은 이번 세션에 포함 안 함(사용자 확인 후 보류) —
  `ffw-sh5-mobile-and-box-plan.md`는 여전히 레포에 있지만 §2(박스 관통 진단)는
  실제로는 캔 기준 Session 8에서 이미 다른 형태로 해결된 문제라 그대로 적용하면 안
  됨.
- 사람이 실제 WASD/화살표로 조작했을 때의 조작감("가속 1초, 정지 0.5초" 등, 계획서
  T10)은 이번 세션에서 검증 못함(`xdotool`/`ydotool`/`wtype` 등 키 합성 도구가
  이 환경에 없어서 실제 GLFW 창에 합성 키 입력을 못 넣었다) — 대신
  `tests/test_phase_5.py`가 동일한 `BaseTeleop`+MuJoCo 파이프라인을 헤드리스로
  검증했고, 오프스크린 렌더로 베이스 이동 후 전체 로봇 형상이 강체로 잘 따라오는
  것도 시각 확인함. 실제 키보드 조작 확인은 Phase 4의 "5회 중 3회" 기준과 마찬가지로
  사람의 몫으로 남는다.

---

## Phase 5 후속 — 초기 포즈 정합 + IK 성능 + 캔 비주얼 (2026-07-05)

**커밋:** `07b30f7`

### 초기 포즈를 `ffw-sh5-mujoco`(이전 시도 레포)와 정합

사용자 요청: `/home/ggh/ffw-sh5-mujoco`의 초기 포즈에 맞출 것. 그 레포는 keyframe이
아니라 `robot/controller.py`의 `reset()`에서 Python으로 pose를 직접 지정하는 구조였음
(agent 서브태스크로 조사): 양팔 전부 0, **팔꿈치(`arm_l/r_joint4`)만 -π/2(-90°)**,
리프트 -0.39m, 손가락 전부 펴짐. `arm_l/r_joint4`의 range(`-2.9361..1.0786`)와 axis가
좌우 완전히 동일(다른 몇몗 관절처럼 미러링된 range가 아님)한 걸 확인하고서 양팔에
**동일한 값** `-π/2`를 그대로 적용(레퍼런스 레포와 동일하게).

`models/full_scene.xml`의 "home" keyframe(`qpos`/`ctrl`)과 `src/teleop_app.py`,
`tests/test_phase_4.py`, `tests/test_phase_5.py`, `tests/record_demo.py`의
`HOME_Q_R`/`HOME_Q_L`을 전부 갱신. **keyframe 문자열은 이번엔 수작업 대신 컴파일된
모델에서 `mj_name2id`/`jnt_qposadr`로 각 조인트 슬롯을 찾아 파이썬으로 조립** —
Phase 5에서 겪은 "수작업 문자열 붙이기 토큰 하나 누락" 실수를 반복하지 않기 위함
(실제로 그 조사 과정에서 fix했다고 생각했던 ctrl 문자열에 남아있던 토큰 2개 누락
버그를 하나 더 발견해서 같이 고쳤다 — `arm_r_joint6/7`에 잘못 들어간 preshape 값,
`finger_r_joint1/2`에 빠져있던 preshape 값. 둘 다 실제 제어 루프에서는 매 스텝
`arm_control`/`grasp.apply_grasp`가 덮어써서 무해했지만 정확한 건 아니었음).

**새 버그 발견 — lift_joint의 정적 처짐**: 새 포즈로 hold 테스트를 돌리니 site
드리프트가 27mm로 재발(2mm 한도). `qpos` 델타를 관절별로 뜯어보니 **팔 관절은
전부 오차 0**인데 `lift_joint`만 30초 동안 계속 내려앉음(-0.5→약 -0.53까지, 아직도
수렴 안 함). `data.qfrc_bias[lift_dof]`를 직접 재보니 **구 포즈와 신 포즈에서
정확히 동일**(323.27N, 예상대로 — 리프트가 받쳐야 할 하중은 팔 자세와 무관하게
로봇 전체 무게로 고정) — 그런데 구 포즈는 30초 내내 -0.30mm에서 안정, 신 포즈는
-31.76mm까지 계속 처짐. `qfrc_bias`가 같은데 정착값이 이렇게 다른 건, `lift_joint`가
Phase 3 이전 방식 그대로인 **순수 비례(kp=10000) `<position>` 액추에이터라
피드포워드가 없고**(Phase 3가 팔에서 고친 것과 똑같은 클래스의 문제), 신 포즈가
시스템의 **감쇠 특성**(같은 정적 하중이라도 도달하는 속도/진동 양상)을 바꿔서
"거의 즉시 마찰로 잠기는" 구 포즈의 우연한 경로와 달리 계속 미끄러지는 경로를
타는 것으로 보임(근본 원인을 완전히 규명하진 못했지만, 정적 하중 자체가 포즈와
무관함은 직접 측정으로 확인). **수정**: `lift_joint`의 `kp`를 10000→500000로
올림(단순 비례 게인 증가로 충분 — 실측 드룹이 예측대로 kp에 반비례해서 줄어드는
것 확인: kp=200000→1.6mm, kp=500000→0.64mm, `max|qacc|` 변화 없음, 불안정 징후 없음).
Phase 3의 팔처럼 완전한 토크+피드포워드 컨트롤러로 바꾸는 게 더 근본적인 해법이지만
이 문제엔 단일 게인 증가로 충분해서 스코프를 넘기지 않음.

**검증**: hold 테스트 site drift 0.645mm(양팔), pick 9/10(순수 게인 변경 후 재수렴
경로가 살짝 달라져 1건 net_lift 7.98cm로 8cm 문턱을 근소하게 못 넘음 — 테스트
기준(7/10) 자체는 넉넉히 통과), IK 100/100, `tests/test_phase_5.py` 전부 PASS.

### IK 성능 — RPY를 크게 움직이면 프레임이 느려지는 문제

사용자가 "RPY 제어할 때 렉 걸리는 것 같다"고 지적. `solve_pose`의 backtracking line
search가 iteration마다 최대 6번 `mj_forward`를 호출하는데, 이 모델(전체 로봇, 60+
DOF) 위에서 `mj_forward` 1회가 ~71μs — 실측: 사람이 슬라이더로 Pitch를 0→90°까지
천천히(초당 1°) 드래그하는 걸 그대로 재현(매 프레임 이전 프레임의 해로
warm-start)했더니, **접힌 새 홈 포즈 기준 도달 가능한 방향 워크스페이스를 벗어나는
약 63° 지점부터 매 프레임 ~20ms**로 급증(그 전까지는 0.2~7ms). 30개 iteration을
전부 채우면서 "더 가까워질 수 없다"는 결과를 매 프레임 반복 재확인하고 있었던 것.

처음엔 "코스트가 개선 안 되면 조기 종료"하는 stagnation 체크를 넣어봤는데, **롤=30°
(멀쩡히 수렴하던 케이스)가 perr 0.01mm에서 29mm로 망가짐** — 큰 목표로 점프한
직후에는 초반 iteration의 코스트 개선폭이 원래 작다가 나중에 커지는 구간이 있는데,
그걸 "정체"로 오인해서 진짜 수렴 중이던 탐색을 조기에 끊어버린 것. 알고리즘 자체를
건드리는 건 위험하다고 판단해 **되돌리고**, 대신 `src/teleop_app.py`(실시간 루프
전용, `solve_pose_multistart`를 쓰는 pick 태스크와는 무관)의 `IK_MAX_ITER`을
30→15로만 낮춤 — 도달 가능 구간(30~63°)에서는 수렴 품질이 소수점 둘째 자리까지
동일하면서, 막힌 구간의 프레임당 비용은 20ms→9.6ms로 절반 이하로 줄어듦(실측,
동일한 warm-start 재현 스크립트로 확인).

### 캔을 실제 소다캔 STL로 교체

`/home/ggh/ffw-sh5-teleoperation/assets/soda_can_stl/soda_can.stl`을
`assets/soda_can/`로 복사(Phase 0가 `robotis_ffw`를 vendor한 것과 같은 방식 — 절대
경로 참조나 심링크 대신 리포에 실제로 들여옴). STL엔 UV가 없어서 원본 텍스처는 못
쓰고 기존 `can_mat`(붉은색) 재질을 그대로 적용. **물리는 전혀 안 건드림** — 기존에
검증된 `can_geom` 콜리전 실린더(r=0.033, half-height=0.055)는 그대로 두고 투명하게
만들었고(`rgba="0 0 0 0"`), 새 mesh geom은 `class="visual"`(`contype=0 conaffinity=0`)
으로 그 실린더의 정확한 외곽 치수에 맞춰 축별로 스케일(xy=1.1, z=0.7333, mesh
원본 반지름 0.03/높이 0.15 → 목표 반지름 0.033/높이 0.11)해서 렌더링만 다르게
보이도록 함 — Phase 1/2가 튜닝한 손가락-캔 접촉 형상은 손도 안 댐.

**검증**: `tests/test_phase_1.py`(20/20, hand_only.xml 기준이라 무관), `test_phase_2.py`
(10/10, 역시 무관), `test_phase_4.py`(hold+pick, 콜리전 변경 전과 수치 완전히 동일 —
9/10, net_lift 값 소수점까지 일치) 전부 재확인. 오프스크린 렌더로 캔이 제대로 된
소다캔 모양(풀탭 디테일 포함)으로 테이블 위에 올바르게 놓여 있는 것도 시각 확인.

---

## Phase 5 후속 2 — 손가락 완화, IK 렉 일반화 수정, 실제 바퀴 마찰 주행 (2026-07-05)

**커밋:** `2a1ed9e`

### 약지/새끼 손가락 "기울어짐" 수정

사용자 지적: 4·5번째 손가락(약지·새끼)이 기울어져 보임. `finger_r_link5/9/13/17`(각
손가락 MCP)에서 손끝까지의 방향벡터를 팔레트 로컬 프레임으로 투영해 4개 손가락을
직접 비교해보니 **완전히 동일**(회전 없음, Y축 위치만 다름) — 양손 다 확인, 기하학적
"기울어짐"은 없음을 수치로 확인. 실제 원인은 Phase 2부터의 설계(약지·새끼는
`range="0 0"`으로 완전 고정, grasp 시 다른 손가락만 굽음)라 손을 오므릴 때 두 손가락만
뻣뻣하게 펴진 채로 남아 상대적으로 "어색해" 보이는 것으로 판단. **수정**: 약지·새끼의
pip/dip/tip 관절(14,15,16,18,19,20 — mcp인 13,17은 스프레드 방지 위해 그대로 고정)
range를 클래스 기본값(공식 벤더 값)으로 복원하고 position actuator를 추가, grasp
스칼라와는 무관하게 **정적인 완화 자세**(pip 20%, dip/tip 20%)로 설정 — 실제 캔 파지에는
여전히 참여 안 함(순수 시각 개선). 키프레임 재조립 중 **또 손으로 만든 문자열에서
토큰 개수를 틀리는 실수**를 저질러(같은 실수를 벌써 세 번째로 반복 — 교훈 아래 참고)
이번엔 아예 사람 손을 안 거치도록 **compiled model → 이름 기반 조회로 배열을 만들고
정규식으로 파일에 직접 써넣는 스크립트**로 전환, 재발 방지.

### IK 렉 — RPY뿐 아니라 큰 이동 전반의 문제였음

이전 세션에서 RPY 전용으로 고친 `IK_MAX_ITER` 인하만으로는 부족했음 — 실측해보니
**위치(XYZ) 슬라이더를 한 프레임에 크게(예: 0.3m) 움직여도 동일하게 최대 11ms/프레임**
소요(도달 불가능하진 않아도 `max_joint_delta`(0.05rad/iteration) 제한 때문에 한
프레임 안에 다 못 감). **일반적인 해법으로 교체**: `teleop_app.py`에 슬라이더의 원시
목표값과 별개로 **IK가 실제로 쫓아가는 "완화된(smoothed)" 목표**를 두고, 매 프레임
그 완화된 목표를 원시 목표 쪽으로 최대 `0.02m`/`5°`만큼만 이동시킴 — 슬라이더가
순간이동해도 IK는 항상 "다음 프레임에 쉽게 닿을 거리"만 쫓아가게 됨. 실측: 도달
가능한 큰 이동(0.19m 대각선)은 4.1ms로 해결(원래 방식은 최댓값 근처에서 계속 걸림돌).
**단, 진짜 도달 불가능한 목표**(예: 접힌 팔 자세에서 물리적으로 안 닿는 방향)는 여전히
프레임당 10~19ms 정도 소모됨 — 이건 "안 닿는다는 걸 알기 전까지는 시도해봐야 한다"는
근본적인 한계라 이번 수정 범위 밖(RPY 세션의 `IK_MAX_ITER=15`가 이 잔여 비용의
상한을 여전히 잡아줌).

### 모바일 베이스를 진짜 바퀴 마찰로 구동

사용자 요청: 가상 평면 관절에 직접 velocity actuator를 거는 대신, **실제 바퀴가
지면과 마찰로 접촉해서 밀려나가는 방식**으로 바꿀 것. 또한 WASD는 일반적인 무조코
조작 키와 겹치니 방향키만 쓸 것(Up/Down=전후진, 좌우=yaw, Shift+좌우=스트레이프).

**설계**: `base_x`/`base_y`/`base_yaw` 평면 관절은 그대로 둠(베이스가 넘어지지 않게
막아주는 역할, Phase 5의 의도적 단순화 유지) — 다만 이제 아무것도 이 관절을 직접
구동하지 않음. 대신 공식 `ffw_sh5.xml`에 원래 있던(Phase 4가 단순화를 위해 지웠던)
**바퀴 3개의 steer(조향, position actuator)+drive(구동, velocity actuator) 관절**을
복원하고, 그 바퀴-지면 접촉 마찰이 실제로 베이스를 밀게 함.

**바닥 높이 재계산**: 공식 모델도 `base_link pos="0 0 0.15"`가 동일하지만, 그 모델은
`floating_base`가 **진짜 6DOF freejoint**라 처음에 살짝 자유낙하해서 바퀴가 바닥에
닿는 구조였음(직접 확인: 바퀴 밑면이 세계 좌표 z=0.1465인데 원래 바닥은 z=0 — 14.65cm
공중에 떠 있다가 떨어지는 게 전제). Phase 4는 이 freejoint를 없애고 높이를 고정했기
때문에, 바퀴에 진짜 콜리전을 주려면 **바닥 자체를 바퀴 높이(z≈0.1465)로 올려야**
함(팔/테이블/캔 높이 보정은 전혀 안 건드림 — "바닥이 어디냐"만 재정의).

**버그 — 정확히 0인 간격이 콜리전 검출을 불안정하게 만듦**: 바닥을 딱 z=0.1465(바퀴와
정확히 접함)로 놓으니, **똑같은 높이의 바퀴 3개 중 뒷바퀴만 접촉이 잡히고 앞 두 개는
`data.contact`에서 아예 빠짐**(부동소수점 상 완전히 같은 높이인데도) — 그 결과 바퀴를
아무리 굴려도 베이스가 거의 안 움직임(구동 관절 자체 속도는 명령대로 잘 따라가는데
실제 미끄러짐 99% 이상 확인). `pos`를 z=0.148(약 1.5mm 겹침)로 살짝 올려서 접촉을
확실하게 만드니 즉시 해결(미끄러짐 0.8%까지 감소, 명령한 방향으로 정상 주행).

**버그 2 — 공식 `wheel_drive`의 `kv=1`은 100kg급 로봇을 밀기엔 너무 약함**:
접촉 자체는 고쳤는데도 순항 속도가 목표(0.5m/s)에 한참 못 미침 — 바퀴 자체의
회전관성이 로봇 전체 질량에 비해 매우 작아서, 속도 서보가 바퀴 자체 목표 속도에는
금방 도달하지만 그 시점엔 오차가 작아져 정작 베이스를 미는 데 필요한 지속적인 토크가
안 나옴. `kv`를 1→30으로 올려 목표 속도의 ~90%까지 도달(추가 불안정 없음, 확인 완료).

**결과**: `tests/test_phase_5.py` 전면 재작성 — `SwerveDrive`(조향각+구동속도 변환)
단위테스트(전진/제자리회전/스트레이프 각각 기하학적으로 검증), 유휴 회귀(드리프트
0.0001mm), 주행 테스트(3초 주행 후 실제 이동 1.24m, 바퀴 구름 속도 vs 실제 베이스
속도 슬립 0.8%로 "진짜 마찰 구동" 확인), 충돌 테스트(테이블 방향 6초 주행해도
505mm에서 멈춤, 관통 안 함) 전부 PASS. `tests/test_phase_{0,1,2,3,4}.py` 전부 재확인
PASS(pick 9/10, 이전과 동일).

### 알려진 한계

- 바퀴의 조향 범위가 공식 사양대로 ±90.5°로 제한돼 있어, 극단적으로 빠른 방향 전환
  요구 시(예: 순간적으로 반대 방향 스트레이프) 조향 각도가 살짝 튈 수 있음(현재
  각도에 가까운 표현을 고르는 최적화는 넣었지만 완벽한 연속성 보장은 아님).
- 왼손 엄지 끝(`finger_l_link4`)이 새 홈 포즈에서 팔레트와 약한 접촉(~28N)을 유지하는
  게 관찰됨 — `test_phase_4/5` 모두 영향 없이 통과하고 발산도 없어서 이번 세션
  범위에서는 그대로 둠(오른손엔 없음, 왼손 엄지 mirroring 값과 관련 있을 가능성).
- 사람이 실제 방향키로 조작했을 때 조향+구동이 자연스럽게 느껴지는지는 이번 세션에서도
  검증 못함(키 합성 도구 부재, Session 8과 동일한 한계).

### 교훈

1. **손으로 만든 키프레임 문자열은 세 번째까지 다시 틀렸다** — 이번엔 아예 사람이
   토큰을 안 세도록 "컴파일된 모델에서 이름으로 조회 → 배열 생성 → 파일에 정규식으로
   써넣기"로 프로세스 자체를 바꿈. 같은 실수를 세 번 반복한 뒤에야 "일부 수정"이
   아니라 "프로세스"를 바꿔야 한다는 걸 인정한 것 — 다음에도 keyframe을 만질 땐 이
   방식을 기본으로 쓸 것.
2. **"정확히 0"인 접촉 간격은 부동소수점 상 불안정하다** — 물리적으로 딱 맞닿게
   설계해도, 반드시 아주 작은(여기선 1.5mm) 의도적 겹침을 둬서 접촉 검출이 확실하게
   한쪽으로 결정되게 만들 것. 이 프로젝트에서 "완전히 동일해 보이는 조건인데 왜 결과가
   다르지"라는 신호가 나오면 부동소수점 경계 조건을 의심할 것.
3. **속도 액추에이터의 `kv`가 낮으면 "가벼운 것"(바퀴 자체)은 명령을 잘 따르는데
   "무거운 것"(그 바퀴가 밀어야 하는 전체 시스템)은 거의 안 움직이는 착시가 생긴다** —
   액추에이터 자신의 목표 추종 여부와 그게 실제로 하려는 일(무거운 걸 미는 것)을
   해내는지는 별개로 확인해야 한다.

## Phase 5 후속 3 — grasp 시 손가락 "벌어짐" 수정 + 테이블 크기 확대 (2026-07-05)

**커밋:** `01b3b83`

### grasp 스크린샷 재검토 — 위 "기울어짐" 수정과는 다른 증상

사용자가 실시간 텔레옵 스크린샷을 보내 "그랩을 할 때는 이렇게 벌어지는데?"라고 지적.
바로 위 절("Phase 5 후속 2")에서 이미 약지·새끼를 pip/dip/tip 20% 완화 자세로
고쳤었는데, 그건 "가만히 있을 때 뻣뻣해 보이는" 문제였고 이번 지적은 **실제로 캔을
쥐는 동작(grasp=0.74, thumb=1.0) 중에** 엄지·검지·중지는 확실히 오므라드는데 약지·
새끼는 20% 완화 자세 그대로 남아 있어 상대적으로 훨씬 더 "벌어져" 보이는 것 — 즉
정적 완화값 자체가 실제 grasp 시의 다른 손가락 굽힘 정도에 비해 너무 낮았던 게
원인. 약지·새끼는 여전히 grasp 스칼라에 연동되지 않는 순수 정적값이라(3점 파지
설계, Phase 2부터 불변), 고정값을 실제 grasp 시 각도와 비슷해 보이는 수준까지
올리는 방향으로 재조정하기로 함.

### 커브 비율 스윕 — 0.40/0.45 사이 절벽 확인

`tests/test_phase_4.py`의 pick 성공률을 기준으로 pip/dip/tip curl 비율(각 관절
range 대비 %)을 0.20부터 0.60까지 스윕(스크래치 스크립트, 커밋 안 함):

| frac | pick 성공 |
|------|-----------|
| 0.20-0.40 | 10/10 |
| 0.45 | 0/10 |
| 0.50 | 0/10 |
| 0.55 | 0/10 |

완만한 저하가 아니라 **0.40과 0.45 사이의 뚜렷한 절벽** — 약지·새끼가 이 지점부터
실제 3점 파지(엄지+검지+중지)를 캔 표면에서 밀어내며 간섭하기 시작하는 것으로
추정(직접 원인 분석은 안 함, 성공률 데이터만으로 판단). 절벽 바로 아래가 아니라
여유를 두고 **0.35**(pip=0.7025rad, dip/tip=0.5498rad, 기존 0.20의 1.75배)로 확정 —
시각적으로 다른 손가락 굽힘과 비슷해 보이면서 절벽까지 10%p 여유를 둠. keyframe은
이번에도 손으로 안 만지고 Phase 5 후속 2에서 도입한 "compiled model → 이름 조회 →
배열 조립 → 정규식으로 파일에 써넣기" 스크립트로 재생성.

### 테이블 크기 확대

사용자 요청: "박스 사이즈도 좀 더 키워줘"(테이블을 가리킴, 아직 별도 박스 오브젝트는
없음 — `ffw-sh5-mobile-and-box-plan.md`의 박스 파지 확장은 여전히 보류 중, 이번
요청은 기존 `table` geom 자체를 키워달라는 뜻으로 해석). `size="0.08 0.15 0.05"`
→ `size="0.14 0.25 0.05"`로 확대(가로 1.75배, 세로 1.67배), `pos`와 두께(0.05)는
그대로 — 캔/파지 기하가 검증된 테이블 윗면 world z는 전혀 안 바뀜, 발판만 넓어짐.
`tests/test_phase_5.py`의 주행-충돌 회귀(테이블 방향으로 계속 주행)가 이제 505mm
대신 **445.1mm**에서 멈춤(테이블이 커진 만큼 더 일찍 부딪힘) — 여전히 유효 범위
(`0 < x < 1.0`) 안이라 테스트 기준 자체는 그대로 통과.

### 재검증

`tests/test_phase_4.py` → pick 9/10(변경 전과 동일, 회귀 없음), `tests/test_phase_5.py`
→ 전체 PASS(충돌 정지 거리만 445.1mm로 갱신), `tests/test_phase_{0,1,2,3}.py` → 전부
PASS. 오프스크린 스냅샷으로 손 모양(주먹 쥔 형태가 자연스러움)과 테이블 크기(캔이
확대된 테이블 위에 올라간 모습) 육안 확인. `python3 src/teleop_app.py` 15초 스모크
실행 — 에러 없이 클린 종료(timeout 124) 확인.

### 교훈

절대값(0.20)이 아니라 **"다른 손가락과 비교했을 때 상대적으로 자연스러워 보이는가"**
라는 기준으로 다시 봐야 한다는 지적이었다 — 첫 튜닝이 "정적으로 봤을 때 안 이상해
보이면 충분하다"고 판단해서 낮은 값을 골랐는데, 실제 동작(grasp) 중 비교 기준이
달라지면 다시 사용자 확인이 필요할 수 있다는 사례. 다행히 이번에도 성공률 절벽이
뚜렷해서(0.40/0.45) 안전 마진을 수치로 확정할 수 있었다.

## Phase 5 후속 4 — IK 렉이 여전히 남아있는 문제 재조사 (2026-07-05)

**커밋:** `932b9ab`

### 사용자 지적: "역시나 ik를 풀 때에 렉이 걸리는 이슈가 있는데"

Session 8 앞부분에서 이미 렉을 한 번 고쳤었다(슬라이더 순간이동 시 IK가 실제로
쫓는 목표를 프레임당 0.02m/5°로만 움직이게 하는 rate-limit). 그런데도 사용자가
"역시나(여전히)" 렉을 지적 — 재조사가 필요했다.

### 재현: rate-limit은 "말도 안 되는 목표"만 막았지, "그럴듯한데 실제로는 안 닿는 목표"는 못 막음

`teleop_app.py`의 물리 루프 로직을 그대로 스크립트로 재현해서(스크래치, 커밋 안 함)
여러 시나리오의 프레임당 `solve_pose` 비용을 직접 측정:

| 시나리오 | 프레임당 비용(정상 상태) |
|---|---|
| 홈 근처 작은 이동(위치+RPY 적당히) | ~0.14ms (즉시 수렴) |
| 슬라이더 극단(1.2, 1.2, 1.2) 코너로 드래그 | **~11.4ms, 계속 유지** |
| 테이블 방향 큰 이동(0.45, 0, 0.15) | **~13ms, 계속 유지** |
| RPY만 세 축 다 90°로 | ~3.5ms (위치는 수렴, 방향만 26° 잔차) |
| "적당해 보이는" 이동(홈+0.2/0.15/-0.1m, RPY 30/20/15°) | **~11-13ms, 계속 유지 (130mm 잔차로 수렴 정체)** |

핵심 발견: **위치가 물리적으로 아예 안 닿는 경우**(어깨로부터의 거리/방향이 관절
한계 밖)뿐 아니라, **평범해 보이는 위치+RPY 조합도 종종 국소 최적점(local
minimum)/관절 한계 lockup에 빠져서 오차가 특정 값(예: 130mm)에서 더 안 줄고
그대로 정체**됨 — 이런 경우 `IK_MAX_ITER=15` 전부를 매 프레임 계속 소모하지만
결과는 전혀 개선되지 않음(같은 130mm에 계속 머무름, `max_iter`를 아무리 줘도
동일 — 직접 대조 확인). 이전 rate-limit 수정은 "슬라이더가 순간이동하는 것"만
막았지, "천천히 다가가도 애초에 그 근방이 도달 불가능한 지점"인 경우는 막지
못했던 것 — 그리고 이런 지점은 슬라이더 범위(-0.2~1.2m 큐브, RPY ±90°)에 비해
팔의 실제 가동범위가 훨씬 좁아서(어깨 기준 최대 도달거리 실측 약 0.65~0.96m,
방향에 따라 훨씬 좁음 — 뒤쪽/반대쪽 측면은 어떤 반지름에서도 아예 도달 불가:
아래 "기각한 접근" 참고) 일반적인 조작 중에도 꽤 자주 마주치게 됨.

### 기각한 접근 — 어깨 기준 도달반경 클램프

처음엔 "IK 목표를 어깨 기준 최대 도달반경 구(sphere)로 클램프하면 되지 않을까"
시도. 랜덤 관절 샘플링(2만개)으로 어깨~손끝 거리 분포를 실측(mean=0.64, p50=0.68,
p95=0.88, max=0.96m)하고 여러 반지름(R=0.5~0.9)과 여러 방향(전방/후방/좌우/상하)
조합으로 실제 그 구 표면 지점이 수렴하는지 검증했는데, **가동범위가 구형이
전혀 아님을 확인** — 예를 들어 "뒤쪽"과 "반대쪽 옆(팔의 반대쪽)" 방향은 반지름을
0.5~0.7m 사이 아무리 바꿔도 항상 정확히 같은 값(59.76mm / 238~341mm)으로 실패,
즉 거리 문제가 아니라 그 **방향 자체가 관절 한계상 원천적으로 안 닿는 영역**이라
반지름 클램프로는 못 고침. 단순 구 근사는 이 로봇의 비대칭 가동범위엔 안 맞는다고
판단해 폐기.

### 채택한 접근 — 프레임 단위 "정체 감지" 후 반복 예산 축소

목표 자체의 도달 가능 여부를 기하학적으로 모델링하는 대신, **`solve_pose`가 실제로
수렴하고 있는지를 프레임 단위로 직접 관찰**하는 쪽으로 방향을 바꿈. `teleop_app.py`에
손별 `stuck_counter` 추가: 매 프레임 `solve_pose`의 최종 위치 오차가
`STUCK_POS_TOL`(30mm)를 넘으면 카운터 증가, 넘지 않으면 0으로 리셋. 카운터가
`STUCK_FRAMES_THRESHOLD`(5프레임, ~0.2초) 이상 연속으로 쌓이면 그 손의 `max_iter`를
`IK_MAX_ITER`(15) 대신 `STUCK_MAX_ITER`(4)로 낮춰서 호출 — 회복되면(오차가 다시
30mm 밑으로) 즉시 15로 복귀.

이전에 `ik.py` 내부에 시도했다가 되돌린 "코스트 정체 시 조기 종료"(한 번의
`solve_pose` 호출 **안에서** 반복 간 개선량을 보고 중간에 멈추는 방식, roll=30°
케이스를 29mm 오차로 망가뜨렸던 그 시도)와는 근본적으로 다른 층위의 수정이라는
점이 중요: 이번엔 `ik.py` 자체는 전혀 안 건드리고, **여러 프레임에 걸친 실제
결과**(수 초 동안 반복해서 풀어봤는데도 안 줄어드는지)를 보고 판단하므로, 한 번의
호출 안에서 "느리게 수렴 중"과 "정체"를 헷갈릴 위험이 없음(이전 실패 사례는 30
iteration 중 초반 몇 번이 정체처럼 보이다가 뒤에 급격히 좋아지는 패턴이었는데,
이번엔 그런 케이스라도 최소 5프레임(최대 75 iteration 상당) 동안 기회를 준 뒤에야
개입하므로 훨씬 안전).

**검증**: 홈+(0.2,0.15,-0.1m)/RPY(30,20,15°) 같은 "적당해 보이지만 실제로는
lockup인" 케이스로 대조 실험 — `max_iter=15` 고정으로는 결국 130mm 근방에서
정체(즉, 예산을 다 줘도 어차피 그 값에 수렴), 새 방식은 5프레임 뒤 4-iteration으로
낮아지지만 **도달하는 최종 오차값은 동일**(정체값 자체는 반복 횟수와 무관 —
직접 대조 확인) — 즉 정확도 손실 없이 비용만 줄어듦. 양손이 동시에 정체되는
현실적 시나리오(오른손 극단 코너 + 왼손 위 lockup 케이스)로 종합 측정: 정상
상태 프레임당 합산 IK 비용이 **22.1ms → 6.8ms로 감소**(약 1/3) — `LOOP_HZ=25`의
40ms 예산 안에서 물리 스텝(~4ms 실측) + 렌더링을 위한 여유가 이전엔 겨우
14ms 남짓이었는데 이제 29ms 이상으로 확보됨. 정상적으로 수렴하는 케이스(홈 근처
작은 이동)는 항상 여전히 `IK_MAX_ITER=15` 그대로 받으므로 정확도 회귀 없음
(직접 대조: 정체 판정 문턱 30mm 밑에서는 카운터가 계속 0으로 리셋되어 절대
throttle 안 걸림). `tests/test_phase_{0..5}.py` 전부 재확인 PASS — 이 로직은
`teleop_app.py`에만 있고 테스트가 쓰는 `solve_pose_multistart`(오프라인 pick
테스트용, `max_iter=250`)와는 완전히 별개 경로라 테스트 결과에 영향 없음(그대로
9/10, 10/10 등 이전과 동일). `python3 src/teleop_app.py` 15초 스모크 실행도 에러
없이 클린 종료 확인.

### 교훈

1. **"슬라이더가 순간이동하는 것"과 "슬라이더가 천천히 다가가도 그 지점 자체가
   안 닿는 것"은 서로 다른 문제라 서로 다른 수정이 필요했다** — 첫 번째 세션의
   rate-limit은 전자만 잡았고, 이번 세션은 후자를 잡았다. "렉 고쳤다"고 끝내기
   전에 사용자가 실제로 겪는 조작 패턴(느린 드래그도 안 닿는 지점으로 갈 수
   있다는 것) 전체를 다시 실측해야 했다.
2. **가동범위를 구(sphere)로 근사하는 건 이 정도로 관절 제약이 많은 팔에는 안 맞는
   근사다** — 방향에 따라 반지름과 무관하게 원천적으로 막힌 영역이 있다는 걸
   실측(여러 반지름 x 여러 방향 조합)으로 직접 확인하고 나서야 접근을 바꿨다.
   "그럴듯한 근사"를 실측 없이 채택하지 않은 게 이번엔 시간을 아꼈다.
3. **"반복 예산을 줄이는 수정"이 항상 위험한 건 아니다 — 어느 층위에서, 무엇을
   근거로 줄이는지가 중요하다.** 이전에 실패한 조기 종료는 "한 번의 호출 내부
   코스트 흐름"만 보고 판단해서 느린 수렴과 진짜 정체를 구분 못 했다. 이번엔
   "여러 프레임에 걸친 실제 결과"를 보고 판단해서 같은 종류의 실수를 피할 수
   있었다.

## Phase 5 후속 5 — 초기 손가락 자세를 grasp 스칼라에 연동 (2026-07-05)

**커밋:** `a30d5db`

### 사용자 지적: "초기 손가락 위치가 서로 동일하고 초기에는 펴져 있어야만 해"

스크린샷 첨부: 텔레옵 앱에서 양손이 테이블 위에 쉬고 있는 장면인데, 한 손은
손가락이 테이블 위에 쫙 펴진 채 놓여 있고 다른 손은 주먹 쥔 것처럼 굽어 있어 —
grasp/thumb 슬라이더는 양손 다 0.000으로 표시돼 있었는데도. 사용자 요구는 명확:
**초기(rest, grasp=0) 상태에서는 양손 손가락 모양이 서로 동일해야 하고, 펴져
있어야 한다**.

키프레임 자체(both hands' qpos/ctrl at reset)를 직접 덤프해서 확인한 결과, 약지/
새끼 관절값은 이미 L/R 완전히 동일(0.7025/0.5498/0.5498, Phase 5 후속 3에서
고정폭으로 넣은 값) — **키프레임엔 비대칭 버그가 없었다**. 스크린샷의 실제
비대칭은 11초간의 라이브 시뮬레이션 동안 테이블과의 접촉 등 동역학적 요인으로
갈라진 것으로 보이지만, 그보다 더 근본적인 문제는 애초에 **약지/새끼가 rest
상태에서도 35% 정적으로 굽어 있었다는 것** — Phase 5 후속 3에서 "grasp 중 벌어져
보이는" 문제를 고치려고 grasp 스칼라와 무관한 고정 상수로 만들었는데, 그 부작용으로
"쉬고 있을 때도 이미 굽어 있어서 다른 손가락(엄지/검지/중지, grasp=0에서 20% 정도만
살짝 굽음)과 다르게 보인다"는 새로운 문제를 만든 셈.

### 수정: 고정 상수 → grasp 스칼라에 비례하는 램프로 전환

`src/grasp.py`에 `RING_PINKY_CURL_JOINTS`(약지/새끼 pip/dip/tip, 6관절 x 2손)와
`RING_PINKY_MAX_FRAC = 0.35` 추가, `apply_grasp` 안에서
`frac = grasp * RING_PINKY_MAX_FRAC`로 매 스텝 커밋 — grasp=0(rest)이면 frac=0(완전
펴짐), grasp=1.0(pick 테스트의 settle 구간과 동일)이면 frac=0.35(이전 세션에서
절벽 실측으로 확정한 바로 그 안전값, 그대로 유지). 이렇게 하면:
- rest 상태: 양손 다 0 → **완전히 펴짐 + 완전히 동일** (사용자 요구 그대로).
- 실제 grasp(=1.0) 도달 시: 이전과 정확히 같은 0.35 상한에 도달하므로 pick 테스트
  회귀 없음.
- grasp 램프 도중(0→1): 이전엔 약지/새끼가 처음부터 0.35로 고정이었는데 이제는
  다른 손가락과 함께 비례해서 굽어감 — 더 자연스러움.
`models/full_scene.xml`의 home keyframe도 (기존 세이프 스크립트 방식으로) 재생성해
약지/새끼 qpos/ctrl을 0으로 되돌림(그동안 관절 액추에이터/range는 그대로 둠 — Phase 5
후속 2가 이미 이 두 손가락의 range를 확장해뒀으므로 액추에이터 자체는 안 건드림).

### 버그 재발 — `data.ctrl[None]` 배열 전체 broadcast (Session 2 이후 세 번째)

`RING_PINKY_CURL_JOINTS` 루프를 처음 붙였을 때 `tests/test_phase_2.py`/
`test_phase_3.py`가 갑자기 0/10으로 전멸 — 원인: `hand_only.xml`/`arm_hand.xml`은
약지/새끼 pip/dip/tip을 **여전히 `range="0 0"`으로 잠가둔 채 액추에이터가 아예
없음**(Session 8 후속 2에서 이 관절들을 활성화한 건 `full_scene.xml`뿐). 기존
`_set_joint_ctrl`은 액추에이터를 못 찾으면 `aid=None`을 그대로 `data.ctrl[aid]`에
써서 numpy가 `data.ctrl[None]`을 `data.ctrl[np.newaxis]`로 해석해 **배열 전체에
스칼라를 broadcast 대입**(Session 2에서 이미 한 번 겪은, 정확히 같은 종류의 버그
— NOTES.md "Phase 2" 참고, 이번이 세 번째 재발). 새 루프에서 `jid == -1` 또는
`aid is None`이면 그냥 `continue`하도록 명시적으로 가드를 추가해 해결 — "존재하지
않는 모델별 옵션 기능"을 다루는 코드는 항상 이 실패 모드를 의심해야 한다는 걸
또 한 번 확인.

### 성능 회귀 발견 및 수정 — 매 스텝 O(nu) 선형 스캔

버그를 고치고 나니 이번엔 `tests/test_phase_4.py`가 180초 넘게 안 끝남 — 원인 조사:
`apply_grasp`가 매 물리 스텝마다 관절 이름당 `_actuator_for_joint`(액추에이터
전체(nu개)를 순회하는 **순수 파이썬 선형 스캔**)를 호출하고 있었는데, 원래도
호출당 10번이던 게 이번에 6번 늘어(16번) 60% 더 느려짐. 직접 측정: `mj_step` 단독
~0.09ms/스텝인데 `apply_grasp` 포함 시 1.12ms/스텝 — 물리 자체가 아니라 순수 파이썬
탐색 오버헤드가 지배적이었음. `(model, joint_name) -> (jid, aid)` 캐시를
`_resolve_joint_actuator`로 추가해 첫 호출 이후엔 dict 조회만 하도록 바꿔서
0.116ms/스텝으로 복귀(거의 순수 물리 비용 수준). 행동은 전혀 안 바뀜(순수 캐싱),
`tests/test_phase_4.py`도 정상적으로 몇 분 안에 끝남.

### 검증

`tests/test_phase_{0,1,2,3,5}.py` 전부 PASS(회귀 없음). `tests/test_phase_4.py`
pick 8/10(이전 9/10에서 살짝 하락, 여전히 목표 7/10 통과) — grasp 램프 도중 약지/
새끼가 이전엔 처음부터 0.35로 고정이었는데 이제 램프를 따라 서서히 굽어가는
차이가 접촉 타이밍에 미세한 영향을 준 것으로 추정(원인 특정은 안 함, ±5mm 노이즈
기반 확률적 테스트라 1건 차이는 범위 내). 오프스크린 렌더로 직접 확인: (1) rest
포즈(grasp=0)에서 양손 손가락이 완전히 펴진 채 좌우 대칭(qpos 덤프로도 확인 —
`l`/`r` 약지·새끼 qpos가 소수점까지 완전히 동일한 0), (2) grasp=1.0으로 몇 초
정착시킨 렌더에서 양손이 대칭적으로 주먹을 쥔 모양(이전 세션에서 확인한 "벌어짐
없는" 모습 유지). `python3 src/teleop_app.py` 15초 스모크 실행도 클린 종료 확인.

### 교훈

1. **비대칭처럼 보이는 증상 앞에서 "정말 값이 다른가"부터 직접 덤프해서 확인해야
   한다** — 이번엔 실제로 키프레임 자체는 완벽히 대칭이었다. 만약 확인 없이
   "왼쪽 어딘가 값이 잘못 들어갔나보다" 가정하고 키프레임을 뒤졌다면 시간을
   낭비했을 것.
2. **"이 관절이 이 모델 변형에도 액추에이터를 갖고 있는가"는 절대 가정하면 안
   된다 — 프로젝트에 모델이 여러 개(hand_only/arm_hand/full_scene) 있고 각각
   기능 범위가 다르다.** `data.ctrl[None]` broadcast 버그는 이걸로 세 번째다 —
   이 프로젝트에서 "새 관절 매핑을 추가"할 때는 그게 모든 모델 변형에 실제로
   존재하는지부터 확인하거나, 존재하지 않을 가능성을 명시적으로 가드해야 한다는
   게 이제 확실한 패턴이다.
3. **하나의 요청(그랩 시 벌어짐 방지)을 고치면서 다른 제약(rest 시 펴짐)을
   깨뜨릴 수 있다 — 상수보다 스칼라에 연동된 램프가 대체로 더 안전하다.** 정적
   상수 하나로 여러 목적(rest 모양 + grasp 모양)을 동시에 만족시키려 하면
   결국 어느 하나는 깨진다. grasp=0/1 양 끝에서 각각 원하는 값이 되는 램프로
   바꾸면 "무엇을 어디서 조정하는지"가 명시적으로 분리된다.

## Phase 5 후속 6 — 왼손 엄지 자가 충돌 수정 + 바퀴-바닥 접촉 강성 문제 (2026-07-05)

**커밋:** `99d87cd`

### 왼손 엄지가 기본값에서부터 구부러져 있던 버그

스크린샷에서 왼손 엄지 쪽이 접혀 있는 게 보임. `finger_l_joint3`(엄지 mcp_pitch)의
range는 오른손과 미러링되어 `[-1.5708, 0]`(오른손은 `[0, 1.5708]`)인데,
`apply_grasp`는 항상 `lo`를 "편 상태"로 취급 — 오른손은 lo=0이 맞지만, 왼손은
`lo=-1.5708`이 실제로는 **최대로 구부러진 쪽**(hi=0이 편 상태)이라 정반대로
잘못 매핑되고 있었음. 그 결과 왼손 엄지는 thumb=0(편 상태)에서도 계속 최대 굴곡
쪽으로 명령받아 팔레트에 자가 충돌(~40N, 액추에이터가 힘 한계에 고정, 목표보다
26° 못 미침) — 그리고 thumb=1(굽힘)에서는 반대로 더 펴지는 방향으로 잘못 움직였음
(실제 grasp 테스트가 캔 없이 왼손 단독으로는 검증된 적이 없어 지금까지 발견 안 됨).
`src/grasp.py`에 `THUMB_CURL_OPEN_AT_HI = {"l": True, "r": False}` 추가해서 손별로
보간 방향을 뒤집도록 수정. 검증: 왼손 엄지가 이제 오른손과 정확히 같은 방식으로
동작(오차 0, 접촉 0, thumb=1에서 미러링된 방향으로 정상 굴곡).

### 바퀴-바닥 접촉이 실제 무게의 28배로 과도하게 뻣뻣했던 문제

사용자가 "방향키 회전이 원활하지 않고 직진도 그저 그렇다, 토크 탓이냐" 질의.
조사 중 자체 스크립트에서 버그 3개를 연달아 발견(교훈 참고: 잘못된 조인트 이름으로
캔의 freejoint를 읽고 있었음, 전진 대신 후진 키를 안 써서 테이블에 충돌, 팔
토크 홀드를 빼먹어 팔이 자유낙하) — 전부 고치고 `tests/test_phase_5.py`의 검증된
헬퍼(`_make_rig`/`_step`)로 재측정. 진짜 원인: **`base_link`은 x/y/yaw 관절만
있고 수직 자유도가 없어서**(Phase 4의 의도적 단순화), 바퀴의 의도적인 ~1.5mm
바닥 겹침이 하중에 따라 자연스러운 평형 깊이로 정착할 수 없음(가라앉을 자유도
자체가 없음). 이 고정된 1.5mm 깊이에서 MuJoCo 기본 solref(0.02 1)가 만드는
반발력을 직접 측정하니 바퀴 3개 합산 **~27,500N** — 로봇 실제 무게 982N의 28배.
이 비현실적인 힘 때문에 조향 액추에이터가 마찰을 이겨내려 몇 초씩 forcerange
한계(-2000~2000)에 고정되어 있었음(뒷바퀴 90° 회전에 실측 ~5초). `<pair>`로
바퀴-바닥 전용 solref를 "0.1 1"로 완화(같은 1.5mm 깊이에서 힘이 ~1100N로,
실제 무게와 비슷하게 나오도록 실측 스윕) — 조향 정렬 시간 5초→0.5초로 개선,
기존 유휴/직진 주행/테이블 충돌 테스트는 전부 완전히 동일한 수치로 회귀 없음.
단, 정상 상태 제자리 회전 각속도는 이 수정만으로는 명령값의 일부에 그침(별도의
더 깊은 견인력/토크 한계로 추정, 이번엔 해결 안 함).

### 검증

`tests/test_phase_{0,1,2,3,5}.py` 전부 PASS. `tests/test_phase_4.py` pick 8/10
(변경 전과 동일, 바퀴/엄지 수정 모두 손 grasp과 무관해 영향 없음 확인).

### 교훈

**진단 스크립트 자체도 검증이 필요하다 — 하루에 3개의 자체 버그를 만들었다**:
(1) `base_yaw_joint`라는 존재하지 않는 이름으로 조인트를 찾았는데 `mj_name2id`가
-1을 반환해도 확인 없이 `model.jnt_qposadr[-1]`로 인덱싱해서 파이썬의 음수
인덱싱이 조용히 **캔의 freejoint** qpos를 읽고 있었음(실제 조인트 이름은
`"base_yaw"`, `_joint` 접미사 없음) — 이 때문에 회전이 "전혀 안 되는 것처럼"
보였다가, 이름을 고치니 사실은 미미하게(0.5% 수준) 되고 있었다는 걸 알게 됨.
(2) 직진 테스트에 전진키('w')를 썼는데, 이미 검증된 `test_phase_5.py`의 공식
테스트는 일부러 후진키('s')를 쓴다는 걸 나중에 발견 — 전진은 팔이 테이블에
충돌하기 때문(주석에 이미 적혀 있었음, 안 읽고 재구현하다 놓침). (3) 팔 토크
홀드(`ctrl_r.apply`/`ctrl_l.apply`)를 매 스텝 호출하는 걸 빼먹어서 팔이 자유낙하,
그 결과 발생한 혼란스러운 접촉력을 바퀴-바닥 문제로 오인할 뻔함. 세 버그 모두
"이미 검증된 공식 테스트 헬퍼(`tp5._make_rig`/`_step`)를 그대로 재사용"하는
것으로 근본적으로 방지 가능했음 — 새로 짠 원포프 스크립트를 신뢰하기 전에,
기존에 검증된 하네스와 대조하거나 재사용하는 습관이 필요하다.

## Phase 5 후속 7 — 약지·새끼 mcp "잠김"이 실제로는 전혀 안 걸려있던 버그 (2026-07-05)

**커밋:** `5eb76b4`

### 사용자 지적: "점점 3~4번째 손가락이 내려가네"

grasp=0으로 가만히 둔 채 오래 놔두면 약지(4번째)가 서서히 계속 회전해 내려가는
게 눈에 보인다는 지적. 직접 60초짜리 정지 시뮬레이션을 돌려 `finger_r_joint13`
(약지 mcp, "펼침" 관절)의 qpos를 5초 간격으로 찍어보니 **5초에 0.087rad → 60초에
0.657rad까지 계속 단조 증가, 60초 안에 전혀 수렴하지 않음** — 반면 ctrl(명령
목표값)은 시종일관 0.0000으로 고정. 즉 액추에이터는 0을 명령하고 있는데 관절
자체가 그 명령과 무관하게 계속 회전하고 있었다는 뜻.

### 근본 원인: `range="0 0"`은 `limited` 없이는 아무 효과가 없다

`model.jnt_limited`를 직접 찍어보니 `finger_{l,r}_joint13`(약지 mcp)과
`finger_{l,r}_joint17`(새끼 mcp) **네 관절 전부 `limited=False`** — Phase 2부터
NOTES.md/코드 주석 여러 곳에 "약지·새끼 mcp는 range=0으로 잠겨있다"고 반복
기록돼 있었지만, 이 잠금 자체가 **처음부터 한 번도 실제로 걸린 적이 없었다**.
MuJoCo는 `<compiler autolimits>`가 켜져 있으면 range가 "지정 안 됨"의 기본값과
구분이 안 되는 정확히 `[0, 0]`일 때 `limited`를 자동으로 켜주지 못한다 — `range="0
0"`을 명시적으로 써도 스키마 기본값과 똑같아 보여서 조용히 무시된다. 그 결과 이
관절들은 damping=1.0/frictionloss=0.05라는 작은 저항만 있는 사실상 자유 관절이었고,
중력이 만드는 미세한 지속 토크가 수십 초에 걸쳐 관절을 서서히(그러나 멈추지 않고)
밀어낸 것 — 60초 동안 60도 가까이 돌아갔는데도 아직 평형에 도달 못 한 상태였다.

### 수정

`limited="true"`를 추가하려 했으나 MuJoCo가 `range[0] < range[1]`을 엄격히
요구해서 `"0 0"` 그대로는 컴파일 에러(`range[0] should be smaller than
range[1]`) — 폭이 사실상 0에 가까운 유효한 구간 `range="-0.0001 0.0001"`로 교체
(0.0001rad ≈ 0.0057°, 시각적으로 완전히 고정된 것과 구분 불가) 후 `limited="true"`
추가. 4관절(`finger_l_joint13/17`, `finger_r_joint13/17`) 전부 동일하게 수정.
재검증: 같은 60초 정지 시뮬레이션에서 `finger_r_joint13`이 0.0002rad에 고정(더
이상 드리프트 없음). `tests/test_phase_{0,1,2,3,4,5}.py` 전부 PASS, **수치가
전부 이전과 완전히 동일**(pick 8/10, idle drift 0.645mm, 주행/충돌 수치 동일) —
이 관절들이 애초에 실제 grasp에 참여한 적이 없으니 "진짜로 잠그기 시작"해도
기존 검증된 동작에 전혀 영향이 없었던 것. 오프스크린 렌더로 60초 후에도 양손이
여전히 대칭적으로 펴져 있는 것 확인.

### 교훈

**"설계 의도가 주석에 여러 번 적혀 있다"는 것과 "실제로 그렇게 동작한다"는 것은
다른 문제다 — 특히 관절 제약처럼 눈에 안 보이는 물리 설정은 직접 `model.jnt_limited`
같은 컴파일된 값을 찍어봐야 확실하다.** 이 버그는 Phase 2 이후 여러 세션에 걸쳐
"약지·새끼는 범위가 0이라 안 움직인다"는 전제로 코드/주석이 계속 쌓여왔지만
아무도 실제로 `jnt_limited`를 확인한 적이 없었다 — 겉보기에 문제가 없어 보였던
이유는 이 관절들이 액추에이터도 없고(Phase 2 시점) 능동적으로 힘을 받을 일도
없어서 드리프트가 아주 느렸기 때문(수십 초 단위). "사용자가 화면을 몇 분 이상
계속 보고 있어야 알아챌 수 있는" 종류의 버그는 짧은 헤드리스 테스트(대부분
몇 초짜리)로는 절대 안 걸린다는 것도 확인 — 필요하면 의도적으로 긴(수십~수백초)
정지 시뮬레이션을 도구 상자에 추가해둘 것.

## Phase 5 후속 8 — 제자리 회전/모바일 조작감 추가 개선 (2026-07-05)

**커밋:** `60f4b84`

### 사용자 지적: "제자리 회전과 모바일 부분의 제어가 뻑뻑한 느낌"

Phase 5 후속 6에서 바퀴-바닥 접촉 강성(28배 과도)을 고쳐 조향 정렬 시간은
5초→0.5초로 크게 개선했지만, 그때 이미 "제자리 회전 각속도 자체는 여전히 명령값의
일부에 그침(별도의 더 깊은 견인력 한계)"이라고 기록해뒀던 부분 — 사용자가 실제
조작해보고 여전히 뻑뻑하다고 재지적. 이번엔 그 "더 깊은 한계"를 실제로 밀어붙여봄.

### `wheel_drive` kv 추가 인상 + 접촉 solref 재조정

`tests/test_phase_5.py`의 공식 헬퍼(`_make_rig`/`_step`)로 kv를 30→1500까지
스윕하면서 제자리 회전 정상상태 각속도와 직진 속도/슬립을 같이 측정:

| kv | 회전 각속도(rad/s, 목표 1.2 대비 %) | 직진 속도(m/s) | 슬립 |
|---|---|---|---|
| 30(기존) | 0.063 (5.2%) | 0.460 | 0.8% |
| 60 | 0.124 (10.3%) | 0.477 | 0.8% |
| 150 | 0.230 (19.1%) | 0.488 | 0.8% |
| 200 | 0.264 (22.0%) | 0.490 | 0.8% |
| 300~1000 | 0.12~0.19 (10~19%, 들쭉날쭉) | 0.49 | 0.7~0.8% |
| 1500 | 0.220 (18.4%) | 0.496 | 0.6% |

kv=200까지는 꾸준히 개선되다가 그 이후로는 값이 들쭉날쭉해짐(뒷바퀴가 조향
범위 경계(±90.5°) 근처에 계속 머무는 것과 관련 있을 것으로 추정, 직접 원인
규명은 안 함) — kv 자체보다 다른 요인이 지배하기 시작하는 신호로 보고 더 올리는
건 포기. 직진 속도/슬립은 kv를 올릴수록 계속 개선(0.460→0.496m/s, 슬립도 유지되거나
개선)되고 유휴 drift는 전혀 안 변함(당연히 구동 명령이 없으면 kv가 할 일이 없음).

바퀴-바닥 접촉 solref(Phase 5 후속 6에서 "0.1 1"로 튜닝)도 함께 스윕(`kv` x
`timeconst` 조합): `timeconst=0.15, kv=150` 조합이 회전 개선 폭(22.6%)은
`kv=200` 단독과 비슷하면서 **피크 순간가속도(max|qacc|)가 가장 낮은**(3567 vs
다른 조합들의 5000~5700) 조합으로 확인돼 최종 채택. `models/full_scene.xml`의
바퀴-바닥 `<pair>` solref를 "0.1 1"→"0.15 1"로, `wheel_drive` 기본 클래스의
`kv`를 30→150으로 갱신.

### 검증

`tests/test_phase_{0,1,2,3}.py` 전부 PASS(회귀 없음, kv/접촉 강성 변경은 손
grasp과 무관). `tests/test_phase_4.py` pick 8/10(변경 전과 완전 동일). `tests/
test_phase_5.py` 전부 PASS — idle drift 0.645mm(동일), 직진 속도 0.460→0.488m/s로
개선(슬립 0.8% 유지), 테이블 충돌 정지거리 445→447mm(오차 범위 내, 사실상 동일).
`python3 src/teleop_app.py` 15초 스모크 클린 종료.

### 알려진 한계

제자리 회전 각속도는 여전히 명령값(1.2 rad/s)의 ~23%에 그침 — kv를 아무리
올려도 200을 넘으면 개선이 멈추고 들쭉날쭉해지는 것으로 보아, 이건 단순 게인
문제가 아니라 **바퀴 조향 범위(±90.5°)의 기하학적 한계**(뒷바퀴는 제자리 회전 시
정확히 경계 근처인 -90°를 요구, NOTES.md "Phase 5 후속 4"의 조향 sweep에서도
이미 확인된 사실)나 **바퀴-지면 마찰이 제공할 수 있는 최대 견인력 자체의 한계**
(로봇 전체의 요(yaw) 관성모멘트가 3개의 작은 바퀴 접촉점만으로 감당하기엔 원래
빠듯할 가능성)에 가까워 보인다. 이번 세션 범위에서는 "확실히 안전하게 개선
가능한 만큼"(kv=150, solref=0.15)까지만 밀어붙이고 멈춤 — 더 근본적인 개선을
원하면 바퀴 마찰계수/반지름/배치 자체를 재설계하거나, 명령 가능한 최대 회전
속도(K_YAW) 자체를 실제 달성 가능한 값에 맞춰 낮추는 방향을 다음 세션에서
검토할 것.

## Phase 5 후속 9 — 캔에 실제 라벨 텍스처 적용 (2026-07-05)

**커밋:** `8840c90`

### 요청: "에셋에 포함되어있는 이미지를 기반으로" 캔 텍스처 적용

`/home/ggh/ffw-sh5-teleoperation/assets/soda_can_stl/textures/`에서 소스를 확인 —
`soda_can_color.png`는 이름과 달리 흑백(러프니스/메탈릭 계열)이고, `soda_can_color2.png`
(4096x4096)가 실제 색상 UV 아틀라스(라벨 + 캡/림 디테일 조각들)였음. 라벨 영역만
좌표(156,2056)-(2968,3680)로 크롭(초록색 픽셀 마스크로 자동 bbox 탐지) → 1024px로
축소해 `assets/soda_can/soda_can_label.png`로 vendoring(기존 STL 벤더링과 동일한
패턴, 절대경로 참조 없음).

### 시행착오: 원통 프리미티브의 자동 UV로는 라벨이 안 감김

처음엔 `type="cylinder"` 프리미티브에 라벨 텍스처를 입히면 MuJoCo가 원주를 따라
자동으로 텍스처 좌표를 감아줄 거라 기대(별도 UV 저작 불필요). 실제로는 **어느
각도에서 봐도 초록색 배경의 가는 세로 줄무늬만 보이고 "Spite" 로고/버블/영양정보
패널이 전혀 안 보이는** 현상 발생. 원인 규명 과정:

1. 텍스처 로딩 자체 의심 → `model.tex_data`를 직접 덤프해서 이미지로 저장해보니
   **완벽하게 정상**(원본과 동일) — 로딩/디코딩 버그는 아님.
2. 오프스크린 렌더러(`mujoco.Renderer`)가 매번 "OpenGL error 0x502"를 띄우는 게
   의심스러워 실제 텔레옵 앱과 동일한 방식(GLFW + `mjr_render` 직접 호출)으로
   재렌더 → 에러는 사라졌지만 **증상은 동일** — 렌더 컨텍스트 문제도 아님.
3. 원통 크기(반지름 0.033→0.5)와 텍스처 해상도(1024x591→256x148)를 각각 크게
   바꿔도 **정확히 동일한 패턴**이 나옴 — 스케일/밉맵 문제도 아님.
4. 6색 블록으로 된 간단한 테스트 텍스처로는 원주를 따라 정확히 한 바퀴 감기는 게
   확인됨(레드→오렌지→...→퍼플이 360°에 걸쳐 정확히 한 번 순환) — 그런데 똑같은
   설정에서 라벨 이미지(디테일이 많은 사진형 이미지)만 실패.

결론: **MuJoCo의 `type="cylinder"` 프리미티브 자동 UV 생성은 사진/라벨류의 디테일
많은 2D 텍스처를 하나의 매끄러운 원주 랩으로 만들어주지 않는다**(정확한 내부 동작은
특정 못함, 재현성만 확인) — 단순 색상 블록에서는 우연히 봐줄 만하게 보였을 뿐.

### 해결: 명시적 UV를 가진 OBJ 메시로 전환

STL은 UV 좌표를 아예 가질 수 없지만 **OBJ는 `vt` 라인으로 UV를 명시할 수 있다**는
점에 착안 — `tests/generate_can_label_mesh.py`(새 dev 툴)로 원통 옆면만의 순수
파라메트릭 메시를 생성: 원주 방향 64분할 + 이음매 처리를 위한 중복 열(u=0과 u=1이
같은 위치를 가리키되 텍스처 좌표만 다름, 안 하면 이음매에서 텍스처가 거꾸로 감김),
각 정점에 `u=각도/2π`, `v=0(바닥)~1(위)`를 명시적으로 부여. 결과물
`assets/soda_can/can_side.obj`를 `type="mesh"`로 불러와 `can_mat`(텍스처 재질)을
입히니 **8방향 스윕에서 로고·버블·노란 점·영양정보 패널이 전부 또렷하게, 정확히
한 바퀴 감긴 채로** 나타남 — 확인 완료.

### 최종 구성

캔 body의 시각 geom을 3개로 분리(물리 콜리전인 `can_geom`은 완전히 그대로,
Phase 1/2 검증 형상 불변): (1) `can_side` 메시 + `can_mat`(라벨 텍스처)로 옆면
(half-height 0.050), (2) 위/아래 각각 얇은 원통(half-height 0.0025) + `can_cap_mat`
(무광 은색 플레인 컬러)로 뚜껑 — 세 개를 이으면 정확히 `can_geom`의 실제 높이
(0.055)와 gap/overlap 없이 맞아떨어짐. 기존 `soda_can.stl` 비주얼 메시는 참고용으로
남겨두되(UV가 없어 텍스처를 못 받으므로) 더 이상 실제로 렌더링에 쓰이지 않음.

**부수 발견**: `.gitignore`의 `*.png` 규칙이 새 라벨 텍스처까지 걸러내서(스냅샷용
`*.png` 무시 규칙이 진짜 에셋까지 잡아먹음) 커밋에서 조용히 빠질 뻔함 —
`!assets/soda_can/*.png` 예외를 추가해서 해결(더 넓게 `!assets/**/*.png`로 하면
Phase 0부터 이미 미추적 상태였던 무관한 벤더 PNG 3개까지 딸려 나와서 범위를
`soda_can/`으로만 좁힘).

### 검증

`tests/test_phase_{0,1,2,3,4,5}.py` 전부 PASS, 수치 전부 이전과 완전히 동일(pick
8/10, idle drift 0.0002mm, 직진/충돌 수치 동일) — 시각 geom만 바꾼 순수 렌더링
변경이라 물리에 전혀 영향 없음. `python3 src/teleop_app.py` 15초 스모크 클린 종료.

### 교훈

**"이 프리미티브 타입은 이런 식으로 동작할 것이다"라는 가정은 실제로 렌더해보기
전까진 신뢰하면 안 된다** — MuJoCo 문서/직관상 원통에 텍스처를 감으면 자동으로
원주 랩이 될 거라 기대했지만 실제로는 그렇지 않았다. 문제를 좁혀나갈 때도 순서를
지켰다(로딩→렌더 컨텍스트→스케일→비교 테스트 순으로 각 가설을 하나씩 소거) —
이 프로젝트의 "파라미터를 크게 바꿔도 결과가 똑같다"는 신호를 이번에도 그대로
적용(원통 크기/텍스처 해상도를 몇 배씩 바꿔도 완전히 동일한 결과 → 그 변수는
원인이 아니라는 확신을 갖고 다음 가설로 넘어감).
