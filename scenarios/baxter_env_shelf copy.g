world {}

Include:'shelf.g'

Include:'./baxter/baxter_new.g'

## delete original gripper frames & joints
Delete right_gripper_base>r_gripper_l_finger_joint
Delete r_gripper_l_finger_joint
Delete r_gripper_l_finger
Delete r_gripper_l_finger_0
Delete r_gripper_l_finger_1
Delete right_gripper_base>r_gripper_r_finger_joint
Delete r_gripper_r_finger_joint
Delete r_gripper_r_finger
Delete r_gripper_r_finger_0
Delete r_gripper_r_finger_1

Delete r_gripper_r_finger>r_gripper_r_finger_tip_joint
Delete r_gripper_r_finger_tip_joint
Delete r_gripper_r_finger_tip
Delete r_gripper_r_finger_tip_0
Delete r_gripper_r_finger_tip_1

Delete r_gripper_l_finger>r_gripper_l_finger_tip_joint
Delete r_gripper_l_finger_tip_joint
Delete r_gripper_l_finger_tip
Delete r_gripper_l_finger_tip_0
Delete r_gripper_l_finger_tip_1


Edit head_nod { q= -0.57 }

Prefix: "R_"
Include: 'baxter_gripper.g'

Edit R_gripper (right_gripper_base){ Q:< d(-180 0 1 0) d(-90 0 0 1) t(0 0 0)> }
Edit R_finger1{ joint:transX Q:<> A:<t(+.06 0 -.08)> limits: [-.05 0.02], contact: -2,  }
Edit R_finger2{ joint:transX mimic:(R_finger1) Q:<> A:<d(180 0 0 1) t(+.06 0 -.08)>, contact: -2, }

gripperCenter (right_endpoint){
    shape:marker, size:[.10], color:[.9 .0 .9],
    Q:<t(0 0 0) )>
}


camera(camera_rgb_optical_frame){
    Q:<t(0. 0. 0.) d(180 0 -1 0) d(180 0 0 1)>, color:[.9 .0 .0],
    shape:marker, size:[.4],
    focalLength:0.895, width:640, height:480, zRange:[.4 70]
}


