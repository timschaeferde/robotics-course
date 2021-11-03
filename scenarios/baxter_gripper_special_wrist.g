gripper{
    shape:ssBox, size:[.12 .04 .03 .01], color:[0.4352, 0.49, 0.5451, 1]
    X:<t(.05 .5 1.) d(-90 0 0 1) d(-45 1 0 0) >
    contact:-1
}

finger1 (gripper){
    shape:ssBox, size:[.01 .02 .1 .005], color:[0.4352, 0.49, 0.5451, 1]
    X:<t(.05 .5 1.) d(-90 0 0 1) d(-45 1 0 0) >   
    contact:-1
}

finger2 (gripper){
    shape:ssBox, size:[.01 .02 .1 .005], color:[0.4352, 0.49, 0.5451, 1]
    X:<t(.05 .5 1.) d(-90 0 0 1) d(-45 1 0 0) >   
    contact:-1
}

gripperCenter (gripper){
    shape:marker, size:[.03], color:[.9 .0 .0],
    Q:<t(0 0 -.05) d(-45 1 0 0)>
}