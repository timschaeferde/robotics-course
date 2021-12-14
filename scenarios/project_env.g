world {}

### walls

wall1 (world){
    shape:ssBox, Q:<t(-1.6 0. .8) d(90 0 1 0)>, size:[1.7 2. .05 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

wall2 (world){
    shape:ssBox, Q:<t(1.6 0. .8) d(90 0 1 0)>, size:[1.7 2. .05 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

wall3 (world){
    shape:ssBox, Q:<t(0. -.9 .35) d(90 1 0 0)>, size:[3.5 0.8 .05 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

wall4 (world){
    shape:ssBox, Q:<t(0. .9 .35) d(90 1 0 0)>, size:[3.5 0.8 .05 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

### two pandas

#L_lift (table){ joint:transZ, limits:[0 .5] }

Prefix: "L_"
Include: '../rai-robotModels/scenarios/panda_fixGripper.g'

Prefix: "R_"
Include: '../rai-robotModels/scenarios/panda_fixGripper.g'

Prefix!
        
Edit L_panda_link0 (world) { Q:<t(-1. 0. 0.) d(0 0 0 1)> }
Edit R_panda_link0 (world)  { Q:<t( 1. 0. 0.) d(180 0 0 1)> }

### camera

camera1(world){
    Q:<t(-1.6 -1. 1.5) d(45 1 0 0) d(55 0 -1 -1) d(15 1 0 0)>,
    shape:marker, size:[.3]}

camera2(world){
    Q:<t(1.6 1. 1.5) d(180 0 0 1) d(45 1 0 0) d(55 0 -1 -1) d(15 1 0 0)>,
    shape:marker, size:[.3]}

camera3(world){
    Q:<t(0. 0 3.0) d(0 1 0 0)>,
    shape:marker, size:[.2]}

### ball

ball 	{  shape:sphere, size:[.03],contact:1, mass:.2 X:<[0.5, 0., 0.05, 0,0,0,1.]> color:[1,0,0] }



