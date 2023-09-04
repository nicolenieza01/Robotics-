
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 


    for (var x in robot.joints) {

        var errorterm = kineval.params.setpoint_target[x] - robot.joints[x].angle;

        if ((Math.abs(errorterm)) > 0.05) {
            return
        };
    }

    
    if (kineval.params.dance_pose_index < kineval.params.dance_sequence_index.length) {

       
        kineval.params.dance_pose_index++;

    } else {
        kineval.params.dance_pose_index = 0;
        
    }
    kineval.setPoseSetpoint(kineval.params.dance_pose_index);
}

    // STENCIL: implement FSM to cycle through dance pose setpoints


kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints

    for (var jointName in robot.joints) {
        var joint = robot.joints[jointName];
        var currentang = joint.angle;
        var error = kineval.params.setpoint_target[jointName] - currentang;
        var control = error * joint.servo.p_gain;
        joint.control = control;
    }
    
}


