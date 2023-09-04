
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
        Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
        + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
        + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
    kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
    kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
    kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
    kineval.params.trial_ik_random.targets += 1;
    textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }

}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    // robot.dx = []              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    // robot.jacobian = []        // Jacobian matrix of current IK iteration matrix size: 6 x N
    // robot.dq = []              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------

    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length
    var endeffector_position_world = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
    robot.dx = [[endeffector_target_world.position[0] - endeffector_position_world[0]],
    [endeffector_target_world.position[1] - endeffector_position_world[1]],
    [endeffector_target_world.position[2] - endeffector_position_world[2]],[0],[0],[0]];

    var jacob = [];

    var indexy = 0; 
    robot.jacobian = [];

    var current = endeffector_joint;

    while (current !== robot.base) {
        

        var temp = [[robot.joints[current].axis[0]], [robot.joints[current].axis[1]], [robot.joints[current].axis[2]], [1]];
        var axworld = matrix_multiply(robot.joints[current].xform, temp);

        var joworld = matrix_multiply(robot.joints[current].xform, [[0], [0], [0], [1]]);

        var diffjoint = [axworld[0] - joworld[0],
        axworld[1] - joworld[1],
        axworld[2] - joworld[2]];


        var enddiff = [endeffector_position_world[0]- joworld[0],endeffector_position_world[1] - joworld[1], endeffector_position_world[2]- joworld[2]];

        var tempcross = vector_cross(diffjoint, enddiff);

        var jacobw = diffjoint;

        robot.jacobian.unshift([tempcross[0], tempcross[1], tempcross[2], jacobw[0], jacobw[1], jacobw[2]]);


        if (robot.links[robot.joints[current].parent].name == robot.base){

            current = robot.base;
        }
        else {
            current = robot.links[robot.joints[current].parent].parent;
        }
        indexy++;
    }
    var jacobT = matrix_transpose(robot.jacobian);
    robot.jacobian = jacobT;

    if (!kineval.params.ik_pseudoinverse) {
        robot.dq = matrix_multiply(matrix_transpose(robot.jacobian), robot.dx);
        
    }
    else{
        
        robot.dq = matrix_multiply(matrix_pseudoinverse(robot.jacobian), robot.dx);
    }
    var newi= indexy-1;

    current = endeffector_joint;
    

    while (current !== robot.base) {

        robot.joints[current].control = kineval.params.ik_steplength * robot.dq[newi];

        newi--;
        
        if (robot.links[robot.joints[current].parent].name != robot.base){

            current = robot.links[robot.joints[current].parent].parent;

        }
        else {

            current = robot.base;
        }

    }
}
    






