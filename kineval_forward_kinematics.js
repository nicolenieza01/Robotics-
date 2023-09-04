
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

     kineval.buildFKTransforms();
}

    // STENCIL: implement buildFKTransforms, which kicks off
    //   a recursive traversal over links and 
    //   joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // To use the keyboard interface, assign the global variables 
    //   "robot_heading" and "robot_lateral", 
    //   which represent the z-axis (heading) and x-axis (lateral) 
    //   of the robot's base in its own reference frame, 
    //   transformed into the world coordinates.
    // The axes should be represented in unit vector form 
    //   as 4x1 homogenous matrices

    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

    kineval.buildFKTransforms = function buildFKTransforms () {

        traverseFKBase();
        var childrenlen = robot.links[robot.base].children.length;
        var indexy = 0;
        while (indexy < childrenlen){
            traverseFKJoint(robot.links[robot.base].children[indexy]);
            indexy++;
        }
    }
    function  rotationmatrix (n,i,c){
        var nicole = matrix_multiply(generate_rotation_matrix_Z(c), generate_rotation_matrix_Y(i));
        nicole = matrix_multiply(nicole, generate_rotation_matrix_X(n));
        return nicole;
    // makes computations easier rather than going thru a bunch more steps 
    }

    
    function traverseFKBase() {
        var heading_vector = [[0],[0],[1],[1]];

        var lateral_vector = [[1],[0],[0],[1]];

        var rotationmatrixy = rotationmatrix(robot.origin.rpy[0],robot.origin.rpy[1],robot.origin.rpy[2]);

        var translationmatrix = generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]);

        robot.links[robot.base].xform = matrix_multiply(translationmatrix,rotationmatrixy);

        // for dance extension - just update the globals 
        robot_heading = matrix_multiply(robot.links[robot.base].xform,heading_vector);

        robot_lateral = matrix_multiply(robot.links[robot.base].xform,lateral_vector);

        if (!robot.links_geom_imported) {
            
        }
        else {
            robot.links[robot.base].xform = matrix_multiply(robot.links[robot.base].xform, matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2)));
        }
   
    }


    function traverseFKJoint(joint_name) {


        var joint = robot.joints[joint_name];


        var mR = rotationmatrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]);
        // rotation matrix 
        var mT = generate_translation_matrix(joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2]); 
        // translational
        var quant = kineval.quaternionFromAxisAngle(robot.joints[joint_name].angle,robot.joints[joint_name].axis);

        var quantmatrix = kineval.quaternionToRotationMatrix(quant);

        var mat = matrix_multiply(mR, quantmatrix);

        var nicole = matrix_multiply(mT, mat);
        joint.xform = matrix_multiply(robot.links[joint.parent].xform, nicole);
        //var mT = generate_translation_matrix(joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2]);
        //joint.xform = matrix_multiply(robot.links[joint.parent].xform, matrix_multiply(mT, mR));
        //var quant = quaternion_from_axisangle(robot.joints[joint_name].angle,robot.joints[joint_name].axis);
        //quant = rotationmatrix(quant);
        //quant = matrix_multiply(mR, quant);

  // Check angle
        if (typeof robot.joints[joint_name].angle !== 'undefined') {
            var axis = [[robot.joints[joint_name].axis[0]], [robot.joints[joint_name].axis[1]], [robot.joints[joint_name].axis[2]], [0]];
            robot.joints[joint_name].xform = matrix_multiply(robot.joints[joint_name].xform, rotationmatrix(axis, robot.joints[joint_name].angle));
        }
  // check control
        else if (typeof robot.joints[joint_name].control !== 'undefined') {
            var axis = [[robot.joints[joint_name].axis[0]], [robot.joints[joint_name].axis[1]], [robot.joints[joint_name].axis[2]], [0]];
            robot.joints[joint_name].xform = matrix_multiply(robot.joints[joint_name].xform, generate_translation_matrix(matrix_multiply(axis, [[robot.joints[joint_name].control]])));
        }

        traverseFKLink(robot.joints[joint_name].child);



    }

    function traverseFKLink(link_name) {
    robot.links[link_name].xform = robot.joints[robot.links[link_name].parent].xform;

    var children = robot.links[link_name].children;

    if (typeof children !== 'undefined' && children.length > 0) {
        for (var i = 0; i < children.length; i++) {
            traverseFKJoint(children[i]);
        }
    }
    else{


        return;
    }
    }





    
