//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

// STENCIL: reference quaternion code has the following functions:
//   quaternion_from_axisangle
//   quaternion_normalize
//   quaternion_to_rotation_matrix
//   quaternion_multiply

// **** Function stencils are provided below, please uncomment and implement them ****//

 kineval.quaternionFromAxisAngle = function quaternion_from_axisangle(axis,angle) {
//     // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
//     var q = {};
    var nicole = {};
    var n = Math.sin(angle/2);
    var i = Math.cos(angle/2);
    
    nicole.a = i;
    nicole.b = axis[0] * n;
    nicole.c = axis[1] *n;
    nicole.d = axis[2] *n;
    
    return nicole;

 }

 kineval.quaternionNormalize = function quaternion_normalize(q1) {
//     // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
//     var q = {};
    var nicole = {};
    var n = q1.a **2;
    var i = q1.b **2;
    var c = q1.c **2;
    var o = q1.d **2;


    var n = Math.sqrt(n +i +c + o);


    nicole.a = q1.a /n;
    nicole.b = q1.b/n;
    nicole.c = q1.c/n;
    nicole.d = q1.d/n;
    return nicole;


 }

 kineval.quaternionMultiply = function quaternion_multiply(q1,q2) {
//     // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
//     var q = {};
    var nicole = {};

    nicole.a  = q1.a *q2.a - q1.b*q2.b - q1.c *q2.c  -q1.d *q2.d;

    nicole.b  = q1.a *q2.b + q1.b * q2.a + q1.c *q2.d  -q1.d *q2.c;

    nicole.c = q1.a *q2.c - q1.b * q2.d + q1.c *q2.a  +q1.d *q2.b;
    
    nicole.d = q1.a *q2.d + q1.b * q2.c - q1.c *q2.b  +q1.d *q2.a;
    
    return nicole;

    

 }

 kineval.quaternionToRotationMatrix = function quaternion_to_rotation_matrix (q) {

//     // returns 4-by-4 2D rotation matrix
    var nicole = [];

    
    nicole.push([1-2*q.c ** 2 -2 *q.d ** 2, 2* q.b *q.c -2 *q.d *q.a, 2* q.b *q.d +2 *q.c *q.a, 0]);


    nicole.push([2 *q.b *q.c +2 *q.d *q.a, 1 -2 *q.b **2 -2 *q.d **2, 2* q.c *q.d -2 *q.b *q.a, 0]);


    nicole.push([2*q.b *q.d -2 *q.c *q.a, 2*q.c * q.d +2 *q.b *q.a, 1-2 *q.b **2-2 *q.c **2, 0]);


    nicole.push([0,0,0,1]);
    
    
    return nicole;



 }