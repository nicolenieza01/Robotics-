//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i = 0;
    var j = 0;

    while (i < m1.length) { // for each row of m1
        mat[i] = [];
        while (j < m1[0].length) { // for each column of m1
            mat[i][j] = m1[i][j];
            j++;
        }
        i++;
    }
    return mat;
}


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//



function matrix_multiply(m1,m2) {
//     // returns 2D array that is the result of m1*m2

    var m = new Array(m1.length);
    var nicole = 0;
        while(nicole < m1.length){
            m[nicole] = new Array(m2[0].length);
            var nn = 0;
            while (nn < m2[0].length){
            
                var robot = 0;
                var k = 0;
                while (k<m2.length){
                    robot = robot + m1[nicole][k]*m2[k][nn];
                    k++;

                }
                m[nicole][nn]=robot;
                nn++;
            }
            nicole++;
        }
        return m;


}

function matrix_transpose(m) {
//     // returns 2D array that is the result of m1*m2
    var mm = [];
    var j =0;
    var nicole = 0;

    while(nicole < m[0].length){

        mm[nicole] = [];

        nicole++;
    }
    var robot = 0;

    while(robot< m[0].length){

        for (j = 0; j < m.length; j++){

        mm[robot][j] = m[j][robot];

        }

        robot++;
    }
    return mm;


 }

 function matrix_pseudoinverse(m) {
    
        
        let svdResult = numeric.svd(m);
        
        
        let cut =  Math.max.apply(svdResult.S) * Math.max(m[0].length, m.length) * 0.000001;
        
        
        let S_inv = svdResult.S.map((value) => (value <= cut ? 0 : 1/value));
      
        let U_transpose = numeric.transpose(svdResult.U);
        
        let dotProduct = numeric.dot(numeric.diag(S_inv), U_transpose);

        let nicole = numeric.dot(svdResult.V, dotProduct);
    
        return nicole;
    }

 
//     // returns pseudoinverse of matrix m

    
// skip for assignemnt 3 


 //}

// function matrix_invert_affine(m) {
//     // returns 2D array that is the invert affine of 4-by-4 matrix m

// skip for assignment 3 

// }

 function vector_normalize(v) {



//     // returns normalized vector for v
    var magnitude = 0;

    var i = 0;

    while ( i < v.length) {

        magnitude += v[i] * v[i];

        i++;
    }
    magnitude = Math.sqrt(magnitude);

    var normalized = [];

    var i = 0; 

    while (i < v.length) {

        normalized.push(v[i] / magnitude);

        i++;
    }
    return normalized;
}


    
 

 function vector_cross(a,b) {
//     // return cross product of vector a and b with both has 3 dimensions

    var v = new Array(a.length);

    v[2] = a[0]*b[1] - a[1]*b[0];

    v[1] = -(a[0]*b[2] - a[2]*b[0]);

    v[0] = a[1]*b[2] - a[2]*b[1];

    return v;

    
 }

 function generate_identity() {
//     // returns 4-by-4 2D array of identity matrix


    var m = new Array(4);

    var nicole = 0;

    while(nicole<4){

        m[nicole] = new Array(4);

        var robot = 0;

        while(robot <4){

            if (nicole==robot){

                m[nicole][robot] = 1;

            } else {

                m[nicole][robot] = 0;

            }

            robot++;
        }
        nicole++;
    }
return m;

    
 }

 function generate_translation_matrix(tx, ty, tz) {
//     // returns 4-by-4 matrix as a 2D array


    var m = generate_identity(4);

    m[0][3] = tx;
    m[1][3] = ty;
    m[2][3] = tz;
    return m;

    
 }

 function generate_rotation_matrix_X(angle) {
//     // returns 4-by-4 matrix as a 2D array, angle is in radians

    var m = generate_identity(4);

    m[1][2] = - Math.sin(angle);
    m[2][2] = Math.cos(angle);
    m[2][1] = Math.sin(angle);
    m[1][1] = Math.cos(angle);
    return m;  

    
 }

 function generate_rotation_matrix_Y(angle) {
//     // returns 4-by-4 matrix as a 2D array, angle is in radians

    var m = generate_identity(4);
   
    m[2][2] = Math.cos(angle);
    m[0][0] = Math.cos(angle);
    m[2][0] = - Math.sin(angle);
    m[0][2] = Math.sin(angle);
    return m; 

    
 }

 function generate_rotation_matrix_Z(angle) {
//     // returns 4-by-4 matrix as a 2D array, angle is in radians

    var m = generate_identity(4);

    m[0][0] = Math.cos(angle);
    m[1][0] = Math.sin(angle);
    m[0][1] = - Math.sin(angle);
    m[1][1] = Math.cos(angle);
    
    return m; 


    
 }