function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    // please use names 'pendulum.angle', 'pendulum.angle_previous', etc. in else codeblock between line 28-30

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment
    pendulum.angle_dot_previous = pendulum.angle_dot;
    pendulum.angle_previous = pendulum.angle;
    var temp = pendulum_acceleration( pendulum, gravity );
    pendulum.angle_dot = pendulum.angle_dot_previous + dt * temp;
    pendulum.angle = pendulum.angle_previous + pendulum.angle_dot_previous * dt;
    }
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration

    }
    else if (numerical_integrator === "velocity verlet") {

    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment
    pendulum.angle_previous = pendulum.angle;

    nicolev = (2 * Math.PI);

    pendulum.angle = ((pendulum.angle + pendulum.angle_dot * dt + pendulum_acceleration(pendulum, gravity) * dt * dt / 2) % (nicolev));

    pendulum.angle_dot = ((pendulum.angle_dot + pendulum_acceleration(pendulum, gravity) * dt) % (nicolev));


    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator
    } 
    else {
        pendulum.angle_previous = pendulum.angle;
        pendulum.angle = (pendulum.angle+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot = (pendulum.angle-pendulum.angle_previous)/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    nicole = - gravity / pendulum.length * Math.sin (pendulum.angle) + pendulum.control / pendulum.mass / pendulum.length / pendulum.length;
    return nicole;

}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time
    
        


    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    switch (pendulum.control_type) {


        case "pd":

            pendulum. servo = {kp:0.6, kd:0.1, ki:0}; 

            break;

        case "pid":

            pendulum.servo = {kp: 0.6, kd: 0.1, ki:0.01}; 

            break;

        case "p":

        pendulum.servo = {kp: 0.5, kd: 0, ki:0}; 

        break;

        default:

        pendulum.servo = {kp:0, kd: 0, ki:0};
        // no control is noted as default 
        break;  
    return pendulum;
}
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error

    var nicole1 = pendulum.desired - pendulum.angle;

    accumulated_error += (nicole1 * dt);

    pendulum.control += (pendulum.servo.kp * nicole1) + (pendulum.servo.ki * accumulated_error) + (pendulum.servo.kd * pendulum.angle_dot);


    return [pendulum, accumulated_error];
}