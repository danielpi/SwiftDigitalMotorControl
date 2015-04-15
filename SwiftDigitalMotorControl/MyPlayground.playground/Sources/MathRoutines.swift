//
//  MathRoutines.swift
//  SwiftDigitalMotorControl
//
//  Created by Daniel Pink on 30/03/2015.
//  Copyright (c) 2015 Electronic Innovations. All rights reserved.
//

import Foundation

// Radians Functions
public func toRadians(x: Double) -> Double {
    return (M_PI / 180) * x
}

public func toDegrees(x: Double) -> Double {
    return (180 / M_PI) * x
}



// Clarke Transform
// This transformation converts balanced three phase quantities into balanced two phase quadrature quantities

public func clarke(phaseA: Double, phaseB: Double, phaseC: Double) -> (Double, Double) {
    let alpha = phaseA
    let beta = (phaseB - phaseC) / sqrt(3.0)
    return (alpha, beta)
}

public func clarke(phaseA: Double, phaseB: Double) -> (Double, Double) {
    let alpha = phaseA
    let beta = (2 * phaseB + phaseA) / sqrt(3.0)
    return (alpha, beta)
}


// Park Transform
// This transformation converts vectors in balanced 2-phase orthogonal stationary system into orthogonal rotating reference frame.

public func park(alpha: Double, beta: Double, angle: Double) -> (Double, Double) {
    let Ds = (alpha * cos(angle)) + (beta * sin(angle))
    let Qs = (-alpha * sin(angle)) + (beta * cos(angle))
    return (Ds, Qs)
}


// Inverse Park Transform
// This transformation projects vectors in orthogonal rotating reference frame into two phase orthogonal stationary frame.

public func iPark(Ds: Double, Qs: Double, angle: Double) -> (Double, Double) {
    let alpha = (Ds * cos(angle)) - (Qs * sin(angle))
    let beta = (Ds * sin(angle)) + (Qs * cos(angle))
    return (alpha, beta)
}


// PID 
// This module implements a 32-bit digital PID controller with anti-windup correction.
// 

/*
PID Terminals
Ref     - Input: reference set-point
Fbk     - Input: feedback
Out     - Output: controller output
c1      - Internal: derivative filter coefficient
c2      - Internal: derivative filter coefficient
Iae     - Output: performance index
Err     - Internal: servo error

PID Parameters
Kr      - Parameter: reference set-point weighting
Kp      - Parameter: proportional loop gain
Ki      - Parameter: integral gain
Kd      - Parameter: derivative gain
Km      - Parameter: derivative weighting
Umax    - Parameter: upper saturation limit
Umin    - Parameter: lower saturation limit
Kiae    - Parameter: IAE scaling

PID Data
up      - Data: proportional term
ui      - Data: integral term
ud      - Data: derivative term
v1      - Data: pre-saturated controller output
i1      - Data: integrator storage: ui(k-1)
d1      - Data: differentiator storage: ud(k-1)
d2      - Data: differentiator storage: d2(k-1)
w1      - Data: saturation record: [u(k-1) - v(k-1)]

*/

// The user needs access to the parameters. This is what defines the particular PID loop. To execute an iteration of the PID loop you must provide the inputs and you expect to receive the outputs. The question is where to the mutating bits of data go?

public struct PIDParameters {
    var Kr: Double = 1.0        // reference set-point weighting
    public var Kp: Double = 1.0        // proportional loop gain
    public var Ki: Double = 0.0        // integral gain
    public var Kd: Double = 0.0        // derivative gain
    var Km: Double = 0.0        // derivitive weighting
    
    var Umax: Double = 100.0    // Upper saturation limit
    var Umin: Double = -100.0   // Lower saturation limit
    var Kiae: Double = 1.0      // IAE Scaling
}

public struct PIDData {
    var up: Double = 0.0        // proportional term
    var ui: Double = 0.0        // integral term
    var ud: Double = 0.0        // derivative term
    var v1: Double = 0.0        // pre-saturated controller output
    var i1: Double = 0.0        // integrator storage: ui(k-1)
    var d1: Double = 0.0        // differentiator storage: ud(k-1)
    var d2: Double = 0.0        // differentiator storage: d2(k-1)
    var w1: Double = 0.0        // saturation record: [u(k-1) - v(k-1)]
    var c1: Double = 0.0   		// derivative filter coefficient 1
    var c2: Double = 0.0   		// derivative filter coefficient 2
}

public class GrandoPID {
    public var parameters: PIDParameters
    var data: PIDData
    
    public init() {
        parameters = PIDParameters()
        data = PIDData()
    }
    
    public func updateWithInput(reference: Double, feedback: Double) -> Double {
        // Proportional term
        data.up = (parameters.Kr * reference) - feedback
        
        // Intergral term
        data.ui = (parameters.Ki * (data.w1 * (reference - feedback))) + data.i1
        data.i1 = data.ui
        
        // Derivitive term
        data.d2 = (parameters.Kd * (data.c1 * ((reference * parameters.Km) - feedback))) - data.d2
        data.ud = data.d2 + data.d1
        data.d1 = data.ud * data.c2
        
        // Control Output
        data.v1 = (parameters.Kp * (data.up + data.ui + data.ud))
        let output = max(min(data.v1, parameters.Umax), parameters.Umin)
        data.w1 = (output == data.v1) ? 1.0 : 0.0
        
        return output
    }
}



/*
/* proportional term */ 																		\
v.data.up = _IQmpy(v.param.Kr, v.term.Ref) - v.term.Fbk;										\
\
/* integral term */ 																			\
v.data.ui = _IQmpy(v.param.Ki, _IQmpy(v.data.w1, (v.term.Ref - v.term.Fbk))) + v.data.i1;		\
v.data.i1 = v.data.ui;																			\
\
/* derivative term */ 																			\
v.data.d2 = _IQmpy(v.param.Kd, _IQmpy(v.term.c1, (_IQmpy(v.term.Ref, v.param.Km) - v.term.Fbk))) - v.data.d2;	\
v.data.ud = v.data.d2 + v.data.d1;																\
v.data.d1 = _IQmpy(v.data.ud, v.term.c2);														\
\
/* control output */ 																			\
v.data.v1 = _IQmpy(v.param.Kp, (v.data.up + v.data.ui + v.data.ud));							\
v.term.Out= _IQsat(v.data.v1, v.param.Umax, v.param.Umin);										\
v.data.w1 = (v.term.Out == v.data.v1) ? _IQ(1.0) : _IQ(0.0);
*/


/*

let speedLoopController = PID(Kp: 0.2, Ki: 0.0005, Kd: 0.0)

while(running) {
    let output = speedLoopController.loop(ref: 0.1, feedback: 0.5)
    
    motors.speed = output
}


speedLoopController.Ki *= 2
speedLoopController.reset()


while(running) {
let output = speedLoopController.loop(ref: 0.1, feedback: 0.5)

motors.speed = output
}


*/






