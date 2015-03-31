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


