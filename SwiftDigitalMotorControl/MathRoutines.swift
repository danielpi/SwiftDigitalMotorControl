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

