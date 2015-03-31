//: Playground - noun: a place where people can play

import Cocoa


toRadians(180)
toDegrees(M_PI)


for t in 1...80 {
    let u = Double(t * (640/80))
    let phaseA = cos(toRadians(u))
    let phaseB = cos(toRadians(u - 120))
    let phaseC = cos(toRadians(u - 240))
    let (alpha, beta) = clarke(phaseA, phaseB, phaseC)
    alpha
    beta
    let (alpha2, beta2) = clarke(phaseA, phaseB)
    alpha2
    beta2
    let (Ds, Qs) = park(alpha, beta, toRadians(u))
    Ds
    Qs
    let (alpha3, beta3) = iPark(Ds, Qs, toRadians(u))
    alpha3
    beta3
}



