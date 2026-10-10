export type HandlingMode = 'rally' | 'refined' | 'baseline';
export type VehicleKind = 'sti' | 'truck';
export type DriveInput = {steer:number; throttle:number; brake:number; handbrake:number};
export type V3 = {x:number; y:number; z:number};
export const VEHICLES = {
  sti: {
    name:'Subaru Impreza WRX STI', mass:1520, scale:1,
    front:-1.27495, back:1.30635, track:1.52434, radius:.31424,
    suspension:.42, mount:-.1, springRate:48000, compressionDamping:4000, reboundDamping:4700,
    suspensionTravel:.23, centerOfMass:-.12, frontAntiRoll:4800, rearAntiRoll:5400,
    // Centre of mass ahead of the body origin (m, -z): ~60/40 front/rear static
    // weight, as the road car. The front-heavy split is what keeps the balance
    // in understeer now that front and rear run the same tire curve.
    centerOfMassForward:-.24,
    powertrain:{engineHorsepower:340,engineMaxRPM:7000,idleRPM:900,gearRatios:[3.636,2.375,1.761,1.346,1.062,.842],finalDriveRatio:3.9,shiftUpRPM:6500,shiftDownRPM:2600,shiftCooldown:.35,reverseRatio:3.545,reverseTorqueScale:.38,engineInertia:.2,clutchTorque:620,launchRPM:4500,shiftTime:.11},
    brake:26000, frontDrive:.43, frontBrake:.6, wheelInertia:1.5,
    differential:{front:{antiSlip:160,torqueSensitivity:.075,coastFactor:.3,split:.5},rear:{antiSlip:250,torqueSensitivity:.15,coastFactor:.55,split:.5},center:{antiSlip:900,torqueSensitivity:0,coastFactor:.25,split:.57}},
    // Corner-balance setup: torque-sensitive center locking releases on coast;
    // softer front/coast coupling lets the loaded front tires turn the car.
    rallyDifferential:{front:{antiSlip:160,torqueSensitivity:.05,coastFactor:.18,split:.5},rear:{antiSlip:250,torqueSensitivity:.15,coastFactor:.35,split:.5},center:{antiSlip:900,torqueSensitivity:.4,coastFactor:.65,split:.57}},
    steering:.55, steerRate:10, steerReturn:13, throttleRate:4.8,
    // Half contact-patch length and caster trail (m) set the aligning torque.
    tirePatch:.09, casterTrail:.024,
    steeringRack:{inertia:2.5,damping:50,handStiffness:60000,handDamping:550,handTorque:1400,relaxedStiffness:600,slipShare:.99,fullLockSpeed:3,slipLockSpeed:25},
    // Contact-patch relaxation lengths (m): distance rolled for slip to build.
    tireRelaxationLength:{lateral:.32,longitudinal:.12},
    corneringAcceleration:6.5, tireGrip:1.02, engineBraking:.45,
    maxSpeed:69.44, length:4.48, width:2.08605,
    camera:{distance:6.65, height:2.15, headingRate:6.5, heightRate:5, hoodForward:1.32, hoodHeight:.86},
  },
  truck: {
    name:'Ford F-150', mass:2450, scale:1,
    front:-1.91953, back:1.60088, track:1.73609, radius:.43383,
    suspension:.46, mount:-.1, springRate:50000, compressionDamping:4900, reboundDamping:5600,
    suspensionTravel:.34, centerOfMass:-.10, frontAntiRoll:4200, rearAntiRoll:4600,
    // ~57/43 front/rear static weight, as the unladen pickup.
    centerOfMassForward:-.41,
    powertrain:{engineHorsepower:430,engineMaxRPM:6500,idleRPM:800,gearRatios:[4.696,2.985,2.146,1.769,1.52,1.275,1,.854,.689,.636],finalDriveRatio:3.55,shiftUpRPM:5900,shiftDownRPM:2400,shiftCooldown:.4,reverseRatio:4.866,reverseTorqueScale:.32,engineInertia:.38,clutchTorque:880,launchRPM:3300,shiftTime:.22},
    brake:32000, frontDrive:.4, frontBrake:.62, wheelInertia:3.2,
    differential:{front:{antiSlip:120,torqueSensitivity:.065,coastFactor:.25,split:.5},rear:{antiSlip:400,torqueSensitivity:.2,coastFactor:.35,split:.5},center:{antiSlip:400,torqueSensitivity:.11,coastFactor:.2,split:.6}},
    rallyDifferential:{front:{antiSlip:120,torqueSensitivity:.045,coastFactor:.18,split:.5},rear:{antiSlip:400,torqueSensitivity:.2,coastFactor:.25,split:.5},center:{antiSlip:400,torqueSensitivity:.2,coastFactor:.18,split:.6}},
    steering:.49, steerRate:9, steerReturn:11, throttleRate:3.1,
    tirePatch:.11, casterTrail:.03,
    steeringRack:{inertia:3.5,damping:85,handStiffness:75000,handDamping:720,handTorque:2000,relaxedStiffness:700,slipShare:.99,fullLockSpeed:3,slipLockSpeed:25},
    tireRelaxationLength:{lateral:.4,longitudinal:.15},
    corneringAcceleration:5.8, tireGrip:.93, engineBraking:.45,
    maxSpeed:55.56, length:5.62207, width:2.33579,
    camera:{distance:8.15, height:2.7, headingRate:4.6, heightRate:3.8, hoodForward:1.8, hoodHeight:1.12},
  },
};
