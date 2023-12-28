/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.subsystems.drive.swerve

import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Acceleration
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

class SwerveModuleConstants {
    var kName = "Name"
    var kDriveId = -1
    var kAzimuthId = -1
    var kCanCoderId = -1

    var kCanCoderNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)

    // general azimuth
    var kInvertAzimuth = false
    var kInvertAzimuthSensorPhase = false
    var kAzimuthBrakeMode = true // neutral mode could change

    //        var kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI) // for azimuth
    var kAzimuthNativeUnitModel = NativeUnitRotationModel(2048.nativeUnits)
    var kAzimuthEncoderHomeOffset = 0.0

    // azimuth motion
    var kAzimuthKp = 1.3
    var kAzimuthKi = 0.05
    var kAzimuthKd = 20.0
    var kAzimuthKf = 0.5421
    var kAzimuthIZone = 25.0
    var kAzimuthCruiseVelocity = SIUnit<Velocity<Radian>>(2.6) // 1698 native units
    var kAzimuthAcceleration = SIUnit<Acceleration<Radian>>(31.26) // 20379 Native Units | 12 * kAzimuthCruiseVelocity
    var kAzimuthClosedLoopAllowableError = 5

    var kDriveKs = 0.0
    var kDriveKv = 0.0
    var kDriveKa = 0.0

    // azimuth current/voltage
    var kAzimuthCurrentLimit = 60.0 // amps
    var kAzimuthEnableCurrentLimit = true

    // general drive
    var kInvertDrive = true
    var kDriveBrakeMode = true // neutral mode could change
    var kWheelDiameter = 4.0 // Probably should tune for each individual wheel maybe
    var kDriveNativeUnitModel = NativeUnitLengthModel(4096.nativeUnits, kWheelDiameter.inches)
    var kDriveKp = 0.0

    // var kDriveMaxSpeed = 10.0
    var kDriveMaxSpeed = 0.0

    // drive current/voltage
    var kDriveCurrentLimit = 50.0 // amps
    var kDriveEnableCurrentLimit = true
    var kDriveMaxVoltage = 11.0 // volts
}
