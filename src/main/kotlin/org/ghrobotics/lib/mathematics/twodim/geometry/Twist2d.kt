/*
 * FRC Team 5190
 * Green Hope Falcons
 */

/*
 * Some implementations and algorithms borrowed from:
 * NASA Ames Robotics "The Cheesy Poofs"
 * Team 254
 */


package org.ghrobotics.lib.mathematics.twodim.geometry

import org.ghrobotics.lib.mathematics.kEpsilon
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.math.absoluteValue


class Twist2d(
    val dxRaw: Double,
    val dyRaw: Double,
    val dThetaRaw: Double
) {

    constructor(
        dx: Length,
        dy: Length,
        dTheta: Rotation2d
    ) : this(
        dx.asMetric.asDouble,
        dy.asMetric.asDouble,
        dTheta.radians
    )

    val dx
        get() = dxRaw.meter
    val dy
        get() = dyRaw.meter
    val dTheta
        get() = dThetaRaw.radians

    val norm
        get() = if (dyRaw == 0.0) dxRaw.absoluteValue else Math.hypot(dxRaw, dyRaw)

    val asPose: Pose2d
        get() {
            val sinTheta = Math.sin(dThetaRaw)
            val cosTheta = Math.cos(dThetaRaw)

            val (s, c) = if (Math.abs(dThetaRaw) < kEpsilon) {
                1.0 - 1.0 / 6.0 * dThetaRaw * dThetaRaw to .5 * dThetaRaw
            } else {
                sinTheta / dThetaRaw to (1.0 - cosTheta) / dThetaRaw
            }
            return Pose2d(
                Translation2d(dxRaw * s - dyRaw * c, dxRaw * c + dyRaw * s),
                Rotation2d(cosTheta, sinTheta, false)
            )
        }

    operator fun times(scale: Double) =
        Twist2d(dxRaw * scale, dyRaw * scale, dThetaRaw * scale)

}