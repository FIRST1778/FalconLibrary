/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.utils

import edu.wpi.first.wpilibj.Timer

fun measureTimeFPGA(block: () -> Unit): Double {
    val start = Timer.getFPGATimestamp()
    block()
    return Timer.getFPGATimestamp() - start
}
