/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.utils

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Watchdog
import kotlinx.coroutines.CoroutineDispatcher
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.CoroutineStart
import kotlinx.coroutines.Job
import kotlinx.coroutines.asCoroutineDispatcher
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.yield
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.wrappers.FalconNotifier
import java.util.concurrent.Executors
import kotlin.coroutines.CoroutineContext

val FalconDispatcher: CoroutineDispatcher = Executors.newFixedThreadPool(2).asCoroutineDispatcher()

fun CoroutineScope.launchFrequency(
    frequency: Int = 50,
    context: CoroutineContext = FalconDispatcher,
    start: CoroutineStart = CoroutineStart.DEFAULT,
    block: suspend CoroutineScope.() -> Unit,
): Job {
    if (frequency <= 0) throw IllegalArgumentException("Frequency cannot be lower then 1!")
    return launch(context, start) {
        loopFrequency(frequency, block)
    }
}

suspend fun CoroutineScope.loopFrequency(
    frequency: Int = 50,
    block: suspend CoroutineScope.() -> Unit,
) {
    val notifier = FalconNotifier(frequency)
    notifier.updateAlarm()

    while (isActive) {
        notifier.waitForAlarm()
        block(this)
        notifier.updateAlarm()
    }
}

class PeriodicScope @PublishedApi internal constructor(val period: Double) {
    @PublishedApi
    internal var isDone = false

    fun stop() {
        isDone = true
    }
}

suspend inline fun periodic(
    period: Double = 0.02,
    watchOverrun: Boolean = false,
    crossinline body: PeriodicScope.() -> Unit,
) {
    val scope = PeriodicScope(period)

    val watchDog = if (watchOverrun) {
        Watchdog(period) {
            DriverStation.reportError("Periodic loop overrun", true)
        }
    } else {
        null
    }

    while (true) {
        watchDog?.reset()
        val dt = measureTimeFPGA {
            body(scope)
        }
        if (scope.isDone) break
        val remainder = period - dt
        if (remainder <= 0.0) {
            yield()
        } else {
            delay(remainder.toLong())
        }
    }
}

suspend inline fun periodic(
    period: SIUnit<Second>,
    watchOverrun: Boolean = false,
    crossinline body: PeriodicScope.() -> Unit,
) = periodic(period.value, watchOverrun, body)
