/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.utils

val <T> Source<T>.monitor get() = SourceMonitor(this)

class SourceMonitor<T>(
    val source: Source<T>,
) {
    var lastValue = source()

    inline fun onChange(block: (T) -> Unit) {
        val newValue = source()
        if (newValue != lastValue) block(newValue)
        lastValue = newValue
    }

    inline fun onWhen(value: T, block: () -> Unit) {
        if (lastValue == value) block()
        onChange { if (it == value) block() }
    }
}

fun SourceMonitor<Boolean>.onChangeToTrue(block: () -> Unit) = onChange { if (it) block() }
fun SourceMonitor<Boolean>.onChangeToFalse(block: () -> Unit) = onChange { if (!it) block() }
fun SourceMonitor<Boolean>.onWhenToTrue(block: () -> Unit) = onWhen(true, block)
fun SourceMonitor<Boolean>.onWhenToFalse(block: () -> Unit) = onWhen(false, block)
