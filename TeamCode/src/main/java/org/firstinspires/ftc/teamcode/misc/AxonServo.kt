package org.firstinspires.ftc.teamcode.misc

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import kotlin.math.abs

/**
 * Utility class for reading positional data from an Axon Servo Encoder. Axons are unique in that
 * they have a secondary, 1-pin wire representing the movement from absolute zero rotation.
 *
 * Please call [cleanUp] after the object is no longer needed to avoid floating threads.
 */
class AxonServo(private val servo: CRServo, private val channel: AnalogInput): CRServo by servo {
    // magic number
    private val toleranceDegrees = 5.0

    /**
     * The rotational difference between starting position and absolute zero, in degrees.
     */
    val initDelta: Double = channel.voltage / 3.3 * 360

    /**
     * The rotational difference between the starting and current positions, in degrees.
     */
    var positionDegrees: Double
        get() = posInternal
        set(new) {
            synchronized(targetPosLock) {
                targetPos = new
            }
        }

    /**
     * The rotational difference between the starting and current positions, normalized
     * between a variable range of 360 degrees (always limited between [-360, 360]).
     */
    var normalizedPosition
        get() = (channel.voltage / 3.3 * 360) - initDelta
        // todo: Verify always goes to closest
        // fixme: inconsistent range = inconsistent results when calling
        set(new) {
            synchronized(targetPosLock) {
                targetPos = posInternal + (new - normalizedPosition)
            }
        }

    override fun setPower(power: Double) {
        synchronized(targetPosLock){
            targetPos = Double.NaN // Clear our target so we aren't overriding them setting their power
            servo.power = power
        }
    }

    private var posInternal: Double = 0.0

    /**
     * Double.NaN if no current target
     */
    private var targetPos: Double = Double.NaN
    private val targetPosLock = Any() // targetPos has to be synchronized to avoid a case where
    // power is set by the user but quickly overwritten by the thread that has already progressed past the check
    // todo: is this the best approach?

    companion object {
        private var warningMessageSet = false
    }

    private var thread = Thread {
        var normalizedPosLastCycle = normalizedPosition
        val timer = ElapsedTime()
        while (!Thread.currentThread().isInterrupted) {
            if (timer.milliseconds() > 250 && !warningMessageSet){
                /*
                * @ 4.8v it takes 140ms to move 60deg. Because we can effectively track 180deg
                * of rotation in a single loop, 250ms is a fair warning level.
                * We want to warn because we could have lost rotation, offsetting automatic controls.
                */
                RobotLog.addGlobalWarningMessage("Warning: Servo encoder tracking loop has gone on for more than 250ms. Servo positional data risks being incorrect and affecting automatic controls. Please report this to a programmer.")
                warningMessageSet = true
            }
            timer.reset()

            val currentNormalizedPosition = normalizedPosition
            // Determine the delta between the current position and the previous position
            val normalizedDelta = currentNormalizedPosition - normalizedPosLastCycle

            var trueDelta = normalizedDelta
            if (normalizedDelta > 180){ // underflow has occurred (i.e. 10 -> 350)
                // 10 -> 350, 340 delta, -20 true delta
                trueDelta -= 360
            } else if (normalizedDelta < -180) {
                // 350 -> 10, -340 delta, 20 true delta
                trueDelta += 360
            }

            // Save the difference to a variable counting our overall rotation
            posInternal += trueDelta
            normalizedPosLastCycle = currentNormalizedPosition

            synchronized(targetPosLock){
                if (!targetPos.isNaN()) {
                    // todo: pid? reference: https://github.com/The-Robotics-Catalyst-Foundation/FIRST-Opensource/blob/main/FTC/RTPAxon/RTPAxon.java
                    // Always run towards the target unless at tolerance, in which case remove the target
                    if (abs(targetPos - posInternal) < toleranceDegrees) {
                        targetPos = Double.NaN
                    } else if (targetPos > posInternal) {
                        servo.power = 0.5
                    } else {
                        servo.power = -0.5
                    }
                }
            }

            try {
                Thread.sleep(50)
            } catch (ignored: InterruptedException) {}
        }
    }.apply { start() }

    fun cleanUp() = thread.interrupt()
}