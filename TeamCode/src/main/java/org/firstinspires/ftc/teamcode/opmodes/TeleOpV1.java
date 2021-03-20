package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.logger.Loggable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OperatorInterface;
import org.firstinspires.ftc.teamcode.Robot;

/** Main OpMode
 *
 */
@TeleOp(name = "TeleOpV1")
public class TeleOpV1 extends CommandOpMode implements Loggable {
    /** The robot
     *
     */
    public Robot robot;
    /** The Operator interface
     *
     */
    public OperatorInterface operatorInterface;

    @Override
    public void uponInit() {
        robot = new Robot();
        operatorInterface = new OperatorInterface(driverGamepad, codriverGamepad, robot);
        super.telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
    }

}