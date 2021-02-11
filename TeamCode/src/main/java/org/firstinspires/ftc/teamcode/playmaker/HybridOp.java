package org.firstinspires.ftc.teamcode.playmaker;

public interface HybridOp {
    /**
     * Called with every loop while in autonomous
     */
    void autonomous_loop();

    /**
     * Called with every loop while in teleop
     */
    void teleop_loop();

    /**
     * Called every loop regardless if auto/teleop
     */
    void hybrid_loop();

}
