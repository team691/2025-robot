package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    // Initialize a Network Table Instance
    private NetworkTable tableInstance = NetworkTableInstance.getDefault()
    .getTable("limelight");

    // Limelight target boolean
    public boolean vTar = false;

    // Send the data to SmartDashboard
    public static final boolean postSmartDashboard = true;

    // Verify connection to SmartDashboard
    public static boolean isConnected(boolean connected) {
        if (postSmartDashboard) {
            SmartDashboard.putBoolean("Limelight Connected", true);
        }
        return connected;
    }

    // Position values for Network Tables based off pipeline

    public double PosX() {
        return tableInstance.getEntry("tx").getDouble(0.0);
    }

    public double PosY() {
        return tableInstance.getEntry("ty").getDouble(0.0);
    }

    public double PosArea() {
        return tableInstance.getEntry("ta").getDouble(0.0);
    }

    public double PosSkew() {
        return tableInstance.getEntry("ts").getDouble(0.0);
    }

    public double PosHor() {
        return tableInstance.getEntry("thor").getDouble(0.0);
    }

    public double PosVert() {
        return tableInstance.getEntry("tvert").getDouble(0.0);
    }

    public double tagId() {
        return tableInstance.getEntry("tid").getDouble(0.0);
    }

    public enum LEDMode {
        // follow pipeline mode
        PIPELINE(0),
        // force LEDs off
        FORCE_OFF(1),
        // force LEDs to blink
        FORCE_BLINK(2),
        // force LEDs on
        FORCE_ON(3);
    
        LEDMode(int value) {
            this.val = value;
        }
        
        public int getCodeValue() {
            return val;
        }

        private int val;
    }    
    
    // Initialize instance of LED modes in Network Tables
    NetworkTableEntry LEDModeEntry = tableInstance.getEntry("ledMode");
    
    // LED Mode
    public final void setLEDMode(LEDMode mode) 
    {
        LEDModeEntry.setNumber(mode.getCodeValue());
    }

    // Check for valid target (disregard is the range/area where target should be ignored)
    // TODO: verify this works???

    // TODO: EDIT LIMELIGHT HARDWARD CONTOURS, SPECIFICALLY THE SORT MODE AND SPECKLES

    public void validTarget(double id) {
        if (tagId() == id)
        {
            vTar = true;
        }
        else
        {
            vTar = false;
        }
    }
}
