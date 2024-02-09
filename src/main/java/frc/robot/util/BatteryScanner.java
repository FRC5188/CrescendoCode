package frc.robot.util;


import edu.wpi.first.wpilibj.SerialPort;
import java.util.Arrays;


public class BatteryScanner {
    public static final String DEFAULT_NAME = "0000-000";

     private static final int NAME_LENGTH = 8;
     private static final byte[] SCAN_COMMAND =
      new byte[] {0x7e, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, (byte) 0xab, (byte) 0xcd};
     private static final byte[] RESPONSE_PREFIX =
      new byte[] {0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31};
     private static final int FULL_RESPONSE_LENGTH = RESPONSE_PREFIX.length + NAME_LENGTH;

  private static String name = DEFAULT_NAME;

  /**
   * Scans the battery. This should be called before the first loop cycle
   *
   * @param timeout The time to wait before giving up
   */
  public static String scanBattery(double timeout) {

        // Only scan on supported robots and in real mode

        try (SerialPort port = new SerialPort(9600, SerialPort.Port.kUSB)) {
          port.setTimeout(timeout);
          port.setWriteBufferSize(SCAN_COMMAND.length);
          port.setReadBufferSize(FULL_RESPONSE_LENGTH);

          port.write(SCAN_COMMAND, SCAN_COMMAND.length);
          byte[] response = port.read(FULL_RESPONSE_LENGTH);

          // Ensure response is correct length
          if (response.length != FULL_RESPONSE_LENGTH) {
            System.out.println(
                "[BatteryTracker] Expected "
                    + FULL_RESPONSE_LENGTH
                    + " bytes from scanner, got "
                    + response.length);
            return name;
          }

          // Ensure response starts with prefix
          for (int i = 0; i < RESPONSE_PREFIX.length; i++) {
            if (response[i] != RESPONSE_PREFIX[i]) {
              System.out.println("[BatteryTracker] Invalid prefix from scanner.  Got data:");
              System.out.println("[BatteryTracker] " + Arrays.toString(response));
              return name;
            }
          }

          // Read name from data
          byte[] batteryNameBytes = new byte[NAME_LENGTH];
          System.arraycopy(response, RESPONSE_PREFIX.length, batteryNameBytes, 0, NAME_LENGTH);
          name = new String(batteryNameBytes);
          System.out.println("[BatteryTracker] Scanned battery " + name);

        } catch (Exception e) {
          System.out.println("[BatteryTracker] Exception while trying to scan battery");
          e.printStackTrace();
        }
         return name;
      }
    

   
  

  /** Returns the name of the last scanned battery. */
  public static String getName() {
    return name;
  }
}
