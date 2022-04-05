package frc.robot;

import java.io.ByteArrayOutputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.net.URLConnection;
import java.nio.file.*;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

public class Snapshot {

    private static final Thread m_thread;
    private static final ZoneId m_utc = ZoneId.of("UTC");
    private static final DateTimeFormatter m_timeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmssSSS")
            .withZone(m_utc);

    private static boolean isUsb = false;
    private static boolean takeSnap = false;
    private static String m_cameraPath = null;
    private static String filePrefix = "FRC";

    static {
        m_thread = new Thread(Snapshot::logMain, "Snapshot");
        m_thread.setDaemon(true);
    }

    public static synchronized void start(String cameraPath) {
        m_thread.start();
        m_cameraPath = cameraPath;
    }

    private static String makeLogDir(String dir) {
        // from DataLogManager
        if (!dir.isEmpty()) {
            return dir;
        }

        isUsb = false;
        if (RobotBase.isReal()) {
            try {
                // prefer a mounted USB drive if one is accessible
                Path usbDir = Paths.get("/u").toRealPath();
                if (Files.isWritable(usbDir)) {
                    isUsb = true;
                    return usbDir.toString();
                }
            } catch (IOException ex) {
                // ignored
            }
        } else {
            // if in simulation, assume we have a USB
            isUsb = true;
        }

        return Filesystem.getOperatingDirectory().getAbsolutePath();
    }

    private static void logMain() {
        ByteArrayOutputStream jpgOut = new ByteArrayOutputStream(100000);

        while (!Thread.interrupted()) {
            DriverStation.waitForData(0.25);
            if (Thread.interrupted()) {
                break;
            }

            if (takeSnap) {
                takeSnap = false;
                String path = makeLogDir("");
                if (!isUsb) {
                    DriverStation.reportWarning("Unable to take snapshot, no USB stick!",false);
                    continue;
                }
                LocalDateTime now = LocalDateTime.now(m_utc);
                String fileName = filePrefix + "_" + m_timeFormatter.format(now) + ".jpg";

                try {
                    int prev = 0;
                    int cur = 0;
                    
                    URL url = new URL(m_cameraPath);
                    URLConnection uc = url.openConnection();
                    InputStream inputStream = uc.getInputStream();
                    
                    while ((inputStream != null)
                            && ((cur = inputStream.read()) >= 0)) {
                        if (prev == 0xFF && cur == 0xD8) {
                            //start of jpeg
                            jpgOut.reset();
                            jpgOut.write((byte) prev);
                        }
                        if (jpgOut != null) {
                            jpgOut.write((byte) cur);
                            //jpeg finished
                            if (prev == 0xFF && cur == 0xD9) {
                                break;
                            }
                        }
                        prev = cur;
                    }
                    inputStream.close();

                    if(jpgOut.size() > 0) {
                        FileOutputStream writer = new FileOutputStream(path + "/" + fileName);
                        jpgOut.writeTo(writer);
                        writer.close();
                        DriverStation.reportWarning("Wrote snapshot to: " + fileName,false);
                    }
                } catch (Exception e) {
                    DriverStation.reportWarning("Unable to take snapshot!",false);
                    e.toString();
                }
            }
        }
    }

    public static void TakeSnapshot() {
        TakeSnapshot("FRC");
    }

    public static void TakeSnapshot(String prefix) {
        takeSnap = true;
        filePrefix = prefix;
    }
}