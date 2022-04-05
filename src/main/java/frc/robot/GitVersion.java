package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;

public class GitVersion implements Serializable {
    static final long SERIAL_VERSION_UID = 12345;
    public String LastCommit;
    public boolean IsModified;
    public Date BuildDate;
    public String BuildAuthor;

    public static GitVersion loadVersion() {
        String path = Filesystem.getDeployDirectory() + "/gitinfo.obj";
        GitVersion obj;

        try {
            FileInputStream fileInputStream = new FileInputStream(path);
            ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream);
            obj = (GitVersion) objectInputStream.readObject();
            objectInputStream.close();
        } catch (Exception e) {
            // generic catch is usually bad, but here we are using it to create a default
            // whenever there is an issue loading it
            obj = new GitVersion();
            obj.BuildDate = new Date();
            obj.IsModified = false;
            obj.BuildAuthor = "Unknown";
            obj.LastCommit = "Unknown";
        }
        return obj;
    }

    // this main function should only be called from Gradle
    public static void main(String[] args) {
        try {
            // setup the commands to run
            GitVersion result = new GitVersion();
            Runtime rt = Runtime.getRuntime();
            Process pr;
            result.BuildDate = new Date();

            // get the user who made the commit
            pr = rt.exec("git config user.name");
            pr.waitFor();
            result.BuildAuthor = new String(pr.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            // remove newline at end of name
            result.BuildAuthor = result.BuildAuthor.substring(0, result.BuildAuthor.length() - 1);

            // run git log to get the last commits hash
            pr = rt.exec("git log -1 --pretty=tformat:%h");
            pr.waitFor();
            result.LastCommit = new String(pr.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            result.LastCommit = result.LastCommit.substring(0, 7);

            // get the status to see if any files have changed
            pr = rt.exec("git status -s");
            String temp = new String(pr.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            result.IsModified = temp.length() != 0;

            // write object file
            FileOutputStream fileOutputStream = new FileOutputStream("src/main/deploy/gitinfo.obj");
            ObjectOutputStream objectOutputStream = new ObjectOutputStream(fileOutputStream);
            objectOutputStream.writeObject(result);
            objectOutputStream.flush();
            objectOutputStream.close();

        } catch (IOException | InterruptedException e) {
            System.exit(1);
        }

        System.exit(0);
    }

    public void printVersions() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SW Version");
        table.getEntry("Build Date").setString(new SimpleDateFormat("MM/dd/yyyy HH:mm:ss").format(BuildDate));
        table.getEntry("Build Author").setString(BuildAuthor);
        table.getEntry("Current Commit").setString(LastCommit);
        table.getEntry("Modified").setBoolean(IsModified);
    }
}
