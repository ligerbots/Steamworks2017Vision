package erik.android.vision.visiontest;

import android.app.AlarmManager;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.os.Environment;
import android.util.Log;
import java.io.File;

public class CrashRestarter extends Thread {
    private Camera2Activity instance;
    private AlarmManager alarmManager;

    public CrashRestarter(Camera2Activity instance) {
        this.instance = instance;
        alarmManager = (AlarmManager) instance.getSystemService(Context.ALARM_SERVICE);
        setName("Crash Restarter");
        setDaemon(true);
        
        if(new File(Environment.getExternalStorageDirectory(), "vision_no_restart").exists()) {
            return;
        }
        
        loop(); // if we crash on init, it needs to run once asap
        start();
    }

    @Override
    public void run() {
        while(true) {
            loop();
            try {
                Thread.sleep(4000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void loop() {
        Log.i("CrashRestarter", "Setting alarm");
        Intent intent = new Intent(instance, AlarmReceiver.class);
        PendingIntent restartIntent = PendingIntent.getBroadcast(instance, 2877, intent, 0);
        alarmManager.setExact(AlarmManager.RTC_WAKEUP, System.currentTimeMillis() + 5000, restartIntent);
    }
}
