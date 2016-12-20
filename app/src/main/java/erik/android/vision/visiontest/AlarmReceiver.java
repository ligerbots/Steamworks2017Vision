package erik.android.vision.visiontest;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.util.Log;

/**
 * This is a BroadcastReceiver that starts the main activity when it's triggered. It's triggered
 * by the 5 second alarm that is constantly set while the app is running
 * @see CrashRestarter
 */
public class AlarmReceiver extends BroadcastReceiver {
    public AlarmReceiver() {
    }

    @Override
    public void onReceive(Context context, Intent intent) {
        Log.i("AlarmReceiver", "onReceive called");
        Intent myIntent = new Intent(context, Camera2Activity.class);
        myIntent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
        context.startActivity(myIntent);
    }
}
