package erik.android.vision.visiontest;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;

/**
 * This is the BroadcastReceiver that is triggered on boot and starts the app
 */
public class BootReceiver extends BroadcastReceiver {
    public BootReceiver() {
    }

    @Override
    public void onReceive(Context context, Intent intent) {
        Intent myIntent = new Intent(context, Camera2Activity.class);
        myIntent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
        context.startActivity(myIntent);
    }
}
