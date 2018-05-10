package com.agendel.SmartLight;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

/* Most of the code in this file is based on
http://jasonmcreynolds.com/?p=388
 */

public class ShakeDetector implements SensorEventListener {

    /*
     * The gForce that is necessary to register as shake.
     * Must be greater than 1G (one earth gravity unit).
     */
    private static final float SHAKE_THRESHOLD_GRAVITY = 2.7F;
    private static final int SHAKE_SLOP_TIME_MS = 500;
    private static final int MOVE_MIN = -15;
    private static final int MOVE_MAX = 10;

    private float[] min = new float[3];
    private float[] max = new float[3];

    private OnShakeListener mListener;
    private long mShakeTimestamp;

    public void setOnShakeListener(OnShakeListener listener) {
        this.mListener = listener;
    }

    public interface OnShakeListener {
        public void onShake();
        public void onMove(int[] values);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // ignore
    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        if (mListener != null) {
            float x = event.values[0];
            float y = event.values[1];
            float z = event.values[2];

            for (int i = 0; i <= 2;) {
                if(min[i]> event.values[i]) {
                    min[i] = event.values[i];
                }
                if(max[i]< event.values[i]) {
                    max[i] = event.values[i];
                }
              i = i +1;
            }

            float gX = x / SensorManager.GRAVITY_EARTH;
            float gY = y / SensorManager.GRAVITY_EARTH;
            float gZ = z / SensorManager.GRAVITY_EARTH;

            // gForce will be close to 1 when there is no movement.
            float gForce = (float)Math.sqrt(gX * gX + gY * gY + gZ * gZ);

            if (gForce > SHAKE_THRESHOLD_GRAVITY) {
                final long now = System.currentTimeMillis();
                // ignore shake events too close to each other (500ms)
                if (mShakeTimestamp + SHAKE_SLOP_TIME_MS > now) {
                    return;
                }

                mShakeTimestamp = now;
                mListener.onShake();
            } else {
                int [] x_c = new int[3];
                for (int i = 0; i <=2; i ++) {
                    x_c[i] = map(event.values[i], MOVE_MIN, MOVE_MAX, 0, 255);
                }

                    mListener.onMove( x_c);
                }
            }

            for (int i = 0; i <= 2;) {
                Log.e("Max"+String.valueOf(i), String.valueOf(max[i]));
                Log.e("Min"+String.valueOf(i), String.valueOf(min[i]));
                i = i +1;
            }
        }


    public int map(float x, float in_min, float in_max, int out_min, int out_max)
    {
        int result =  Math.round((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
        return (result > out_max) ? out_max : (result < out_min ? out_min: result );
    }


}