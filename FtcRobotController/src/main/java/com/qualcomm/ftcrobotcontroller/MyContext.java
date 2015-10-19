package com.qualcomm.ftcrobotcontroller;
import android.content.Context;

/**
 * Created by Phil on 8/9/2015.
 *
 */
public class MyContext
{
    private static Context mContext;
    /**
     * Constructor
     */
    public MyContext(Context me) {
        if (me != null)
            mContext = me ;
    }

    public Context getContext() {
        return mContext;
    }
}