/***************************************************************************
 *   Copyright (C) 2013 by Ralf Kaestner                                   *
 *   ralf.kaestner@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

package ch.ethz.naro;

import com.google.common.base.Preconditions;

import android.app.Activity;
import android.app.DialogFragment;
import android.app.AlertDialog;
import android.content.SharedPreferences;
import android.content.SharedPreferences.OnSharedPreferenceChangeListener;
import android.content.Intent;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.drawable.Drawable;
import android.graphics.drawable.LevelListDrawable;
import android.graphics.PorterDuff;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MenuInflater;
import android.widget.RelativeLayout;
import android.widget.Gallery;
import android.widget.ImageView;

import org.ros.node.NodeMain;
import org.ros.node.Node;
import org.ros.node.ConnectedNode;
import org.ros.namespace.GraphName;

import org.ros.android.MasterChooserDialogFragment;
import org.ros.android.MasterChooserDialogFragmentListener;
import org.ros.android.NodeMainExecutorService;
import org.ros.android.NodeMainExecutorServiceListener;
import org.ros.android.NodeMainExecutorServiceConnection;

import java.lang.Float;

import ch.ethz.naro.GalleryAdapter;
import ch.ethz.naro.VirtualJoystick;
import ch.ethz.naro.VirtualSlider;
import ch.ethz.naro.JoyPublisher;
import ch.ethz.naro.GetVoltageClient;
import ch.ethz.naro.SetSpeedClient;
import ch.ethz.naro.GetLimitsClient;

public class MainActivity
  extends Activity
  implements MasterChooserDialogFragmentListener,
    NodeMainExecutorServiceListener, NodeMain {

  private static final String PREFS_KEY_NODE_NAME = "pref_key_node_name";
  private static final String PREFS_KEY_NAMESPACE = "pref_key_namespace";
  private static final String PREFS_KEY_JOY_TOPIC = "pref_key_joy_topic";
  private static final String PREFS_KEY_GET_VOLTAGE_SERVER =
    "pref_key_get_voltage_server";
  private static final String PREFS_KEY_MIN_BATTERY_VOLTAGE =
    "pref_key_battery_min_voltage";
  private static final String PREFS_KEY_MAX_BATTERY_VOLTAGE =
    "pref_key_battery_max_voltage";
  private static final String PREFS_KEY_SET_SPEED_SERVER =
    "pref_key_set_speed_server";
  private static final String PREFS_KEY_GET_LIMITS_SERVER =
    "pref_key_get_limits_server";
  private static final String PREFS_KEY_INVERT_LIMITS =
    "pref_key_limits_invert";
  private static final String PREFS_KEY_JOYSTICK_LEFT_CHANNEL_X =
    "pref_key_joystick_left_channel_x";
  private static final String PREFS_KEY_JOYSTICK_LEFT_LOCK_X =
    "pref_key_joystick_left_lock_x";
  private static final String PREFS_KEY_JOYSTICK_LEFT_CHANNEL_Y =
    "pref_key_joystick_left_channel_y";
  private static final String PREFS_KEY_JOYSTICK_LEFT_LOCK_Y =
    "pref_key_joystick_left_lock_y";
  private static final String PREFS_KEY_JOYSTICK_RIGHT_CHANNEL_X =
    "pref_key_joystick_right_channel_x";
  private static final String PREFS_KEY_JOYSTICK_RIGHT_LOCK_X =
    "pref_key_joystick_right_lock_x";
  private static final String PREFS_KEY_JOYSTICK_RIGHT_CHANNEL_Y =
    "pref_key_joystick_right_channel_y";
  private static final String PREFS_KEY_JOYSTICK_RIGHT_LOCK_Y =
    "pref_key_joystick_right_lock_y";
    
  private NodeMainExecutorService service;
  private NodeMainExecutorServiceConnection serviceConnection;  
  private ConnectedNode node = null;

  MenuItem connectAction;
  MenuItem batteryAction;
  MenuItem cylinderAction;
  
  private VirtualJoystick joystickLeft;
  private VirtualJoystick joystickRight;
  private VirtualSlider slider;
  
  private JoyPublisher joyPublisher;
  private GetVoltageClient getVoltageClient;
  private SetSpeedClient setSpeedClient;
  private GetLimitsClient getLimitsClient;

  private OnSharedPreferenceChangeListener preferenceChangeListener;
  
  private Handler connectHandler;
  private Handler disconnectHandler;
  private Handler getVoltageHandler;
  private Handler getLimitsHandler;
  
  public MainActivity() {
    service = new NodeMainExecutorService();
    serviceConnection = new NodeMainExecutorServiceConnection(this);
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main_activity);
    
    Gallery gallery = (Gallery)findViewById(R.id.gallery);
    GalleryAdapter adapter = new GalleryAdapter(this);
    adapter.setImagesResource(R.drawable.img_gallery);
    gallery.setAdapter(adapter);
    
    PreferenceManager.setDefaultValues(this,
      R.xml.preferences_connection, false);
    SharedPreferences preferences = 
      PreferenceManager.getDefaultSharedPreferences(this);
    joyPublisher = new JoyPublisher(getDefaultNodeName().toString(),
      preferences.getString(PREFS_KEY_JOY_TOPIC, "joy"));
    getVoltageClient = new GetVoltageClient(getDefaultNodeName().toString(),
      preferences.getString(PREFS_KEY_GET_VOLTAGE_SERVER, "get_voltage"));
    setSpeedClient = new SetSpeedClient(getDefaultNodeName().toString(),
      preferences.getString(PREFS_KEY_SET_SPEED_SERVER, "set_speed"));
    getLimitsClient = new GetLimitsClient(getDefaultNodeName().toString(),
      preferences.getString(PREFS_KEY_GET_LIMITS_SERVER, "get_limits"));
      
    joystickLeft = (VirtualJoystick)findViewById(R.id.joystick_left);
    joystickLeft.addListener(joyPublisher);
    joystickRight = (VirtualJoystick)findViewById(R.id.joystick_right);
    joystickRight.addListener(joyPublisher);    
    slider = (VirtualSlider)findViewById(R.id.slider);
    slider.addListener(setSpeedClient);
    
    applyPreferences(preferences);
  }

  @Override
  protected void onStart() {
    super.onStart();

    Intent intent = new Intent(this, NodeMainExecutorService.class);
    intent.setAction(NodeMainExecutorService.ACTION_START);
    intent.putExtra(NodeMainExecutorService.EXTRA_NOTIFICATION_TICKER,
      getString(R.string.service_start_ticker));
    intent.putExtra(NodeMainExecutorService.EXTRA_NOTIFICATION_TITLE,
      getString(R.string.service_title));
    startService(intent);
    Preconditions.checkState(bindService(intent, serviceConnection,
      BIND_AUTO_CREATE), "Failed to bind NodeMainExecutorService.");
  }

  @Override
  protected void onDestroy() {
    unbindService(serviceConnection);
    Intent intent = new Intent(this, NodeMainExecutorService.class);
    stopService(intent);
    
    super.onDestroy();
  }
  
  @Override
  public boolean onCreateOptionsMenu(Menu menu) {
    MenuInflater inflater = getMenuInflater();
    inflater.inflate(R.menu.main_menu, menu);
    inflater.inflate(R.menu.main_actions, menu);
    
    connectAction = menu.findItem(R.id.action_connect);
    connectAction.getIcon().setAlpha(96);
    batteryAction = menu.findItem(R.id.action_battery);
    batteryAction.getIcon().setAlpha(96);
    cylinderAction = menu.findItem(R.id.action_cylinder);
    cylinderAction.getIcon().setAlpha(96);
    
    preferenceChangeListener = new OnSharedPreferenceChangeListener() {
      @Override
      public void onSharedPreferenceChanged(SharedPreferences preferences,
          String key) {
        applyPreferences(preferences);
      }
    };
    SharedPreferences preferences = 
      PreferenceManager.getDefaultSharedPreferences(this);
    preferences.registerOnSharedPreferenceChangeListener(
      preferenceChangeListener);

    connectHandler = new Handler() {
      public void handleMessage(Message message) {
        Drawable connectIcon = connectAction.getIcon();
        connectIcon.setLevel(1);
        connectIcon.setAlpha(255);
      }
    };

    disconnectHandler = new Handler() {
      public void handleMessage(Message message) {
        Drawable connectIcon = connectAction.getIcon();
        connectIcon.setLevel(0);
        connectIcon.setAlpha(96);
        
        Drawable batteryIcon = batteryAction.getIcon();
        batteryIcon.setLevel(0);
        batteryIcon.setAlpha(96);
        
        Drawable cylinderIcon = cylinderAction.getIcon();
        cylinderIcon.setLevel(0);
        cylinderIcon.setAlpha(96);
      }
    };

    getVoltageHandler = new Handler() {
      public void handleMessage(Message message) {
        float batteryVoltage = ((Float)message.obj).floatValue();
        Drawable batteryIcon = batteryAction.getIcon();
        
        if (batteryVoltage > 0.0f) {
          SharedPreferences preferences = 
            PreferenceManager.getDefaultSharedPreferences(MainActivity.this);
          
          float minBatteryVoltage = Float.parseFloat(
            preferences.getString(PREFS_KEY_MIN_BATTERY_VOLTAGE, "0.0"));
          float maxBatteryVoltage = Float.parseFloat(
            preferences.getString(PREFS_KEY_MAX_BATTERY_VOLTAGE, "0.0"));

          int level = 0;
          if (batteryVoltage < minBatteryVoltage)
            level = 1;
          else if (batteryVoltage > maxBatteryVoltage)
            level = 100;
          else
            level = (int)Math.round((batteryVoltage-minBatteryVoltage)/
              (maxBatteryVoltage-minBatteryVoltage)*100.0f);

          ((LevelListDrawable)batteryIcon).setLevel(level);
          batteryIcon.setAlpha(255);
        }
        else {
          ((LevelListDrawable)batteryIcon).setLevel(0);
          batteryIcon.setAlpha(96);
        }
      }
    };
    getVoltageClient.setGetVoltageHandler(getVoltageHandler);
        
    getLimitsHandler = new Handler() {
      public void handleMessage(Message message) {
        GetLimitsClient.Limits limits = (GetLimitsClient.Limits)message.obj;
        Drawable cylinderIcon = cylinderAction.getIcon();
        
        if (limits != null) {
          SharedPreferences preferences = 
            PreferenceManager.getDefaultSharedPreferences(MainActivity.this);
            
          boolean invertLimits = preferences.getBoolean(
            PREFS_KEY_INVERT_LIMITS, false);
            
          boolean empty = limits.analog1;
          boolean full = limits.analog2;
          if (invertLimits) {
            boolean swap = empty;
            empty = full;
            full = swap;
          }
          
          int level = 2;
          if (empty)
            level = 1;
          else if (full)
            level = 3;
            
          ((LevelListDrawable)cylinderIcon).setLevel(level);
          cylinderIcon.setAlpha(255);
        }
        else {
          ((LevelListDrawable)cylinderIcon).setLevel(0);
          cylinderIcon.setAlpha(96);
        }
      }
    };
    getLimitsClient.setGetLimitsHandler(getLimitsHandler);    
    
    return true;
  }

  @Override
  public GraphName getDefaultNodeName() {
    SharedPreferences preferences = 
      PreferenceManager.getDefaultSharedPreferences(this);      
    return GraphName.of(preferences.getString(PREFS_KEY_NODE_NAME,
      "naro_app"));
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    this.node = connectedNode;
        
    Message message = connectHandler.obtainMessage();
    message.obj = connectedNode;
    connectHandler.sendMessage(message);
  }

  @Override
  public void onShutdown(Node node) {
  };

  @Override
  public void onShutdownComplete(Node node) {
    this.node = null;
    
    Message message = disconnectHandler.obtainMessage();
    message.obj = node;
    disconnectHandler.sendMessage(message);    
  }

  @Override
  public void onError(Node node, Throwable throwable) {
  }
  
  public boolean onConnectClick(MenuItem item) {
    if (node == null) {
      DialogFragment dialog = new MasterChooserDialogFragment();
      dialog.show(getFragmentManager(), "MasterChooserDialogFragment");
    }
    else {
      AlertDialog.Builder builder = new AlertDialog.Builder(this);
      builder.setMessage(R.string.alert_disconnect_message)
        .setTitle(String.format(getString(R.string.alert_disconnect_title),
          service.getMasterUri().getHost()))
        .setIconAttribute(android.R.attr.alertDialogIcon)
        .setPositiveButton(android.R.string.ok,
            new DialogInterface.OnClickListener() {
          public void onClick(DialogInterface dialog, int id) {
            onDisconnectClick();
          }
        })
        .setNegativeButton(android.R.string.cancel,
            new DialogInterface.OnClickListener() {
          public void onClick(DialogInterface dialog, int id) {
          }
        });
      AlertDialog dialog = builder.create();
      dialog.show();
    }
    
    return true;
  }
  
  public void onConnectClick(MasterChooserDialogFragment dialog) {
    service.setMasterUri(dialog.getMasterUri());
    service.setNamespace(dialog.getNamespace());

    service.connect(this);
    service.connect(joyPublisher);
    service.connect(getVoltageClient);
    service.connect(setSpeedClient);
    service.connect(getLimitsClient);
  }

  public void onDisconnectClick() {
    service.disconnect(joyPublisher);
    service.disconnect(getVoltageClient);
    service.disconnect(setSpeedClient);
    service.disconnect(getLimitsClient);
    service.disconnect(this);
  }
  
  public void onCancelClick(MasterChooserDialogFragment dialog) {
  }

  public void onShutdown(NodeMainExecutorService service) {
    finish();
  }

  public boolean onPreferencesClick(MenuItem item) {
    Intent intent = new Intent(this, PreferenceActivity.class);
    startActivity(intent);
    
    return true;
  }
  
  public boolean onExitClick(MenuItem item) {
    finish();
    return true;
  }
  
  protected void applyPreferences(SharedPreferences preferences) {
    int leftChannelX = Integer.parseInt(preferences.getString(
      PREFS_KEY_JOYSTICK_LEFT_CHANNEL_X, "0"));
    int leftChannelY = Integer.parseInt(preferences.getString(
      PREFS_KEY_JOYSTICK_LEFT_CHANNEL_Y, "1"));
    VirtualJoystick.Lock leftLock = new VirtualJoystick.Lock(
      preferences.getBoolean(PREFS_KEY_JOYSTICK_LEFT_LOCK_X, false),
      preferences.getBoolean(PREFS_KEY_JOYSTICK_LEFT_LOCK_Y, false));
    joystickLeft.setLock(leftLock);
    joyPublisher.setAxes(joystickLeft, leftChannelX, leftChannelY);
      
    int rightChannelX = Integer.parseInt(preferences.getString(
      PREFS_KEY_JOYSTICK_RIGHT_CHANNEL_X, "3"));
    int rightChannelY = Integer.parseInt(preferences.getString(
      PREFS_KEY_JOYSTICK_RIGHT_CHANNEL_Y, "2"));
    VirtualJoystick.Lock rightLock = new VirtualJoystick.Lock(
      preferences.getBoolean(PREFS_KEY_JOYSTICK_RIGHT_LOCK_X, false),
      preferences.getBoolean(PREFS_KEY_JOYSTICK_RIGHT_LOCK_Y, false));
    joystickRight.setLock(rightLock);
    joyPublisher.setAxes(joystickRight, rightChannelX, rightChannelY);
  }
}
