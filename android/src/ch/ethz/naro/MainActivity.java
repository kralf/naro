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
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MenuInflater;
import android.content.Intent;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.drawable.Drawable;
import android.graphics.drawable.LevelListDrawable;
import android.graphics.PorterDuff;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.widget.RelativeLayout;
import android.widget.Gallery;
import android.widget.ImageView;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

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

public class MainActivity
  extends Activity
  implements MasterChooserDialogFragmentListener,
    NodeMainExecutorServiceListener, NodeMain {

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
    
  private NodeMainExecutorService service;
  private NodeMainExecutorServiceConnection serviceConnection;  
  private ConnectedNode node = null;

  MenuItem connectAction;
  MenuItem batteryAction;
  
  private VirtualJoystick joyLeft;
  private VirtualJoystick joyRight;
  private VirtualSlider slider;
  
  private JoyPublisher joyPublisher;
  private GetVoltageClient getVoltageClient;
  private SetSpeedClient setSpeedClient;

  private Handler connectHandler;
  private Handler disconnectHandler;
  private Handler getVoltageHandler;
  
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
    
    SharedPreferences preferences = 
      PreferenceManager.getDefaultSharedPreferences(this);
    joyPublisher = new JoyPublisher(getString(R.string.node_name),
      preferences.getString(PREFS_KEY_JOY_TOPIC, "joy"));
    getVoltageClient = new GetVoltageClient(getString(R.string.node_name),
      preferences.getString(PREFS_KEY_GET_VOLTAGE_SERVER, "get_voltage"));
    setSpeedClient = new SetSpeedClient(getString(R.string.node_name),
      preferences.getString(PREFS_KEY_SET_SPEED_SERVER, "set_speed"));
      
    RelativeLayout layoutLeftJoystick = (RelativeLayout)findViewById(
      R.id.layout_left_joystick);
    joyLeft = new VirtualJoystick(layoutLeftJoystick, 165, 165, 125,
      getString(R.string.left_joystick_title));
    joyPublisher.setAxes(joyLeft, 0, 1);
    joyLeft.lockAxis('x', true);
    joyLeft.addListener(joyPublisher);
    
    RelativeLayout layoutRightJoystick = (RelativeLayout)findViewById(
      R.id.layout_right_joystick);
    joyRight = new VirtualJoystick(layoutRightJoystick, 165, 165, 125,
      getString(R.string.right_joystick_title));
    joyPublisher.setAxes(joyRight, 3, 2);
    joyRight.lockAxis('y', true);
    joyRight.addListener(joyPublisher);
    
    RelativeLayout layoutSlider = (RelativeLayout)findViewById(
      R.id.layout_slider);
    slider = new VirtualSlider(layoutSlider, 500, 80,
      getString(R.string.slider_title));
    slider.addListener(setSpeedClient);
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
        
    return true;
  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of(getString(R.string.node_name));
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
  }

  public void onDisconnectClick() {
    service.disconnect(joyPublisher);
    service.disconnect(getVoltageClient);
    service.disconnect(setSpeedClient);
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
}
