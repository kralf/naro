/***************************************************************************
 *   Copyright (C) 2013 by Jonas Eichenberger                              *
 *   jonasei@ethz.ch                                                       *
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

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.Node;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.namespace.NameResolver;
import org.ros.message.Time;

import android.util.Log;

import java.util.Timer;
import java.util.TimerTask;
import java.util.ArrayList;
import java.util.Hashtable;

import ch.ethz.naro.VirtualJoystick;

public class JoyPublisher
  extends AbstractNodeMain
  implements VirtualJoystick.Listener {
  
  private String graphNamespace;
  private String topic;
  private ConnectedNode node;
  private Publisher<sensor_msgs.Joy> publisher;
  
  private Timer timer;
  private TimerTask timerTask;

  private float[] axes;
  private Hashtable<VirtualJoystick, ArrayList<Integer> > joystickAxes;
  
  private float x = 0;
  private float y = 0;
  
  public JoyPublisher(String graphNamespace, String topic) {
    this.graphNamespace = graphNamespace;
    this.topic = topic;

    joystickAxes = new Hashtable<VirtualJoystick, ArrayList<Integer> >();
    axes = new float[8];
  }
  
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of(graphNamespace+"/"+topic);
  }
  
  @Override
  public void onStart(ConnectedNode connectedNode) {
    node = connectedNode;
    
    NameResolver resolver = node.getResolver();
    publisher = node.newPublisher(resolver.resolve("joy"),
      sensor_msgs.Joy._TYPE);
    
    timerTask = new TimerTask() {
      @Override
      public void run() {
        if (publisher != null) {
          Time currentTime = node.getCurrentTime();

          sensor_msgs.Joy joy = publisher.newMessage();
          joy.getHeader().setStamp(currentTime);
          
          joy.setAxes(axes);

          publisher.publish(joy);
        }
      }
    };

    timer = new Timer();
    timer.schedule(timerTask, 0, 100);
  }

  @Override
  public void onShutdownComplete(Node node) {
    publisher = null;
    this.node = null;
  }
  
  @Override
  public void onMove(VirtualJoystick joystick) {
    ArrayList<Integer> joystickAxes = new ArrayList<Integer>();
    joystickAxes.add(0);
    joystickAxes.add(1);
  
    if (this.joystickAxes.containsKey(joystick)) {
      joystickAxes = this.joystickAxes.get(joystick);
    }
  
    axes[joystickAxes.get(0)] = joystick.getPosition().getX();
    axes[joystickAxes.get(1)] = joystick.getPosition().getY();
  }
  
  public void setAxes(VirtualJoystick joystick, int x, int y) {
    ArrayList<Integer> joystickAxes = new ArrayList<Integer>();
    joystickAxes.add(x);
    joystickAxes.add(y);
    
    this.joystickAxes.put(joystick, joystickAxes);
  }
}
