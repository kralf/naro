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
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.namespace.NameResolver;
import org.ros.message.Time;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;

import android.os.Handler;
import android.os.Message;
import android.util.Log;

import java.lang.Float;
import java.util.Timer;
import java.util.TimerTask;
import java.util.ArrayList;
import java.util.Hashtable;

import ch.ethz.naro.VirtualSlider;

public class SetSpeedClient
  extends AbstractNodeMain
  implements ServiceResponseListener<naro_smc_srvs.SetSpeedResponse>,
    VirtualSlider.Listener {
  
  private String graphNamespace;
  private String setSpeedServer;
  
  private ConnectedNode node;
  private ServiceClient<naro_smc_srvs.SetSpeedRequest,
    naro_smc_srvs.SetSpeedResponse> setSpeedClient;
  private Handler getVoltageHandler;
  
  private float speed = 0.0f;
  
  private Timer timer;
  private TimerTask timerTask;

  public SetSpeedClient(String graphNamespace, String setSpeedServer) {
    this.graphNamespace = graphNamespace;
    this.setSpeedServer = setSpeedServer;
  }
  
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of(graphNamespace+"/set_speed_client");
  }
  
  @Override
  public void onStart(ConnectedNode connectedNode) {
    node = connectedNode;
    
    try {
      NameResolver resolver = node.getResolver();
      setSpeedClient = node.newServiceClient(resolver.resolve(
        setSpeedServer), naro_smc_srvs.SetSpeed._TYPE);
    }
    catch (ServiceNotFoundException exception) {
      setSpeedClient = null;
    }

    timerTask = new TimerTask() {
      @Override
      public void run() {
        if (setSpeedClient != null) {
          naro_smc_srvs.SetSpeedRequest request =
            setSpeedClient.newMessage();
          request.setSpeed(speed);
          request.setStart(true);

          setSpeedClient.call(request, SetSpeedClient.this);
        }
      }
    };

    timer = new Timer();
    timer.schedule(timerTask, 0, 100);
  }

  @Override
  public void onShutdownComplete(Node node) {
    speed = 0.0f;
  
    setSpeedClient = null;
    this.node = null;
  }

  @Override
  public void onSuccess(naro_smc_srvs.SetSpeedResponse response) {
  }

  @Override
  public void onFailure(RemoteException exception) {
  }
  
  @Override
  public void onMove(VirtualSlider slider) {
    speed = slider.getX();
  }
}
