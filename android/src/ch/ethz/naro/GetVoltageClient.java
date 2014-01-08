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

public class GetVoltageClient
  extends AbstractNodeMain
  implements ServiceResponseListener<naro_smc_srvs.GetVoltageResponse> {
  
  private String graphNamespace;
  private String getVoltageServer;
  
  private ConnectedNode node;
  private ServiceClient<naro_smc_srvs.GetVoltageRequest,
    naro_smc_srvs.GetVoltageResponse> getVoltageClient;
  private Handler getVoltageHandler;
  
  private float voltage = 0.0f;
  
  private Timer timer;
  private TimerTask timerTask;

  public GetVoltageClient(String graphNamespace, String getVoltageServer) {
    this.graphNamespace = graphNamespace;
    this.getVoltageServer = getVoltageServer;
  }
  
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of(graphNamespace+"/get_voltage_client");
  }
  
  @Override
  public void onStart(ConnectedNode connectedNode) {
    node = connectedNode;
    
    try {
      NameResolver resolver = node.getResolver();
      getVoltageClient = node.newServiceClient(resolver.resolve(
        getVoltageServer), naro_smc_srvs.GetVoltage._TYPE);
    }
    catch (ServiceNotFoundException exception) {
      getVoltageClient = null;
    }

    timerTask = new TimerTask() {
      @Override
      public void run() {
        if (getVoltageClient != null) {
          naro_smc_srvs.GetVoltageRequest request =
            getVoltageClient.newMessage();

          getVoltageClient.call(request, GetVoltageClient.this);
        }
      }
    };

    timer = new Timer();
    timer.schedule(timerTask, 0, 1000);
  }

  @Override
  public void onShutdownComplete(Node node) {
    voltage = 0.0f;
  
    getVoltageClient = null;
    this.node = null;
  }

  @Override
  public void onSuccess(naro_smc_srvs.GetVoltageResponse response) {
    voltage = response.getVoltage();
    
    if (getVoltageHandler != null) {
      Message message = getVoltageHandler.obtainMessage();
      message.obj = new Float(voltage);
      getVoltageHandler.sendMessage(message);
    }
  }

  @Override
  public void onFailure(RemoteException exception) {
  }
  
  public float getVoltage() {
    return voltage;
  }

  public void setGetVoltageHandler(Handler handler) {
    getVoltageHandler = handler;
  }
}
