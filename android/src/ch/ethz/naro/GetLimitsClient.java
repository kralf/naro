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

public class GetLimitsClient
  extends AbstractNodeMain
  implements ServiceResponseListener<naro_smc_srvs.GetLimitsResponse> {
  
  public class Limits {
    public boolean analog1 = false;
    public boolean analog2 = false;
  
    public Limits(short limits) {
      fromShort(limits);
    };
    
    public void fromShort(short limits) {
      analog1 = ((limits & naro_smc_srvs.GetLimitsResponse.ANALOG1) != 0);
      analog2 = ((limits & naro_smc_srvs.GetLimitsResponse.ANALOG2) != 0);
    };
  };
  
  private String graphNamespace;
  private String getLimitsServer;
  
  private ConnectedNode node;
  private ServiceClient<naro_smc_srvs.GetLimitsRequest,
    naro_smc_srvs.GetLimitsResponse> getLimitsClient;
  private Handler getLimitsHandler;
  
  private Limits limits = null;
  
  private Timer timer;
  private TimerTask timerTask;

  public GetLimitsClient(String graphNamespace, String getLimitsServer) {
    this.graphNamespace = graphNamespace;
    this.getLimitsServer = getLimitsServer;
  }
  
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of(graphNamespace+"/get_limits_client");
  }
  
  @Override
  public void onStart(ConnectedNode connectedNode) {
    node = connectedNode;
    
    try {
      NameResolver resolver = node.getResolver();
      getLimitsClient = node.newServiceClient(resolver.resolve(
        getLimitsServer), naro_smc_srvs.GetLimits._TYPE);
    }
    catch (ServiceNotFoundException exception) {
      getLimitsClient = null;
    }

    timerTask = new TimerTask() {
      @Override
      public void run() {
        if (getLimitsClient != null) {
          naro_smc_srvs.GetLimitsRequest request =
            getLimitsClient.newMessage();

          getLimitsClient.call(request, GetLimitsClient.this);
        }
      }
    };

    timer = new Timer();
    timer.schedule(timerTask, 0, 250);
  }

  @Override
  public void onShutdownComplete(Node node) {
    limits = null;
  
    getLimitsClient = null;
    this.node = null;
  }

  @Override
  public void onSuccess(naro_smc_srvs.GetLimitsResponse response) {
    if (limits == null)
      limits = new Limits(response.getLimits());
    else
      limits.fromShort(response.getLimits());
    
    if (getLimitsHandler != null) {
      Message message = getLimitsHandler.obtainMessage();
      message.obj = limits;
      getLimitsHandler.sendMessage(message);
    }
  }

  @Override
  public void onFailure(RemoteException exception) {
  }
  
  public Limits getLimits() {
    return limits;
  }

  public void setGetLimitsHandler(Handler handler) {
    getLimitsHandler = handler;
  }
}
