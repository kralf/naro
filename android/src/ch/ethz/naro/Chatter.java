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

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class Chatter extends AbstractNodeMain {
  private Publisher<std_msgs.String> publisher;
  
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("chatter");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    publisher = connectedNode.newPublisher("chatter", std_msgs.String._TYPE);
  }
  
  public void sendMessage(String text) {
    std_msgs.String message = publisher.newMessage();
    message.setData(text);
    
    publisher.publish(message);
  }
}
