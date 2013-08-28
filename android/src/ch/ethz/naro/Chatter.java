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
