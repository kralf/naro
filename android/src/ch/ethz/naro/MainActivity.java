package ch.ethz.naro;

import android.app.Activity;
import android.os.Bundle;

// import org.ros.android.RosActivity;
// import org.ros.node.NodeMainExecutor;

public class MainActivity extends Activity
{
  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main_activity);
  }

  public MainActivity() {
//     super("Naro Nanins", "Naro Nanins");
  }

//   @Override
//   protected void init(NodeMainExecutor nodeMainExecutor) {
//   }
}
