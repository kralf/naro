package ch.ethz.naro;

import android.app.Activity;
import android.app.DialogFragment;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MenuInflater;
import android.graphics.drawable.Drawable;
import android.graphics.PorterDuff;
import android.os.Bundle;

// import org.ros.android.RosActivity;
// import org.ros.node.NodeMainExecutor;

import org.ros.android.MasterChooserDialogFragment;

public class MainActivity extends Activity
{
  @Override
  public void onCreate(Bundle savedInstanceState) {
    Drawable connectIcon = getResources().getDrawable(
      R.drawable.ic_action_connect);
    connectIcon.setColorFilter(0xff555555, PorterDuff.Mode.MULTIPLY);
  
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main_activity);
  }

  public MainActivity() {
//     super("Naro Nanins", "Naro Nanins");
  }

//   @Override
//   protected void init(NodeMainExecutor nodeMainExecutor) {
//   }

  @Override
  public boolean onCreateOptionsMenu(Menu menu) {
    MenuInflater inflater = getMenuInflater();
    inflater.inflate(R.menu.main_menu, menu);
    inflater.inflate(R.menu.main_actions, menu);
    return true;
  }
  
  public boolean onConnectClick(MenuItem item) {
    DialogFragment dialog = new MasterChooserDialogFragment();
    dialog.show(getFragmentManager(), "MasterChooserDialogFragment");
    return true;
  }

  public void onSendButtonClick(View view) {
  }
}
