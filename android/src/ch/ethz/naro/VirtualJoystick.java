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

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import android.annotation.SuppressLint;
import android.app.ActionBar.LayoutParams;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.CompoundButton;
import android.widget.CompoundButton.OnCheckedChangeListener;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.Switch;

public class VirtualJoystick
  implements OnCheckedChangeListener {
  
  public interface Listener {
    public void onMove(VirtualJoystick joystick);
  }
  
  private ImageView stick;
  private Bitmap pic;
  private Bitmap picRes;
  private RelativeLayout layout;
  private RelativeLayout.LayoutParams params;
//   private Switch lockX;
//   private Switch lockY;
  
  private int bitWidth;
  private int bitHeight;
  private int startPosX;
  private int startPosY;
  private int radius;
  
  private boolean dragging = false;
  private boolean draggingPos = true;
  
  private boolean moveX = true;
  private boolean moveY = true;
  
  private int posX;
  private int posY;
  
  private String name;
  
  private List listeners = new ArrayList();

  public VirtualJoystick(RelativeLayout layout, int midPointX, int midPointY,
      int radius, String name) {   
    bitWidth = (int)Math.round((float)radius*0.6666);
    bitHeight = bitWidth;
    
    this.startPosX = midPointX;
    this.startPosY = midPointY;
    this.radius = radius;
  
    posX = startPosX-bitWidth/2;
    posY = startPosY-bitWidth/2;
    
    this.name = name;
    
    Context context = layout.getContext();
    
    this.layout = layout;
    stick = new ImageView(layout.getContext());
    
    pic = BitmapFactory.decodeResource(layout.getResources(),
      R.drawable.joystick);
    picRes = Bitmap.createScaledBitmap(pic, bitWidth, bitHeight, false);
    stick.setImageBitmap(picRes);
    params = new RelativeLayout.LayoutParams(LayoutParams.WRAP_CONTENT,
      LayoutParams.WRAP_CONTENT);
    params.setMargins(posX, posY, 0, 0);
    stick.setLayoutParams(params);  
    
    int textHeight = 40;
    int wScreen = startPosX+radius;
    int hScreen= startPosY+radius+textHeight;
    Bitmap pallet = Bitmap.createBitmap(wScreen, hScreen,
      Bitmap.Config.ARGB_8888);
    Canvas canvas = new Canvas(pallet);
    Paint paint = new Paint(); 
    paint.setStyle(Paint.Style.STROKE);
    paint.setFlags(Paint.ANTI_ALIAS_FLAG);
    paint.setStrokeWidth(2.0f);
    canvas.drawCircle(startPosX, startPosY, radius, paint);
    canvas.drawLine(startPosX, startPosY+radius, startPosX,
      startPosY-radius, paint);
    canvas.drawLine(startPosX+radius, startPosY, startPosX-radius,
      startPosY, paint);
    
    Paint text = new Paint();
    text.setTextSize(20);
    text.setStyle(Paint.Style.FILL);
    text.setFlags(Paint.ANTI_ALIAS_FLAG);
    text.setTextAlign(Paint.Align.CENTER);
    canvas.drawText(name, startPosX, startPosY+radius+24, text);
    
    ImageView circle = new ImageView(layout.getContext());
    circle.setImageBitmap(pallet);
    
//     lockY = new Switch(layout.getContext());
//     lockY.setX(startPosX-95);
//     lockY.setY(startPosY-radius-70);
//     lockY.setOnCheckedChangeListener(this);
//     lockY.setId(2);
//     lockY.setChecked(true);

//     lockX = new Switch(layout.getContext());
//     lockX.setX(startPosX+radius+15-53);
//     lockX.setY(startPosY-25);
//     lockX.setRotation(90);
//     lockX.setOnCheckedChangeListener(this);
//     lockX.setId(1);
//     lockX.setChecked(true);
    
//     layout.addView(lockX);
//     layout.addView(lockY);
    layout.addView(circle);
    layout.addView(stick);
    
    layout.setOnTouchListener(myListener);
  }
    
  public void addListener(Listener listener) {
    listeners.add(listener);
  }
  
  public void removeListeners(Listener listener) {
    listeners.remove(listener);
  }
  
  private void firePosition() {
    Iterator it = listeners.iterator();
    
    while (it.hasNext())
      ((Listener)it.next()).onMove(this);
  }
  
  @Override
  public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
//     int id = buttonView.getId();
    
//     if (id == 1) {
//       lockAxis('x', isChecked);
//     }
//     if (id == 2) {
//       this.lockAxis('y', isChecked);
//     }
  }
  
  OnTouchListener myListener = new OnTouchListener(){
    @Override
    public boolean onTouch(View v, MotionEvent event){
      updatePos(event);
        return true;
    }};
  
  public void lockAxis(char axis, boolean bol) {
    if (axis == 'x')
      this.moveX = !bol;
    if (axis == 'y')
      this.moveY = !bol;
  }
  
  public float getDx() {
    return ((float)-(this.startPosX-this.posX-bitWidth/2))/radius;
  }
  
  public float getDy() {
    return (float)(this.startPosY-this.posY-bitHeight/2)/radius;
  }
  
  private void updatePos(MotionEvent event) {
    double tabX = event.getX();
    double tabY = event.getY();
    
    int tabXint = (int)Math.round(event.getX());
    int tabYint = (int)Math.round(event.getY());
    
    int posX = tabXint-bitWidth/2;
    int posY = tabYint-bitHeight/2;
      
    int disX = tabXint-startPosX;
    int disY = tabYint-startPosY;
    
    if ( event.getAction() == MotionEvent.ACTION_DOWN) {
      if(disX*disX+disY*disY < bitWidth*bitWidth) {
        dragging = true;
      }
    }
    else if (event.getAction() == MotionEvent.ACTION_UP) {
      dragging = false;
      draggingPos = true;
    }
      
    if(disX*disX+disY*disY > radius*radius) {
      draggingPos = false;
    }
    else {
      draggingPos = true;
    }
    
    if(dragging) {
      if(!draggingPos) {

        double dX = tabX-startPosX;
        double dY = tabY-startPosY;
        
        double num = Math.sqrt(1+Math.pow(dY/dX, 2));
        int dx = (int) Math.round(radius/num);
        int dy = (int) Math.round(Math.sqrt(radius*radius-dx*dx));
        
        if (dX < 0)
          dx = -dx;
        if (dY < 0)
          dy = -dy;
        
        posX = startPosX+dx-bitWidth/2;
        posY = startPosY+dy-bitHeight/2;
      }

      if (!moveX) 
        posX = startPosX-bitWidth/2;
      if (!moveY) 
        posY = startPosY-bitWidth/2;
      
      this.posX = posX;
      this.posY = posY;
      params.setMargins(posX, posY, 0, 0);
      stick.setLayoutParams(params);
    } 
    else {
      this.posX = startPosX-bitWidth/2;
      this.posY = startPosY-bitHeight/2;
      params.setMargins(this.posX, this.posY, 0, 0);
      stick.setLayoutParams(params);
    }
    
    firePosition();
  }
  
  public float getX() {
    return -(float)(posX-startPosX+bitWidth/2)/(float)radius;
  }

  public float getY() {
    return -(float)(posY-startPosY+bitWidth/2)/(float)radius;
  }
}
