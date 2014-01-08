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

public class VirtualSlider {
  
  public interface Listener {
    public void onMove(VirtualSlider slider);
  }
  
  private ImageView bar;
  private ImageView stick;
  private Bitmap pic;
  private Bitmap picRes;
  private RelativeLayout layout;
  private RelativeLayout.LayoutParams params;
  
  private int bitWidth;
  private int bitHeight;
  private int startPosX;
  private int width;
  private int height;
  
  private boolean dragging = false;
  private boolean draggingPos = true;
  
  private int posX;
  
  private String name;
  
  private List listeners = new ArrayList();

  public VirtualSlider(RelativeLayout layout, int width, int height,
      String name) {   
    bitWidth = (int)Math.round((float)height*0.8);
    bitHeight = bitWidth;
    
    this.startPosX = width/2;
    this.width = width;
    this.height = height;
  
    this.posX = startPosX-bitWidth/2;
    
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
    params.setMargins(posX, height/2-bitHeight/2, 0, 0);
    stick.setLayoutParams(params);  
    
    int textHeight = 20;
    int wScreen = width;
    int hScreen= height+textHeight*3/2+4;
    Bitmap pallet = Bitmap.createBitmap(wScreen, hScreen,
      Bitmap.Config.ARGB_8888);
    Canvas canvas = new Canvas(pallet);
    Paint paint = new Paint(); 
    paint.setStyle(Paint.Style.STROKE);
    paint.setFlags(Paint.ANTI_ALIAS_FLAG);
    paint.setStrokeWidth(2.0f);
    canvas.drawLine(bitWidth/2, height/2, width-bitWidth/2, height/2, paint);
    
    Paint text = new Paint();
    text.setTextSize(textHeight);
    text.setStyle(Paint.Style.FILL);
    text.setFlags(Paint.ANTI_ALIAS_FLAG);
    text.setTextAlign(Paint.Align.CENTER);
    canvas.drawText(name, startPosX, height+textHeight+4, text);
    
    bar = new ImageView(layout.getContext());
    bar.setImageBitmap(pallet);
    
    layout.addView(bar);
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
  
  OnTouchListener myListener = new OnTouchListener(){
    @Override
    public boolean onTouch(View v, MotionEvent event){
      updatePos(event);
        return true;
    }};
  
  public float getDx() {
    return ((float)-(this.startPosX-this.posX-bitWidth/2))/width;
  }
  
  private void updatePos(MotionEvent event) {
    double tabX = event.getX()-bar.getLeft();   
    int tabXint = (int)Math.round(event.getX()-bar.getLeft());
    int posX = tabXint-bitWidth/2;
    int disX = tabXint-startPosX;
    
    if (event.getAction() == MotionEvent.ACTION_DOWN) {
      if(Math.abs(disX) < Math.abs(bitWidth)) {
        dragging = true;
      }
    }
    else if (event.getAction() == MotionEvent.ACTION_UP) {
      dragging = false;
      draggingPos = true;
    }
      
    if(Math.abs(disX) > width/2-bitWidth/2) {
      draggingPos = false;
    }
    else {
      draggingPos = true;
    }
    
    if(dragging) {
      if(!draggingPos) {
        double dx = tabX-startPosX;
        
        if (dx > width/2-bitWidth/2)
          dx = width/2-bitWidth/2;
        else if (dx < -(width/2-bitWidth/2))
          dx = -(width/2-bitWidth/2);
        
        posX = (int)Math.round(startPosX+dx-bitWidth/2);
      }

      this.posX = posX;
      params.setMargins(posX, height/2-bitHeight/2, 0, 0);
      stick.setLayoutParams(params);
    } 
    else {
      this.posX = startPosX-bitWidth/2;
      params.setMargins(this.posX, height/2-bitHeight/2, 0, 0);
      stick.setLayoutParams(params);
    }
    
    firePosition();
  }
  
  public float getX() {
    return (float)(posX-startPosX+bitWidth/2)/
      (float)(width/2-bitWidth/2);
  }
}
