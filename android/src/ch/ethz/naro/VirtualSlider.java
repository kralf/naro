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

import android.content.Context;
import android.content.res.TypedArray;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.drawable.Drawable;
import android.util.Log;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

public class VirtualSlider
  extends View {
  
  public interface Listener {
    public void onMove(VirtualSlider slider);
  }
  
  protected float position = 0.0f;
  protected Drawable thumb = null;
  protected float lineWidth = 5.0f;
  protected int lineColor = Color.BLACK;
  protected float lineAlpha = 0.3f;
  protected List listeners = new ArrayList();
  
  private boolean dragging = false;
  
  public VirtualSlider(Context context) {
    super(context);
  }
  
  public VirtualSlider(Context context, AttributeSet attrs) {
    super(context, attrs);

    TypedArray attrArray = context.getTheme().obtainStyledAttributes(
      attrs, R.styleable.VirtualSlider, 0, 0);

    try {
      thumb = attrArray.getDrawable(R.styleable.VirtualSlider_thumb);
      lineWidth = attrArray.getFloat(R.styleable.VirtualSlider_lineWidth,
        lineWidth);
      lineColor = attrArray.getColor(R.styleable.VirtualSlider_lineColor,
        lineColor);
      lineAlpha = attrArray.getFloat(R.styleable.VirtualSlider_lineAlpha,
        lineAlpha);
    } finally {
      attrArray.recycle();
    }
  }
    
  public float getPosition() {
    return position;
  }
  
  public void setPosition(float position) {
    if (position > 1.0f)
      position = 1.0f;
    else if (position < -1.0f)
      position = -1.0f;
      
    if (position != this.position) {
      this.position = position;

      invalidate();
      notifyListeners();
    }
  }
  
  public void setThumb(Drawable thumb) {
    this.thumb = thumb;
    invalidate();
  }
  
  public Drawable getThumb() {
    return thumb;
  }
  
  public void setLineWidth(float lineWidth) {
    this.lineWidth = lineWidth;
    invalidate();
  }
  
  public float getLineWidth() {
    return lineWidth;
  }
  
  public void setLineColor(int lineColor) {
    this.lineColor = lineColor;
    invalidate();
  }
  
  public float getLineColor() {
    return lineColor;
  }
  
  public void setLineAlpha(float lineAlpha) {
    this.lineAlpha = lineAlpha;
    invalidate();
  }
  
  public float getLineAlpha() {
    return lineAlpha;
  }
  
  public float getThumbRadius() {
    return 0.5f*getHeight();
  }
  
  public void addListener(Listener listener) {
    listeners.add(listener);
  }
  
  public void removeListeners(Listener listener) {
    listeners.remove(listener);
  }
  
  @Override
  protected void onDraw(Canvas canvas) {
    super.onDraw(canvas);
    
    float width = getWidth();
    float height = getHeight();
    float thumbRadius = getThumbRadius();

    Paint paint = new Paint();
    paint.setColor(lineColor);
    paint.setAlpha((int)(lineAlpha*255.0f));
    paint.setStyle(Paint.Style.STROKE);
    paint.setFlags(Paint.ANTI_ALIAS_FLAG);
    paint.setStrokeWidth(lineWidth);
    paint.setStrokeCap(Paint.Cap.ROUND);
    canvas.drawLine(thumbRadius, thumbRadius, width-thumbRadius,
      thumbRadius, paint);
      
    if (thumb != null) {
      float positionX = position*(0.5f*width-thumbRadius);
      thumb.setBounds((int)(0.5f*width+positionX-thumbRadius), 0,
        (int)(0.5f*width+positionX+thumbRadius), (int)(2.0f*thumbRadius));
      thumb.draw(canvas);
    }
  }
  
  @Override
  public boolean onTouchEvent(MotionEvent event) {
    float width = getWidth();
    float thumbRadius = getThumbRadius();    
    float touchX = event.getX()-0.5f*width;
    
    if (event.getAction() == MotionEvent.ACTION_DOWN) {
      if(Math.abs(touchX) < Math.abs(thumbRadius))
        dragging = true;
    }
    else if (event.getAction() == MotionEvent.ACTION_UP)
      dragging = false;
      
    if(dragging)
      setPosition(touchX/(0.5f*width-thumbRadius));
    else
      setPosition(0.0f);
      
    return true;
  }
  
  @Override
  protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
    super.onMeasure(widthMeasureSpec, heightMeasureSpec);
  }
  
  private void notifyListeners() {
    Iterator it = listeners.iterator();
    
    while (it.hasNext())
      ((Listener)it.next()).onMove(this);
  }
}
