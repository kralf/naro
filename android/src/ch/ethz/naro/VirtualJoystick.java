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
import java.util.Hashtable;
import java.util.Iterator;
import java.util.List;

import android.content.Context;
import android.content.res.TypedArray;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.drawable.Drawable;
import android.util.Log;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

public class VirtualJoystick
  extends View {
  
  public enum Axis {
    X,
    Y;
  };
  
  public static class Position {
    protected float x;
    protected float y;
    
    public Position() {
      this(0.0f, 0.0f);
    }
    
    public Position(float x, float y)  {
      setX(x);
      setY(y);
    }
    
    public Position(Position src) {
      x = src.x;
      y = src.y;
    }
    
    void setX(float x) {
      this.x = x;
    }
    
    float getX() {
      return x;
    }

    void setY(float y) {
      this.y = y;
    }
    
    float getY() {
      return y;
    }
  };
  
  public static class Lock {
    protected boolean x;
    protected boolean y;
    
    public Lock() {
      this(false, false);
    }
    
    public Lock(boolean x, boolean y)  {
      setX(x);
      setY(y);
    }
    
    public Lock(Lock src) {
      x = src.x;
      y = src.y;
    }
    
    void setX(boolean x) {
      this.x = x;
    }
    
    boolean getX() {
      return x;
    }

    void setY(boolean y) {
      this.y = y;
    }
    
    boolean getY() {
      return y;
    }
  };
  
  public interface Listener {
    public void onMove(VirtualJoystick joystick);
  }
  
  protected Position position = new Position();
  protected Lock lock = new Lock();
  protected Drawable thumb = null;
  protected int thumbRadius = 50;
  protected float lineWidth = 5.0f;
  protected int lineColor = Color.BLACK;
  protected float lineAlpha = 0.3f;
  protected List listeners = new ArrayList();
  
  private boolean dragging = false;
  
  public VirtualJoystick(Context context) {
    super(context);
  }
  
  public VirtualJoystick(Context context, AttributeSet attrs) {
    super(context, attrs);

    TypedArray attrArray = context.getTheme().obtainStyledAttributes(
      attrs, R.styleable.VirtualJoystick, 0, 0);

    try {
      thumb = attrArray.getDrawable(R.styleable.VirtualJoystick_thumb);
      thumbRadius = attrArray.getDimensionPixelSize(
        R.styleable.VirtualJoystick_thumbRadius, thumbRadius);
      lock.setX(attrArray.getBoolean(R.styleable.VirtualJoystick_lockX,
        lock.getX()));
      lock.setY(attrArray.getBoolean(R.styleable.VirtualJoystick_lockY,
        lock.getY()));
      lineWidth = attrArray.getFloat(R.styleable.VirtualJoystick_lineWidth,
        lineWidth);
      lineColor = attrArray.getColor(R.styleable.VirtualJoystick_lineColor,
        lineColor);
      lineAlpha = attrArray.getFloat(R.styleable.VirtualJoystick_lineAlpha,
        lineAlpha);
    } finally {
      attrArray.recycle();
    }
  }
    
  public void setPosition(Position position) {
    float x = position.getX();
    float y = position.getY();
  
    if (lock.getX())
      x = 0.0f;
    if (lock.getY())
      y = 0.0f;
  
    if (x > 1.0f)
      x = 1.0f;
    else if (x < -1.0f)
      x = -1.0f;
  
    if (y > 1.0f)
      y = 1.0f;
    else if (y < -1.0f)
      y = -1.0f;
      
    float norm = (float)Math.sqrt(x*x+y*y);
    if (norm > 1.0f) {
      x /= norm;
      y /= norm;
    }
  
    if ((this.position.getX() != x) || (this.position.getY() != y)) {
      this.position.setX(x);
      this.position.setY(y);

      invalidate();
      notifyListeners();
    }
  }
  
  public void setPosition(float x, float y) {
    setPosition(new Position(x, y));
  }
  
  public Position getPosition() {
    return position;
  }
  
  public float getPosition(Axis axis) {
    if (axis == Axis.Y)
      return position.getY();
    else
      return position.getX();
  }
  
  public void setLock(Lock lock) {
    if (this.lock != lock) {
      this.lock = lock;
      setPosition(position);
    }
  }
  
  public Lock getLock() {
    return lock;
  }
  
  public boolean getLock(Axis axis) {
    if (axis == Axis.Y)
      return lock.getY();
    else
      return lock.getX();
  }
  
  public void setThumb(Drawable thumb) {
    this.thumb = thumb;
    invalidate();
  }
  
  public Drawable getThumb() {
    return thumb;
  }
  
  public void setThumbRadius(int thumbRadius) {
    this.thumbRadius = thumbRadius;
    invalidate();
  }
  
  public int getThumbRadius() {
    return thumbRadius;
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
  
  public float getRadius() {
    return 0.5f*Math.min(getWidth(), getHeight())-thumbRadius;
  }
  
  public void addListener(Listener listener) {
    listeners.add(listener);
  }
  
  public void removeListeners(Listener listener) {
    listeners.remove(listener);
  }
  
  public void lock(Axis axis) {
    if (axis == Axis.Y)
      setLock(new Lock(lock.getX(), true));
    else
      setLock(new Lock(true, lock.getY()));
  }
  
  public void unlock(Axis axis) {
    if (axis == Axis.Y)
      setLock(new Lock(lock.getX(), false));
    else
      setLock(new Lock(false, lock.getY()));
  }
  
  @Override
  protected void onDraw(Canvas canvas) {
    super.onDraw(canvas);
    
    float width = getWidth();
    float height = getHeight();
    float radius = getRadius();
    float thumbRadius = getThumbRadius();

    Paint paint = new Paint();
    Path path = new Path();
    paint.setColor(lineColor);
    paint.setAlpha((int)(lineAlpha*255.0f));
    paint.setStyle(Paint.Style.STROKE);
    paint.setFlags(Paint.ANTI_ALIAS_FLAG);
    paint.setStrokeWidth(lineWidth);
    paint.setStrokeCap(Paint.Cap.ROUND);
    path.addCircle(0.5f*width, 0.5f*height, radius, Path.Direction.CW);
    path.moveTo(0.5f*width-radius, 0.5f*height);
    path.lineTo(0.5f*width+radius, 0.5f*height);
    path.moveTo(0.5f*width, 0.5f*height-radius);
    path.lineTo(0.5f*width, 0.5f*height+radius);
    canvas.drawPath(path, paint);
      
    if (thumb != null) {
      thumb.setBounds(
        (int)(0.5f*width+position.getX()*radius-thumbRadius),
        (int)(0.5f*height+position.getY()*radius-thumbRadius),
        (int)(0.5f*width+position.getX()*radius+thumbRadius),
        (int)(0.5f*height+position.getY()*radius+thumbRadius));
      thumb.draw(canvas);
    }
  }
  
  @Override
  public boolean onTouchEvent(MotionEvent event) {
    float width = getWidth();
    float height = getHeight();
    float radius = getRadius();
    float thumbRadius = getThumbRadius();
    float touchX = event.getX()-0.5f*width;
    float touchY = event.getY()-0.5f*height;
    float touchRadius = (float)Math.sqrt(touchX*touchX+touchY*touchY);
    
    if (event.getAction() == MotionEvent.ACTION_DOWN) {
      if(touchRadius < thumbRadius)
        dragging = true;
    }
    else if (event.getAction() == MotionEvent.ACTION_UP)
      dragging = false;
      
    if(dragging)
      setPosition(touchX/radius, touchY/radius);      
    else
      setPosition(0.0f, 0.0f);
        
    return true;
  }
  
  @Override
  protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {    
    int widthMode = MeasureSpec.getMode(widthMeasureSpec);
    int heightMode = MeasureSpec.getMode(heightMeasureSpec);
    int width = MeasureSpec.getSize(widthMeasureSpec);
    int height = MeasureSpec.getSize(heightMeasureSpec);

    if ((widthMode != MeasureSpec.EXACTLY) && 
        (heightMode != MeasureSpec.UNSPECIFIED))
      width = height;
    else if ((heightMode != MeasureSpec.EXACTLY) && 
        (widthMode != MeasureSpec.UNSPECIFIED))
      height = width;
    
    setMeasuredDimension(width, height);
  }
  
  private void notifyListeners() {
    Iterator it = listeners.iterator();
    
    while (it.hasNext())
      ((Listener)it.next()).onMove(this);
  }
}
