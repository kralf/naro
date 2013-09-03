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

import android.content.Context;
import android.graphics.drawable.Drawable;
import android.graphics.drawable.DrawableContainer;
import android.view.View;
import android.view.ViewGroup;
import android.widget.BaseAdapter;
import android.widget.Gallery;
import android.widget.ImageView;
import android.util.Log;

public class GalleryAdapter
  extends BaseAdapter {

  private Context context;
  private DrawableContainer images;
  
  public GalleryAdapter(Context context) {
    this.context = context;
  }

  @Override
  public int getCount() {
    if (images != null)
      Log.i("NARO", String.format("%d", ((DrawableContainer.DrawableContainerState)
          images.getConstantState()).getChildCount()));
    if (images != null)
      return ((DrawableContainer.DrawableContainerState)
        images.getConstantState()).getChildCount();
    else
      return 0;
  }

  @Override
  public Object getItem(int position) {
    return null;
  }

  @Override
  public long getItemId(int position) {
    return 0;
  }

  @Override
  public View getView(int position, View convertView, ViewGroup parent) {
    Drawable image = ((DrawableContainer.DrawableContainerState)
      images.getConstantState()).getChildren()[position];

    ImageView imageView = new ImageView(this.context);
    ViewGroup.LayoutParams layoutParams = new Gallery.LayoutParams(
      ViewGroup.LayoutParams.FILL_PARENT, ViewGroup.LayoutParams.FILL_PARENT);
    imageView.setLayoutParams(layoutParams);
    imageView.setScaleType(ImageView.ScaleType.CENTER_INSIDE);
    imageView.setImageDrawable(image);

    return imageView;
  }

  public void setImagesDrawables(DrawableContainer images) {
    this.images = images;
    notifyDataSetChanged();
  }

  public void setImagesResource(int id) {
    this.images = (DrawableContainer)context.getResources().getDrawable(id);
    notifyDataSetChanged();
  }
}
