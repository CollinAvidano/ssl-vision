#include "plugin_greyscale.h"
#include <image.h>
#include <iostream>

PluginGreyscale::PluginGreyscale(FrameBuffer *buffer)
    : VisionPlugin(buffer), settings(new VarList("Greyscale")) {}

ProcessResult PluginGreyscale::process(FrameData *data,
                                       RenderOptions *options) {

  Image<raw8> *img_greyscale;
  if ((img_greyscale = reinterpret_cast<Image<raw8> *>(
           data->map.get("greyscale"))) == nullptr) {
    img_greyscale = reinterpret_cast<Image<raw8> *>(
        data->map.insert("greyscale", new Image<raw8>()));
  }

  // make sure image is allocated:
  //
  // TODO(dschwab): Does this check if allocated or does it always allocate?
  img_greyscale->allocate(data->video.getWidth(), data->video.getHeight());

  // TODO(dschwab): Add options to control greyscale conversion
  // formula. For now just use pixel averaging.
  //
  // TODO(dschwab): Would probably be faster to just loop over the raw
  // pixel buffer, issue is that the data might not be in rgb format.
  for (int i = 0; i < data->video.getWidth(); ++i) {
    for (int j = 0; j < data->video.getHeight(); ++j) {
      const auto pixel = data->video.getRgb(i, j);
      // TODO(dschwab): Might want to do averaging as floats and then round
      *(img_greyscale->getPixelPointer(i, j)) =
          (pixel.r + pixel.g + pixel.b) / 3;
    }
  }

  return ProcessingOk;
}

VarList *PluginGreyscale::getSettings() { return settings.get(); }

std::string PluginGreyscale::getName() { return "Greyscale"; }
