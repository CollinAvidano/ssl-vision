#ifndef PLUGIN_GREYSCALE_H
#define PLUGIN_GREYSCALE_H

#include <memory>
#include <string>
#include <visionplugin.h>

class PluginGreyscale : public VisionPlugin {
protected:
  std::unique_ptr<VarList> settings;

public:
  PluginGreyscale(FrameBuffer *buffer);
  ~PluginGreyscale() override = default;

  ProcessResult process(FrameData *data, RenderOptions *options) override;

  VarList *getSettings() override;

  std::string getName() override;
};

#endif /* PLUGIN_GREYSCALE_H */
