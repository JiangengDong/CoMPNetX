#include <boost/bind.hpp>
#include <openrave/plugin.h>

#include "Problem.h"

using namespace OpenRAVE;

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string &interfacename, std::istream &sinput, EnvironmentBasePtr penv) {
    if (type == PT_Planner && interfacename == "compnetx") {
        return InterfaceBasePtr(new CoMPNetX::Problem(std::move(penv), sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO &info) {
    info.interfacenames[PT_Planner].push_back("CoMPNetX");
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin() {
}
