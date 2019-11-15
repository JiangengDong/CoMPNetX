#include <openrave/plugin.h>
#include <boost/bind.hpp>

#include "Problem.h"

using namespace OpenRAVE;

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string &interfacename, std::istream &sinput, EnvironmentBasePtr penv) {
//    OpenRAVE::RaveSetDebugLevel(ATLASMPNET_DEBUGLEVEL);
    if (type == PT_Planner && interfacename == "atlasmpnet") {
        return InterfaceBasePtr(new AtlasMPNet::Problem(std::move(penv), sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO &info) {
    info.interfacenames[PT_Planner].push_back("AtlasMPNet");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin() {
}

