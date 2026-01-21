#include "server/Server.hpp"
#include "scene/Scene.hpp"
#include "component/RenderComponent.hpp"
#include "Camera.hpp"

#include "SimplePathTracer.hpp"

using namespace std;
using namespace NRenderer;

namespace SimplePathTracer
{
    class Adapter : public RenderComponent
    {
        void render(SharedScene spScene) {
            SimplePathTracerRenderer renderer{spScene};
            auto renderResult = renderer.render();
            auto [ pixels, width, height ]  = renderResult;
            getServer().screen.set(pixels, width, height);
            renderer.release(renderResult);
        }
    };
}

const static string description = 
    "Photon Mapping. ";

REGISTER_RENDERER(Photon_mapping, description, SimplePathTracer::Adapter);
