#pragma once
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>

#include <libobsensor/ObSensor.hpp>

namespace vsdk = ::viam::sdk;

namespace orbbec {

void startOrbbecSDK(ob::Context& ctx);

class Orbbec final : public vsdk::Camera, public vsdk::Reconfigurable {
   public:
    Orbbec(vsdk::Dependencies deps, vsdk::ResourceConfig cfg);
    ~Orbbec();
    void reconfigure(const vsdk::Dependencies& deps, const vsdk::ResourceConfig& cfg) override;
    vsdk::ProtoStruct do_command(const vsdk::ProtoStruct& command) override;
    raw_image get_image(std::string mime_type, const vsdk::ProtoStruct& extra) override;
    image_collection get_images() override;
    point_cloud get_point_cloud(std::string mime_type, const vsdk::ProtoStruct& extra) override;
    properties get_properties() override;
    std::vector<vsdk::GeometryConfig> get_geometries(const vsdk::ProtoStruct& extra) override;

   private:
    struct state_;
    std::unique_ptr<struct state_> state_;
    std::mutex state_mu_;
    static std::unique_ptr<struct state_> configure_(vsdk::Dependencies deps, vsdk::ResourceConfig cfg);
};

}  // namespace orbbec
