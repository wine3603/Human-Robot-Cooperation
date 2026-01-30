#ifndef BAICHUAN_ARM_DYN_MODELS_H
#define BAICHUAN_ARM_DYN_MODELS_H

#include <moying_mcr_arm_dynamics_models/basic_dynamics_model.h>
//大族动力学模型
#include <moying_mcr_arm_dynamics_models/elfin/elfin_dynamics_models.h>

namespace baichuan{

namespace arm{

namespace dynamics{

    std::shared_ptr<BasicDynamicsModel> createDynModel(std::string arm_type, std::string mounted_way)
    {
        std::shared_ptr<BasicDynamicsModel> dyn_model;
        if (arm_type == "elfin_03")
        {
            if (mounted_way == "ground")
            {
                dyn_model = std::make_shared<elfin::Elfin3Pose1DynamicsModel>();
            }
            else if (mounted_way == "wall")
            {
                dyn_model = std::make_shared<elfin::Elfin3Pose2DynamicsModel>();
            }
        }
        else if (arm_type == "elfin_05")
        {
            if (mounted_way == "ground")
            {
                dyn_model = std::make_shared<elfin::Elfin5Pose1DynamicsModel>();
            }
        }
        else if (arm_type == "elfin_05l")
        {
            // TODO:暂时还没有5公斤加长版的动力学模型
            dyn_model = nullptr;
        }
        else if (arm_type == "elfin_10")
        {
            if (mounted_way == "ground")
            {
                dyn_model = std::make_shared<elfin::Elfin10Pose1DynamicsModel>();
            }
        }
        else if (arm_type == "elfin_10l")
        {
            // TODO:暂时还没有10公斤加长版的动力学模型
            dyn_model = nullptr;
        }
        else if (arm_type == "elfin_15")
        {
            if (mounted_way == "ground")
            {
                dyn_model = std::make_shared<elfin::Elfin15Pose1DynamicsModel>();
            }
        }
        
        return dyn_model;
    }
}
}
}
#endif
