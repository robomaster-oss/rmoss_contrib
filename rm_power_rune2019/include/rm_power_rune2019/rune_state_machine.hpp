/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/

#ifndef RM_POWER_RUNE_2019_RUNE_STATE_MACHINE_H
#define RM_POWER_RUNE_2019_RUNE_STATE_MACHINE_H

#include <string>
#include "opencv2/opencv.hpp"
#include "rm_power_rune2019/rune_def.hpp"

namespace rm_power_rune2019
{
      enum class FmsResultCode {shoot, wait0, wait1, ignore};
      typedef struct
      {
            int state = 0; //表示当前亮灯个数1-5,0表示初始状态
            cv::Point2f point[5];
            float err[5];
            float err_thre = 60;
            float err_num = 0;
            int errmax_id = 0;
            float err_sum = 0;
            int target_err_min_id = 0;
      } RuneState;

      // 能量机关识别算法部分，状态机。
      class RuneStateMachine
      {
      public:
            RuneStateMachine(){};
            ~RuneStateMachine(){};
      public:
            FmsResultCode update(std::vector<ArmorDescriptor> &armors);
            void clear();
            ArmorDescriptor getTargetArmor();
      private:
            int calcNewState(std::vector<ArmorDescriptor> &armors, RuneState &old_state, RuneState &new_state);

      private:
            RuneState state_;
            ArmorDescriptor target_armor_;
      };
} // namespace rm_power_rune2019

#endif //RM_POWER_RUNE_2019_RUNE_STATE_MACHINE_H
