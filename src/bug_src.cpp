//
// Created by wang on 20-1-7.
//

#include "bug_header.h"

void DataProcessor::LoadResult() {
        FineRegistrationData fine_reg_data;
        Vec6 se3 = Vec6::Zero();
        fine_reg_data.se3s.emplace_back(se3);

        datas_.push_back(fine_reg_data);

}
