// This file is a part of the orca framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Main contributor(s): Antoine Hoarau, hoarau@isir.upmc.fr
//
// This software is a computer program whose purpose is to [describe
// functionalities and technical features of your software].
//
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

#pragma once

#include <orca/common/Wrench.h>
#include <orca/common/PIDFunctions.h>
#include <orca/task/GenericTask.h>

namespace orca
{
namespace task
{

using math::Vector6d;

class WrenchTask : public GenericTask, public common::PIDFunctions<6>
{
public:
    WrenchTask();

    void setBaseFrame(const std::string& base_ref_frame);

    void setControlFrame(const std::string& control_frame);

    const std::string& getBaseFrame() const;

    const std::string& getControlFrame() const;
    
    void setCurrent(const Vector6d& current_wrench_at_control_frame);
    
    void setDesired(const Vector6d& wrench_at_control_frame);

    void setIntegralDt(double dt);

    void updateAffineFunction();

    void resize();
protected:
    Vector6d wrench_des_;
    Vector6d current_wrench_;
    Vector6d integral_error_;
    double dt_ = 0;
    Eigen::MatrixXd jacobian_transpose_;
    common::Wrench wrench_;

};

}
}
