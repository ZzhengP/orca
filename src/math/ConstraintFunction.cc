#include <orca/math/ConstraintFunction.h>
using namespace orca::math;

void ConstraintFunction::print() const
{
    std::cout << "Constraint Matrix C size " << getSize() << std::endl;
    std::cout << getConstraintMatrix() << std::endl;
    std::cout << "Lower Bound lb" << std::endl;
    std::cout << getLowerBound() << std::endl;
    std::cout << "Upper Bound ub" << std::endl;
    std::cout << getUpperBound() << std::endl;
}

const Eigen::VectorXd& ConstraintFunction::getLowerBound() const
{
    return lower_bound_;
}

const Eigen::VectorXd& ConstraintFunction::getUpperBound() const
{
    return upper_bound_;
}

Eigen::VectorXd& ConstraintFunction::lowerBound()
{
    return lower_bound_;
}

Eigen::VectorXd& ConstraintFunction::upperBound()
{
    return upper_bound_;
}

const Eigen::MatrixXd& ConstraintFunction::getConstraintMatrix() const
{
    return C_;
}

Eigen::MatrixXd& ConstraintFunction::constraintMatrix()
{
    return C_;
}

void ConstraintFunction::resize(int new_rows,int new_cols)
{
    const int old_rows = C_.rows();
    const int old_cols = C_.cols();
    
    bool rows_changed = ( old_rows != new_rows );
    bool cols_changed = ( old_cols != new_cols );

    if(not rows_changed and not cols_changed)
    {
        // No Need to resize anything
        return;
    }
    
    // Add new rows only
    if( rows_changed and not cols_changed )
    {
        // Keep old data, and add zeros if matrix is growing
        C_.conservativeResize(new_rows,Eigen::NoChange);
        
        // If matrix if growing
        if(new_rows > old_rows)
        {
            // Initialize the new rows at zero
            C_.block(old_rows,0,new_rows - old_rows,old_cols).setZero();
            lower_bound_.conservativeResizeLike( Eigen::VectorXd::Constant(new_rows,   - math::Infinity) );
            upper_bound_.conservativeResizeLike( Eigen::VectorXd::Constant(new_rows,     math::Infinity) );
        }
        else
        {
            // If we remove rows
            lower_bound_.conservativeResize(new_rows);
            lower_bound_.conservativeResize(new_rows);
        }
    }
    // Add new cols only
    // Size of lower/upper band does not change
    if( not rows_changed and cols_changed )
    {
        C_.conservativeResize(Eigen::NoChange,new_cols);
        if(new_cols > old_cols)
        {
            // Initialize the new cols at zero
            C_.block(0,old_cols,old_rows,new_cols-old_cols).setZero();
        }
    }
    
    // Add both
    if( rows_changed and cols_changed )
    {
        C_.conservativeResizeLike( Eigen::MatrixXd::Zero(new_rows,new_cols) );
        lower_bound_.conservativeResizeLike( Eigen::VectorXd::Constant(new_rows,   - math::Infinity) );
        upper_bound_.conservativeResizeLike( Eigen::VectorXd::Constant(new_rows,     math::Infinity) );
        // add rows and remove cols
        // if(new_rows > old_rows and new_cols < old_cols)
        // {
        //     
        // }
        // // remove rows and remove cols
        // if(new_rows < old_rows and new_cols < old_cols)
        // {
        //     
        // }
        // // remove rows and add cols
        // if(new_rows < old_rows and new_cols > old_cols)
        // {
        //     
        // }
        // // add rows and add cols
        // if(new_rows > old_rows and new_cols > old_cols)
        // {
        //     
        // }
    }
}

Size ConstraintFunction::getSize() const
{
    return Size(C_.rows(),C_.cols());
}

int ConstraintFunction::rows() const
{
    return C_.rows();
}

int ConstraintFunction::cols() const
{
    return C_.cols();
}
