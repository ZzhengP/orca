/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_ANGULAR_FORCE_VECTOR_3_H
#define IDYNTREE_ANGULAR_FORCE_VECTOR_3_H

#include <iDynTree/Core/ForceVector3.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>

namespace iDynTree
{
    class Position;
    class PositionSemantics;
    class LinearForceVector3;
    class LinearForceVector3Semantics;

    /**
     * Class providing the semantics for any angular force vector (torque or angular momentum).
     */
    class AngularForceVector3Semantics: public ForceVector3Semantics<AngularForceVector3Semantics>
    {
    protected:
        int point;

    public:
        /**
         * Constructors:
         */
        inline AngularForceVector3Semantics() {}
        AngularForceVector3Semantics(int _point, int _body, int _refBody, int _coordinateFrame);
        AngularForceVector3Semantics(const AngularForceVector3Semantics & other);

        /**
         * Semantics operations
         * Compute the semantics of the result given the semantics of the operands.
         */
        bool changePoint(const PositionSemantics & newPoint,
                         const LinearForceVector3Semantics & otherLinear,
                         AngularForceVector3Semantics & resultAngular) const;

        static bool compose(const AngularForceVector3Semantics & op1,
                            const AngularForceVector3Semantics & op2,
                            AngularForceVector3Semantics & result);
    };


    /**
     * Class providing the raw coordinates and semantics for any torque vector
     *
     * \ingroup iDynTreeCore
     *
     * A motion vector can be used to describe an angular momentum or force,
     * and implement the adjoint transformations common to these geometric relations.
     *
     */
    class AngularForceVector3: public ForceVector3<AngularForceVector3>
    {
    public:
        typedef MotionForce_traits<AngularForceVector3>::SemanticsType SemanticsType;

        /**
         * constructors
         */
        inline AngularForceVector3() {}
        AngularForceVector3(const double x, const double y, const double z);
        AngularForceVector3(const double* in_data, const unsigned int in_size);
        AngularForceVector3(const AngularForceVector3 & other);
        AngularForceVector3(const Vector3& other);

        /**
         * Geometric operations
         */
        AngularForceVector3 changePoint(const Position & newPoint,
                                        const LinearForceVector3 & otherLinear) const;
    };

    typedef AngularForceVector3 AngMomentum;
    typedef AngularForceVector3 Torque;
}

#endif /* IDYNTREE_ANGULAR_FORCE_VECTOR_3_H */
