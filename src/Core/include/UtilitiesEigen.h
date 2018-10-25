//
//  UtilitiesEigen.h
//  Gauss
//
//  Created by David Levin on 2/11/17.
//
//

#ifndef UtilitiesEigen_h
#define UtilitiesEigen_h

#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/src/SparseCore/SparseSelfAdjointView.h>
#include <Utilities.h>
#include <World.h>

//Modal Analysis using Spectra

#include <stdexcept>

//some useful types
namespace Eigen {
    template<typename DataType>
    using Vector3x = Eigen::Matrix<DataType, 3,1>;
    
    template<typename DataType>
    using Vector6x = Eigen::Matrix<DataType, 6,1>;
    
    template<typename DataType>
    using VectorXx = Eigen::Matrix<DataType, Eigen::Dynamic, 1>;
    
    template<typename DataType>
    using MatrixXx = Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>;
    
    template<typename DataType>
    using Matrix33x = Eigen::Matrix<DataType, 3, 3>;
    
    //Comes up a lot in constitutive models
    template<typename DataType>
    using Matrix66x = Eigen::Matrix<DataType, 6,6>;
    
    //useful maps
    template<typename DataType>
    using Map3x = Eigen::Map<Vector3x<DataType> >;
    
    
}

namespace Gauss {
    //state ptr direct to eigen map for a single property (position or velocity)
    template<unsigned int Property, typename World>
    Eigen::Map<Eigen::VectorXd> mapStateEigen(World &world) {
        std::tuple<double *, unsigned int> ptr = world.getState().template getStatePtr<Property>();
        return Eigen::Map<Eigen::VectorXd>(std::get<0>(ptr), std::get<1>(ptr));
    }
    
    //state ptr for the whole thing
    template<typename World>
    Eigen::Map<Eigen::VectorXd> mapStateEigen(World &world) {
        std::tuple<double *, unsigned int> ptr = world.getState().getStatePtr();
        return Eigen::Map<Eigen::VectorXd>(std::get<0>(ptr), std::get<1>(ptr));
    }
    
    template<typename DOF, typename DataType, typename ...SystemTypes, typename ...ForceTypes, typename ...ConstraintTypes>
    inline Eigen::Map<Eigen::VectorXd> mapDOFEigen(DOF &dof, World<DataType,
                                                   std::tuple<SystemTypes...>,
                                                   std::tuple<ForceTypes...>,
                                                   std::tuple<ConstraintTypes...> > &world) {
        std::tuple<double *, unsigned int> qPtr = dof.getPtr(world.getState());
        //set position DOF and check
        return Eigen::Map<Eigen::VectorXd>(std::get<0>(qPtr), dof.getNumScalarDOF());
        
    }
    
    template<typename DOF, typename DataType>
    inline Eigen::Map<Eigen::VectorXd> mapDOFEigen(DOF &dof, const State<DataType> &state) {
        std::tuple<double *, unsigned int> qPtr = dof.getPtr(state);
        //set position DOF and check
        return Eigen::Map<Eigen::VectorXd>(std::get<0>(qPtr), dof.getNumScalarDOF());
    }
    
    //functor for getting position of a DOF
    template <typename DataType, typename DOF>
    class PositionEigen {
    public:
        inline PositionEigen(DOF *dof=nullptr, Eigen::Vector3x<DataType> p = Eigen::Vector3x<DataType>()) { m_dof = dof; m_p = p; }
        inline Eigen::Vector3x<DataType> operator()(const State<DataType> &state) const { return m_p+ mapDOFEigen(*m_dof, state); }
        inline DOF * getDOF() { return m_dof; }
    protected:
        DOF *m_dof;
        Eigen::Vector3x<DataType> m_p;
    };
}


#endif /* UtilitiesEigen_h */
