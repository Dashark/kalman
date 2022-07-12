#ifndef KALMAN_TRACKING_LIDAR_SYSTEMMODEL_HPP_
#define KALMAN_TRACKING_LIDAR_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>

namespace KalmanTracking
{
namespace LidarTarget
{

/**
 * @brief System state vector-type for a Lidar object
 *
 * This is a system state for a very simple Lidar object that
 * is characterized by its (x,y,z)-Position.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 7>
{
public:
    KALMAN_VECTOR(State, T, 7)
    
    //! X-position
    static constexpr size_t X = 0;
    //! Y-Position
    static constexpr size_t Y = 1;
    //! Z-Position
    static constexpr size_t Z = 2;
    //! Length
    static constexpr size_t L = 3;
    //! Width
    static constexpr size_t W = 4;
    //! Height
    static constexpr size_t H = 5;
    //! Intensity
    static constexpr size_t I = 6;
    
    T x()       const { return (*this)[ X ]; }
    T y()       const { return (*this)[ Y ]; }
    T z()       const { return (*this)[ Z ]; }
    T length()       const { return (*this)[ L ]; }
    T width()        const { return (*this)[ W ]; }
    T height()       const { return (*this)[ H ]; }
    T intensity()    const { return (*this)[ I ]; }
    
    T& x()      { return (*this)[ X ]; }
    T& y()      { return (*this)[ Y ]; }
    T& z()      { return (*this)[ Z ]; }
    T& length()       { return (*this)[ L ]; }
    T& width()        { return (*this)[ W ]; }
    T& height()       { return (*this)[ H ]; }
    T& intensity()    { return (*this)[ I ]; }
};

/**
 * @brief System control-input vector-type for a Lidar object
 *
 * This is the system control-input of a very simple Lidar object.
 * 它控制目标在三维方向上的变化量，它们同时隐含了目标的运动方向
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 7>
{
public:
    KALMAN_VECTOR(Control, T, 7)
    
    //! delta X velocity
    static constexpr size_t dX = 0;
    //! delta Y velocity
    static constexpr size_t dY = 1;
    //! delta Z velocity
    static constexpr size_t dZ = 2;
    //! delta X acceleration
    static constexpr size_t ddX = 3;
    //! delta Y acceleration
    static constexpr size_t ddY = 4;
    //! delta Z acceleration
    static constexpr size_t ddZ = 5;
    //! delta Intensity
    static constexpr size_t dI = 6;
    
    T dx()       const { return (*this)[ dX ]; }
    T dy()       const { return (*this)[ dY ]; }
    T dz()       const { return (*this)[ dZ ]; }
    T ddx()       const { return (*this)[ ddX ]; }
    T ddy()       const { return (*this)[ ddY ]; }
    T ddz()       const { return (*this)[ ddZ ]; }
    T di()       const { return (*this)[ dI ]; }
    
    T& dx()      { return (*this)[ dX ]; }
    T& dy()      { return (*this)[ dY ]; }
    T& dz()      { return (*this)[ dZ ]; }
    T& ddx()      { return (*this)[ ddX ]; }
    T& ddy()      { return (*this)[ ddY ]; }
    T& ddz()      { return (*this)[ ddZ ]; }
    T& di()      { return (*this)[ dI ]; }
};

/**
 * @brief System model for a simple Lidar object
 *
 * This is the system model defining how our Lidar object moves from one 
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef KalmanTracking::LidarTarget::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanTracking::LidarTarget::Control<T> C;
    
    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_;
        
        // New x-position given by old x-position plus change in x-direction
        x_.x() = x.x() + u.dx();
        x_.y() = x.y() + u.dy();
        x_.z() = x.z() + u.dz();
        x_.length() = x.length() + u.dl();
        x_.width() = x.width() + u.dw();
        x_.height() = x.height() + u.dh();
        x_.intensity() = x.intensity() + u.di();
        
        // Return transitioned state vector
        return x_;
    }
    
protected:
    /** 不简单，线性模型按照非线性泰勒级数展开，每次都要更新矩阵F，不更新是纯线性方程
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    /*
    void updateJacobians( const S& x, const C& u )
    {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setZero();
        
        // partial derivative of x.x() w.r.t. x.x()
        this->F( S::X, S::X ) = 1;
        // partial derivative of x.x() w.r.t. x.theta()
        this->F( S::X, S::THETA ) = -std::sin( x.theta() + u.dtheta() ) * u.v();
        
        // partial derivative of x.y() w.r.t. x.y()
        this->F( S::Y, S::Y ) = 1;
        // partial derivative of x.y() w.r.t. x.theta()
        this->F( S::Y, S::THETA ) = std::cos( x.theta() + u.dtheta() ) * u.v();
        
        // partial derivative of x.theta() w.r.t. x.theta()
        this->F( S::THETA, S::THETA ) = 1;
        
        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
        // TODO: more sophisticated noise modelling
        //       i.e. The noise affects the the direction in which we move as 
        //       well as the velocity (i.e. the distance we move)
    }
    */
};

} // namespace LidarTarget
} // namespace KalmanTracking

#endif