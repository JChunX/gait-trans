#pragma once

#include <memory>
#include <optional>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/dense_output.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

/// A general initial value problem (or IVP) representation class, that allows
/// evaluating the ๐ฑ(t; ๐ค) solution function to the given ODE
/// d๐ฑ/dt = f(t, ๐ฑ; ๐ค), where f : t โจฏ ๐ฑ โ โโฟ, t โ โ, ๐ฑ โ โโฟ, ๐ค โ โแต,
/// provided an initial condition ๐ฑ(tโ; ๐ค) = ๐ฑโ. The parameter vector ๐ค
/// allows for generic IVP definitions, which can later be solved for any
/// instance of said vector.
///
/// By default, an explicit 3rd order RungeKutta integration scheme is used.
///
/// The implementation of this class performs basic computation caching,
/// optimizing away repeated integration whenever the IVP is solved for
/// increasing values of time t while both initial conditions and parameters
/// are kept constant, e.g. if solved for tโ > tโ first, solving for tโ > tโ
/// will only require integrating from tโ onward.
///
/// Additionally, IntegratorBase's dense output support can be leveraged to
/// efficiently approximate the IVP solution within closed intervals of t.
/// This is convenient when there's a need for a more dense sampling of the
/// IVP solution than what would be available through either fixed or
/// error-controlled step integration (for a given accuracy), or when the IVP
/// is to be solved repeatedly for arbitrarily many t values within a given
/// interval. See documentation of the internally held IntegratorBase subclass
/// instance (either the default or a user-defined one, set via
/// reset_integrator()) for further reference on the specific dense output
/// technique in use.
///
/// For further insight into its use, consider the following examples:
///
/// - The momentum ๐ฉ of a particle of mass m that is traveling through a
///   volume of a gas with dynamic viscosity ฮผ can be described by
///   d๐ฉ/dt = -ฮผ * ๐ฉ/m. At time tโ, the particle carries an initial momentum
///   ๐ฉโ. In this context, t is unused (the ODE is autonomous), ๐ฑ โ ๐ฉ,
///   ๐ค โ [m, ฮผ], tโ = 0, ๐ฑโ โ ๐ฉโ, d๐ฑ/dt = f(t, ๐ฑ; ๐ค) = -kโ * ๐ฑ / kโ.
///
/// - The velocity ๐ฏ of the same particle in the same exact conditions as
///   before, but when a time varying force ๐(t) is applied to it, can be
///   be described by d๐ฏ/dt = (๐(t) - ฮผ * ๐ฏ) / m. In this context, ๐ฑ โ ๐ฏ,
///   ๐ค โ [m, ฮผ], ๐ฑโ โ ๐ฏโ, d๐ฑ/dt = f(t, ๐ฑ; ๐ค) = (๐(t) - kโ * ๐ฑ) / kโ.
///
/// @tparam_nonsymbolic_scalar
template <typename T>
class InitialValueProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InitialValueProblem);

  /// Default integration accuracy in the relative tolerance sense.
  static const double kDefaultAccuracy;
  /// Default initial integration step size.
  static const T kInitialStepSize;
  /// Default maximum integration step size.
  static const T kMaxStepSize;

  /// General ODE system d๐ฑ/dt = f(t, ๐ฑ; ๐ค) function type.
  ///
  /// @param t The independent scalar variable t โ โ.
  /// @param x The dependent vector variable ๐ฑ โ โโฟ.
  /// @param k The vector of parameters ๐ค โ โแต.
  /// @return The derivative vector d๐ฑ/dt โ โโฟ.
  using OdeFunction = std::function<VectorX<T>(const T& t, const VectorX<T>& x,
                                               const VectorX<T>& k)>;

  /// Constructs an IVP described by the given @p ode_function, using @p x0 as
  /// initial conditions, and parameterized with @p k.
  ///
  /// @param ode_function The ODE function f(t, ๐ฑ; ๐ค) that describes the state
  ///                     evolution over time.
  /// @param x0 The initial state vector ๐ฑโ โ โโฟ.
  /// @param k The parameter vector ๐ค โ โแต.  By default m=0 (no parameters).
  InitialValueProblem(const OdeFunction& ode_function,
                      const Eigen::Ref<const VectorX<T>>& x0,
                      const Eigen::Ref<const VectorX<T>>& k = Vector0<T>{});

  /// Solves the IVP from the initial time @p t0 up to time @p tf, using the
  /// initial state vector ๐ฑโ and parameter vector ๐ค provided in the
  /// constructor.
  /// @throws std::exception if t0 > tf.
  VectorX<T> Solve(const T& t0, const T& tf) const;

  /// Solves and yields an approximation of the IVP solution x(t; ๐ค) for the
  /// closed time interval between the given initial time @p t0 and the given
  /// final time @p tf, using initial state ๐ฑโ and parameter vector ๐ค provided
  /// in the constructor.
  ///
  /// To this end, the wrapped IntegratorBase instance solves this IVP,
  /// advancing time and state from tโ and ๐ฑโ = ๐ฑ(tโ) to @p tf and ๐ฑ(@p tf),
  /// creating a dense output over that [tโ, @p tf] interval along the way.
  ///
  /// @param tf The IVP will be solved up to this time, which must be โฅ tโ.
  /// Usually, tโ < @p tf as an empty dense output would result if tโ = @p tf.
  /// @returns A dense approximation to ๐ฑ(t; ๐ค) with ๐ฑ(tโ; ๐ค) = ๐ฑโ,
  /// defined for tโ โค t โค tf.
  /// @note The larger the given @p tf value is, the larger the approximated
  ///       interval will be. See documentation of the specific dense output
  ///       technique in use for reference on performance impact as this
  ///       interval grows.
  /// @throws std::exception if t0 > tf.
  std::unique_ptr<DenseOutput<T>> DenseSolve(const T& t0, const T& tf) const;

  /// Resets the internal integrator instance by in-place
  /// construction of the given integrator type.
  ///
  /// A usage example is shown below.
  /// @code{.cpp}
  ///    ivp.reset_integrator<RungeKutta2Integrator<T>>(max_step);
  /// @endcode
  ///
  /// @param args The integrator type-specific arguments.
  /// @returns The new integrator instance.
  /// @tparam Integrator The integrator type, which must be an
  ///         IntegratorBase subclass.
  /// @tparam Args The integrator specific argument types.
  /// @warning This operation invalidates pointers returned by
  ///          InitialValueProblem::get_integrator() and
  ///          InitialValueProblem::get_mutable_integrator().
  template <typename Integrator, typename... Args>
  Integrator* reset_integrator(Args&&... args) {
    integrator_ =
        std::make_unique<Integrator>(*system_, std::forward<Args>(args)...);
    integrator_->reset_context(context_.get());
    return static_cast<Integrator*>(integrator_.get());
  }

  /// Gets a reference to the internal integrator instance.
  const IntegratorBase<T>& get_integrator() const {
    DRAKE_DEMAND(integrator_ != nullptr);
    return *integrator_.get();
  }

  /// Gets a mutable reference to the internal integrator instance.
  IntegratorBase<T>& get_mutable_integrator() {
    DRAKE_DEMAND(integrator_ != nullptr);
    return *integrator_.get();
  }

 private:
  // Resets the context / integrator between multiple solves.
  void ResetState() const;

  // IVP ODE solver integration context.
  std::unique_ptr<Context<T>> context_;
  // IVP system representation used for ODE solving.
  std::unique_ptr<System<T>> system_;
  // Numerical integrator used for IVP ODE solving.
  std::unique_ptr<IntegratorBase<T>> integrator_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::InitialValueProblem)
