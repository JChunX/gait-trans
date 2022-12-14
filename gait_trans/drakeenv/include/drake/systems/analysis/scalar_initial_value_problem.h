#pragma once

#include <memory>
#include <optional>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/initial_value_problem.h"
#include "drake/systems/analysis/scalar_view_dense_output.h"

namespace drake {
namespace systems {

/// A thin wrapper of the InitialValueProblem class to provide a simple
/// interface when solving scalar initial value problems i.e. when evaluating
/// the x(t; ๐ค) solution function to the given ODE dx/dt = f(t, x; ๐ค),
/// where f : t โจฏ x โ  โ , t โ โ, x โ โ, ๐ค โ โแต, along with an initial
/// condition x(tโ; ๐ค) = xโ. The parameter vector ๐ค allows for generic IVP
/// definitions, which can later be solved for any instance of said vector.
///
/// Note the distinction from general initial value problems where
/// f : t โจฏ ๐ฑ โ โโฟ and ๐ฑ โ โโฟ, addressed by the class being wrapped. While
/// every scalar initial value problem could be written in vector form, this
/// wrapper keeps both problem definition and solution in their scalar form
/// with almost zero overhead, leading to clearer code if applicable.
/// Moreover, this scalar form facilitates single-dimensional quadrature
/// using methods for solving initial value problems.
///
/// See InitialValueProblem class documentation for information on caching
/// support and dense output usage for improved efficiency in scalar IVP
/// solving.
///
/// For further insight into its use, consider the following examples of scalar
/// IVPs:
///
/// - The population growth of an hypothetical bacteria colony is described
///   by dN/dt = r * N. The colony has Nโ subjects at time tโ. In this
///   context, x โ N, xโ โ Nโ, ๐ค โ [r], dx/dt = f(t, x; ๐ค) = ๐คโ * x.
///
/// - The charge Q stored in the capacitor of a (potentially equivalent) series
///   RC circuit driven by a time varying voltage source E(t) can be described
///   by dQ/dt = (E(t) - Q / Cs) / Rs, where Rs refers to the resistor's
///   resistance and Cs refers to the capacitor's capacitance. In this context,
///   and assuming an initial stored charge Qโ at time tโ, x โ Q, ๐ค โ [Rs, Cs],
///   xโ โ Qโ, dx/dt = f(t, x; ๐ค) = (E(t) - x / ๐คโ) / ๐คโ.
///
/// @tparam_nonsymbolic_scalar
template <typename T>
class ScalarInitialValueProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScalarInitialValueProblem);

  /// Scalar ODE dx/dt = f(t, x; ๐ค) function type.
  ///
  /// @param t The independent variable t โ โ .
  /// @param x The dependent variable x โ โ .
  /// @param k The parameter vector ๐ค โ โแต.
  /// @return The derivative dx/dt โ โ.
  using ScalarOdeFunction =
      std::function<T(const T& t, const T& x, const VectorX<T>& k)>;

  /// Constructs a scalar IVP described by the given @p scalar_ode_function,
  /// using given @p x0 as initial conditions, and parameterized with @p k.
  ///
  /// @param scalar_ode_function The ODE function f(t, ๐ฑ; ๐ค) that describes
  /// the state evolution over time. @param x0 The initial state ๐ฑโ โ โ.
  /// @param k The parameter vector ๐ค โ โแต.  By default m=0 (no parameters).
  ScalarInitialValueProblem(
      const ScalarOdeFunction& scalar_ode_function, const T& x0,
      const Eigen::Ref<const VectorX<T>>& k = Vector0<T>{});

  /// Solves the IVP from time @p t0 up to time @p tf, using the initial state
  /// ๐ฑโ and parameter vector ๐ค provided in the constructor.
  /// @throws std::exception if t0 > tf.
  T Solve(const T& t0, const T& tf) const;

  /// Solves and yields an approximation of the IVP solution x(t; ๐ค) for the
  /// closed time interval between the initial time @p t0 and the final time @p
  /// tf, using initial state ๐ฑโ and parameter vector ๐ค provided in the
  /// constructor.
  ///
  /// To this end, the wrapped IntegratorBase instance solves this IVP,
  /// advancing time and state from tโ and ๐ฑโ = ๐ฑ(@p t0) to @p tf and ๐ฑ(@p
  /// tf), creating a dense output over that [@p t0, @p tf] interval along the
  /// way.
  ///
  /// @param tf The IVP will be solved up to this time, which must be โฅ @p t0.
  /// Usually, @p t0 < @p tf as an empty dense output would result if @p t0 =
  /// @p tf.
  /// @returns A dense approximation to ๐ฑ(t; ๐ค) with ๐ฑ(t0; ๐ค) = ๐ฑโ,
  /// defined for t0 โค t โค tf.
  /// @note The larger the given @p tf value is, the larger the approximated
  ///       interval will be. See documentation of the specific dense output
  ///       technique in use for reference on performance impact as this
  ///       interval grows.
  /// @throws std::exception if t0 > tf.
  std::unique_ptr<ScalarDenseOutput<T>> DenseSolve(const T& t0,
                                                   const T& tf) const;

  /// Resets the internal integrator instance by in-place
  /// construction of the given integrator type.
  ///
  /// A usage example is shown below.
  /// @code{.cpp}
  ///    scalar_ivp.reset_integrator<RungeKutta2Integrator<T>>(max_step);
  /// @endcode
  ///
  /// @param args The integrator type-specific arguments.
  /// @returns The new integrator instance.
  /// @tparam Integrator The integrator type, which must be an
  ///                    IntegratorBase subclass.
  /// @tparam Args The integrator specific argument types.
  /// @warning This operation invalidates pointers returned by
  ///          ScalarInitialValueProblem::get_integrator() and
  ///          ScalarInitialValueProblem::get_mutable_integrator().
  template <typename Integrator, typename... Args>
  Integrator* reset_integrator(Args&&... args) {
    return vector_ivp_->template reset_integrator<Integrator>(
        std::forward<Args>(args)...);
  }

  /// Gets a reference to the internal integrator instance.
  const IntegratorBase<T>& get_integrator() const {
    return vector_ivp_->get_integrator();
  }

  /// Gets a mutable reference to the internal integrator instance.
  IntegratorBase<T>& get_mutable_integrator() {
    return vector_ivp_->get_mutable_integrator();
  }

 private:
  // Vector IVP representation of this scalar IVP.
  std::unique_ptr<InitialValueProblem<T>> vector_ivp_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ScalarInitialValueProblem)
