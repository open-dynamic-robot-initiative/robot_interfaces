/**
 * @file
 * @brief Serialization functions for serializing Eigen matrices and arrays
 *        with cereal.
 * @authors Azoth, eudoxos
 * @date 2020-01-15
 * @license CC BY-SA 4.0
 * @todo Move this to some "serialization tools" package.
 *
 * Taken from https://stackoverflow.com/a/51944389/2095383.
 */
#include <Eigen/Eigen>
#include <type_traits>

namespace cereal
{
template <class Archive, class Derived>
inline typename std::enable_if<
    traits::is_output_serializable<BinaryData<typename Derived::Scalar>,
                                   Archive>::value,
    void>::type
save(Archive& ar, Eigen::PlainObjectBase<Derived> const& m)
{
    typedef Eigen::PlainObjectBase<Derived> ArrT;
    if (ArrT::RowsAtCompileTime == Eigen::Dynamic) ar(m.rows());
    if (ArrT::ColsAtCompileTime == Eigen::Dynamic) ar(m.cols());
    ar(binary_data(m.data(), m.size() * sizeof(typename Derived::Scalar)));
}

template <class Archive, class Derived>
inline typename std::enable_if<
    traits::is_input_serializable<BinaryData<typename Derived::Scalar>,
                                  Archive>::value,
    void>::type
load(Archive& ar, Eigen::PlainObjectBase<Derived>& m)
{
    typedef Eigen::PlainObjectBase<Derived> ArrT;
    Eigen::Index rows = ArrT::RowsAtCompileTime, cols = ArrT::ColsAtCompileTime;
    if (rows == Eigen::Dynamic) ar(rows);
    if (cols == Eigen::Dynamic) ar(cols);
    m.resize(rows, cols);
    ar(binary_data(m.data(),
                   static_cast<std::size_t>(rows * cols *
                                            sizeof(typename Derived::Scalar))));
}
}  // namespace cereal
