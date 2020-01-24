/**
 * @file
 * @brief Serialization functions for serializing Eigen matrices and arrays
 *        with cereal.
 * @authors Azoth, eudoxos
 * @date 2020-01-15
 * @license CC BY-SA 4.0
 * @todo Move this to some "serialization tools" package.
 *
 * Taken from https://stackoverflow.com/a/51944389/2095383 with minor
 * modifications.
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
save(Archive& archive, const Eigen::PlainObjectBase<Derived>& object)
{
    typedef Eigen::PlainObjectBase<Derived> ArrT;

    // only add dimensions to the serialized data when they are dynamic
    if (ArrT::RowsAtCompileTime == Eigen::Dynamic)
    {
        archive(object.rows());
    }
    if (ArrT::ColsAtCompileTime == Eigen::Dynamic)
    {
        archive(object.cols());
    }

    archive(binary_data(object.data(),
                        object.size() * sizeof(typename Derived::Scalar)));
}

template <class Archive, class Derived>
inline typename std::enable_if<
    traits::is_input_serializable<BinaryData<typename Derived::Scalar>,
                                  Archive>::value,
    void>::type
load(Archive& archive, Eigen::PlainObjectBase<Derived>& object)
{
    typedef Eigen::PlainObjectBase<Derived> ArrT;

    Eigen::Index rows = ArrT::RowsAtCompileTime, cols = ArrT::ColsAtCompileTime;
    // information about dimensions are only serialized for dynamic-size types
    if (rows == Eigen::Dynamic)
    {
        archive(rows);
    }
    if (cols == Eigen::Dynamic)
    {
        archive(cols);
    }

    object.resize(rows, cols);
    archive(binary_data(object.data(),
                        static_cast<std::size_t>(
                            rows * cols * sizeof(typename Derived::Scalar))));
}
}  // namespace cereal
