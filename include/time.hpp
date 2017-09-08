#ifndef TIME_HPP
#define TIME_HPP

#include <utility>
#include <limits>

#include "interval.hpp"

// discrete time
using dtime_t = long long;

// dense time
using dense_t = double;

inline Interval<dtime_t> I(const dtime_t &a, const dtime_t &b)
{
	return Interval<dtime_t>{a, b};
}

// inline Interval<dense_t> I(const dense_t &a, const dense_t &b)
// {
// 	return Interval<dense_t>{a, b};
// }

namespace Time_model {

	template<typename T>
	struct constants
	{
		static constexpr T infinity()
		{
			return std::numeric_limits<T>::max();
		}

		static constexpr T epsilon()
		{
			return T(1);
		}
	};


	template<>
	struct constants<dense_t>
	{
		static constexpr dense_t infinity()
		{
			return std::numeric_limits<dense_t>::infinity();
		}

		static constexpr dense_t epsilon()
		{
			return dense_t(0);
		}

	};

}


#endif

