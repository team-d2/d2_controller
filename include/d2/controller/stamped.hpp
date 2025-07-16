#ifndef D2__CONTROLLER__STAMPED_HPP_
#define D2__CONTROLLER__STAMPED_HPP_

namespace d2::controller
{

template<class Data>
struct Stamped
{
  std::uint64_t stamp_nanosec;
  Data data;
};

};

#endif // D2__CONTROLLER__STAMPED_HPP_
