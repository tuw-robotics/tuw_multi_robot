#ifndef TUW_RW_VECTOR_H
#define TUW_RW_VECTOR_H

#include <vector>
#include <mutex>

namespace tuw_rqt_ordermanager
{
template <class T>
class RWVector
{
  public:
    RWVector(){}

    //TODO: shared locking

    void push_back(T t)
    {
      vec_.push_back(t);
    }

    T at(int i) const
    {
      T t = vec_.at(i);
      return t;
    }

    int size() const
    {
      int size = vec_.size();
      return size;
    }

    void clear()
    {
      vec_.clear();
    }

    void lock()
    {
      mtx_vec_.lock();
    }

    void unlock()
    {
      mtx_vec_.unlock();
    }

  private:
    std::vector<T> vec_;
    mutable std::mutex mtx_vec_;
};

}
# endif
