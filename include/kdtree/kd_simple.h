#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <memory>
#include <sys/time.h>

double getTime() {
  struct timeval tv;
  gettimeofday(&tv, 0);
  return (double)tv.tv_sec*1000.f+(double)tv.tv_usec*1e-3;
}

template <typename VectorType_>
struct KDNode_{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  using VectorType=VectorType_;
  static constexpr int Dim=VectorType::RowsAtCompileTime;
  using Scalar=typename VectorType::Scalar;
  using ContainerType=std::vector<VectorType_, Eigen::aligned_allocator<VectorType> >;
  using KDNode=KDNode_<VectorType>;
  using KDNodePtr=std::unique_ptr<KDNode_<VectorType>>;
  ContainerType& _container;
  VectorType _mean;
  VectorType _direction;
  VectorType _normal;
  size_t _idx_start=0, _idx_end=0;
  KDNodePtr _left, _right;

  inline size_t size() const {return _idx_end-_idx_start;}
  
  inline bool whichSide(const VectorType& v) const {
    return _direction.dot(v-_mean)>0;
  }

  inline bool isLeaf() const {
    return !_left && !_right;
  }
  
  inline size_t splitRange() const{
    int s=_idx_start;
    int e=_idx_end-1;
    while (s<=e) {
      bool left_or_right=whichSide(_container[s]);
      if (left_or_right) {
        ++s;
      } else {
        std::swap(_container[s], _container[e]);
        --e;
      }
    }
    return s;
  }

  void label(){
    VectorType accumulator=VectorType::Zero();
    using CovarianceType=Eigen::Matrix<Scalar, Dim, Dim>;
    CovarianceType cov=CovarianceType::Zero();
    for (size_t i=_idx_start; i<_idx_end; ++i) {
      auto& v=_container[i];
      accumulator+=v;
      cov+=v*v.transpose();
    }
    _mean=accumulator/size();
    cov *= 1./(size());
    cov-=_mean*_mean.transpose();
    static Eigen::SelfAdjointEigenSolver<CovarianceType> eig;
    eig.computeDirect(cov);
    _direction=eig.eigenvectors().col(Dim-1);
    _normal=eig.eigenvectors().col(0);
  }

  KDNode_(ContainerType& cont, size_t start_range, size_t end_range, int min_points=10):
    _container(cont),
    _left(nullptr),
    _right(nullptr)
  {
    using namespace std;
    _idx_start=start_range;
    _idx_end=end_range;
    label();
    if (size()<=min_points) {
      //cerr << this << " is_leaf " << size() <<endl;
      return;
    }
    auto middle=splitRange();
    // cerr << this << " idx_start: " << _idx_start << " _idx_end: " << _idx_end << " _middle: " << middle << endl;
    _left.reset(new KDNode(_container, _idx_start, middle, min_points));
    _right.reset(new KDNode(_container, middle, _idx_end, min_points));
  }
  
  KDNode_(ContainerType& c, int min_points=10):
    KDNode_(c, 0, c.size(), min_points) {}
  
  inline size_t searchFast(const VectorType& v) const {
    const KDNode* aux=this;
    while (! aux->isLeaf()) {
      aux = aux->whichSide(v) ? aux->_left.get() : aux->_right.get();
    }
    Scalar min_distance=std::numeric_limits<Scalar>::max();
    size_t min_idx=0;
    for (auto i=aux->_idx_start; i<aux->_idx_end; ++i){
      Scalar d2=(v-_container[i]).squaredNorm();
      if (d2<min_distance) {
        min_distance=d2;
        min_idx=i;
      }
    }
    return min_idx;
  }
};
