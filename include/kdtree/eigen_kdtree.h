#include <memory>
#include "split.h"
#include "eigen_covariance.h"
#include "brute_force_search.h"

template <typename IteratorType_>
class TreeNode_ {
public:
  using IteratorType = IteratorType_;
  using PointType = typename IteratorType_::value_type;
  using Scalar    = typename PointType::Scalar;
  static constexpr int Dim = PointType::RowsAtCompileTime;
  using CovarianceType = Eigen::Matrix<Scalar, Dim-1, Dim-1>;
  using ThisType  = TreeNode_<IteratorType>;
  using PtrType = std::unique_ptr < ThisType  >;
  using AnswerType = std::vector<PointType* >;
  
  TreeNode_(IteratorType begin_,
            IteratorType end_,
            int max_points_in_leaf=20):
    _begin(begin_),
    _end(end_)
  {
    int num_points=std::distance(_begin, _end);
    if (num_points < max_points_in_leaf)
      return;
    CovarianceType cov;
    computeMeanAndCovariance(_mean, cov, _begin, _end);
    _normal = largestEigenVector(cov);

    IteratorType middle = split(_begin, _end,
                                [&](const PointType& v)->bool {
                                  return (v.tail(Dim-1)-_mean).dot(_normal) < Scalar(0);
                                }
                                );
    _left  = PtrType(new ThisType(_begin, middle, max_points_in_leaf) );
    _right = PtrType(new ThisType(middle, _end,   max_points_in_leaf) );
  }

  void fastSearch(AnswerType& answers,
                  const PointType& query,
                  Scalar norm) {
    if (! _left && !_right) {
      bruteForceSearch(answers, _begin, _end, query, norm);
      return;
    }
    Scalar distance_from_split_plane =  (query.tail(Dim-1)-_mean).dot(_normal);
    if (distance_from_split_plane<Scalar(0))
      _left->fastSearch(answers,query,norm);
    else
      _right->fastSearch(answers,query,norm);
  }

  void fullSearch(AnswerType& answers,
                  const PointType& query,
                  Scalar norm) {
    if (! _left && !_right) {
      bruteForceSearch(answers, _begin, _end, query, norm);
      return;
    }
    Scalar distance_from_split_plane =  (query.tail(Dim-1)-_mean).dot(_normal);
    if (distance_from_split_plane < -norm )
      _left->fullSearch(answers,query, norm);
    else if (distance_from_split_plane > norm )
      _right->fullSearch(answers,query,norm);
    else {
     _left->fullSearch(answers,query, norm);
      _right->fullSearch(answers,query,norm);
    }
  }


  PointType* fullSearchCustom( const PointType& query,
                                  Scalar norm) {
    if (! _left && !_right) {
      return bruteForceSearchCustom(_begin, _end, query, norm);
    }
    Scalar distance_from_split_plane =  (query.tail(Dim-1)-_mean).dot(_normal);

    if (distance_from_split_plane < -norm ){
      return _left->fullSearchCustom(query, norm);
    }
    else if (distance_from_split_plane > norm ){
      return _right->fullSearchCustom(query, norm);
    }

    PointType* point_left  = _left->fullSearchCustom(query, norm);
    PointType* point_right = _right->fullSearchCustom(query, norm);
    Scalar distance_left = norm*norm;
    Scalar distance_right = norm*norm;

    if (point_left)
      distance_left=((*point_left)-query).tail(Dim-1).squaredNorm();
    if (point_right)
      distance_right=((*point_right)-query).tail(Dim-1).squaredNorm();
    if (distance_left < distance_right){
      return point_left;
    }
    else{
      return point_right;
    }
  }

protected:
  Eigen::Matrix<Scalar,Dim-1, 1> _mean, _normal;
  IteratorType _begin, _end;
  PtrType _left, _right;
};
